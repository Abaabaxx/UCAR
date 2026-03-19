import cv2
import numpy as np

# --- 参数配置区 ---
# 输入视频的路径 (请确保视频文件与此脚本在同一目录下, 或提供完整路径)
VIDEO_PATH = '/home/lby/CURSOR/follow_line/my_try/图像预处理/视频和图片/拉窗帘原始视频（平视）/7.28省.mp4'
# 高斯模糊参数
GAUSSIAN_KERNEL_SIZE = (5, 5)  # 高斯核大小
GAUSSIAN_SIGMA_X = 0  # 标准差，0表示根据核大小自动计算
# Canny边缘检测参数
CANNY_LOW_THRESHOLD = 50
CANNY_HIGH_THRESHOLD = 150
# 图像翻转参数
PERFORM_HORIZONTAL_FLIP = True  # 是否执行水平翻转
# 起始点寻找参数
START_POINT_SCAN_STEP = 10  # 向上扫描的步长（像素）
# 路径规划参数
CENTER_LINE_OFFSET = -55  # 从右边线向左偏移的像素数
# 逆透视变换矩阵（从鸟瞰图坐标到原始图像坐标的映射）
INVERSE_PERSPECTIVE_MATRIX = np.array([
    [-3.365493,  2.608984, -357.317062],
    [-0.049261,  1.302389, -874.095796],
    [ 0.000029,  0.007556,   -4.205510]
], dtype=np.float32)
# 形态学操作的卷积核
DILATE_KERNEL = np.ones((15, 15), np.uint8)  # 较大的核用于膨胀，强力连接断点
ERODE_KERNEL = np.ones((7, 7), np.uint8)   # 较小的核用于腐蚀，精细去除噪点
# 在IPM鸟瞰图上进行分析的ROI区域参数
IPM_ROI_Y = 240  # ROI起始Y坐标
IPM_ROI_H = 240  # ROI高度
IPM_ROI_X = 0    # ROI起始X坐标
IPM_ROI_W = 640  # ROI宽度

# --- 可视化开关 ---
# 设置为 True 来显示对应的处理阶段窗口，设置为 False 来隐藏
SHOW_ORIGINAL = False
SHOW_GRAYSCALE = False
SHOW_GAUSSIAN = False
SHOW_CANNY = False
SHOW_IPM_RAW = False        # 显示原始的、未经形态学处理的鸟瞰图
SHOW_IPM_MORPHED = False     # 显示经过形态学处理后的完整鸟瞰图
SHOW_FINAL_ROI = True       # 显示最终用于分析的ROI区域

# 定义沿墙走的搜索模式（Follow The Wall）
# 顺时针方向搜索，用于沿着右侧赛道内边界行走
# 这个顺序确保了我们的"机器人"能够正确地沿着赛道的内侧边界前进
FTW_SEEDS = [
    (-1, 0),    # 左
    (-1, -1),   # 左上
    (0, -1),    # 上
    (1, -1),    # 右上
    (1, 0),     # 右
    (1, 1),     # 右下
    (0, 1),     # 下
    (-1, 1)     # 左下
]

def follow_the_wall(image, start_point):
    """
    使用沿墙走(Follow The Wall)算法跟踪右侧边界
    
    参数:
    image: 二值图像
    start_point: 起始点坐标 (x, y)
    
    返回:
    boundary_points: 边界点列表
    """
    # 初始化边界点列表
    boundary_points = []
    current_point = start_point
    h, w = image.shape[:2]
    
    # 主循环，最多迭代400次以防止死循环
    for _ in range(400):
        # 将当前点添加到边界点列表中
        boundary_points.append(current_point)
        
        # 初始化候选点列表
        candidates = []
        
        # 遍历8个方向，使用索引以便获取下一个方向
        for i in range(8):
            # 获取当前邻居点A的坐标
            dx_a, dy_a = FTW_SEEDS[i]
            A_x = current_point[0] + dx_a
            A_y = current_point[1] + dy_a
            
            # 获取下一个邻居点B的坐标（使用循环索引）
            dx_b, dy_b = FTW_SEEDS[(i + 1) % 8]
            B_x = current_point[0] + dx_b
            B_y = current_point[1] + dy_b
            
            # 检查两个点是否都在图像范围内
            if (0 <= A_x < w and 0 <= A_y < h and 
                0 <= B_x < w and 0 <= B_y < h):
                # 执行"邻居配对"判断：A点为黑(0)，B点为白(255)
                if image[A_y, A_x] == 0 and image[B_y, B_x] == 255:
                    # 将黑点（A点）作为候选点添加
                    candidates.append((A_x, A_y))
        
        # 如果没有找到候选点，则中断循环
        if not candidates:
            break
        
        # 选择y坐标最小的候选点作为下一个点
        next_point = min(candidates, key=lambda p: p[1])
        current_point = next_point
    
    return boundary_points

def extract_final_border(image_height, raw_points):
    """
    从原始轮廓点集中提取每行一个点的最终边线
    
    参数:
    image_height: ROI图像的高度
    raw_points: 原始轮廓点列表 [(x1,y1), (x2,y2), ...]
    
    返回:
    final_border: 长度为image_height的数组，每个元素是该行边线的x坐标，-1表示该行没有边线
    """
    # 初始化最终边线数组，所有值设为-1表示未找到边线
    final_border = np.full(image_height, -1, dtype=int)
    # 用于记录已处理的行
    found_rows = set()
    
    # 按照原始顺序遍历轮廓点
    for x, y in raw_points:
        # 如果这一行还没有记录过边线点
        if y not in found_rows:
            # 记录这一行的x坐标
            final_border[y] = x
            # 标记这一行已处理
            found_rows.add(y)
    
    return final_border

def process_video():
    """
    加载视频，将每一帧转换为灰度图，并同时显示原始帧和灰度帧
    """
    # 计算正向透视变换矩阵（从原始图像坐标到鸟瞰图坐标的映射）
    try:
        forward_perspective_matrix = np.linalg.inv(INVERSE_PERSPECTIVE_MATRIX)
    except np.linalg.LinAlgError:
        print("错误: 提供的矩阵是奇异矩阵, 无法求逆。请检查矩阵参数。")
        return
    
    # 打开视频文件
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"错误: 无法打开视频文件 '{VIDEO_PATH}'")
        return
    
    # 获取视频的帧率(FPS)并计算帧间延迟
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        print("警告: 无法获取视频帧率, 将默认使用30 FPS。")
        fps = 30
    delay = int(1000 / fps)
    
    print("视频处理已启动... 按 'q' 键退出。")
    
    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            print("视频播放结束。")
            break
            
        # 执行水平翻转（如果启用）
        if PERFORM_HORIZONTAL_FLIP:
            frame = cv2.flip(frame, 1)
        
        # 将原始帧转换为灰度图
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 对灰度图应用高斯模糊
        blurred_frame = cv2.GaussianBlur(gray_frame, GAUSSIAN_KERNEL_SIZE, GAUSSIAN_SIGMA_X)
        
        # 应用Canny边缘检测
        canny_edges = cv2.Canny(blurred_frame, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)
        
        # 应用逆透视变换（使用正向变换矩阵生成鸟瞰图）
        height, width = frame.shape[:2]
        ipm_frame = cv2.warpPerspective(canny_edges, forward_perspective_matrix, (width, height))
        
        # 对完整鸟瞰图应用形态学闭运算（先膨胀后腐蚀）
        dilated_frame = cv2.dilate(ipm_frame, DILATE_KERNEL)
        morphed_full_ipm = cv2.erode(dilated_frame, ERODE_KERNEL)
        
        # 从处理后的鸟瞰图中截取ROI区域
        final_roi_frame = morphed_full_ipm[IPM_ROI_Y:IPM_ROI_Y + IPM_ROI_H, IPM_ROI_X:IPM_ROI_X + IPM_ROI_W]
        
        # 显示原始帧、灰度帧、高斯模糊后的帧、Canny边缘检测结果、原始鸟瞰图、形态学处理后的完整鸟瞰图和最终ROI
        if SHOW_ORIGINAL:
            cv2.imshow('Original Frame', frame)
        if SHOW_GRAYSCALE:
            cv2.imshow('Grayscale Frame', gray_frame)
        if SHOW_GAUSSIAN:
            cv2.imshow('Gaussian Blurred Frame', blurred_frame)
        if SHOW_CANNY:
            cv2.imshow('Canny Edges', canny_edges)
        if SHOW_IPM_RAW:
            cv2.imshow('IPM Bird-eye View', ipm_frame)
        if SHOW_IPM_MORPHED:
            cv2.imshow('Morphed Full IPM', morphed_full_ipm)
        if SHOW_FINAL_ROI:
            # 对ROI区域进行二值化处理
            _, binary_roi_frame = cv2.threshold(final_roi_frame, 5, 255, cv2.THRESH_BINARY)
            
            # 为了在上面画图，我们先将ROI二值图转换为BGR彩色图
            roi_display = cv2.cvtColor(binary_roi_frame, cv2.COLOR_GRAY2BGR)

            # 获取ROI的尺寸
            roi_h, roi_w = binary_roi_frame.shape[:2]
            
            # 初始化中心点和起始点坐标
            center_x = roi_w // 2
            right_start_point = None
            current_scan_y = None  # 用于记录最终找到的扫描线位置

            # 从底部开始，每隔START_POINT_SCAN_STEP个像素向上扫描，寻找右边线起始点
            for y in range(roi_h - 1, 0, -START_POINT_SCAN_STEP):
                # 从中心向右扫描寻找右边线的内侧起始点
                for x in range(center_x, roi_w - 1):
                    if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x + 1] == 255:
                        right_start_point = (x + 1, y)
                        current_scan_y = y
                        break
                
                # 如果找到了起始点，就停止向上扫描
                if right_start_point is not None:
                    break
            
            # 可视化：画出扫描线和找到的起始点
            if current_scan_y is not None:
                # 画一条蓝色的线表示我们从哪里找到了起始点
                cv2.line(roi_display, (0, current_scan_y), (roi_w, current_scan_y), (255, 0, 0), 1)
            
            if right_start_point:
                # 在右起始点画一个红色的圆
                cv2.circle(roi_display, right_start_point, 5, (0, 0, 255), -1)
                
                # 使用沿墙走算法寻找右边界
                right_points = follow_the_wall(binary_roi_frame, right_start_point)
                
                # 提取最终的右边线
                final_right_border = None
                if right_points:
                    final_right_border = extract_final_border(roi_h, right_points)
                
                # 如果成功提取到右边线，计算并绘制中心线
                if final_right_border is not None:
                    center_line = []
                    for y, x in enumerate(final_right_border):
                        if x != -1:  # 如果该行有右边线点
                            # 绘制右边线点（粉色）
                            cv2.circle(roi_display, (x, y), 2, (255, 192, 203), -1)
                            # 计算中心线点
                            center_x = x + CENTER_LINE_OFFSET
                            if 0 <= center_x < roi_w:  # 确保中心线点在图像范围内
                                center_line.append((center_x, y))
                    
                    # 如果有足够的点，绘制中心线
                    if len(center_line) > 1:
                        # 将点列表转换为numpy数组，并重塑为绘制多边形所需的格式
                        center_line_array = np.array(center_line).reshape((-1, 1, 2))
                        # 绘制中心线（亮绿色，粗线）
                        cv2.polylines(roi_display, [center_line_array], False, (0, 255, 0), 2)
            
            cv2.imshow('Final ROI', roi_display)
        
        # 等待按键，'q'键用于退出
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    process_video() 