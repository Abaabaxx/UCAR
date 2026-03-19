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

# 定义邻域搜索模式
# 顺时针方向，用于从左起始点开始，沿赛道外侧（黑色区域）向内寻找左边界
SEEDS_L = [
    (-1, 0),    # 左
    (-1, -1),   # 左上
    (0, -1),    # 上
    (1, -1),    # 右上
    (1, 0),     # 右
    (1, 1),     # 右下
    (0, 1),     # 下
    (-1, 1)     # 左下
]

# 逆时针方向，用于从右起始点开始，沿赛道外侧（黑色区域）向内寻找右边界
SEEDS_R = [
    (1, 0),     # 右
    (1, -1),    # 右上
    (0, -1),    # 上
    (-1, -1),   # 左上
    (-1, 0),    # 左
    (-1, 1),    # 左下
    (0, 1),     # 下
    (1, 1)      # 右下
]

def trace_boundary(image, start_point, seeds):
    """
    使用八邻域爬线算法跟踪边界
    
    参数:
    image: 二值图像
    start_point: 起始点坐标 (x, y)
    seeds: 邻域搜索模式列表
    
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
            dx_a, dy_a = seeds[i]
            A_x = current_point[0] + dx_a
            A_y = current_point[1] + dy_a
            
            # 获取下一个邻居点B的坐标（使用循环索引）
            dx_b, dy_b = seeds[(i + 1) % 8]
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
            
            # 定义扫描线的位置（从下往上数20个像素，增加鲁棒性）
            start_row = roi_h - 20
            center_x = roi_w // 2

            # 初始化起始点坐标
            left_start_point = None
            right_start_point = None

            # 从中心向左扫描寻找左边线的内侧起始点
            for x in range(center_x, 0, -1):
                if binary_roi_frame[start_row, x] == 0 and binary_roi_frame[start_row, x - 1] == 255:
                    left_start_point = (x - 1, start_row)  # 记录白色点的位置
                    break
            
            # 从中心向右扫描寻找右边线的内侧起始点
            for x in range(center_x, roi_w - 1):
                if binary_roi_frame[start_row, x] == 0 and binary_roi_frame[start_row, x + 1] == 255:
                    right_start_point = (x + 1, start_row)  # 记录白色点的位置
                    break
            
            # 可视化：画出扫描线和找到的起始点
            # 画一条蓝色的线表示我们从哪里开始扫描
            cv2.line(roi_display, (0, start_row), (roi_w, start_row), (255, 0, 0), 1)
            
            # 初始化最终边线数组
            final_left_border = None
            final_right_border = None
            
            if left_start_point:
                # 在左起始点画一个绿色的圆
                cv2.circle(roi_display, left_start_point, 5, (0, 255, 0), -1)
                # 使用八邻域爬线算法寻找左边界，注意这里使用了SEEDS_R（交换了爬线算法）
                left_points = trace_boundary(binary_roi_frame, left_start_point, SEEDS_R)
                # 提取最终的左边线
                if left_points:
                    final_left_border = extract_final_border(roi_h, left_points)
            
            if right_start_point:
                # 在右起始点画一个红色的圆
                cv2.circle(roi_display, right_start_point, 5, (0, 0, 255), -1)
                # 使用八邻域爬线算法寻找右边界，注意这里使用了SEEDS_L（交换了爬线算法）
                right_points = trace_boundary(binary_roi_frame, right_start_point, SEEDS_L)
                # 提取最终的右边线
                if right_points:
                    final_right_border = extract_final_border(roi_h, right_points)
            
            # 可视化最终的边线点
            if final_left_border is not None:
                for y, x in enumerate(final_left_border):
                    if x != -1:  # 如果这一行有边线点
                        cv2.circle(roi_display, (x, y), 2, (0, 255, 0), -1)  # 亮绿色
            
            if final_right_border is not None:
                for y, x in enumerate(final_right_border):
                    if x != -1:  # 如果这一行有边线点
                        cv2.circle(roi_display, (x, y), 2, (255, 192, 203), -1)  # 粉色
            
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