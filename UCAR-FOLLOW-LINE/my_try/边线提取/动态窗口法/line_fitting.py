import cv2
import numpy as np

# --- 参数配置区 ---
# 输入视频的路径 (请确保视频文件与此脚本在同一目录下, 或提供完整路径)
VIDEO_PATH = '/home/lby/CURSOR/follow_line/my_try/视频和图片/拉窗帘原始视频（平视）/最后录制.mp4'
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
SHOW_LANE_FITTING = True    # 显示车道线拟合结果

def find_lane_pixels(binary_warped):
    # 获取图像下半部分的直方图
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    # 创建一个输出图像来绘制和可视化结果
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))
    # 找到直方图的左右两个峰值
    midpoint = int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # 设置滑动窗口的数量
    nwindows = 20
    # 设置窗口的高度
    window_height = int(binary_warped.shape[0]//nwindows)
    # 识别图像中所有非零像素的x和y坐标
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # 为每个窗口更新当前位置
    leftx_current = leftx_base
    rightx_current = rightx_base

    # 设置窗口的宽度 +/- margin
    margin = 50
    # 设置重新定位窗口的最小像素数
    minpix = 50
    # 创建空列表以接收左、右车道像素索引
    left_lane_inds = []
    right_lane_inds = []

    # 遍历窗口
    for window in range(nwindows):
        # 识别x和y的窗口边界
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        
        # 在可视化图像上绘制窗口
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),
        (win_xleft_high,win_y_high),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xright_low,win_y_low),
        (win_xright_high,win_y_high),(0,255,0), 2) 
        
        # 识别窗口内的非零像素
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        
        # 将这些索引附加到列表中
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        # 如果找到的像素>minpix，则将下一个窗口重新定位到它们的平均位置
        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

    # 连接索引数组
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # 如果数组为空，则传递
        pass

    # 提取左、右线像素位置
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty, out_img

def fit_polynomial(binary_warped):
    # 找到我们的车道像素
    leftx, lefty, rightx, righty, out_img = find_lane_pixels(binary_warped)

    # 对x和y位置进行二阶多项式拟合
    try:
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
    except TypeError:
        # 避免因找不到线条而导致的错误
        print('函数无法拟合一条线!')
        left_fit = [1,1,1]
        right_fit = [1,1,1]

    # 生成用于绘图的x和y值
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    try:
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    except TypeError:
        # 避免因拟合失败而导致的错误
        print('函数无法拟合一条线!')
        left_fitx = 1*ploty**2 + 1*ploty
        right_fitx = 1*ploty**2 + 1*ploty
    
    return left_fit, right_fit, left_fitx, right_fitx, ploty

def draw_lane_on_original(original_frame, binary_warped, Minv, left_fitx, right_fitx, ploty):
    # 创建一个图像来绘制线条
    warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    # 将x和y点重塑为cv2.fillPoly()的可用格式
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    # 在扭曲的空白图像上绘制车道
    cv2.fillPoly(color_warp, np.int_([pts]), (0,0,255))  # 红色车道线

    # 使用反透视矩阵将空白扭曲回原始图像空间
    newwarp = cv2.warpPerspective(color_warp, Minv, (original_frame.shape[1], original_frame.shape[0])) 
    # 将结果与原始图像组合
    result = cv2.addWeighted(original_frame, 1, newwarp, 0.5, 0)
    return result

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
        
        # --- 车道线拟合与可视化 ---
        lane_fit_result = frame # 默认为原始帧
        final_roi_display = cv2.cvtColor(final_roi_frame, cv2.COLOR_GRAY2BGR) # 默认显示无拟合线的ROI
        try:
            # 使用完整的形态学处理后的鸟瞰图进行拟合
            left_fit, right_fit, left_fitx, right_fitx, ploty = fit_polynomial(morphed_full_ipm)
            
            # 1. 将拟合的车道线（红色）绘制到原始翻转后的帧上
            lane_fit_result = draw_lane_on_original(frame, morphed_full_ipm, INVERSE_PERSPECTIVE_MATRIX, left_fitx, right_fitx, ploty)

            # 2. 在 Final ROI 窗口上绘制拟合线 (蓝色以区分)
            #   - 生成ROI区域对应的y坐标(在完整鸟瞰图坐标系中)
            ploty_roi = np.linspace(IPM_ROI_Y, IPM_ROI_Y + IPM_ROI_H - 1, IPM_ROI_H)
            #   - 计算这些y坐标对应的x坐标
            left_fitx_roi = left_fit[0]*ploty_roi**2 + left_fit[1]*ploty_roi + left_fit[2]
            right_fitx_roi = right_fit[0]*ploty_roi**2 + right_fit[1]*ploty_roi + right_fit[2]
            #   - 将x坐标转换到ROI的局部坐标系
            left_fitx_roi_local = left_fitx_roi - IPM_ROI_X
            right_fitx_roi_local = right_fitx_roi - IPM_ROI_X
            #   - 将y坐标转换到ROI的局部坐标系
            ploty_roi_local = np.linspace(0, IPM_ROI_H - 1, IPM_ROI_H)

            #   - 准备绘制点集并绘制
            left_line_pts = np.array(list(zip(left_fitx_roi_local, ploty_roi_local))).astype(np.int32)
            right_line_pts = np.array(list(zip(right_fitx_roi_local, ploty_roi_local))).astype(np.int32)
            cv2.polylines(final_roi_display, [left_line_pts], isClosed=False, color=(255, 0, 0), thickness=2)
            cv2.polylines(final_roi_display, [right_line_pts], isClosed=False, color=(255, 0, 0), thickness=2)

        except Exception as e:
            print(f"车道线拟合时发生错误: {e}")
            # 如果拟合失败，则将结果设置为原始帧，以便继续显示
            lane_fit_result = frame

        
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
            cv2.imshow('Final ROI', final_roi_display)
        if SHOW_LANE_FITTING:
            cv2.imshow('Lane Fitting Result', lane_fit_result)
        
        # 等待按键，'q'键用于退出
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    process_video() 