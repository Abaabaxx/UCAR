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
# 在IPM鸟瞰图上进行分析的ROI区域参数 (参考show_ipm_roi.py)
IPM_ROI_Y = 240  # ROI起始Y坐标
IPM_ROI_H = 240  # ROI高度
IPM_ROI_X = 0    # ROI起始X坐标
IPM_ROI_W = 640  # ROI宽度
# 形态学操作的卷积核
DILATE_KERNEL = np.ones((15, 15), np.uint8)  # 较大的核用于膨胀，强力连接断点
ERODE_KERNEL = np.ones((7, 7), np.uint8)   # 较小的核用于腐蚀，精细去除噪点

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
        
        # 从IPM鸟瞰图中截取ROI区域
        ipm_roi_frame = ipm_frame[IPM_ROI_Y:IPM_ROI_Y + IPM_ROI_H, IPM_ROI_X:IPM_ROI_X + IPM_ROI_W]
        
        # 应用形态学闭运算（先膨胀后腐蚀）来修复断裂的线段
        dilated_frame = cv2.dilate(ipm_roi_frame, DILATE_KERNEL)
        morphed_roi_frame = cv2.erode(dilated_frame, ERODE_KERNEL)
        
        # 显示原始帧、灰度帧、高斯模糊后的帧、Canny边缘检测结果、完整鸟瞰图、ROI区域和形态学处理后的结果
        cv2.imshow('Original Frame', frame)
        cv2.imshow('Grayscale Frame', gray_frame)
        cv2.imshow('Gaussian Blurred Frame', blurred_frame)
        cv2.imshow('Canny Edges', canny_edges)
        cv2.imshow('IPM Bird-eye View', ipm_frame)
        cv2.imshow('IPM ROI View', ipm_roi_frame)
        cv2.imshow('Morphed ROI View', morphed_roi_frame)
        
        # 等待按键，'q'键用于退出
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    process_video() 