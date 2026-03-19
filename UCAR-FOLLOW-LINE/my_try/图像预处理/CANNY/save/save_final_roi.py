import cv2
import numpy as np
import os

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

# --- 视频保存配置 ---
SAVE_VIDEO = True
OUTPUT_DIR = 'my_try/视频和图片/CANNY预处理视频'  # 输出视频的保存目录

def process_video():
    """
    加载视频，对每一帧进行处理，并将最终的ROI区域保存为新的视频文件
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
    
    # 获取视频的帧率(FPS)
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        print("警告: 无法获取视频帧率, 将默认使用30 FPS。")
        fps = 30
    
    # 初始化视频写入器 (如果SAVE_VIDEO为True)
    video_writer = None
    if SAVE_VIDEO:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        video_filename = os.path.basename(VIDEO_PATH)
        base_name, _ = os.path.splitext(video_filename)
        output_filename = f"{base_name}_图像预处理结果.mp4"
        output_path = os.path.join(OUTPUT_DIR, output_filename)
        
        output_video_size = (IPM_ROI_W, IPM_ROI_H)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(output_path, fourcc, fps, output_video_size)
        
        if not video_writer.isOpened():
            print(f"错误: 无法创建视频写入器于 '{output_path}'")
            return
        else:
            print(f"视频处理启动，将保存到: {output_path}")
    else:
        print("视频处理已启动... (保存功能已禁用)")
    
    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            print("视频处理结束。")
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
        
        # 如果启用了保存功能，则将处理后的帧写入文件
        if SAVE_VIDEO and video_writer is not None and video_writer.isOpened():
            # 将单通道二值图像转换为三通道BGR图像以便保存
            output_frame = cv2.cvtColor(final_roi_frame, cv2.COLOR_GRAY2BGR)
            video_writer.write(output_frame)
    
    # 释放资源
    cap.release()
    if video_writer is not None:
        video_writer.release()
    print("程序已退出。")

if __name__ == '__main__':
    process_video() 