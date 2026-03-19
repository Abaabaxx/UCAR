import cv2
import numpy as np

# --- 参数配置区 ---
# 输入视频的路径 (请确保视频文件与此脚本在同一目录下, 或提供完整路径)
VIDEO_PATH = '/home/lby/CURSOR/follow_line/my_try/视频和图片/拉窗帘原始视频（平视）/最后录制.mp4'
# 高斯模糊参数
GAUSSIAN_KERNEL_SIZE = (5, 5)  # 高斯核大小
GAUSSIAN_SIGMA_X = 0  # 标准差，0表示根据核大小自动计算
# 图像翻转参数
PERFORM_HORIZONTAL_FLIP = True  # 是否执行水平翻转

def process_video():
    """
    加载视频，将每一帧转换为灰度图，并同时显示原始帧和灰度帧
    """
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
        
        # 显示原始帧、灰度帧和高斯模糊后的帧
        cv2.imshow('Original Frame', frame)
        cv2.imshow('Grayscale Frame', gray_frame)
        cv2.imshow('Gaussian Blurred Frame', blurred_frame)
        
        # 等待按键，'q'键用于退出
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    process_video() 