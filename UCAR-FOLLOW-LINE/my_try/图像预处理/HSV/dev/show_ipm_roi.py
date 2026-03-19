import cv2
import numpy as np

# --- 1. 参数配置区 ---

# 在这里粘贴您从【全图】标定中获得的3x3逆向变换矩阵
INVERSE_IPM_MATRIX = np.array([
    [-3.365493,  2.608984, -357.317062],
    [-0.049261,  1.302389, -874.095796],
    [ 0.000029,  0.007556,   -4.205510]
], dtype=np.float64)

# 定义您期望的输出【全图】俯视图的尺寸（宽度, 高度）
OUTPUT_SIZE = (640, 480)

# 定义在【IPM俯视图】上进行分析的ROI区域
IPM_ROI_Y = 240  # 示例：从俯视图的垂直中点开始截取
IPM_ROI_H = 240  # 示例：截取下半部分
IPM_ROI_X = 0
IPM_ROI_W = 640

# 输入视频的路径 (请确保视频文件与此脚本在同一目录下, 或提供完整路径)
VIDEO_PATH = '/home/lby/CURSOR/follow_line/follow_line_mp4/high/high_1.mp4'

# --- 2. 主程序区 (通常无需修改) ---

def run_ipm_video_test():
    """
    运行逆透视变换视频测试
    """
    # 计算正向变换矩阵 (这是warpPerspective函数实际需要的)
    try:
        ipm_matrix = np.linalg.inv(INVERSE_IPM_MATRIX)
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

    print("视频处理中... 按 'q' 键退出。")

    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            print("视频播放结束。")
            break

        # 新增：对原始帧进行水平镜像翻转，以校正摄像头问题
        frame = cv2.flip(frame, 1)
        
        # 1. 首先，对【整张原始图像】应用逆透视变换
        # 使用线性插值(INTER_LINEAR)以在Jetson上获得最佳性能
        ipm_frame = cv2.warpPerspective(frame, ipm_matrix, OUTPUT_SIZE, flags=cv2.INTER_LINEAR)

        # 2. 然后，从【已生成的IPM俯视图】中截取用于分析的ROI
        analysis_roi = ipm_frame[IPM_ROI_Y : IPM_ROI_Y + IPM_ROI_H, IPM_ROI_X : IPM_ROI_X + IPM_ROI_W]

        # 显示原始视图、完整的IPM视图和最终用于分析的ROI
        cv2.imshow('Original Video', frame)
        cv2.imshow('Full IPM View', ipm_frame)
        cv2.imshow('Analysis ROI', analysis_roi)

        # 等待按键，'q'键用于退出
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    run_ipm_video_test()