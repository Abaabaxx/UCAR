import cv2
import numpy as np

# --- 1. 参数配置区 ---

# 在这里粘贴您从标定中获得的3x3逆向变换矩阵
# 这个矩阵描述了如何从“俯视图”坐标换算回“原始图像”坐标
INVERSE_IPM_MATRIX = np.array([
    [-1.874777,  1.432237, -140.732033],
    [-0.026319,  0.326944, -234.944954],
    [-0.000017,  0.004203,   -2.182227]
], dtype=np.float64)

# 定义您期望的输出俯视图的尺寸（宽度, 高度）
OUTPUT_SIZE = (640, 480)

# 输入视频的路径 (请确保视频文件与此脚本在同一目录下, 或提供完整路径)
VIDEO_PATH = '/home/lby/CURSOR/follow_line/follow_line_mp4/low/low_1.mp4'

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

        # 应用逆透视(鸟瞰图)变换
        ipm_frame = cv2.warpPerspective(frame, ipm_matrix, OUTPUT_SIZE, flags=cv2.INTER_LINEAR)

        # 显示原始视图和IPM视图
        cv2.imshow('Original Video', frame)
        cv2.imshow('IPM View (Bird\'s-Eye View)', ipm_frame)

        # 等待按键，'q'键用于退出
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    run_ipm_video_test()