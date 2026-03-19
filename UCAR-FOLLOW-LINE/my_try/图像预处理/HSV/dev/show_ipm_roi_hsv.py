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
VIDEO_PATH = '/home/lby/CURSOR/follow_line/my_try/视频和图片/拉窗帘原始视频（平视）/最后录制.mp4'

# --- 2. 主程序区 (通常无需修改) ---

def run_hsv_tuner():
    """
    运行HSV阈值调节器，用于寻找最佳的颜色分割参数
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

    # 创建控制窗口和滑动条
    cv2.namedWindow('HSV Controls')
    cv2.createTrackbar('H_min', 'HSV Controls', 0, 179, lambda x: None)
    cv2.createTrackbar('H_max', 'HSV Controls', 179, 179, lambda x: None)
    cv2.createTrackbar('S_min', 'HSV Controls', 0, 255, lambda x: None)
    cv2.createTrackbar('S_max', 'HSV Controls', 255, 255, lambda x: None)
    cv2.createTrackbar('V_min', 'HSV Controls', 0, 255, lambda x: None)
    cv2.createTrackbar('V_max', 'HSV Controls', 255, 255, lambda x: None)

    print("HSV调节器已启动... 按 'q' 键退出，按 's' 键保存当前HSV参数。")

    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            print("视频播放结束，重新开始播放。")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 重置到视频开始
            continue

        # 新增：对原始帧进行水平镜像翻转，以校正摄像头问题
        frame = cv2.flip(frame, 1)
        
        # 1. 首先，对【整张原始图像】应用逆透视变换
        # 使用线性插值(INTER_LINEAR)以在Jetson上获得最佳性能
        ipm_frame = cv2.warpPerspective(frame, ipm_matrix, OUTPUT_SIZE, flags=cv2.INTER_LINEAR)

        # 2. 然后，从【已生成的IPM俯视图】中截取用于分析的ROI
        analysis_roi = ipm_frame[IPM_ROI_Y : IPM_ROI_Y + IPM_ROI_H, IPM_ROI_X : IPM_ROI_X + IPM_ROI_W]

        # 3. 从滑动条获取当前的HSV阈值
        h_min = cv2.getTrackbarPos('H_min', 'HSV Controls')
        h_max = cv2.getTrackbarPos('H_max', 'HSV Controls')
        s_min = cv2.getTrackbarPos('S_min', 'HSV Controls')
        s_max = cv2.getTrackbarPos('S_max', 'HSV Controls')
        v_min = cv2.getTrackbarPos('V_min', 'HSV Controls')
        v_max = cv2.getTrackbarPos('V_max', 'HSV Controls')
        
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # 4. 将用于分析的ROI从BGR色彩空间转换到HSV色彩空间
        hsv_roi = cv2.cvtColor(analysis_roi, cv2.COLOR_BGR2HSV)

        # 5. 根据HSV阈值，生成二值化蒙版 (mask)
        mask = cv2.inRange(hsv_roi, lower_bound, upper_bound)

        # 显示原始分析区域和二值化蒙版结果
        cv2.imshow('Analysis ROI', analysis_roi)
        cv2.imshow('HSV Mask', mask)

        # 等待按键，'q'键用于退出，'s'键用于保存当前参数
        key = cv2.waitKey(delay) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # 保存当前HSV参数到文本文件
            with open('hsv_parameters.txt', 'w') as f:
                f.write(f"H_min: {h_min}\n")
                f.write(f"H_max: {h_max}\n")
                f.write(f"S_min: {s_min}\n")
                f.write(f"S_max: {s_max}\n")
                f.write(f"V_min: {v_min}\n")
                f.write(f"V_max: {v_max}\n")
            print(f"HSV参数已保存到 hsv_parameters.txt")

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出。")

if __name__ == '__main__':
    run_hsv_tuner() 