import cv2
import numpy as np

# --- 参数配置区 ---
# 请将此路径修改为您准备的"黑白分明"测试视频的实际路径
VIDEO_PATH = '/home/lby/CURSOR/follow_line/my_try/视频和图片/拉窗帘原始视频（平视）/最后录制.mp4'

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

# --- 主程序区 ---
def validate_otsu():
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

    # 计算帧间延迟
    fps = cap.get(cv2.CAP_PROP_FPS)
    delay = int(1000 / fps) if fps > 0 else 33

    print("方案D验证程序: 原图 -> Otsu二值化 -> IPM -> IPM上的ROI... 按 'q' 键退出。")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("视频播放结束。")
            break
            
        # 对原始帧进行水平镜像翻转，以校正摄像头问题
        frame = cv2.flip(frame, 1)

        # 1. 首先，将【整张原始图像】转换为灰度图
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. 然后，对整张灰度图应用大津法进行二值化
        ret_otsu, full_otsu_mask = cv2.threshold(gray_frame, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 3. 接着，对【已二值化的全图】应用逆透视变换
        # 注意：这里的输入是 full_otsu_mask
        ipm_otsu_mask = cv2.warpPerspective(full_otsu_mask, ipm_matrix, OUTPUT_SIZE, flags=cv2.INTER_LINEAR)

        # 4. 最后，从变换后的俯视图中，裁剪出用于分析的ROI
        analysis_roi = ipm_otsu_mask[IPM_ROI_Y : IPM_ROI_Y + IPM_ROI_H, IPM_ROI_X : IPM_ROI_X + IPM_ROI_W]

        # 显示关键步骤的结果以供诊断
        cv2.imshow('0. Original Frame', frame)
        cv2.imshow('1. Otsu on Full Frame', full_otsu_mask)
        cv2.imshow('2. IPM on Otsu Mask', ipm_otsu_mask)
        cv2.imshow('3. Final Analysis ROI', analysis_roi)

        # 等待按键
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    validate_otsu() 