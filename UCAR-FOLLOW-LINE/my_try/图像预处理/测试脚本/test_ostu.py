import cv2
import numpy as np

# --- 参数配置区 ---
# 请将此路径修改为您准备的"黑白分明"测试视频的实际路径
# 原始视频：VIDEO_PATH = '/home/lby/CURSOR/follow_line/follow_line_mp4/high/high_1.mp4'
# 逆透视视频：VIDEO_PATH = '/home/lby/CURSOR/follow_line/my_try/逆透视视频/平视/省赛/1.mp4'
VIDEO_PATH = '/home/lby/CURSOR/follow_line/follow_line_mp4/high/high_2.mp4'

# --- 主程序区 ---
def validate_otsu():
    # 打开视频文件
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"错误: 无法打开视频文件 '{VIDEO_PATH}'")
        return

    # 计算帧间延迟
    fps = cap.get(cv2.CAP_PROP_FPS)
    delay = int(1000 / fps) if fps > 0 else 33

    print("纯粹的大津法验证程序... 按 'q' 键退出。")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("视频播放结束。")
            break

        # 1. 将原始帧转换为灰度图
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. 对灰度图应用大津法进行自适应二值化
        ret_otsu, otsu_mask = cv2.threshold(gray_frame, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 显示原始帧和应用大津法后的结果
        cv2.imshow('Original Frame', frame)
        cv2.imshow('Otsu Mask', otsu_mask)

        # 等待按键
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    validate_otsu() 