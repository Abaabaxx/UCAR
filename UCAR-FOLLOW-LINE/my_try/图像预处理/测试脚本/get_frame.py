import cv2

# 该脚本用于提取.mp4中一帧图像，然后使用这个图像进行标定，然后使用这个图像进行逆透视变换

# --- 您需要修改的参数 ---
VIDEO_PATH = '/home/lby/CURSOR/follow_line/follow_line_mp4/low/low_bd.mp4'  # 您的视频文件路径
OUTPUT_IMAGE_NAME = '/home/lby/CURSOR/follow_line/my_try/image/low_bd.png'  # 保存的图片文件名 可以写保存的路径
# --- 修改结束 ---

# 读取视频
cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    print(f"错误: 无法打开视频文件 {VIDEO_PATH}")
else:
    # 只读取第一帧
    ret, frame = cap.read()
    if ret:
        # 保存这一帧为图片文件
        cv2.imwrite(OUTPUT_IMAGE_NAME, frame)
        height, width, _ = frame.shape
        print(f"成功! 已将视频的第一帧保存为 '{OUTPUT_IMAGE_NAME}'")
        print(f"这张图片的真实分辨率是: {width}x{height}")
    else:
        print("错误: 无法从视频中读取帧。")
    cap.release()