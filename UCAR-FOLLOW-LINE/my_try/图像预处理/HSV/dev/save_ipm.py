import cv2
import numpy as np
import os

# 脚本功能：读取一个视频，对其应用IPM变换，并将结果保存为另一个视频文件。

# --- 1. 参数配置区 ---

# 在这里粘贴您从标定中获得的3x3逆向变换矩阵
# 这个矩阵描述了如何从"俯视图"坐标换算回"原始图像"坐标
INVERSE_IPM_MATRIX = np.array([
    [-3.365493,  2.608984, -357.317062],
    [-0.049261,  1.302389, -874.095796],
    [ 0.000029,  0.007556,   -4.205510]
], dtype=np.float64)

# 定义您期望的输出俯视图的尺寸（宽度, 高度）
OUTPUT_SIZE = (640, 480)

# 输入视频的路径 (请确保视频文件与此脚本在同一目录下, 或提供完整路径)
VIDEO_PATH = '/home/lby/CURSOR/follow_line/follow_line_mp4/high/high_2.mp4'

# 输出视频的目录
OUTPUT_DIR = '/home/lby/CURSOR/follow_line/my_try/video'

# --- 2. 主程序区 (通常无需修改) ---

def create_ipm_video(input_path: str, output_path: str, forward_ipm_matrix: np.ndarray, size: tuple) -> None:
    """
    读取输入视频，应用IPM变换，并将结果保存为新的视频文件
    
    Args:
        input_path: 输入视频的路径
        output_path: 输出视频的路径
        forward_ipm_matrix: 正向IPM变换矩阵
        size: 输出视频的尺寸 (宽度, 高度)
    """
    # 打开视频文件
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        print(f"错误: 无法打开视频文件 '{input_path}'")
        return

    # 获取视频的帧率(FPS)
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0:
        print("警告: 无法获取视频帧率, 将默认使用30 FPS。")
        fps = 30
    
    # 获取视频的宽度和高度
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # 创建视频写入器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, size)
    
    print(f"处理视频中...")
    
    # 处理每一帧
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 新增：对原始帧进行水平镜像翻转，以校正摄像头问题
        frame = cv2.flip(frame, 1)      
           
        # 应用逆透视(鸟瞰图)变换
        ipm_frame = cv2.warpPerspective(frame, forward_ipm_matrix, size, flags=cv2.INTER_LINEAR)
        
        # 写入变换后的帧
        out.write(ipm_frame)
    
    # 释放资源
    cap.release()
    out.release()
    print(f"视频处理完成。")

if __name__ == '__main__':
    # 计算正向变换矩阵
    try:
        ipm_matrix = np.linalg.inv(INVERSE_IPM_MATRIX)
    except np.linalg.LinAlgError:
        print("错误: 提供的矩阵是奇异矩阵, 无法求逆。请检查矩阵参数。")
        exit(1)
    
    # 动态构建输出路径
    video_filename = os.path.basename(VIDEO_PATH)
    base_name, ext = os.path.splitext(video_filename)
    output_filename = f"{base_name}_ipm.mp4"
    output_path = os.path.join(OUTPUT_DIR, output_filename)
    
    print(f"开始处理视频: {VIDEO_PATH}")
    print(f"输出将保存到: {output_path}")
    
    # 调用核心功能
    create_ipm_video(VIDEO_PATH, output_path, ipm_matrix, OUTPUT_SIZE)
    
    print(f"处理完成! 文件已保存到: {output_path}")