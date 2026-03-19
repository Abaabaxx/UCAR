#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
MP3转WAV批量转换脚本

功能：
- 扫描/home/ucar/lby_ws/src/xf_mic_asr_offline/voices目录下的所有MP3文件
- 保持原目录结构，将文件转换为WAV格式
- 保存到/home/ucar/lby_ws/src/xf_mic_asr_offline/voices_wav目录
"""

import os
import subprocess
import shutil

def convert_mp3_to_wav(mp3_path, wav_path):
    """
    将MP3文件转换为WAV格式
    
    参数:
        mp3_path: MP3文件路径
        wav_path: 输出WAV文件路径
    
    返回:
        bool: 转换成功返回True，否则返回False
    """
    try:
        # 确保输出目录存在
        output_dir = os.path.dirname(wav_path)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        # 使用ffmpeg进行转换
        command = ['ffmpeg', '-i', mp3_path, '-acodec', 'pcm_s16le', 
                  '-ar', '44100', '-y', wav_path]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        
        # 检查转换结果
        if process.returncode != 0:
            print("错误: 转换失败 {} -> {}: {}".format(mp3_path, wav_path, stderr))
            return False
            
        if not os.path.exists(wav_path):
            print("错误: 输出文件未创建: {}".format(wav_path))
            return False
            
        print("成功: {} -> {}".format(mp3_path, wav_path))
        return True
    
    except Exception as e:
        print("异常: 处理 {} 时出错: {}".format(mp3_path, str(e)))
        return False

def batch_convert_directory(source_dir, target_dir):
    """
    批量转换目录下的所有MP3文件为WAV文件
    
    参数:
        source_dir: 源目录（包含MP3文件）
        target_dir: 目标目录（将保存WAV文件）
    """
    # 统计计数
    total_files = 0
    success_count = 0
    error_count = 0
    
    # 遍历源目录下的所有文件
    for root, dirs, files in os.walk(source_dir):
        for filename in files:
            if filename.lower().endswith('.mp3'):
                total_files += 1
                
                # 构建源文件和目标文件路径
                source_path = os.path.join(root, filename)
                
                # 计算相对路径
                rel_path = os.path.relpath(source_path, source_dir)
                
                # 构建目标路径，替换扩展名
                target_path = os.path.join(target_dir, rel_path.replace('.mp3', '.wav'))
                
                # 转换文件
                if convert_mp3_to_wav(source_path, target_path):
                    success_count += 1
                else:
                    error_count += 1
    
    # 打印结果摘要
    print("\n转换完成！")
    print("总计文件数: {}".format(total_files))
    print("成功转换: {}".format(success_count))
    print("转换失败: {}".format(error_count))

def main():
    # 定义源目录和目标目录
    source_directory = "/home/ucar/lby_ws/src/xf_mic_asr_offline/voices"
    target_directory = "/home/ucar/lby_ws/src/xf_mic_asr_offline/voices_wav"
    
    # 检查源目录是否存在
    if not os.path.exists(source_directory):
        print("错误: 源目录不存在: {}".format(source_directory))
        return
    
    # 如果目标目录已存在，询问是否覆盖
    if os.path.exists(target_directory):
        response = raw_input("目标目录已存在，是否清空并继续？(y/n): ")
        if response.lower() == 'y':
            shutil.rmtree(target_directory)
        else:
            print("操作已取消")
            return
    
    # 创建目标目录
    try:
        os.makedirs(target_directory)
    except OSError as e:
        print("创建目标目录失败: {}".format(str(e)))
        return
    
    print("开始转换...")
    batch_convert_directory(source_directory, target_directory)

if __name__ == "__main__":
    main()