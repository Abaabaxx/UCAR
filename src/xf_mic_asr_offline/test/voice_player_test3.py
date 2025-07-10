#!/usr/bin/env python2
# -*- coding: utf-8 -*-

'''
使用说明：
1. 请确保退出 Conda 环境，使用系统级别的 Python 2.7 环境。
2. 启动方式：
   - 使用以下命令启动 ROS 核心：
     $ roscore
   - 在另一个终端中运行语音播放节点：
     $ rosrun xf_mic_asr_offline voice_player.py
3. 功能：
   - 该节点订阅 ROS 话题 `/status_manager/voice_status`。
   - 当接收到话题消息时，根据话题消息内容（字典中的键名）播放对应的音频文件。
4. 支持的话题消息内容（键名）：
   - 任务获取相关：
     - task_get_veg
     - task_get_fruit
     - task_get_dessert
   - 物品获取相关：
     - get_apple
     - get_banana
     - get_cake
     - get_cola
     - get_milk
     - get_pepper
     - get_potato
     - get_tomato
     - get_watermelon
   - 房间相关：
     - room_A
     - room_B
     - room_C
   - 路径相关：
     - way_1
     - way_2
   - 任务完成相关：
     - task_fin_apple_banana
     - task_fin_watermelon_banana
     - 等其他任务完成语音（详见代码中的 `files_to_load` 字典）。
5. 使用方法：
   - 发布消息到 `/status_manager/voice_status` 话题，消息内容为上述键名之一。
   - 示例：
     $ rostopic pub /status_manager/voice_status std_msgs/String "data: 'task_get_dessert'"
6. 注意事项：
   - 确保音频文件目录结构正确，且文件存在。
   - 确保已安装 `ffmpeg` 和 `pydub` 依赖：
     $ sudo apt-get install ffmpeg
     $ pip install pydub
   - 程序运行时会自动将 MP3 文件转换为 WAV 格式，并在退出时清理临时文件。
'''

import rospy
from std_msgs.msg import String
from pydub import AudioSegment  # 导入 pydub.AudioSegment 用于加载和处理音频文件
from pydub.playback import play # 导入 pydub.playback.play 用于播放音频
import os   # 导入 os 模块用于文件路径操作
import subprocess   # 导入 subprocess 模块用于调用外部 ffmpeg 将 MP3 文件转换为 WAV 格式
import signal   # 导入 signal 模块用于捕获 Ctrl+C 信号，优雅的关闭程序
import sys  # 导入 sys 模块用于ctal+c 退出后，调用sys.exit()退出程序
import tempfile # 导入 tempfile 模块用于创建临时文件
import atexit   # 导入 atexit 模块用于注册退出时的清理函数

# 定义语音播放器类
class VoicePlayer:
    def __init__(self):
        """初始化语音播放器"""
        # 创建临时文件列表
        self.temp_files = []
        # 注册退出时的清理函数
        atexit.register(self.cleanup_temp_files)
        
        # 检查ffmpeg是否可用
        try:
            subprocess.check_output(['ffmpeg', '-version'])
            rospy.loginfo("ffmpeg 可用")
        except:
            rospy.logerr("ffmpeg 不可用，请安装: sudo apt-get install ffmpeg")
            return

        # 初始化节点
        rospy.init_node('voice_player_node', anonymous=True)
        
        # 语音文件根目录路径
        self.voice_path = "/home/ucar/lby_ws/src/xf_mic_asr_offline/voices/"
        if not os.path.exists(self.voice_path):
            rospy.logerr("语音文件目录不存在: %s", self.voice_path)
            return
        yu
        # 创建空字典并预加载语音文件
        self.voices_files = {}
        self.load_voices_files()
        
        # 订阅语音播放状态
        rospy.Subscriber("/status_manager/voice_status", String, self.voice_callback)
        
        rospy.loginfo("语音播放节点初始化完成")

    def cleanup_temp_files(self):
        """清理临时文件"""
        for temp_file in self.temp_files:
            try:
                if os.path.exists(temp_file):
                    os.remove(temp_file)
            except Exception as e:
                rospy.logwarn("清理临时文件失败: %s", str(e))
        self.temp_files = []

    def convert_to_wav(self, mp3_path):
        """将MP3转换为WAV格式"""
        try:
            wav_file = tempfile.mktemp(suffix='.wav')
            self.temp_files.append(wav_file)
            
            command = ['ffmpeg', '-i', mp3_path, '-acodec', 'pcm_s16le', 
                      '-ar', '44100', '-y', wav_file]
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()
            
            if process.returncode != 0:
                rospy.logerr("音频转换失败，返回码: %s", process.returncode)
                rospy.logerr("标准错误: %s", stderr)
                return None
                
            if not os.path.exists(wav_file):
                rospy.logerr("WAV文件未生成")
                return None
                
            return wav_file
            
        except Exception as e:
            rospy.logerr("音频转换失败: %s", str(e))
            return None

    def load_voices_files(self):
        """预加载所有语音文件"""
        try:
            # 定义要加载的文件列表
            files_to_load = {
                # task_get 目录
                "task_get_dessert": "task_get/dessert_QR.mp3",
                "task_get_fruit": "task_get/fruit_QR.mp3",
                "task_get_veg": "task_get/veg_QR.mp3",
                
                # get 目录
                "get_apple": "get/apple_get.mp3",
                "get_banana": "get/banana_get.mp3",
                "get_cake": "get/cake_get.mp3",
                "get_cola": "get/cola_get.mp3",
                "get_milk": "get/milk_get.mp3",
                "get_pepper": "get/pepper_get.mp3",
                "get_potato": "get/potato_get.mp3",
                "get_tomato": "get/tomato_get.mp3",
                "get_watermelon": "get/watermelon_get.mp3",
                
                # room 目录
                "room_A": "room/A_room.mp3",
                "room_B": "room/B_room.mp3",
                "room_C": "room/C_room.mp3",
                
                # way 目录
                "way_1": "way/way_1.mp3",
                "way_2": "way/way_2.mp3",
                
                # task_fin 目录
                "task_fin_apple_banana": "task_fin/fin_apple_banana.mp3",
                "task_fin_apple_watermelon": "task_fin/fin_apple_watermelon.mp3",
                "task_fin_banana_apple": "task_fin/fin_banana_apple.mp3",
                "task_fin_banana_watermelon": "task_fin/fin_banana_watermelon.mp3",
                "task_fin_cake_coke": "task_fin/fin_cake_coke.mp3",
                "task_fin_cake_milk": "task_fin/fin_cake_milk.mp3",
                "task_fin_chili_potato": "task_fin/fin_chili_potato.mp3",
                "task_fin_chili_tomato": "task_fin/fin_chili_tomato.mp3",
                "task_fin_coke_cake": "task_fin/fin_coke_cake.mp3",
                "task_fin_coke_milk": "task_fin/fin_coke_milk.mp3",
                "task_fin_milk_cake": "task_fin/fin_milk_cake.mp3",
                "task_fin_milk_coke": "task_fin/fin_milk_coke.mp3",
                "task_fin_potato_chili": "task_fin/fin_potato_chili.mp3",
                "task_fin_potato_tomato": "task_fin/fin_potato_tomato.mp3",
                "task_fin_tomato_chili": "task_fin/fin_tomato_chili.mp3",
                "task_fin_tomato_potato": "task_fin/fin_tomato_potato.mp3",
                "task_fin_watermelon_apple": "task_fin/fin_watermelon_apple.mp3",
                "task_fin_watermelon_banana": "task_fin/fin_watermelon_banana.mp3"
            }
            
            # 逐个加载文件
            for key, filepath in files_to_load.items():
                try:
                    full_path = os.path.join(self.voice_path, filepath)
                    if not os.path.exists(full_path):
                        rospy.logwarn("文件不存在: %s", full_path)
                        continue
                        
                    rospy.loginfo("正在加载: %s", filepath)
                    # 转换为WAV
                    wav_file = self.convert_to_wav(full_path)
                    if wav_file:
                        self.voices_files[key] = AudioSegment.from_wav(wav_file)
                        rospy.loginfo("成功加载: %s", key)
                    
                except Exception as e:
                    rospy.logerr("加载文件 %s 失败: %s", filepath, str(e))
                    continue
                    
            rospy.loginfo("完成音频加载，共加载 %d 个文件", len(self.voices_files))
            
        except Exception as e:
            rospy.logerr("语音文件加载失败: %s", str(e))

    def voice_callback(self, msg):
        """语音播放回调函数"""
        try:
            status = msg.data
            rospy.loginfo("收到播放请求: %s", status)
            
            if status in self.voices_files:
                rospy.loginfo("开始播放音频...")
                play(self.voices_files[status])
                rospy.loginfo("播放完成")
            else:
                rospy.logwarn("未找到对应音频: %s", status)
                
        except Exception as e:
            rospy.logerr("播放出错: %s", str(e))

    def run(self):
        """运行节点"""
        try:
            rospy.loginfo("节点开始运行...")
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("收到用户中断")
        except Exception as e:
            rospy.logerr("运行出错: %s", str(e))
        finally:
            self.cleanup_temp_files()
            rospy.loginfo("节点退出")

def signal_handler(sig, frame):
    """信号处理函数"""
    rospy.loginfo("接收到退出信号")
    rospy.signal_shutdown("用户中断")
    sys.exit(0)

if __name__ == '__main__':
    try:
        # 注册信号处理
        signal.signal(signal.SIGINT, signal_handler)
        
        # 创建并运行节点
        player = VoicePlayer()
        player.run()
    except Exception as e:
        rospy.logerr("程序异常: %s", str(e))
    finally:
        rospy.loginfo("程序结束")