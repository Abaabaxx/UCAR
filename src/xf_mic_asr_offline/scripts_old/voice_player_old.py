#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from pydub import AudioSegment
from pydub.playback import play
import os
import signal
import sys

# 在初始化前取消设置DISPLAY环境变量（仅对当前 Python 脚本进程及其子进程生效，不会影响终端或系统环境）
os.environ.pop('DISPLAY', None)  # 首选方法，直接在Python环境中取消设置

class VoicePlayer:
    def __init__(self):
        rospy.init_node('voice_player_node', anonymous=True)
        
        self.voice_path = "/home/ucar/lby_ws/src/xf_mic_asr_offline/voices_wav/"
        if not os.path.exists(self.voice_path):
            rospy.logerr("语音文件目录不存在: %s", self.voice_path)
            return
        
        self.voices_files = {}
        self.load_voices_files()
        
        rospy.Subscriber("/status_manager/voice_status", String, self.voice_callback)
        rospy.loginfo("语音播放节点初始化完成")

    def load_voices_files(self):
        try:
            files_to_load = {
                "task_get_dessert": "task_get/dessert_QR.wav",
                "task_get_fruit": "task_get/fruit_QR.wav",
                "task_get_veg": "task_get/veg_QR.wav",
                "get_apple": "get/apple_get.wav",
                "get_banana": "get/banana_get.wav",
                "get_cake": "get/cake_get.wav",
                "get_cola": "get/cola_get.wav",
                "get_milk": "get/milk_get.wav",
                "get_pepper": "get/pepper_get.wav",
                "get_potato": "get/potato_get.wav",
                "get_tomato": "get/tomato_get.wav",
                "get_watermelon": "get/watermelon_get.wav",
                "room_A": "room/A_room.wav",
                "room_B": "room/B_room.wav",
                "room_C": "room/C_room.wav",
                "way_1": "way/way_1.wav",
                "way_2": "way/way_2.wav",
                "task_fin_apple_banana": "task_fin/fin_apple_banana.wav",
                "task_fin_apple_watermelon": "task_fin/fin_apple_watermelon.wav",
                "task_fin_banana_apple": "task_fin/fin_banana_apple.wav",
                "task_fin_banana_watermelon": "task_fin/fin_banana_watermelon.wav",
                "task_fin_cake_coke": "task_fin/fin_cake_coke.wav",
                "task_fin_cake_milk": "task_fin/fin_cake_milk.wav",
                "task_fin_chili_potato": "task_fin/fin_chili_potato.wav",
                "task_fin_chili_tomato": "task_fin/fin_chili_tomato.wav",
                "task_fin_coke_cake": "task_fin/fin_coke_cake.wav",
                "task_fin_coke_milk": "task_fin/fin_coke_milk.wav",
                "task_fin_milk_cake": "task_fin/fin_milk_cake.wav",
                "task_fin_milk_coke": "task_fin/fin_milk_coke.wav",
                "task_fin_potato_chili": "task_fin/fin_potato_chili.wav",
                "task_fin_potato_tomato": "task_fin/fin_potato_tomato.wav",
                "task_fin_tomato_chili": "task_fin/fin_tomato_chili.wav",
                "task_fin_tomato_potato": "task_fin/fin_tomato_potato.wav",
                "task_fin_watermelon_apple": "task_fin/fin_watermelon_apple.wav",
                "task_fin_watermelon_banana": "task_fin/fin_watermelon_banana.wav"
            }
            
            for key, filepath in files_to_load.items():
                try:
                    full_path = os.path.join(self.voice_path, filepath)
                    if not os.path.exists(full_path):
                        rospy.logwarn("文件不存在: %s", full_path)
                        continue
                        
                    rospy.loginfo("正在加载: %s", filepath)
                    # 直接加载WAV文件
                    self.voices_files[key] = AudioSegment.from_wav(full_path)
                    rospy.loginfo("成功加载: %s", key)
                    
                except Exception as e:
                    rospy.logerr("加载文件 %s 失败: %s", filepath, str(e))
                    continue
                    
            rospy.loginfo("完成音频加载，共加载 %d 个文件", len(self.voices_files))
            
        except Exception as e:
            rospy.logerr("语音文件加载失败: %s", str(e))

    def voice_callback(self, msg):
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
        try:
            rospy.loginfo("节点开始运行...")
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("收到用户中断")
        except Exception as e:
            rospy.logerr("运行出错: %s", str(e))
        finally:
            rospy.loginfo("节点退出")

def signal_handler(sig, frame):
    rospy.loginfo("接收到退出信号")
    rospy.signal_shutdown("用户中断")
    sys.exit(0)

if __name__ == '__main__':
    try:
        signal.signal(signal.SIGINT, signal_handler)
        player = VoicePlayer()
        player.run()
    except Exception as e:
        rospy.logerr("程序异常: %s", str(e))
    finally:
        rospy.loginfo("程序结束")