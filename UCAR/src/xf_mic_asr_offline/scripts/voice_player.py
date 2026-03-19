#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from pydub import AudioSegment
from pydub.playback import play
import os
import signal
import sys

# 导入您的自定义服务消息
from xf_mic_asr_offline.srv import VoiceCmd, VoiceCmdResponse

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
        
        # 创建使用您的自定义服务消息类型的服务
        self.voice_service = rospy.Service('/robot/voice_cmd', VoiceCmd, self.voice_service_callback)
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
                "task_fin_cake_cola": "task_fin/fin_cake_cola.wav",
                "task_fin_cake_milk": "task_fin/fin_cake_milk.wav",
                "task_fin_pepper_potato": "task_fin/fin_pepper_potato.wav",
                "task_fin_pepper_tomato": "task_fin/fin_pepper_tomato.wav",
                "task_fin_cola_cake": "task_fin/fin_cola_cake.wav",
                "task_fin_cola_milk": "task_fin/fin_cola_milk.wav",
                "task_fin_milk_cake": "task_fin/fin_milk_cake.wav",
                "task_fin_milk_cola": "task_fin/fin_milk_cola.wav",
                "task_fin_potato_pepper": "task_fin/fin_potato_pepper.wav",
                "task_fin_potato_tomato": "task_fin/fin_potato_tomato.wav",
                "task_fin_tomato_pepper": "task_fin/fin_tomato_pepper.wav",
                "task_fin_tomato_potato": "task_fin/fin_tomato_potato.wav",
                "task_fin_watermelon_apple": "task_fin/fin_watermelon_apple.wav",
                "task_fin_watermelon_banana": "task_fin/fin_watermelon_banana.wav",
                # --- 新增的重复物品播报 ---
                "task_fin_apple_apple": "task_fin/fin_apple_apple.wav",
                "task_fin_banana_banana": "task_fin/fin_banana_banana.wav",
                "task_fin_watermelon_watermelon": "task_fin/fin_watermelon_watermelon.wav",
                "task_fin_cake_cake": "task_fin/fin_cake_cake.wav",
                "task_fin_cola_cola": "task_fin/fin_cola_cola.wav",
                "task_fin_milk_milk": "task_fin/fin_milk_milk.wav",
                "task_fin_pepper_pepper": "task_fin/fin_pepper_pepper.wav",
                "task_fin_potato_potato": "task_fin/fin_potato_potato.wav",
                "task_fin_tomato_tomato": "task_fin/fin_tomato_tomato.wav"
            }
            
            # 在开始加载文件前输出提示
            rospy.loginfo("正在加载文件...")
            
            total_files = len(files_to_load)
            loaded_count = 0
            failed_count = 0
            
            for key, filepath in files_to_load.items():
                try:
                    full_path = os.path.join(self.voice_path, filepath)
                    if not os.path.exists(full_path):
                        failed_count += 1
                        continue
                        
                    # 直接加载WAV文件，不输出加载过程的日志
                    self.voices_files[key] = AudioSegment.from_wav(full_path)
                    loaded_count += 1
                    
                except Exception as e:
                    failed_count += 1
                    continue
            
            # 判断是否所有文件都成功加载
            if loaded_count == total_files:
                rospy.loginfo("完成音频加载，成功加载所有音频文件，共计 %d 个", loaded_count)
            else:
                rospy.loginfo("完成音频加载，共加载 %d 个文件，失败 %d 个", loaded_count, failed_count)
            
        except Exception as e:
            rospy.logerr("语音文件加载失败: %s", str(e))

    # 使用您的自定义服务消息类型的回调函数
    def voice_service_callback(self, req):
        response = VoiceCmdResponse()
        try:
            voice_cmd = req.voice_cmd
            rospy.loginfo("收到播放请求: %s", voice_cmd)
            
            if voice_cmd in self.voices_files:
                rospy.loginfo("开始播放音频...")
                play(self.voices_files[voice_cmd])
                rospy.loginfo("播放完成")
                response.success = True
                # 返回英文消息
                response.message = "Successfully played voice: " + voice_cmd
            else:
                rospy.logwarn("未找到对应音频: %s", voice_cmd)
                response.success = False
                # 返回英文消息
                response.message = "Voice not found: " + voice_cmd
                
        except Exception as e:
            rospy.logerr("播放出错: %s", str(e))
            response.success = False
            # 返回英文消息
            response.message = "Playback error: " + str(e)
        
        return response

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