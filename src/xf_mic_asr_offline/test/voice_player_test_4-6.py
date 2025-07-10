#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import os
import subprocess
import signal
import sys
import threading

class VoicePlayer:
    def __init__(self):
        # 先初始化节点，确保日志正常工作
        rospy.init_node('voice_player_node', anonymous=True)
        
        # 设置预先转换好的WAV文件路径
        self.voice_path = "/home/ucar/lby_ws/src/xf_mic_asr_offline/voices_wav/"
        if not os.path.exists(self.voice_path):
            rospy.logerr("语音文件目录不存在: %s", self.voice_path)
            return
        
        # 创建字典存储WAV文件路径
        self.voices_files = {}
        self.load_voices_files()
        
        # 当前播放进程
        self.current_process = None
        self.play_lock = threading.Lock()
        
        # 创建订阅者
        rospy.Subscriber("/status_manager/voice_status", String, self.voice_callback)
        rospy.loginfo("语音播放节点初始化完成")

    def load_voices_files(self):
        """加载WAV文件路径映射"""
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
            
            # 验证文件存在性
            for key, filepath in files_to_load.items():
                full_path = os.path.join(self.voice_path, filepath)
                if os.path.exists(full_path):
                    self.voices_files[key] = full_path
                    rospy.loginfo("已加载: %s -> %s", key, full_path)
                else:
                    rospy.logwarn("文件不存在: %s", full_path)
                    
            rospy.loginfo("完成音频路径加载，共加载 %d 个文件", len(self.voices_files))
            
        except Exception as e:
            rospy.logerr("语音文件路径加载失败: %s", str(e))

    def play_with_aplay(self, wav_path):
        """使用aplay命令播放WAV文件"""
        try:
            with self.play_lock:
                # 终止正在播放的音频（如有）
                self.stop_current_playback()
                
                # 使用aplay播放新音频
                self.current_process = subprocess.Popen(
                    ['aplay', wav_path], 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.PIPE
                )
                
                # 启动监视线程
                threading.Thread(target=self._monitor_playback, args=(wav_path,)).start()
                
            return True
        except Exception as e:
            rospy.logerr("播放失败: %s", str(e))
            return False
    
    def _monitor_playback(self, wav_path):
        """监控播放进程完成情况"""
        if self.current_process:
            self.current_process.wait()
            if self.current_process.returncode == 0:
                rospy.loginfo("播放完成: %s", os.path.basename(wav_path))
            else:
                stdout, stderr = self.current_process.communicate()
                rospy.logwarn("播放异常退出，返回码: %d, 错误: %s", 
                            self.current_process.returncode, stderr)

    def stop_current_playback(self):
        """停止当前播放进程"""
        if self.current_process and self.current_process.poll() is None:
            try:
                self.current_process.terminate()
                self.current_process.wait(timeout=1)
            except:
                try:
                    self.current_process.kill()
                except:
                    pass

    def voice_callback(self, msg):
        """语音播放回调"""
        try:
            status = msg.data
            rospy.loginfo("收到播放请求: %s", status)
            
            if status in self.voices_files:
                wav_path = self.voices_files[status]
                rospy.loginfo("开始播放音频: %s", wav_path)
                # 使用非阻塞方式启动播放
                threading.Thread(target=self.play_with_aplay, args=(wav_path,)).start()
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
            self.stop_current_playback()
            rospy.loginfo("节点退出")

def signal_handler(sig, frame):
    """信号处理函数"""
    rospy.loginfo("接收到退出信号")
    rospy.signal_shutdown("用户中断")
    sys.exit(0)

if __name__ == '__main__':
    try:
        # 注册信号处理器
        signal.signal(signal.SIGINT, signal_handler)
        # 创建并运行节点
        player = VoicePlayer()
        player.run()
    except Exception as e:
        rospy.logerr("程序异常: %s", str(e))
    finally:
        rospy.loginfo("程序结束")