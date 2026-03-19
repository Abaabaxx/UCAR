#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from pydub import AudioSegment
from pydub.playback import play
import os
import subprocess
import signal
import sys
import tempfile
import atexit

class VoicePlayer:
    def __init__(self):
        """初始化语音播放器"""
        # 临时文件列表
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
        
        # 语音文件路径
        self.voice_path = "/home/ucar/lby_ws/src/xf_mic_asr_offline/voices/"
        if not os.path.exists(self.voice_path):
            rospy.logerr("语音文件目录不存在: %s", self.voice_path)
            return
        
        # 预加载语音文件
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

    def load_voices_files(self):
        """预加载所有语音文件"""
        try:
            # 先测试加载一个文件
            test_file = os.path.join(self.voice_path, "task_get/dessert_QR.mp3")
            if not os.path.exists(test_file):
                rospy.logerr("测试文件不存在: %s", test_file)
                return
                
            # 创建临时WAV文件
            wav_file = tempfile.mktemp(suffix='.wav')
            self.temp_files.append(wav_file)
            
            try:
                rospy.loginfo("尝试转换音频文件为 WAV 格式...")
                # 使用ffmpeg转换音频
                command = ['ffmpeg', '-i', test_file, '-acodec', 'pcm_s16le', 
                         '-ar', '44100', '-y', wav_file]
                process = subprocess.Popen(command, 
                                        stdout=subprocess.PIPE, 
                                        stderr=subprocess.PIPE)
                stdout, stderr = process.communicate()
                
                if process.returncode != 0:
                    rospy.logerr("音频转换失败，返回码: %s", process.returncode)
                    rospy.logerr("标准错误: %s", stderr)
                    return
                
                if not os.path.exists(wav_file):
                    rospy.logerr("WAV文件未生成")
                    return
                    
                rospy.loginfo("音频转换成功")
                test_audio = AudioSegment.from_wav(wav_file)
                rospy.loginfo("WAV 文件加载成功")
                
                # 保存到字典
                self.voices_files = {
                    "test": test_audio
                }
                rospy.loginfo("测试文件加载成功")
                
            except Exception as e:
                rospy.logerr("音频处理失败: %s", str(e))
                return
                
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