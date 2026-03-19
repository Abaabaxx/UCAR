#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from pydub import AudioSegment
from pydub.playback import play
import os
import signal
import sys
import time
import socket

def check_ros_env():
    """检查ROS环境配置"""
    print("\n=== ROS环境检查 ===")
    ros_vars = ['ROS_MASTER_URI', 'ROS_IP', 'ROS_HOSTNAME', 'ROS_ROOT']
    for var in ros_vars:
        value = os.environ.get(var, '未设置')
        print("{}: {}".format(var, value))

def check_ros_master():
    """检查ROS Master连接状态"""
    print("\n=== ROS Master检查 ===")
    try:
        master = rospy.get_master()
        pid = master.getPid()
        print("ROS Master运行中，PID: {}".format(pid))
        return True
    except Exception as e:
        print("无法连接ROS Master: {}".format(e))
        return False

class VoicePlayer:
    def __init__(self):
        """初始化语音播放器"""
        try:
            # 环境检查
            check_ros_env()
            if not check_ros_master():
                raise Exception("请确保roscore已启动")

            # 音频测试
            print("\n=== 音频功能测试 ===")
            test_file = os.path.join("/home/ucar/lby_ws/src/xf_mic_asr_offline/voices/", 
                                   "task_get/dessert_QR.mp3")
            if not os.path.exists(test_file):
                raise Exception("测试文件不存在: {}".format(test_file))
                
            print("加载测试音频文件...")
            test_audio = AudioSegment.from_file(test_file)
            print("音频加载成功! 时长: {:.2f}秒".format(len(test_audio)/1000))

            # ROS节点初始化
            print("\n=== ROS节点初始化 ===")
            print("正在初始化节点...")
            rospy.init_node('voice_player_test', anonymous=True, disable_signals=True)
            print("节点初始化完成")

            # 音频文件管理
            self.voice_path = "/home/ucar/lby_ws/src/xf_mic_asr_offline/voices/"
            self.voices_files = {"test": test_audio}
            print("音频文件加载完成")

            # 话题订阅
            rospy.Subscriber("/status_manager/voice_status", String, self.voice_callback)
            print("话题订阅成功")

        except Exception as e:
            print("\n初始化失败: {}".format(e))
            raise

    def voice_callback(self, msg):
        """语音播放回调函数"""
        try:
            status = msg.data
            print("\n收到播放请求: {}".format(status))
            
            if status in self.voices_files:
                print("开始播放音频...")
                play(self.voices_files[status])
                print("播放完成")
            else:
                print("未找到对应音频: {}".format(status))
                
        except Exception as e:
            print("播放出错: {}".format(e))

    def run(self):
        """运行节点"""
        try:
            print("\n=== 节点运行 ===")
            print("等待播放请求...")
            rospy.spin()
        except KeyboardInterrupt:
            print("\n收到用户中断")
        except Exception as e:
            print("\n运行出错: {}".format(e))
        finally:
            print("节点退出")

if __name__ == '__main__':
    try:
        # 信号处理
        def signal_handler(sig, frame):
            print("\n接收到退出信号")
            rospy.signal_shutdown("用户中断")
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        print("=== 语音播放测试节点 ===")
        player = VoicePlayer()
        player.run()

    except Exception as e:
        print("\n程序异常: {}".format(e))
    finally:
        print("\n程序结束")