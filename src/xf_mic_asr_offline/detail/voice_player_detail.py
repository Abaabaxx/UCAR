#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
语音播放节点(voice_player_node)

功能描述：
---------
1. 加载并播放WAV格式的语音文件
2. 通过ROS话题接收播放指令

支持的话题消息内容（键名）：
--------------
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

使用方法：
--------
1. 启动节点：
   rosrun xf_mic_asr_offline voice_player_node
   python2 /home/ucar/lby_ws/src/xf_mic_asr_offline/scripts/voice_player.py

2. 发布话题控制播放：
   rostopic pub /status_manager/voice_status std_msgs/String "data: '语音指令'"
   
   示例：
   
   # 播放 "取到苹果" 提示音
   rostopic pub /status_manager/voice_status std_msgs/String "data: 'get_apple'"

话题说明：
--------
订阅话题：
- 名称：/status_manager/voice_status
- 类型：std_msgs/String
- 说明：接收要播放的语音标识符

环境变量设置：
-----------
取消设置DISPLAY环境变量的原因：
1. 避免在无GUI环境下运行时出现相关警告
2. 确保在远程SSH连接时也能正常运行
3. 防止某些音频库尝试打开GUI界面

语音文件目录结构：
-----------
voices_wav/
├── task_get/     # "采购任务" 提示音
├── get/          # "取到物品" 提示音
├── room/         # "房间" 提示音
├── way/          # "路口" 提示音
└── task_fin/     # "任务完成" 提示音

注意事项：
--------
1. 确保voices_wav目录存在且包含所有必要的WAV文件
2. 文件必须是WAV格式，推荐采样率44100Hz，16位深度
3. 节点使用pydub库播放音频，确保系统已安装该库
4. 使用Ctrl+C可以安全退出节点
5. 如果找不到指定的语音文件，节点会通过ROS日志系统提供警告信息

错误处理：
--------
1. 目录不存在：输出错误日志并返回
2. 文件加载失败：跳过该文件并继续加载其他文件
3. 播放请求无效：通过警告日志通知
4. 播放出错：通过错误日志通知具体原因

# 1. 启动ROS核心
roscore

# 2. 启动语音播放节点
rosrun xf_mic_asr_offline voice_player_node

# 3. 播放测试示例
# 播放物品识别语音
rostopic pub /status_manager/voice_status std_msgs/String "data: 'get_apple'"

# 播放任务开始提示音
rostopic pub /status_manager/voice_status std_msgs/String "data: 'task_get_fruit'"

# 播放房间提示音
rostopic pub /status_manager/voice_status std_msgs/String "data: 'room_A'"

# 播放任务完成提示音
rostopic pub /status_manager/voice_status std_msgs/String "data: 'task_fin_apple_banana'"
"""

# 导入需要的库
import rospy  # ROS Python接口
from std_msgs.msg import String  # ROS标准消息类型
from pydub import AudioSegment  # 音频处理库
from pydub.playback import play  # 音频播放功能
import os  # 文件系统操作
import signal  # 信号处理
import sys  # 系统相关功能

# 取消DISPLAY环境变量设置，避免GUI相关的问题
os.environ.pop('DISPLAY', None)

class VoicePlayer:
    """语音播放节点的主类，处理语音文件的加载和播放"""
    
    def __init__(self):
        """初始化语音播放节点"""
        # 初始化ROS节点
        rospy.init_node('voice_player_node', anonymous=True)
        
        # 设置语音文件目录
        self.voice_path = "/home/ucar/lby_ws/src/xf_mic_asr_offline/voices_wav/"
        if not os.path.exists(self.voice_path):
            rospy.logerr("语音文件目录不存在: %s", self.voice_path)
            return
        
        # 初始化语音文件字典
        self.voices_files = {}
        self.load_voices_files()
        
        # 订阅语音播放控制话题
        rospy.Subscriber("/status_manager/voice_status", String, self.voice_callback)
        rospy.loginfo("语音播放节点初始化完成")

    def load_voices_files(self):
        """加载所有语音文件"""
        try:
            # 定义需要加载的语音文件映射
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
            
            # 遍历加载每个语音文件
            for key, filepath in files_to_load.items():
                try:
                    full_path = os.path.join(self.voice_path, filepath)
                    if not os.path.exists(full_path):
                        rospy.logwarn("文件不存在: %s", full_path)
                        continue
                    
                    rospy.loginfo("正在加载: %s", filepath)
                    self.voices_files[key] = AudioSegment.from_wav(full_path)
                    rospy.loginfo("成功加载: %s", key)
                    
                except Exception as e:
                    rospy.logerr("加载文件 %s 失败: %s", filepath, str(e))
                    continue
            
            rospy.loginfo("完成音频加载，共加载 %d 个文件", len(self.voices_files))
            
        except Exception as e:
            rospy.logerr("语音文件加载失败: %s", str(e))

    def voice_callback(self, msg):
        """处理语音播放请求的回调函数"""
        try:
            status = msg.data
            rospy.loginfo("收到播放请求: %s", status)
            
            # 检查并播放请求的语音
            if status in self.voices_files:
                rospy.loginfo("开始播放音频...")
                play(self.voices_files[status])
                rospy.loginfo("播放完成")
            else:
                rospy.logwarn("未找到对应音频: %s", status)
                
        except Exception as e:
            rospy.logerr("播放出错: %s", str(e))

    def run(self):
        """运行节点主循环"""
        try:
            rospy.loginfo("节点开始运行...")
            rospy.spin()  # 保持节点运行
        except KeyboardInterrupt:
            rospy.loginfo("收到用户中断")
        except Exception as e:
            rospy.logerr("运行出错: %s", str(e))
        finally:
            rospy.loginfo("节点退出")

def signal_handler(sig, frame):
    """信号处理函数，处理Ctrl+C等中断信号"""
    rospy.loginfo("接收到退出信号")
    rospy.signal_shutdown("用户中断")
    sys.exit(0)

if __name__ == '__main__':
    try:
        # 注册信号处理函数
        signal.signal(signal.SIGINT, signal_handler)
        # 创建并运行语音播放节点
        player = VoicePlayer()
        player.run()
    except Exception as e:
        rospy.logerr("程序异常: %s", str(e))
    finally:
        rospy.loginfo("程序结束")