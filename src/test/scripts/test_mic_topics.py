#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, Int8
from xf_mic_asr_offline.msg import Pcm_Msg

class MicTopicsTester:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('mic_topics_tester', anonymous=True)
        rospy.loginfo("话题测试节点已启动...")

        # 订阅所有相关话题
        self.awake_sub = rospy.Subscriber('/mic/awake/angle', Int32, self.awake_callback)
        self.major_mic_sub = rospy.Subscriber('/mic/major_mic', Int8, self.major_mic_callback)
        self.pcm_sub = rospy.Subscriber('/mic/pcm/deno', Pcm_Msg, self.pcm_callback)

        # 记录收到的消息数量
        self.awake_count = 0
        self.major_mic_count = 0
        self.pcm_count = 0

        # 记录开始时间
        self.start_time = rospy.get_time()

    def awake_callback(self, msg):
        """唤醒角度话题回调"""
        self.awake_count += 1
        current_time = rospy.get_time() - self.start_time
        rospy.loginfo("[%.2f秒] 收到唤醒角度消息: %d (第%d条消息)", 
                      current_time, msg.data, self.awake_count)

    def major_mic_callback(self, msg):
        """主麦克风话题回调"""
        self.major_mic_count += 1
        current_time = rospy.get_time() - self.start_time
        rospy.loginfo("[%.2f秒] 收到主麦克风消息: %d (第%d条消息)", 
                      current_time, msg.data, self.major_mic_count)

    def pcm_callback(self, msg):
        """音频数据话题回调"""
        self.pcm_count += 1
        current_time = rospy.get_time() - self.start_time
        rospy.loginfo("[%.2f秒] 收到PCM数据消息，长度: %d (第%d条消息)", 
                      current_time, len(msg.pcm_buf), self.pcm_count)

    def print_status(self):
        """定期打印状态信息"""
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            rospy.loginfo("\n=== 话题统计信息 ===")
            rospy.loginfo("运行时间: %.2f 秒", rospy.get_time() - self.start_time)
            rospy.loginfo("唤醒角度消息数: %d", self.awake_count)
            rospy.loginfo("主麦克风消息数: %d", self.major_mic_count)
            rospy.loginfo("PCM数据消息数: %d", self.pcm_count)
            rospy.loginfo("==================\n")
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = MicTopicsTester()
        tester.print_status()
    except rospy.ROSInterruptException:
        pass