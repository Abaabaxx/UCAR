#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, Int8
from xf_mic_asr_offline.msg import Pcm_Msg

class MicTopicsTester:
    def __init__(self):
        rospy.init_node('mic_topics_tester', anonymous=True)
        rospy.loginfo("话题测试节点已启动...")

        # 添加订阅者信息打印
        rospy.loginfo("正在订阅话题:")
        rospy.loginfo("- /mic/awake/angle (std_msgs/Int32)")
        rospy.loginfo("- /mic/major_mic (std_msgs/Int8)")
        rospy.loginfo("- /mic/pcm/deno (xf_mic_asr_offline/Pcm_Msg)")

        # 检查话题是否存在
        topics = rospy.get_published_topics()
        rospy.loginfo("\n当前已发布的话题列表:")
        for topic, type_name in topics:
            rospy.loginfo("话题: %s, 类型: %s", topic, type_name)

        # 订阅话题并添加队列大小
        self.awake_sub = rospy.Subscriber('/mic/awake/angle', Int32, self.awake_callback, queue_size=10)
        self.major_mic_sub = rospy.Subscriber('/mic/major_mic', Int8, self.major_mic_callback, queue_size=10)
        self.pcm_sub = rospy.Subscriber('/mic/pcm/deno', Pcm_Msg, self.pcm_callback, queue_size=1000)

        self.awake_count = 0
        self.major_mic_count = 0
        self.pcm_count = 0
        self.start_time = rospy.get_time()

    def awake_callback(self, msg):
        current_time = rospy.get_time() - self.start_time
        self.awake_count += 1
        rospy.loginfo("[%.2f秒] 收到唤醒角度消息: %d (第%d条消息)", 
                      current_time, msg.data, self.awake_count)

    def major_mic_callback(self, msg):
        current_time = rospy.get_time() - self.start_time
        self.major_mic_count += 1
        rospy.loginfo("[%.2f秒] 收到主麦克风消息: %d (第%d条消息)", 
                      current_time, msg.data, self.major_mic_count)
        # 添加详细的主麦克风信息
        rospy.loginfo("主麦克风详细信息: data=%d", msg.data)

    def pcm_callback(self, msg):
        current_time = rospy.get_time() - self.start_time
        self.pcm_count += 1
        rospy.loginfo("[%.2f秒] 收到PCM数据消息，数据长度: %d (第%d条消息)", 
                      current_time, len(msg.pcm_buf), self.pcm_count)
        # 添加PCM数据的前几个字节信息
        if len(msg.pcm_buf) > 0:
            rospy.loginfo("PCM数据前10个字节: %s", str(msg.pcm_buf[:10]))

    def print_status(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            rospy.loginfo("\n=== 话题统计信息 ===")
            rospy.loginfo("运行时间: %.2f 秒", rospy.get_time() - self.start_time)
            rospy.loginfo("唤醒角度消息数: %d", self.awake_count)
            rospy.loginfo("主麦克风消息数: %d", self.major_mic_count)
            rospy.loginfo("PCM数据消息数: %d", self.pcm_count)
            
            # 添加话题连接状态检查
            awake_connections = self.awake_sub.get_num_connections()
            major_mic_connections = self.major_mic_sub.get_num_connections()
            pcm_connections = self.pcm_sub.get_num_connections()
            
            rospy.loginfo("\n话题连接状态:")
            rospy.loginfo("- /mic/awake/angle: %d 个发布者", awake_connections)
            rospy.loginfo("- /mic/major_mic: %d 个发布者", major_mic_connections)
            rospy.loginfo("- /mic/pcm/deno: %d 个发布者", pcm_connections)
            rospy.loginfo("==================\n")
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = MicTopicsTester()
        tester.print_status()
    except rospy.ROSInterruptException:
        pass