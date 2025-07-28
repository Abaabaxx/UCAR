#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import time

#持续前进的测试脚本
# 定义控制参数
FORWARD_SPEED = 0.2  # 前进速度，单位：米/秒
LOOP_RATE = 10      # 发布频率，单位：Hz

def stop(pub):
    """发布停止指令"""
    stop_msg = Twist()
    pub.publish(stop_msg)
    rospy.sleep(0.1)  # 稍微等待一下，确保消息被发送

def test_forward(pub, rate):
    """测试持续前进"""
    print("开始测试持续前进...")
    print("前进速度: %.2f m/s" % FORWARD_SPEED)
    
    # 创建Twist消息
    twist_msg = Twist()
    twist_msg.linear.x = FORWARD_SPEED  # 设置前进速度
    twist_msg.angular.z = 0.0           # 确保不旋转
    
    try:
        while not rospy.is_shutdown():
            # 发布速度指令
            pub.publish(twist_msg)
            # 按照设定的频率延时
            rate.sleep()
            
    except KeyboardInterrupt:
        print("\n收到键盘中断，停止运动...")
        stop(pub)

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('forward_test_node')
        
        # 创建速度指令的发布者
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 创建频率控制器
        rate = rospy.Rate(LOOP_RATE)
        
        # 等待1秒，确保发布者已经完全初始化
        rospy.sleep(1)
        
        # 执行前进测试
        test_forward(pub, rate)
        
    except rospy.ROSInterruptException:
        pass
    finally:
        # 确保在程序退出时停止小车
        if 'pub' in locals():
            stop(pub)
            print("已发送停止指令，程序结束。")


