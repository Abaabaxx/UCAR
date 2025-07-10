#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped

"""
修改说明：
1. 添加状态机重置功能
2. 执行顺序：重置状态机 -> 设置初始位置 -> 清除代价地图

运行方式：
python setup.py

作者：abaabaxxx
创建时间：2025-04-29 13:46:50
"""

def reset_state_machine():
    """调用重置状态机服务"""
    try:
        rospy.wait_for_service('/reset_state_machine', timeout=5.0)
        reset_service = rospy.ServiceProxy('/reset_state_machine', Trigger)
        response = reset_service()
        if response.success:
            rospy.loginfo("状态机重置成功: %s", response.message)
            return True
        else:
            rospy.logwarn("状态机重置失败: %s", response.message)
            return False
    except rospy.ROSException as e:
        rospy.logerr("服务调用失败: %s", str(e))
        return False

def clear_costmaps():
    """调用清除代价地图服务"""
    try:
        rospy.wait_for_service('/move_base/clear_costmaps', timeout=5.0)
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps_service()
        rospy.loginfo("代价地图清除成功")
        return True
    except rospy.ROSException as e:
        rospy.logerr("服务调用失败: %s", str(e))
        return False

def set_initial_pose():
    """发布机器人初始位姿到 /initialpose 话题"""
    # 创建初始位姿的发布者
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    # 等待发布者注册
    rospy.sleep(0.5)
    
    # 创建 PoseWithCovarianceStamped 消息
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"  # 设置坐标系
    pose.header.stamp = rospy.Time.now()
    
    # 设置位置
    pose.pose.pose.position.x = 0.25
    pose.pose.pose.position.y = 0.25
    pose.pose.pose.position.z = 0.0
    
    # 设置方向（四元数，假设偏航角为0）
    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y = 0.0
    pose.pose.pose.orientation.z = 0.0
    pose.pose.pose.orientation.w = 1.0
    
    # 设置协方差
    # x, y, z, roll, pitch, yaw 的对角线值
    pose.pose.covariance[0] = 0.0025  # x方向
    pose.pose.covariance[7] = 0.0025  # y方向
    pose.pose.covariance[35] = 0.0171  # 偏航角
    
    # 发布消息
    pub.publish(pose)
    rospy.loginfo("初始位姿已发布")
    
    # 等待消息处理
    rospy.sleep(0.5)

if __name__ == "__main__":
    try:
        # 初始化ROS节点
        rospy.init_node('setup_robot_pose', anonymous=True)
        
        # 第一步：重置状态机
        rospy.loginfo("第一步：重置状态机...")
        if reset_state_machine():
            rospy.loginfo("状态机重置成功")
        else:
            rospy.logwarn("状态机重置失败")
        
        # 第二步：设置初始位姿
        rospy.loginfo("第二步：设置初始位姿...")
        set_initial_pose()
        
        # 第三步：清除代价地图
        rospy.loginfo("第三步：清除代价地图...")
        if clear_costmaps():
            rospy.loginfo("代价地图清除成功")
        else:
            rospy.logwarn("代价地图清除失败")
        
        rospy.loginfo("所有设置步骤已完成！")
    
    except rospy.ROSInterruptException:
        rospy.logerr("ROS被中断")
        pass
    except Exception as e:
        rospy.logerr("出现错误: %s", str(e))