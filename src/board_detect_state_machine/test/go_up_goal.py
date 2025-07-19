#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('direct_navigator_node')
    
    # 实例化Action客户端
    rospy.loginfo("正在连接move_base服务器...")
    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
    # 等待Action服务器
    move_base_client.wait_for_server()
    rospy.loginfo("已连接到move_base服务器")
    
    # 目标位姿定义
    target_pose = PoseStamped()
    target_pose.header.frame_id = "map"
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = 0.75
    target_pose.pose.position.y = 4.50
    target_pose.pose.position.z = 0.0
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.7071
    target_pose.pose.orientation.w = 0.7071
    
    # 创建并发送导航目标
    goal = MoveBaseGoal()
    goal.target_pose = target_pose
    
    rospy.loginfo("发送导航目标: x=%.2f, y=%.2f", target_pose.pose.position.x, target_pose.pose.position.y)
    move_base_client.send_goal(goal)
    
    # 等待导航结果，设置60秒超时
    rospy.loginfo("等待导航结果...")
    succeeded = move_base_client.wait_for_result(rospy.Duration(60.0))
    status = move_base_client.get_state()
    
    # 结果处理与日志
    if succeeded and status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("导航成功到达目标点!")
    else:
        rospy.logerr("导航失败! 状态码: %d", status)