#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import actionlib
import yaml
import os
import glob
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


def load_yaml_goals(goals_dir):
    """加载YAML文件中的目标点"""
    goal_poses = []
    
    try:
        # 获取所有goal*.yaml文件
        yaml_files = glob.glob(os.path.join(goals_dir, 'goal*.yaml'))
        
        # 如果没有找到文件，返回空列表
        if not yaml_files:
            rospy.logerr("在{}目录下未找到任何goal*.yaml文件".format(goals_dir))
            return goal_poses
        
        # 按文件名数字排序
        yaml_files.sort(key=lambda f: int(''.join(filter(str.isdigit, os.path.basename(f)))))
        
        rospy.loginfo("找到{}个目标点文件".format(len(yaml_files)))
        
        for file_path in yaml_files:
            try:
                # 读取YAML文件
                with open(file_path, 'r') as yaml_file:
                    goal_data = yaml.safe_load(yaml_file)
                
                # 从YAML数据创建PoseStamped对象
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = goal_data['header']['frame_id']
                # header.stamp在发送前设置，这里可以先设为0
                pose_stamped.header.stamp = rospy.Time(0)
                
                pose_stamped.pose.position.x = goal_data['pose']['position']['x']
                pose_stamped.pose.position.y = goal_data['pose']['position']['y']
                pose_stamped.pose.position.z = goal_data['pose']['position']['z']
                
                pose_stamped.pose.orientation.x = goal_data['pose']['orientation']['x']
                pose_stamped.pose.orientation.y = goal_data['pose']['orientation']['y']
                pose_stamped.pose.orientation.z = goal_data['pose']['orientation']['z']
                pose_stamped.pose.orientation.w = goal_data['pose']['orientation']['w']
                
                # 添加到目标点列表
                goal_poses.append(pose_stamped)
                rospy.loginfo("已加载目标点: {}".format(os.path.basename(file_path)))
            
            except Exception as e:
                rospy.logerr("处理文件{}时出错: {}".format(file_path, str(e)))
    
    except Exception as e:
        rospy.logerr("加载YAML目标点时出错: {}".format(str(e)))
    
    return goal_poses


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('yaml_goal_tester')
    
    # 定义目标点目录路径
    goals_dir = '/home/ucar/lby_ws/src/board_detect/goals'
    
    # 加载YAML文件中的目标点
    goal_poses = load_yaml_goals(goals_dir)
    
    # 检查是否成功加载了目标点
    if not goal_poses:
        rospy.logerr("没有找到有效的目标点，程序终止。")
        exit(1)
    
    rospy.loginfo("成功加载了{}个目标点".format(len(goal_poses)))
    
    # 初始化Action Client
    rospy.loginfo("等待导航服务器...")
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("导航服务器连接成功!")
    
    # 遍历所有目标点，发送并等待每个目标
    for i, pose_stamped in enumerate(goal_poses):
        # 创建MoveBaseGoal实例
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        # 在发送前更新时间戳
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 打印当前要发送的目标点信息
        rospy.loginfo("正在发送第{}个目标点 (总共{}个)".format(i + 1, len(goal_poses)))
        rospy.loginfo("目标位置: x={:.2f}, y={:.2f}".format(pose_stamped.pose.position.x, pose_stamped.pose.position.y))
        
        # 发送目标点
        client.send_goal(goal)
        
        # 阻塞式等待导航结果
        rospy.loginfo("等待导航结果...")
        client.wait_for_result()
        
        # 获取导航状态
        status = client.get_state()
        
        # 根据状态打印结果
        status_str = {
            0: "PENDING",
            1: "ACTIVE",
            2: "PREEMPTED",
            3: "SUCCEEDED",
            4: "ABORTED",
            5: "REJECTED",
            6: "PREEMPTING",
            7: "RECALLING",
            8: "RECALLED",
            9: "LOST"
        }.get(status, "UNKNOWN({})".format(status))
        
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("目标点{}导航成功!".format(i + 1))
        else:
            rospy.logerr("目标点{}导航失败: {}".format(i + 1, status_str))
        
        # 短暂延时，方便观察
        rospy.sleep(1.0)
    
    rospy.loginfo("所有目标点尝试发送完毕。")