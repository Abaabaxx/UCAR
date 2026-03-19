#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 在 RVIZ中 可视化 /goals_up /goals_down 中的目标点

import rospy
import rospkg
import yaml
import os
import glob
from geometry_msgs.msg import PoseArray, Pose

class StaticGoalPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('static_goal_publisher', anonymous=True)
        
        # 创建发布者，发布PoseArray消息
        self.pose_publisher = rospy.Publisher('/static_goals_array', PoseArray, queue_size=10)
        
        # 创建PoseArray消息实例
        self.pose_array_msg = PoseArray()
        
        # 加载目标点
        self.load_goals()
        
        # 创建定时器，每秒发布一次
        rospy.Timer(rospy.Duration(1.0), self.publish_poses)
        
        rospy.loginfo("Static goal publisher initialized.")
    
    def load_goals(self):
        # 获取board_detect包的路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('board_detect')
        
        # 构建goals_down和goals_up目录的完整路径
        goals_down_path = os.path.join(package_path, 'goals_down')
        goals_up_path = os.path.join(package_path, 'goals_up')
        
        # 查找所有yaml文件
        down_yaml_files = glob.glob(os.path.join(goals_down_path, '*.yaml'))
        up_yaml_files = glob.glob(os.path.join(goals_up_path, '*.yaml'))
        
        # 合并文件列表
        yaml_files = down_yaml_files + up_yaml_files
        
        # 初始化PoseArray消息
        self.pose_array_msg.header.frame_id = 'map'
        
        # 遍历所有yaml文件
        for yaml_file in yaml_files:
            try:
                # 打开并解析yaml文件
                with open(yaml_file, 'r') as f:
                    goal_data = yaml.safe_load(f)
                
                # 创建Pose消息
                pose = Pose()
                
                # 设置位置
                pose.position.x = goal_data['pose']['position']['x']
                pose.position.y = goal_data['pose']['position']['y']
                pose.position.z = goal_data['pose']['position']['z']
                
                # 设置方向
                pose.orientation.x = goal_data['pose']['orientation']['x']
                pose.orientation.y = goal_data['pose']['orientation']['y']
                pose.orientation.z = goal_data['pose']['orientation']['z']
                pose.orientation.w = goal_data['pose']['orientation']['w']
                
                # 添加到PoseArray中
                self.pose_array_msg.poses.append(pose)
                
                rospy.loginfo("Loaded goal from file: %s", yaml_file)
                
            except Exception as e:
                rospy.logerr("Error loading goal from file %s: %s", yaml_file, str(e))
        
        rospy.loginfo("Loaded %d goals in total.", len(self.pose_array_msg.poses))
    
    def publish_poses(self, event):
        # 更新时间戳
        self.pose_array_msg.header.stamp = rospy.Time.now()
        
        # 发布消息
        self.pose_publisher.publish(self.pose_array_msg)
        
        rospy.loginfo("Publishing %d static goals.", len(self.pose_array_msg.poses))

if __name__ == '__main__':
    try:
        # 创建并启动节点
        goal_publisher = StaticGoalPublisher()
        
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Static goal publisher node terminated.")