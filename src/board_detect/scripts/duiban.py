#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# --- 参数配置区 ---
LIDAR_TOPIC = "/scan"                   # 激光雷达话题名称
AVOIDANCE_ANGLE_DEG = 40.0              # 监控的前方角度范围（正负各20度）
AVOIDANCE_POINT_THRESHOLD = 7          # 触发避障的点数阈值
AVOIDANCE_DISTANCE_M = 0.42              # 触发停止的距离阈值 (米)
LINEAR_SPEED = 0.1                      # 机器人前进的速度 (米/秒)
CONTROL_LOOP_HZ = 30                    # 主控制循环的频率 (赫兹)

class ForwardUntilObstacleNode:
    """
    一个简单的ROS节点，控制机器人直线前进，直到前方一定距离内检测到障碍物则停止。
    """
    def __init__(self):
        """
        初始化节点、发布者、订阅者和主循环。
        """
        # 初始化核心状态标志
        self.obstacle_detected = False

        # 初始化ROS发布者和订阅者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(LIDAR_TOPIC, LaserScan, self.scan_callback)

        # 启动一个定时器来运行主控制循环
        self.main_loop_timer = rospy.Timer(rospy.Duration(1.0/CONTROL_LOOP_HZ), self.main_control_loop)
        
        rospy.loginfo("节点已启动，正在向前移动...")

    def scan_callback(self, msg):
        """
        处理激光雷达数据，检测前方障碍物。
        这段逻辑直接复用自 shengsai巡线.py。
        """
        # 如果已经检测到障碍物，就不再进行计算，节省资源
        if self.obstacle_detected:
            return

        try:
            # 计算0度（正前方）的索引
            center_index = int((0.0 - msg.angle_min) / msg.angle_increment)
            
            # 计算角度偏移对应的索引数量
            angle_rad = np.deg2rad(AVOIDANCE_ANGLE_DEG / 2.0) # 注意是半个角度
            index_offset = int(angle_rad / msg.angle_increment)
            
            # 确定扫描的起始和结束索引
            start_index = center_index - index_offset
            end_index = center_index + index_offset
            
            # 遍历指定范围内的点，统计满足条件的障碍物点数
            obstacle_points_count = 0
            for i in range(start_index, end_index):
                distance = msg.ranges[i]
                # 检查距离是否在有效且危险的范围内 (忽略0和inf)
                if 0 < distance < AVOIDANCE_DISTANCE_M:
                    obstacle_points_count += 1
            
            # 如果满足条件的点数超过阈值，则更新障碍物检测标志
            if obstacle_points_count > AVOIDANCE_POINT_THRESHOLD:
                self.obstacle_detected = True

        except Exception as e:
            rospy.logwarn_throttle(1.0, "障碍物检测出错: %s", str(e))

    def main_control_loop(self, timer_event):
        """
        主控制循环，根据障碍物检测结果决定前进或停止。
        """
        if self.obstacle_detected:
            # 如果检测到障碍物，停止机器人并关闭节点
            rospy.loginfo("检测到障碍物！停止并关闭节点...")
            self.stop()
            # 发出关闭信号，rospy.spin()会因此退出
            rospy.signal_shutdown("Obstacle detected, task finished.")
        else:
            # 如果没有障碍物，则继续前进
            twist_msg = Twist()
            twist_msg.linear.x = LINEAR_SPEED
            self.cmd_vel_pub.publish(twist_msg)

    def stop(self):
        """
        发布一个全零的速度指令，使机器人停止。
        """
        rospy.loginfo("发送停止指令...")
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('forward_until_obstacle_node', anonymous=True)
        
        # 创建并运行节点
        node = ForwardUntilObstacleNode()
        
        # 注册一个关闭钩子，确保在节点退出（如Ctrl+C）时，机器人也能停止
        rospy.on_shutdown(node.stop)
        
        # 保持节点运行，直到被外部（或内部的signal_shutdown）关闭
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("节点已关闭。")

