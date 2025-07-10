#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 该版本接收激光雷达扫描数据和地图数据，去除在RVIZ中可视化ROI区域的相关代码（ROI区域的设置由另一版本的代码进行测试）

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg
from shapely.geometry import Point, Polygon

class RadarProcessor:
    def __init__(self):
        rospy.init_node('radar_processor')
        
        # 存储地图数据
        self.map = None
        
        # TF监听器用于坐标转换
        self.listener = tf.TransformListener()
        
        # 用于可视化的发布器
        self.marker_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=10)
        
        # 订阅地图和激光扫描数据
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 定义ROI区域（预设的地图区域）
        # 上半平面
        self.roi_polygon = Polygon([(-0.2, 3.5), (2.7, 3.5), (2.7, 5.2), (-0.2, 5.2)])
        
        # 用于存储当前可视化的标记
        self.current_markers = {}
        
        rospy.loginfo("RadarProcessor初始化完成，等待激光雷达数据...")

    def map_callback(self, msg):
        """接收并存储地图数据"""
        self.map = msg
        rospy.loginfo("地图数据已接收")
    
    def scan_callback(self, scan):
        """处理激光雷达扫描数据"""
        if self.map is None:
            rospy.logwarn("尚未接收到地图数据，跳过处理")
            return
        
        # 具体处理逻辑将在这里实现
        # ...

if __name__ == '__main__':
    try:
        processor = RadarProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass