#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg
from std_msgs.msg import ColorRGBA

# 该版本基于显示所有聚类的算法的基础上进行修改，筛选出长度在45厘米到60厘米之间的直线聚类
# 该版本针对"线拟合与偏差"聚类算法，筛选并可视化长度在45厘米到60厘米之间的直线聚类
# 原始的所有聚类（未经过长度筛选的）不再显示


class ClusterVisualizer:
    def __init__(self):
        rospy.init_node('obstacle_filter_all', anonymous=True)
        
        self.map = None
        self.listener = tf.TransformListener()
        
        # 发布者 - 更改了话题名称以反映筛选后的聚类
        self.marker_pub = rospy.Publisher('/filtered_length_clusters', MarkerArray, queue_size=10)
        
        # 订阅者
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 预定义颜色列表，用于区分不同的聚类
        self.colors = [
            self.create_color(1.0, 0.0, 0.0),  # 红色
            self.create_color(0.0, 1.0, 0.0),  # 绿色
            self.create_color(0.0, 0.0, 1.0),  # 蓝色
            self.create_color(1.0, 1.0, 0.0),  # 黄色
            self.create_color(1.0, 0.0, 1.0),  # 品红
            self.create_color(0.0, 1.0, 1.0),  # 青色
            self.create_color(1.0, 0.5, 0.0),  # 橙色
            self.create_color(0.5, 0.0, 1.0),  # 紫色
            self.create_color(0.0, 0.5, 0.0),  # 深绿色
            self.create_color(0.5, 0.5, 0.5),  # 灰色
        ]
        
        # 保留关键算法参数
        self.max_line_deviation = 0.025  # 最大线偏差（米）
        self.min_points_for_fitting = 4   # 进行线拟合所需的最小点数
        
        # 长度筛选阈值
        self.min_length = 0.45  # 最小长度（米）
        self.max_length = 0.60  # 最大长度（米）
        
        rospy.loginfo("ClusterVisualizer initialized, waiting for scan data...")
        rospy.loginfo("Filtering clusters with length between %.2fm and %.2fm", self.min_length, self.max_length)

    def map_callback(self, msg):
        self.map = msg
    
    def scan_callback(self, scan):
        if self.map is None:
            return
        
        # 获取激光雷达数据在地图坐标系中的位置
        try:
            (trans, rot) = self.listener.lookupTransform('/map', scan.header.frame_id, rospy.Time(0))
            transform_matrix = self.listener.fromTranslationRotation(trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        
        angle = scan.angle_min
        points = []
        
        for r in scan.ranges:
            if r >= scan.range_min and r <= scan.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                
                # 转换到地图坐标系
                point_in_base_link = np.array([x, y, 0, 1])
                point_in_map = np.dot(transform_matrix, point_in_base_link)
                
                map_x = point_in_map[0]
                map_y = point_in_map[1]
                
                points.append((map_x, map_y))
            angle += scan.angle_increment
        
        # 使用基于线拟合与偏差的聚类方法
        clusters = self.cluster_by_distance_and_line_deviation(points)
        
        # 筛选长度在指定范围内的聚类
        filtered_clusters = []
        
        for cluster in clusters:
            if len(cluster) >= 2:
                start_point = np.array(cluster[0])
                end_point = np.array(cluster[-1])
                length = np.linalg.norm(end_point - start_point)
                
                if self.min_length <= length <= self.max_length:
                    filtered_clusters.append(cluster)
                    rospy.logdebug("Cluster PASSED length filter: length=%.3fm", length)
                else:
                    rospy.logdebug("Cluster FAILED length filter: length=%.3fm", length)
            else:
                rospy.logdebug("Cluster too short, needs at least 2 points")
        
        # 创建MarkerArray用于可视化筛选后的聚类
        marker_array = MarkerArray()
        marker_id = 0
        
        # 遍历所有筛选后的聚类并创建标记
        for i, cluster in enumerate(filtered_clusters):
            # 创建线条标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "filtered_length_clusters"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0  # 设置为单位四元数，避免未初始化警告
            
            # 设置线条宽度
            marker.scale.x = 0.015  # 保持较细的线条宽度
            
            # 循环使用颜色列表中的颜色
            color_index = i % len(self.colors)
            color = self.colors[color_index]
            
            # 设置颜色
            marker.color.r = color.r
            marker.color.g = color.g
            marker.color.b = color.b
            marker.color.a = 1.0
            
            # 添加聚类中的所有点
            for point in cluster:
                p = self.create_point(point[0], point[1], 0)
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        # 实时发布筛选后的聚类标记
        self.marker_pub.publish(marker_array)
        
        rospy.loginfo("Published %d filtered clusters (length %.2f-%.2fm)", 
                     len(marker_array.markers), self.min_length, self.max_length)
    
    # 保留原有的基于线拟合与偏差的聚类函数
    def cluster_by_distance_and_line_deviation(self, points, proximity_threshold=0.05):
        clusters = []
        current_cluster = []
        
        for i, point in enumerate(points):
            # 转换点为numpy数组，方便计算
            point_np = np.array(point)
            
            if len(current_cluster) == 0:
                # 当前聚类为空，加入第一个点
                current_cluster.append(point)
            elif len(current_cluster) < self.min_points_for_fitting:
                # 点数不足以进行线拟合，只检查距离
                prev_point_np = np.array(current_cluster[-1])
                distance = np.linalg.norm(point_np - prev_point_np)
                
                if distance <= proximity_threshold:
                    # 距离足够近，加入当前聚类
                    current_cluster.append(point)
                else:
                    # 距离太远，开始新的聚类
                    clusters.append(current_cluster)
                    current_cluster = [point]
            else:
                # 点数足够进行线拟合
                prev_point_np = np.array(current_cluster[-1])
                
                # 先检查距离
                distance = np.linalg.norm(point_np - prev_point_np)
                
                if distance > proximity_threshold:
                    # 距离太远，开始新的聚类
                    rospy.loginfo("Splitting cluster due to distance: %.3fm > %.3fm", 
                                 distance, proximity_threshold)
                    clusters.append(current_cluster)
                    current_cluster = [point]
                    continue
                
                # 提取当前聚类中所有点的坐标
                cluster_points = np.array(current_cluster)
                x_coords = cluster_points[:, 0]
                y_coords = cluster_points[:, 1]
                
                # 检查是否为垂直线
                x_std = np.std(x_coords)
                y_std = np.std(y_coords)
                
                # 计算点到线的距离
                if x_std < 1e-6:  # 垂直线情况
                    # 垂直线：x = 常数
                    x_mean = np.mean(x_coords)
                    deviation = abs(point_np[0] - x_mean)
                    line_type = "vertical"
                elif y_std < 1e-6:  # 水平线情况
                    # 水平线：y = 常数
                    y_mean = np.mean(y_coords)
                    deviation = abs(point_np[1] - y_mean)
                    line_type = "horizontal"
                else:
                    # 使用最小二乘法拟合直线
                    # 判断哪个变量更分散来决定拟合方向
                    if x_std > y_std:
                        # 拟合 y = mx + c
                        coeffs = np.polyfit(x_coords, y_coords, 1)
                        m, c = coeffs
                        
                        # 从 y = mx + c 转换为一般式 mx - y + c = 0 --> Ax + By + C = 0
                        A, B, C = m, -1, c
                        
                        # 计算点到直线的垂直距离
                        deviation = abs(A * point_np[0] + B * point_np[1] + C) / np.sqrt(A**2 + B**2)
                        line_type = "y=f(x)"
                    else:
                        # 拟合 x = my + c
                        coeffs = np.polyfit(y_coords, x_coords, 1)
                        m, c = coeffs
                        
                        # 从 x = my + c 转换为一般式 x - my - c = 0 --> Ax + By + C = 0
                        A, B, C = 1, -m, -c
                        
                        # 计算点到直线的垂直距离
                        deviation = abs(A * point_np[0] + B * point_np[1] + C) / np.sqrt(A**2 + B**2)
                        line_type = "x=f(y)"
                
                # 比较偏差与阈值
                if deviation <= self.max_line_deviation:
                    # 偏差在允许范围内，加入当前聚类
                    current_cluster.append(point)
                else:
                    # 偏差太大，开始新的聚类
                    rospy.loginfo("Splitting cluster due to line deviation: %.3fm > %.3fm (line type: %s)", 
                                 deviation, self.max_line_deviation, line_type)
                    clusters.append(current_cluster)
                    current_cluster = [point]
        
        # 不要忘记添加最后一个聚类
        if current_cluster:
            clusters.append(current_cluster)
        
        return clusters
    
    def create_point(self, x, y, z):
        p = geometry_msgs.msg.Point()
        p.x = x
        p.y = y
        p.z = z
        return p
    
    def create_color(self, r, g, b, a=1.0):
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color

if __name__ == '__main__':
    try:
        cluster_visualizer = ClusterVisualizer()
        rospy.loginfo("Starting ClusterVisualizer node...")
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ClusterVisualizer node terminated.")