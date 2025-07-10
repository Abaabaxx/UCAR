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

#该版本专注于可视化所有聚类的线条
#新增：该版本专注于可视化所有聚类的线条（可视化的线条较细，同一聚类线条颜色不变）

class ClusterVisualizer:
    def __init__(self):
        rospy.init_node('obstacle_filter_all', anonymous=True)
        
        self.map = None
        self.listener = tf.TransformListener()
        
        # 发布者 - 更改了话题名称
        self.marker_pub = rospy.Publisher('/all_clusters_markers', MarkerArray, queue_size=10)
        
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
        
        # 新增：存储上一帧的聚类信息，包括位置和颜色
        self.previous_clusters = []
        
        # 新增：聚类匹配的距离阈值（单位：米）
        self.matching_threshold = 0.15
        
        # 新增：下一个可用的颜色索引
        self.next_color_index = 0
        
        rospy.loginfo("ClusterVisualizer initialized, waiting for scan data...")

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
        
        # 聚类点
        clusters = self.cluster_points(points)
        
        # 创建MarkerArray用于可视化所有聚类
        marker_array = MarkerArray()
        marker_id = 0
        
        # 为当前帧处理的聚类信息
        current_clusters = []
        # 记录已经匹配过的旧聚类索引
        matched_old_indices = set()
        
        # 遍历所有聚类并创建标记
        for cluster in clusters:
            if len(cluster) < 2:
                continue
                
            # 计算聚类的中心点（代表性位置）
            start_point = np.array(cluster[0])
            end_point = np.array(cluster[-1])
            center_x = (start_point[0] + end_point[0]) / 2
            center_y = (start_point[1] + end_point[1]) / 2
            center = (center_x, center_y)
            
            # 查找最近的旧聚类
            best_match_idx = -1
            min_distance = float('inf')
            
            for idx, old_cluster in enumerate(self.previous_clusters):
                if idx in matched_old_indices:
                    continue  # 跳过已经被匹配过的旧聚类
                
                old_center = old_cluster['center']
                distance = np.sqrt((center[0] - old_center[0])**2 + (center[1] - old_center[1])**2)
                
                if distance < min_distance and distance < self.matching_threshold:
                    min_distance = distance
                    best_match_idx = idx
            
            # 决定分配什么颜色
            if best_match_idx >= 0:
                # 匹配成功，使用旧聚类的颜色
                color = self.previous_clusters[best_match_idx]['color']
                matched_old_indices.add(best_match_idx)
                rospy.logdebug("Cluster matched with old cluster at index %d, distance: %.3fm", best_match_idx, min_distance)
            else:
                # 没有匹配，分配新颜色
                color = self.colors[self.next_color_index % len(self.colors)]
                self.next_color_index += 1
                rospy.logdebug("New cluster detected, assigned new color")
            
            # 存储当前聚类信息
            current_clusters.append({
                'center': center,
                'color': color
            })
                
            # 创建线条标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "all_clusters"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0  # 设置为单位四元数，避免未初始化警告
            
            # 修改：设置更细的线条 (0.01 - 0.02 米)
            marker.scale.x = 0.015  # 更细的线条宽度
            
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
        
        # 实时发布所有聚类的标记
        self.marker_pub.publish(marker_array)
        
        # 更新存储的聚类信息
        self.previous_clusters = current_clusters
        
        rospy.loginfo("Published %d clusters", len(marker_array.markers))
            
    def cluster_points(self, points, threshold=0.05):
        clusters = []
        current_cluster = []
        
        for i, point in enumerate(points):
            if not current_cluster:
                current_cluster.append(point)
            else:
                distance = np.linalg.norm(np.array(point) - np.array(current_cluster[-1]))
                if distance <= threshold:
                    current_cluster.append(point)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [point]
        
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