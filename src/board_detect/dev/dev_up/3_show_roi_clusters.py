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
# 新增：导入shapely库用于区域筛选
from shapely.geometry import Point, Polygon

# 该版本特性：
# 1. 使用"线拟合与偏差"聚类算法对激光点云进行聚类
# 2. 按长度筛选聚类（45-60厘米）
# 3. 按区域筛选聚类（只保留完全在矩形ROI内的聚类）


class ClusterVisualizer:
    def __init__(self):
        rospy.init_node('obstacle_filter_all', anonymous=True)
        
        self.map = None
        self.listener = tf.TransformListener()
        
        # 发布者 - 用于可视化筛选后的聚类
        self.marker_pub = rospy.Publisher('/filtered_roi_clusters', MarkerArray, queue_size=10)
        
        # 新增：发布者 - 用于可视化ROI区域
        self.roi_pub = rospy.Publisher('/roi_polygon', Marker, queue_size=1)
        
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
        
        # 新增：定义ROI多边形区域
        self.polygon = Polygon([(-0.2, 3), (2.7, 3), (2.7, 5.2), (-0.2, 5.2)])
        
        rospy.loginfo("ClusterVisualizer initialized, waiting for scan data...")
        rospy.loginfo("Filtering clusters with length between %.2fm and %.2fm", self.min_length, self.max_length)
        rospy.loginfo("ROI Polygon defined for filtering: [(-0.2, 3.0), (2.7, 3.0), (2.7, 5.2), (-0.2, 5.2)]")
        
        # 新增：在初始化后立即发布ROI多边形用于可视化
        self.publish_roi_polygon()

    def map_callback(self, msg):
        self.map = msg
    
    # 新增：发布ROI多边形可视化标记的函数
    def publish_roi_polygon(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "roi_polygon"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        
        # 设置线条属性
        marker.scale.x = 0.03  # 线宽
        marker.color.r = 0.0
        marker.color.g = 0.8   # 绿色
        marker.color.b = 0.2
        marker.color.a = 0.8   # 半透明
        
        # 添加多边形的顶点，注意要闭合
        vertices = [(-0.2, 3), (2.7, 3), (2.7, 5.2), (-0.2, 5.2), (-0.2, 3)]
        for vertex in vertices:
            p = self.create_point(vertex[0], vertex[1], 0.05)  # 稍微抬高一点，避免被地图覆盖
            marker.points.append(p)
        
        # 发布ROI多边形标记
        self.roi_pub.publish(marker)
        rospy.loginfo("Published ROI polygon visualization")
    
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
        
        # 新增：区域筛选步骤 - 只保留完全在ROI内的聚类
        final_filtered_clusters = []
        for cluster in filtered_clusters:
            is_fully_inside = True  # 假设聚类完全在ROI内
            for point_coords in cluster:
                point_geom = Point(point_coords[0], point_coords[1])
                if not self.polygon.contains(point_geom):
                    is_fully_inside = False  # 发现一个点在ROI外，整个聚类不符合
                    rospy.logdebug("Cluster point (%.2f, %.2f) outside ROI, rejecting cluster.", 
                                  point_coords[0], point_coords[1])
                    break  # 不需要检查此聚类的其他点
            
            if is_fully_inside:
                final_filtered_clusters.append(cluster)
                rospy.logdebug("Cluster PASSED ROI filter.")
        
        # 创建MarkerArray用于可视化最终筛选后的聚类
        marker_array = MarkerArray()
        marker_id = 0
        
        # 遍历所有最终筛选后的聚类并创建标记
        for i, cluster in enumerate(final_filtered_clusters):
            # 创建线条标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "filtered_roi_clusters"
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
        
        # 实时发布最终筛选后的聚类标记
        self.marker_pub.publish(marker_array)
        
        # 定期重新发布ROI多边形可视化（确保它始终可见）
        if rospy.Time.now().to_sec() % 5 < 0.1:  # 大约每5秒更新一次
            self.publish_roi_polygon()
        
        rospy.loginfo("Published %d final clusters (after length & ROI filter)", 
                     len(marker_array.markers))
    
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