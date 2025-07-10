#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from shapely.geometry import Point, Polygon
import os

class ObstacleFilter:
    def __init__(self):
        rospy.init_node('obstacle_filter')
        
        self.map = None
        self.listener = tf.TransformListener()
        
        # 输出文件路径
        self.output_file_path = "/home/ucar/ucar_ws/src/xwctest/src/board.txt"
        if os.path.exists(self.output_file_path):
            os.remove(self.output_file_path) 
            with open(self.output_file_path, "w") as file:
                pass

        self.marker_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=10)
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.current_markers = {}
        self.front_markers = []  # 用于存储前方20cm的标记

        # 定义矩形区域的四个点
        self.polygon = Polygon([(-0.2, 3.5), (2.7, 3.5), (2.7, 5.2), (-0.2, 5.2)])
        
        # 新增：用于存储计算好的MarkerArray
        self.stored_marker_array = None
        
        # 新增：标志位，表示是否已完成计算
        self.computation_done = False
        
        # 新增：创建定时器，定期发布存储的标记
        self.publish_timer = rospy.Timer(rospy.Duration(0.5), self.publish_markers_loop)
        
        rospy.loginfo("ObstacleFilter initialized, waiting for scan data...")

    def map_callback(self, msg):
        self.map = msg
    
    def scan_callback(self, scan):
        # 新增：如果已经完成计算，则直接返回
        if self.computation_done:
            return
            
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
        
        # 计算每个聚类的长度并过滤
        filtered_obstacles = self.filter_obstacles_by_length(clusters, 0.4, 0.6)
        
        # 过滤在指定矩形区域内的障碍物
        filtered_obstacles_in_area = self.filter_obstacles_in_area(filtered_obstacles)
        
        # 只有当找到符合条件的障碍物时才处理
        if filtered_obstacles_in_area:
            # 修改：现在process_obstacles会返回marker_array而不是直接发布
            self.stored_marker_array = self.process_obstacles(filtered_obstacles_in_area)
            
            # 设置标志位，表示已完成计算
            self.computation_done = True
            
            rospy.loginfo("Obstacle processing completed. Switching to continuous publishing mode.")
        else:
            rospy.loginfo("No valid obstacles found in this scan. Waiting for next scan...")
        
        # 移除: 不再调用rospy.signal_shutdown()
    
    # 新增：定时器回调函数，用于持续发布存储的标记
    def publish_markers_loop(self, event):
        if self.stored_marker_array is not None:
            # 更新所有标记的时间戳
            for marker in self.stored_marker_array.markers:
                marker.header.stamp = rospy.Time.now()
            
            # 发布标记
            self.marker_pub.publish(self.stored_marker_array)
            
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
    
    def filter_obstacles_by_length(self, clusters, min_length, max_length):
        filtered_clusters = []
        
        for cluster in clusters:
            if len(cluster) < 2:
                continue
            
            start_point = np.array(cluster[0])
            end_point = np.array(cluster[-1])
            length = np.linalg.norm(end_point - start_point)
            
            if min_length <= length <= max_length:
                filtered_clusters.append(cluster)
        
        return filtered_clusters

    def filter_obstacles_in_area(self, obstacles):
        filtered_obstacles = []
        for obstacle in obstacles:
            obstacle_points = [Point(p[0], p[1]) for p in obstacle]
            if all(self.polygon.contains(point) for point in obstacle_points):
                filtered_obstacles.append(obstacle)
        return filtered_obstacles

    def process_obstacles(self, obstacles):
        # 修改：创建一个本地变量marker_array，最后返回而不是直接发布
        marker_array = MarkerArray()
        marker_id = 0
        existing_ids = set()

        for obstacle in obstacles:
            # 添加障碍物线条标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = marker_id
            existing_ids.add(marker_id)
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1  # 线条宽度
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            # 改为永久显示（不再设置lifetime）
            
            # 添加这一行 - 初始化四元数为单位四元数
            marker.pose.orientation.w = 1.0  # 设置为单位四元数 (表示无旋转)
            
            for point in obstacle:
                p = self.create_point(point[0], point[1], 0)
                marker.points.append(p)
            
            marker_array.markers.append(marker)

            # 计算障碍物中心点
            center_x = (obstacle[0][0] + obstacle[-1][0]) / 2
            center_y = (obstacle[0][1] + obstacle[-1][1]) / 2
                
            # 计算障碍物的方向向量
            direction = np.array([obstacle[-1][0] - obstacle[0][0], obstacle[-1][1] - obstacle[0][1]])
            norm_direction = direction / np.linalg.norm(direction)
            
            # 计算障碍物的法线向量（逆时针旋转90度）
            normal_vector = np.array([-norm_direction[1], norm_direction[0]])
            
            # 计算前方40cm处的坐标（沿法线方向前进40cm）
            front_x = center_x + normal_vector[0] * 0.40
            front_y = center_y + normal_vector[1] * 0.40
            
            # 计算面朝障碍物的四元数，调整为面向障碍物方向
            angle = np.arctan2(normal_vector[1], normal_vector[0])
            quaternion = quaternion_from_euler(0, 0, angle + np.pi)  # 添加 np.pi 以确保前方点面向障碍物
            
            # 打印信息
            rospy.loginfo("Front point: (%f, %f), Quaternion: (%f, %f, %f, %f)",
                        front_x, front_y, quaternion[0], quaternion[1], quaternion[2], quaternion[3])

            # 将前方位姿信息写入文件
            with open(self.output_file_path, 'a') as f:
                f.write('{},{},{},{}\n'.format(front_x, front_y, quaternion[2], quaternion[3]))

            # 创建前方点的标记（球体）
            front_marker = Marker()
            front_marker.header.frame_id = "map"
            front_marker.header.stamp = rospy.Time.now()
            front_marker.ns = "front_points"
            front_marker.id = marker_id
            front_marker.type = Marker.SPHERE
            front_marker.action = Marker.ADD
            front_marker.pose.position.x = front_x
            front_marker.pose.position.y = front_y
            front_marker.pose.position.z = 0
            front_marker.pose.orientation.x = quaternion[0]
            front_marker.pose.orientation.y = quaternion[1]
            front_marker.pose.orientation.z = quaternion[2]
            front_marker.pose.orientation.w = quaternion[3]
            front_marker.scale.x = 0.1
            front_marker.scale.y = 0.1
            front_marker.scale.z = 0.1
            front_marker.color.g = 1.0
            front_marker.color.a = 1.0
            # 改为永久显示（不再设置lifetime）
            
            marker_array.markers.append(front_marker)
            marker_id += 1
            
            # 添加法向量箭头标记
            normal_marker = Marker()
            normal_marker.header.frame_id = "map"
            normal_marker.header.stamp = rospy.Time.now()
            normal_marker.ns = "normal_vectors"
            normal_marker.id = marker_id
            normal_marker.type = Marker.ARROW  # 设置为箭头类型
            normal_marker.action = Marker.ADD
            
            # 设置箭头起点为障碍物中心点
            normal_marker.pose.position.x = center_x
            normal_marker.pose.position.y = center_y
            normal_marker.pose.position.z = 0
            
            # 设置箭头方向为法向量方向
            normal_angle = np.arctan2(normal_vector[1], normal_vector[0])
            normal_quaternion = quaternion_from_euler(0, 0, normal_angle)
            normal_marker.pose.orientation.x = normal_quaternion[0]
            normal_marker.pose.orientation.y = normal_quaternion[1]
            normal_marker.pose.orientation.z = normal_quaternion[2]
            normal_marker.pose.orientation.w = normal_quaternion[3]
            
            # 设置箭头大小
            normal_marker.scale.x = 0.3  # 箭头长度
            normal_marker.scale.y = 0.03  # 箭杆粗细
            normal_marker.scale.z = 0.05  # 箭头头部粗细
            
            # 设置箭头颜色为蓝色
            normal_marker.color.b = 1.0
            normal_marker.color.a = 1.0
            # 改为永久显示（不再设置lifetime）
            
            marker_array.markers.append(normal_marker)
            marker_id += 1
            
        # 修改：返回marker_array而不是直接发布
        return marker_array
    
    def create_point(self, x, y, z):
        p = geometry_msgs.msg.Point()
        p.x = x
        p.y = y
        p.z = z
        return p

if __name__ == '__main__':
    try:
        obstacle_filter = ObstacleFilter()
        rospy.loginfo("Starting ObstacleFilter node...")
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ObstacleFilter node terminated.")