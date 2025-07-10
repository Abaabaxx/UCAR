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
        self.output_file_path = "/home/ucar/lby_ws/src/board_detect/board_goal/board_goal.txt"
        if os.path.exists(self.output_file_path):
            os.remove(self.output_file_path) 
            with open(self.output_file_path, "w") as file:
                pass

        self.marker_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=10)
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.current_markers = {}
        self.front_markers = []  # 用于存储前方20cm的标记

        self.polygon = Polygon([(-0.2, 3.5), (2.7, 3.5), (2.7, 5.2), (-0.2, 5.2)])





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
        
        # 计算每个聚类的长度并过滤
        filtered_obstacles = self.filter_obstacles_by_length(clusters, 0.4, 0.7)
        
        # 过滤在指定矩形区域内的障碍物
        filtered_obstacles_in_area = self.filter_obstacles_in_area(filtered_obstacles)
        
        # 处理过滤后的障碍物
        self.process_obstacles(filtered_obstacles_in_area)
        rospy.signal_shutdown("Processed first scan, shutting down.")

    
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
        
        marker_array = MarkerArray()
        marker_id = 0
        existing_ids = set()

        for obstacle in obstacles:
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
            marker.lifetime = rospy.Duration(0.5)  # 标记持续时间
            
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
            
            # 计算前方40cm处的坐标（沿法线方向前进45cm）
            front_x = center_x + normal_vector[0] * 0.40
            front_y = center_y + normal_vector[1] * 0.40
            
            # 计算面朝障碍物的四元数，调整为面向障碍物方向
            angle = np.arctan2(normal_vector[1], normal_vector[0])
            quaternion = quaternion_from_euler(0, 0, angle + np.pi)  # 添加 np.pi 以确保前方点面向障碍物
            
            # 打印信息
            rospy.loginfo("Front point: (%f, %f), Quaternion: (%f, %f, %f, %f)",front_x,front_y,quaternion[0],quaternion[1],quaternion[2],quaternion[3])

            # 将前方20cm处的点和四元数写入文件
            with open(self.output_file_path, 'a') as f:
                # f.write(f'{front_x},{front_y},{quaternion[2]},{quaternion[3]}\n')
                f.write('{},{},{},{}\n'.format(front_x, front_y, quaternion[2], quaternion[3]))

            # 创建前方20cm处点的标记
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
            front_marker.lifetime = rospy.Duration(0.5)  # 标记持续时间
            
            marker_array.markers.append(front_marker)

            # 更新标记ID
            marker_id += 1
        
        # 添加删除旧标记的操作
        for marker_id in list(self.current_markers.keys()):
            if marker_id not in existing_ids:
                delete_marker = Marker()
                delete_marker.header.frame_id = "map"
                delete_marker.header.stamp = rospy.Time.now()
                delete_marker.ns = "obstacles"
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)
                del self.current_markers[marker_id]

        self.current_markers = {marker.id: marker for marker in marker_array.markers if marker.action != Marker.DELETE}
        
        self.marker_pub.publish(marker_array)
    
    def create_point(self, x, y, z):
        p = geometry_msgs.msg.Point()
        p.x = x
        p.y = y
        p.z = z
        return p

if __name__ == '__main__':
    ObstacleFilter()
    rospy.spin()
