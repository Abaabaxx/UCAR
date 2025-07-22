#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from __future__ import division

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf
import numpy as np
import math
from geometry_msgs.msg import Pose
from shapely.geometry import Point as ShapelyPoint, Polygon
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import os  # 用于文件和目录操作
import yaml  # 用于生成 YAML 文件
import time  # 用于可能的延时

# 该版本特性：
# 1. 使用"线拟合与偏差"聚类算法对激光点云进行聚类
# 2. 按长度筛选聚类（39-60厘米）
# 3. 按区域筛选聚类（只保留完全在矩形ROI内的聚类）
# 4. 使用与ObstacleFilter.py相同逻辑计算目标位姿并发布
# 5. 新增：连续处理10帧激光雷达数据，跟踪匹配目标点!!!!!!!!!!!!!!!!!!!!!!
# 6. 新增：计算每个目标点的平均位姿
# 7. 将计算出的最终平均位姿保存为YAML文件，适合用作ROS导航目标
# 8. 安全关闭节点，避免AttributeError错误
# target_distance 是目标航点在激光雷达前方的距离
# self.num_scans_to_average 要处理并且计算的的总帧数


class ClusterVisualizer:
    def __init__(self):
        rospy.init_node('obstacle_filter_all', anonymous=True)
        
        self.map = None
        self.listener = tf.TransformListener()
        
        # 订阅者 - 保存引用以便后续取消注册
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 保留关键算法参数
        self.max_line_deviation = 0.025  # 最大线偏差（米）
        self.min_points_for_fitting = 4   # 进行线拟合所需的最小点数
        
        # 长度筛选阈值
        self.min_length = 0.39  # 最小长度（米）
        self.max_length = 0.6  # 最大长度（米）
        
        # 定义ROI多边形区域
        self.polygon = Polygon([(-0.2, 4.1), (2.7, 4.1), (2.7, 1.7), (-0.2, 1.7)])
        
        # 目标位姿参数
        self.target_distance = 0.50  # 前方目标距离（米）
        
        # 新增：定义精确的过滤模板和阈值
        self.filter_target_x = 2.0
        self.filter_target_y = 4.75
        self.filter_target_yaw = 0.0  # 目标 Yaw (弧度)

        self.filter_pos_threshold = 0.1   # 位置距离阈值 (米)
        self.filter_angle_threshold_deg = 10.0  # 角度阈值 (度)
        self.filter_angle_threshold_rad = np.radians(self.filter_angle_threshold_deg)  # 转换为弧度
        
        # 新增：多帧处理相关参数
        self.scan_count = 0  # 当前处理的帧数
        self.num_scans_to_average = 5  # 要平均的总帧数
        self.tracked_targets = []  # 追踪中的目标列表
        
        # 新增：目标匹配阈值
        self.match_pos_threshold = 0.1  # 10 cm
        self.match_angle_threshold_deg = 10.0  # 10度
        self.match_angle_threshold_rad = np.radians(self.match_angle_threshold_deg)
        
        # 目标目录路径
        self.goals_dir = "/home/ucar/lby_ws/src/board_detect/goals_down"
        
        print("节点已初始化，将处理 {} 帧激光扫描数据...".format(self.num_scans_to_average))

    def map_callback(self, msg):
        self.map = msg
    
    def is_close(self, pose_data1, pose_data2):
        """
        判断两个位姿是否满足匹配条件：位置距离小于阈值且偏航角差异小于阈值
        """
        pos1 = pose_data1['position']
        pos2 = pose_data2['position']
        quat1 = pose_data1['orientation']
        quat2 = pose_data2['orientation']

        # 计算位置距离
        distance = np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        if distance > self.match_pos_threshold:
            return False

        # 计算角度差异
        euler1 = euler_from_quaternion(quat1)
        euler2 = euler_from_quaternion(quat2)
        yaw1 = euler1[2]
        yaw2 = euler2[2]
        
        # 计算最小角度差 (0 到 pi 之间)
        abs_diff = abs(yaw1 - yaw2)
        angle_diff = min(abs_diff, 2 * np.pi - abs_diff)
        
        return angle_diff <= self.match_angle_threshold_rad
    
    def scan_callback(self, scan):
        # 检查是否已处理足够的帧数
        if self.scan_count >= self.num_scans_to_average:
            return
        
        if self.map is None:
            return
        
        # 获取激光雷达数据在地图坐标系中的位置
        try:
            (trans, rot) = self.listener.lookupTransform('/map', scan.header.frame_id, rospy.Time(0))
            transform_matrix = self.listener.fromTranslationRotation(trans, rot)
            
            # 保存激光雷达当前位置（可选，用于检查法线方向）
            self.lidar_position = np.array([trans[0], trans[1]])
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
        
        # 区域筛选步骤 - 只保留完全在ROI内的聚类
        final_filtered_clusters = []
        for cluster in filtered_clusters:
            is_fully_inside = True  # 假设聚类完全在ROI内
            for point_coords in cluster:
                point_geom = ShapelyPoint(point_coords[0], point_coords[1])
                if not self.polygon.contains(point_geom):
                    is_fully_inside = False  # 发现一个点在ROI外，整个聚类不符合
                    break  # 不需要检查此聚类的其他点
            
            if is_fully_inside:
                final_filtered_clusters.append(cluster)
        
        # 为位姿数组创建容器
        calculated_poses = []
        
        # 遍历所有最终筛选后的聚类
        for cluster in final_filtered_clusters:
            # 计算该聚类的目标位姿，采用ObstacleFilter.py的逻辑
            if len(cluster) >= 2:
                # a. 计算聚类的起始点、终点和中心点
                start_point = np.array(cluster[0])
                end_point = np.array(cluster[-1])
                
                center_x = (start_point[0] + end_point[0]) / 2.0
                center_y = (start_point[1] + end_point[1]) / 2.0
                
                # b. 计算方向向量并归一化
                direction = end_point - start_point
                norm = np.linalg.norm(direction)
                
                # 确保方向向量不为零
                if norm < 1e-6:
                    continue
                
                norm_direction = direction / norm
                
                # c. 计算法线向量（垂直于方向向量，逆时针旋转90度）
                normal_vector = np.array([-norm_direction[1], norm_direction[0]])
                
                # 确认法线指向外侧
                # 计算从激光雷达到聚类中心的向量
                center_point = np.array([center_x, center_y])
                lidar_to_center = center_point - self.lidar_position
                
                # 如果法线与从激光雷达到中心点的向量点积为负，说明法线指向"内侧"，需要翻转
                if np.dot(normal_vector, lidar_to_center) < 0:
                    normal_vector = -normal_vector
                
                # d. 计算目标位置
                target_x = center_x - normal_vector[0] * self.target_distance
                target_y = center_y - normal_vector[1] * self.target_distance
                target_position = (target_x, target_y)
                
                # e. 计算目标姿态（指向聚类的方向）
                angle = np.arctan2(normal_vector[1], normal_vector[0])
                target_yaw = angle
                target_quaternion = quaternion_from_euler(0, 0, target_yaw)
                
                # f. 将计算的位姿添加到列表中
                pose_data = {
                    'position': target_position,
                    'orientation': target_quaternion
                }
                calculated_poses.append(pose_data)
        
        # 筛选掉位置和姿态同时接近指定值的目标位姿
        final_valid_poses = []
        for pose_data in calculated_poses:
            current_pos = pose_data['position']
            current_quat = pose_data['orientation']

            # 计算位置距离
            distance = np.sqrt((current_pos[0] - self.filter_target_x)**2 + 
                               (current_pos[1] - self.filter_target_y)**2)

            # 计算角度差异
            euler = euler_from_quaternion(current_quat)
            current_yaw = euler[2]

            # 计算与 filter_target_yaw 的最小角度差
            abs_diff = abs(current_yaw - self.filter_target_yaw)
            angle_diff = min(abs_diff, 2 * np.pi - abs_diff)

            # 判断是否需要滤除 (位置和角度都接近才滤除)
            filter_this_pose = (distance <= self.filter_pos_threshold) and (angle_diff <= self.filter_angle_threshold_rad)

            # 添加到最终列表
            if not filter_this_pose:
                final_valid_poses.append(pose_data)
        
        # 输出当前帧的结果到终端
        print("------- 帧 {}/{} -------".format(self.scan_count + 1, self.num_scans_to_average))
        if final_valid_poses:
            print("检测到 {} 个有效目标位姿".format(len(final_valid_poses)))
        else:
            print("此帧中未检测到有效的目标位姿")
        
        # 数据关联 - 匹配当前帧检测到的位姿与已追踪的目标
        if final_valid_poses:
            # 初始化匹配记录数组
            matched_current_poses = [False] * len(final_valid_poses)
            
            if self.tracked_targets:
                updated_tracked_targets = [False] * len(self.tracked_targets)
                
                # 尝试将当前帧的位姿与已追踪的目标匹配
                for idx_curr, current_pose in enumerate(final_valid_poses):
                    for idx_track, tracked_target in enumerate(self.tracked_targets):
                        # 如果这个追踪目标在当前帧还没有匹配
                        if not updated_tracked_targets[idx_track]:
                            # 检查当前位姿与追踪目标的最后位姿是否匹配
                            if self.is_close(current_pose, tracked_target['last_pose_data']):
                                # 获取当前位姿的yaw角，用于计算方向向量
                                euler = euler_from_quaternion(current_pose['orientation'])
                                current_yaw = euler[2]
                                
                                # 更新追踪目标的累积数据
                                tracked_target['sum_x'] += current_pose['position'][0]
                                tracked_target['sum_y'] += current_pose['position'][1]
                                tracked_target['sum_vec_x'] += np.cos(current_yaw)
                                tracked_target['sum_vec_y'] += np.sin(current_yaw)
                                tracked_target['count'] += 1
                                tracked_target['last_pose_data'] = current_pose
                                
                                # 标记匹配状态
                                matched_current_poses[idx_curr] = True
                                updated_tracked_targets[idx_track] = True
                                
                                print("目标 #{} 匹配成功，已追踪 {} 次".format(
                                    tracked_target['id'], tracked_target['count']))
                                
                                # 一个当前位姿只匹配一个追踪目标
                                break
            
            # 将未匹配的当前位姿添加为新的追踪目标
            for idx_curr, matched in enumerate(matched_current_poses):
                if not matched:
                    current_pose = final_valid_poses[idx_curr]
                    euler = euler_from_quaternion(current_pose['orientation'])
                    current_yaw = euler[2]
                    
                    # 创建新的追踪目标
                    new_target = {
                        'id': len(self.tracked_targets) + 1,  # 简单递增ID
                        'sum_x': current_pose['position'][0],
                        'sum_y': current_pose['position'][1],
                        'sum_vec_x': np.cos(current_yaw),
                        'sum_vec_y': np.sin(current_yaw),
                        'count': 1,
                        'last_pose_data': current_pose
                    }
                    
                    self.tracked_targets.append(new_target)
                    print("新增追踪目标 #{}，位置=({:.3f}, {:.3f}), 偏航角={:.1f}度".format(
                        new_target['id'], 
                        current_pose['position'][0], 
                        current_pose['position'][1], 
                        np.degrees(current_yaw)))
        
        # 当前帧处理完毕，增加计数
        self.scan_count += 1
        
        # 如果已收集足够帧数，进行最终处理
        if self.scan_count >= self.num_scans_to_average:
            print("\n已收集 {} 帧数据，开始计算最终目标位姿并保存...".format(self.num_scans_to_average))
            self.finalize_and_save_targets()
            self.shutdown_node()
    
    def finalize_and_save_targets(self):
        """
        计算每个追踪目标的平均位姿，并保存为YAML文件
        """
        if not self.tracked_targets:
            print("没有检测到任何追踪目标，无需保存。")
            return
        
        # 计算最终位姿
        final_poses_to_save = []
        
        print("\n----- 最终追踪结果 -----")
        for target in self.tracked_targets:
            # 计算平均位置
            avg_x = target['sum_x'] / target['count']
            avg_y = target['sum_y'] / target['count']
            
            # 计算平均方向向量，并转换为偏航角
            avg_vec_x = target['sum_vec_x'] / target['count']
            avg_vec_y = target['sum_vec_y'] / target['count']
            avg_yaw = math.atan2(avg_vec_y, avg_vec_x)
            
            # 转换为四元数
            avg_quat = quaternion_from_euler(0, 0, avg_yaw)
            
            print("目标 #{}: 平均位置=({:.3f}, {:.3f}), 平均偏航角={:.1f}度, 出现次数: {}/{}".format(
                target['id'], avg_x, avg_y, np.degrees(avg_yaw), target['count'], self.num_scans_to_average))
            
            # 构建最终位姿数据
            final_pose_data = {
                'id': target['id'],
                'position': (float(avg_x), float(avg_y)),  # 显式转换为标准Python float
                'orientation': (float(avg_quat[0]), float(avg_quat[1]), float(avg_quat[2]), float(avg_quat[3])),  # 显式转换为标准Python float
                'count': target['count']
            }
            
            final_poses_to_save.append(final_pose_data)
        
        # 按ID排序最终位姿列表
        final_poses_to_save.sort(key=lambda x: x['id'])
        
        # 检查并创建目标目录
        if not os.path.exists(self.goals_dir):
            try:
                os.makedirs(self.goals_dir)
                print("创建目标目录: {}".format(self.goals_dir))
            except OSError as e:
                print("错误：无法创建目录 {}: {}".format(self.goals_dir, e))
        
        # 清理目录中已存在的 .yaml 文件
        try:
            for filename in os.listdir(self.goals_dir):
                if filename.endswith(".yaml"):
                    file_path = os.path.join(self.goals_dir, filename)
                    try:
                        os.remove(file_path)
                    except OSError as e:
                        print("错误：无法删除文件 {}: {}".format(file_path, e))
        except OSError as e:
            print("错误：无法访问目录 {} 进行清理: {}".format(self.goals_dir, e))
        
        # 保存最终位姿为YAML文件
        print("\n开始保存目标位姿到YAML文件...")
        for i, pose_data in enumerate(final_poses_to_save):
            goal_index = i + 1  # 文件名从1开始
            pos = pose_data['position']
            quat = pose_data['orientation']
            
            # 构建符合目标格式的Python字典，确保所有数值都是标准Python float类型
            goal_dict = {
                'header': {
                    'seq': goal_index,
                    'stamp': {'secs': 0, 'nsecs': 0},
                    'frame_id': "map"
                },
                'pose': {
                    'position': {
                        'x': float(pos[0]),  # 显式转换为标准Python float
                        'y': float(pos[1]),  # 显式转换为标准Python float
                        'z': 0.0
                    },
                    'orientation': {
                        'x': float(quat[0]),  # 显式转换为标准Python float
                        'y': float(quat[1]),  # 显式转换为标准Python float
                        'z': float(quat[2]),  # 显式转换为标准Python float
                        'w': float(quat[3])   # 显式转换为标准Python float
                    }
                }
            }
            
            # 定义YAML文件名和完整路径
            yaml_filename = "goal{}.yaml".format(goal_index)
            yaml_filepath = os.path.join(self.goals_dir, yaml_filename)
            
            # 写入YAML文件
            try:
                with open(yaml_filepath, 'w') as outfile:
                    yaml.dump(goal_dict, outfile, default_flow_style=False)
                print("已保存: {}".format(yaml_filepath))
            except IOError as e:
                print("错误：无法写入文件 {}: {}".format(yaml_filepath, e))
            except yaml.YAMLError as e:
                print("错误：生成YAML时出错 {}: {}".format(yaml_filepath, e))
        
        print("所有目标位姿已保存完成。")
    
    def shutdown_node(self):
        """
        安全关闭节点，避免AttributeError
        """
        # 在关闭节点之前，先取消注册订阅者
        print("取消注册订阅者...")
        try:
            if hasattr(self, 'map_sub') and self.map_sub:
                self.map_sub.unregister()
                self.map_sub = None
            if hasattr(self, 'scan_sub') and self.scan_sub:
                self.scan_sub.unregister()
                self.scan_sub = None
        except Exception as e:
            print("警告：取消注册订阅者时出错: {}".format(e))
        
        # 添加短暂延时，确保订阅者完全取消注册
        time.sleep(0.5)
        
        print("处理完成。节点即将关闭...")
        rospy.signal_shutdown("已处理{}帧扫描并完成任务。".format(self.num_scans_to_average))
    
    # 基于线拟合与偏差的聚类函数 (移除了所有日志输出)
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
                    clusters.append(current_cluster)
                    current_cluster = [point]
        
        # 不要忘记添加最后一个聚类
        if current_cluster:
            clusters.append(current_cluster)
        
        return clusters

if __name__ == '__main__':
    try:
        cluster_visualizer = ClusterVisualizer()
        print("启动节点，等待处理激光数据...")
        # 保持节点运行直到完成处理
        rospy.spin()
    except rospy.ROSInterruptException:
        print("节点被中断。")