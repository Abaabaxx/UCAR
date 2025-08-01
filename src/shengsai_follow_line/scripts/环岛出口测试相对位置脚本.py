#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# ==============================================================================
# 状态四: DRIVE_TO_CENTER (直行到入口板中心) - 参数
# ==============================================================================
# --- 检测参数 (右侧短板) ---
DRIVE_TO_CENTER_TARGET_ANGLE_DEG = -90.0  # 扫描中心: 右侧 (-90度)
DRIVE_TO_CENTER_SCAN_RANGE_DEG = 90.0     # 扫描范围: 中心±45度
DRIVE_TO_CENTER_MIN_DIST_M = 0.2          # 最小检测距离
DRIVE_TO_CENTER_MAX_DIST_M = 1.5          # 最大检测距离
DRIVE_TO_CENTER_MIN_LENGTH_M = 0.4        # 短板最小长度 (米)
DRIVE_TO_CENTER_MAX_LENGTH_M = 0.6        # 短板最大长度 (米)

# ==============================================================================
# 全局激光雷达参数 (适用于所有状态)
# ==============================================================================
LIDAR_TOPIC = "/scan"                   # 激光雷达话题名称
BOARD_DETECT_CLUSTER_TOL_M = 0.05       # 聚类时，点与点之间的最大距离
BOARD_DETECT_MIN_CLUSTER_PTS = 5        # 一个有效聚类最少的点数

# ==============================================================================
# 机器人物理参数
# ==============================================================================
# 根据 `rosrun tf tf_echo base_link laser_frame` 的输出,
# 激光雷达安装在机器人旋转中心(base_link)后方0.1米处。
LIDAR_X_OFFSET_M = -0.1

# 观察角度容忍度
OBSERVATION_ANGLE_TOL_DEG = 20.0  # 移动跟踪时宽容的角度阈值 (度)

class RightBoardDetector:
    def _visualize_board_markers(self, scan_msg, cluster_array, center_x_m, lateral_error_m, coeffs, x_std, y_std, debug_marker_array):
        """
        可视化板子的中心点和法向量
        """
        # 1. 可视化中心点 (一个黄色的球体)
        center_marker = Marker()
        center_marker.header.frame_id = scan_msg.header.frame_id
        center_marker.header.stamp = rospy.Time.now()
        center_marker.ns = "debug_info_ns"
        center_marker.id = 100 # 使用一个较大的ID，避免与聚类点冲突
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        
        center_marker.pose.position.x = center_x_m
        center_marker.pose.position.y = lateral_error_m
        center_marker.pose.position.z = 0
        center_marker.pose.orientation.w = 1.0
        
        center_marker.scale.x = 0.1
        center_marker.scale.y = 0.1
        center_marker.scale.z = 0.1
        
        center_marker.color.a = 1.0
        center_marker.color.r = 1.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0 # 黄色
        
        center_marker.lifetime = rospy.Duration(0.5)
        debug_marker_array.markers.append(center_marker)

        # 2. 可视化法向量 (一个从中心点出发的紫色箭头)
        normal_marker = Marker()
        normal_marker.header.frame_id = scan_msg.header.frame_id
        normal_marker.header.stamp = rospy.Time.now()
        normal_marker.ns = "debug_info_ns"
        normal_marker.id = 101
        normal_marker.type = Marker.ARROW
        normal_marker.action = Marker.ADD

        # 箭头的起点是聚类的中心
        start_p = Point(x=center_x_m, y=lateral_error_m, z=0)

        # 箭头的终点代表法向量方向
        end_p = Point()
        
        # 根据拟合方向计算基础法向量
        if coeffs is not None:
            if x_std > y_std: # 拟合 y = mx + c
                slope = coeffs[0]
                # 法向量方向 (-slope, 1)
                normal_vector = np.array([-slope, 1.0])
            else: # 拟合 x = my + c
                slope = coeffs[0]
                # 法向量方向 (1, -slope)
                normal_vector = np.array([1.0, -slope])

            # 检查并确保法线指向外侧 (远离雷达原点)
            lidar_to_center = np.array([center_x_m, lateral_error_m])
            if np.dot(normal_vector, lidar_to_center) < 0:
                normal_vector = -normal_vector # 翻转法线

            # 归一化并设置箭头终点
            norm = np.linalg.norm(normal_vector)
            if norm > 1e-6:
                unit_normal = normal_vector / norm
                end_p.x = center_x_m + unit_normal[0] * 0.5 # 箭头长度0.5米
                end_p.y = lateral_error_m + unit_normal[1] * 0.5
                end_p.z = 0
                
                normal_marker.points.append(start_p)
                normal_marker.points.append(end_p)
        
        normal_marker.scale.x = 0.02 # 箭杆直径
        normal_marker.scale.y = 0.04 # 箭头宽度
        
        normal_marker.color.a = 1.0
        normal_marker.color.r = 1.0
        normal_marker.color.g = 0.0
        normal_marker.color.b = 1.0 # 紫色

        normal_marker.lifetime = rospy.Duration(0.5)
        debug_marker_array.markers.append(normal_marker)
        
        # 发布调试标记
        self.debug_markers_pub.publish(debug_marker_array)

    def __init__(self):
        rospy.init_node('right_board_detector', anonymous=True)
        
        # 创建激光雷达订阅者
        self.scan_sub = rospy.Subscriber(LIDAR_TOPIC, LaserScan, self.scan_callback)
        
        # 创建RViz可视化发布者
        self.clusters_pub = rospy.Publisher('/line_follower/lidar_clusters', MarkerArray, queue_size=10)
        self.debug_markers_pub = rospy.Publisher('/line_follower/debug_markers', MarkerArray, queue_size=10)
        
        rospy.loginfo("右侧板子距离测试程序已启动，等待激光雷达数据...")
        
    def _find_board(self, scan_msg, target_angle_deg, scan_range_deg, alignment_mode, 
                    min_dist_m=0.25, max_dist_m=1.5, min_length_m=0.45, max_length_m=0.62, 
                    angle_tol_deg=9.0):
        """
        通用的板子检测函数，可以在任意方向寻找平行或垂直的板子
        
        参数:
        scan_msg: 激光雷达数据
        target_angle_deg: 目标扫描的中心角度（度）。0表示正前方，-90表示正右方，90表示正左方
        scan_range_deg: 扫描的角度范围（度）。例如60表示在中心角度的±30度范围内扫描
        alignment_mode: 对齐模式，可以是'PERPENDICULAR'（垂直）或'PARALLEL'（平行）
        min_dist_m: 考虑的最小距离 (米)
        max_dist_m: 考虑的最大距离 (米)
        min_length_m: 聚类的最小长度 (米)
        max_length_m: 聚类的最大长度 (米)
        angle_tol_deg: 角度容忍度
        
        返回:
        tuple: (是否找到符合条件的板子, 中心点X坐标, 中心点Y坐标, 角度偏差)
        """
        try:
            # 初始化调试MarkerArray
            debug_marker_array = MarkerArray()
            # 添加一个DELETEALL标记，以清除上一帧的调试标记
            clear_marker = Marker()
            clear_marker.id = 0
            clear_marker.ns = "debug_info_ns"
            clear_marker.action = Marker.DELETEALL
            debug_marker_array.markers.append(clear_marker)
            
            # 1. 数据筛选：只考虑指定角度和距离范围内的点
            center_angle_rad = np.deg2rad(target_angle_deg)
            scan_half_range_rad = np.deg2rad(scan_range_deg / 2.0)
            
            # 计算角度索引范围
            center_index = int((center_angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
            angle_index_range = int(scan_half_range_rad / scan_msg.angle_increment)
            start_index = max(0, center_index - angle_index_range)
            end_index = min(len(scan_msg.ranges), center_index + angle_index_range)
            
            # 提取有效点的坐标
            points = []
            for i in range(start_index, end_index):
                distance = scan_msg.ranges[i]
                if min_dist_m <= distance <= max_dist_m:
                    angle = scan_msg.angle_min + i * scan_msg.angle_increment
                    x = distance * np.cos(angle)
                    y = distance * np.sin(angle)
                    points.append((x, y))
            
            if len(points) < BOARD_DETECT_MIN_CLUSTER_PTS:
                return (False, 0.0, 0.0, 999.0)
            
            # 2. 简单距离聚类
            clusters = []
            current_cluster = []
            
            for i, point in enumerate(points):
                if len(current_cluster) == 0:
                    current_cluster.append(point)
                else:
                    # 计算与前一个点的距离
                    prev_point = current_cluster[-1]
                    distance = np.sqrt((point[0] - prev_point[0])**2 + (point[1] - prev_point[1])**2)
                    
                    if distance <= BOARD_DETECT_CLUSTER_TOL_M:
                        current_cluster.append(point)
                    else:
                        # 距离太远，开始新聚类
                        if len(current_cluster) >= BOARD_DETECT_MIN_CLUSTER_PTS:
                            clusters.append(current_cluster)
                        current_cluster = [point]
            
            # 不要忘记最后一个聚类
            if len(current_cluster) >= BOARD_DETECT_MIN_CLUSTER_PTS:
                clusters.append(current_cluster)
            
            # --- [开始] 可视化所有找到的聚类 ---
            marker_array = MarkerArray()

            # 1. 创建一个特殊的Marker用于清除上一帧的所有标记
            clear_marker = Marker()
            clear_marker.id = 0
            clear_marker.ns = "lidar_clusters_ns" # 使用一个命名空间
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)

            # 2. 遍历所有找到的聚类，并为每一个都创建一个可视化标记
            for i, cluster in enumerate(clusters):
                marker = Marker()
                marker.header.frame_id = scan_msg.header.frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = "lidar_clusters_ns"
                marker.id = i + 1 # ID 0 已被DELETEALL使用
                marker.type = Marker.POINTS  # 将每个聚类显示为一组点
                marker.action = Marker.ADD

                marker.pose.orientation.w = 1.0
                
                # 设置点的大小
                marker.scale.x = 0.03
                marker.scale.y = 0.03

                # 根据聚类的索引号赋予不同颜色（红/绿交替）
                marker.color.a = 1.0  # 不透明
                marker.color.r = float(i % 2 == 0)
                marker.color.g = float(i % 2 != 0)
                marker.color.b = 0.0

                marker.lifetime = rospy.Duration(0.5)

                # 将聚类中的所有点添加到marker消息中
                for x, y in cluster:
                    p = Point(x=x, y=y, z=0)
                    marker.points.append(p)
                
                marker_array.markers.append(marker)
            
            # 3. 在所有marker都准备好后，只发布一次MarkerArray
            self.clusters_pub.publish(marker_array)
            # --- [结束] 可视化代码 ---
            
            # 3. 聚类验证和角度检测
            for cluster in clusters:
                if len(cluster) < BOARD_DETECT_MIN_CLUSTER_PTS:
                    continue
                
                # 计算聚类长度
                start_point = np.array(cluster[0])
                end_point = np.array(cluster[-1])
                length = np.linalg.norm(end_point - start_point)
                
                if not (min_length_m <= length <= max_length_m):
                    continue
                
                # 线性拟合并计算角度
                cluster_array = np.array(cluster)
                x_coords = cluster_array[:, 0]
                y_coords = cluster_array[:, 1]
                
                # 判断拟合方向
                x_std = np.std(x_coords)
                y_std = np.std(y_coords)
                
                if x_std < 1e-6:  # 垂直线
                    angle_deg = 90.0
                elif y_std < 1e-6:  # 水平线
                    angle_deg = 0.0
                else:
                    if x_std > y_std:
                        # 拟合 y = mx + c
                        coeffs = np.polyfit(x_coords, y_coords, 1)
                        slope = coeffs[0]
                        angle_rad = np.arctan(slope)
                    else:
                        # 拟合 x = my + c
                        coeffs = np.polyfit(y_coords, x_coords, 1)
                        slope = coeffs[0]
                        angle_rad = np.arctan(1.0 / slope) if slope != 0 else np.pi/2
                    
                    angle_deg = abs(np.rad2deg(angle_rad))
                
                # 根据对齐模式进行判断
                if alignment_mode == 'PERPENDICULAR':
                    deviation = abs(angle_deg - 90)
                    if deviation <= angle_tol_deg:
                        # 找到了一个垂直的板子
                        center_x_m = np.mean(cluster_array[:, 0])  # 前向距离（X轴）
                        lateral_error_m = np.mean(cluster_array[:, 1])  # 横向偏差（Y轴）
                        
                        # 可视化中心点和法向量
                        self._visualize_board_markers(scan_msg, cluster_array, center_x_m, lateral_error_m, 
                                                    coeffs if 'coeffs' in locals() else None, 
                                                    x_std, y_std, debug_marker_array)
                        
                        return (True, center_x_m, lateral_error_m, deviation)
                        
                elif alignment_mode == 'PARALLEL':
                    deviation = angle_deg  # 平行时，角度应接近0度
                    if deviation <= angle_tol_deg:
                        # 找到了一个平行的板子
                        center_x_m = np.mean(cluster_array[:, 0])  # 前向距离（X轴）
                        lateral_error_m = np.mean(cluster_array[:, 1])  # 横向偏差（Y轴）
                        
                        # 可视化中心点和法向量
                        self._visualize_board_markers(scan_msg, cluster_array, center_x_m, lateral_error_m, 
                                                    coeffs if 'coeffs' in locals() else None, 
                                                    x_std, y_std, debug_marker_array)
                        
                        return (True, center_x_m, lateral_error_m, deviation)
            
            return (False, 0.0, 0.0, 999.0)
            
        except Exception as e:
            rospy.logwarn_throttle(5, "板子检测出错: %s", str(e))
            return (False, 0.0, 0.0, 999.0)
    
    def scan_callback(self, msg):
        """
        处理激光雷达数据，检测右侧短板
        """
        # 右侧短板检测
        board_found, board_center_x, board_center_y, board_angle_dev = self._find_board(
            msg,
            DRIVE_TO_CENTER_TARGET_ANGLE_DEG,
            DRIVE_TO_CENTER_SCAN_RANGE_DEG,
            'PARALLEL',
            DRIVE_TO_CENTER_MIN_DIST_M,
            DRIVE_TO_CENTER_MAX_DIST_M,
            DRIVE_TO_CENTER_MIN_LENGTH_M,
            DRIVE_TO_CENTER_MAX_LENGTH_M,
            OBSERVATION_ANGLE_TOL_DEG  # 使用宽容阈值(20°)进行目标跟踪
        )
        
        # 计算板子长度（如果找到了板子）
        length = 0.0  # 默认长度为0
        
        if board_found:
            # 计算相对于机器人中心(base_link)的坐标
            # 激光雷达在base_link后方0.1米处，所以需要补偿
            # 正确的计算: (激光雷达的读数) + (激光雷达的位置) = 板子相对于机器人中心的位置
            base_link_x = board_center_x + LIDAR_X_OFFSET_M  # 加上偏移量（因为LIDAR_X_OFFSET_M已经是负值）
            base_link_y = board_center_y  # Y方向无需补偿
            
            rospy.loginfo("\n"
                         "============================================================\n"
                         "检测到右侧短板:\n"
                         "  - 相对于激光雷达(laser_frame)的坐标: (x=%.3f, y=%.3f)m\n"
                         "  - 相对于机器人中心(base_link)的坐标: (x=%.3f, y=%.3f)m\n"
                         "  - 板子长度: %.3fm\n"
                         "  - 角度: %.1f度\n"
                         "============================================================",
                         board_center_x, board_center_y,
                         base_link_x, base_link_y,
                         length,  # 使用当前聚类的长度
                         board_angle_dev)
        else:
            rospy.logwarn_throttle(1, "未检测到右侧短板")

if __name__ == '__main__':
    try:
        detector = RightBoardDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass