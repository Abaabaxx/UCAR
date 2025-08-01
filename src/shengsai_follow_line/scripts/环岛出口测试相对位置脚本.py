#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

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
    def __init__(self):
        rospy.init_node('right_board_detector', anonymous=True)
        
        # 创建激光雷达订阅者
        self.scan_sub = rospy.Subscriber(LIDAR_TOPIC, LaserScan, self.scan_callback)
        
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
                        
                        return (True, center_x_m, lateral_error_m, deviation)
                        
                elif alignment_mode == 'PARALLEL':
                    deviation = angle_deg  # 平行时，角度应接近0度
                    if deviation <= angle_tol_deg:
                        # 找到了一个平行的板子
                        center_x_m = np.mean(cluster_array[:, 0])  # 前向距离（X轴）
                        lateral_error_m = np.mean(cluster_array[:, 1])  # 横向偏差（Y轴）
                        
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
        
        if board_found:
            # 计算相对于机器人中心(base_link)的坐标
            # 激光雷达在base_link后方0.1米处，所以需要补偿
            base_link_x = board_center_x - LIDAR_X_OFFSET_M
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
                         0.0 if 'length' not in locals() else length,  # 如果length未定义，使用0.0
                         board_angle_dev)
        else:
            rospy.logwarn_throttle(1, "未检测到右侧短板")

if __name__ == '__main__':
    try:
        detector = RightBoardDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass