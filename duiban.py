#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse


"""
rosservice call /board_aligner_node/start_alignment "data: true"

rosservice call /board_aligner_node/start_alignment "data: false"
"""

class BoardAligner:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('board_aligner_node')
        
        # ==============================================================================
        # --- 可调参数区 ---
        # ==============================================================================
        
        # --- 1. 目标板子物理属性 ---
        self.MIN_LENGTH_M = 0.37       # 板子最小长度 (米)
        self.MAX_LENGTH_M = 0.60       # 板子最大长度 (米)
        
        # --- 2. 最终停靠目标 ---
        self.TARGET_DISTANCE_M = 0.32  # 最终与板子的目标距离 (米)
        
        # --- 3. 激光雷达检测参数 ---
        self.DETECTION_SCAN_RANGE_DEG = 120.0  # 扫描范围: 正前方中心±(此值/2)度
        self.DETECTION_MIN_DIST_M = 0.2        # 检测板子的最小距离
        self.DETECTION_MAX_DIST_M = 0.8        # 检测板子的最大距离
        self.DETECTION_ANGLE_TOL_DEG = 30.0    # **观察容差**: 检测时，板子角度偏离90度的最大容忍值
        
        # --- 4. 控制参数 ---
        self.ROTATION_SPEED_DEG = 10.0     # 对准时的旋转速度 (度/秒)
        self.LINEAR_SPEED_M_S = 0.1        # 前后移动速度 (米/秒)
        self.LATERAL_SPEED_M_S = 0.1       # 横向平移速度 (米/秒)
        
        # --- 5. 完成任务的精度容差 ---
        self.ALIGN_ANGLE_TOL_DEG = 3.0     # **对准容差**: 角度对准完成的误差容忍值 (度)
        self.ALIGN_LATERAL_TOL_M = 0.04    # 横向对准完成的误差容忍值 (米)
        self.ALIGN_DISTANCE_TOL_M = 0.05   # 最终距离完成的误差容忍值 (米)

        # --- 6. 机器人与传感器物理参数 ---
        self.LIDAR_X_OFFSET_M = -0.1         # 激光雷达在机器人中心后方的偏移量
        self.BOARD_DETECT_CLUSTER_TOL_M = 0.05 # 聚类时，点与点之间的最大距离
        self.BOARD_DETECT_MIN_CLUSTER_PTS = 5  # 一个有效聚类最少的点数
        
        # ==============================================================================
        # --- 内部变量与ROS设置 ---
        # ==============================================================================
        
        # 将角度从度转换为弧度
        self.ROTATION_SPEED_RAD = np.deg2rad(self.ROTATION_SPEED_DEG)
        
        # 状态定义
        self.STATE_IDLE = 0                # 空闲状态
        self.STATE_ALIGNING_YAW = 1        # 正对板子
        self.STATE_ALIGNING_LATERAL = 2    # 平移到中心
        self.STATE_ALIGNING_LONGITUDINAL = 3  # 前后移动到目标距离
        self.STATE_DONE = 4                # 任务完成
        
        # 初始化状态
        self.current_state = self.STATE_IDLE
        
        # 板子检测结果
        self.is_board_found = False
        self.board_center_x_m = 0.0
        self.board_lateral_error_m = 0.0
        self.board_angle_error_deg = 0.0
        
        # 创建ROS接口
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.align_service = rospy.Service('~start_alignment', SetBool, self.handle_start_alignment)
        
        # 创建控制循环定时器 (20Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        
        rospy.loginfo("板子对准节点已启动，等待服务调用...")
    
    def handle_start_alignment(self, req):
        """处理服务调用，启动或停止对准任务"""
        if req.data:
            if self.current_state == self.STATE_IDLE:
                self.current_state = self.STATE_ALIGNING_YAW
                rospy.loginfo("开始板子对准任务")
                return SetBoolResponse(True, "对准任务已启动")
            else:
                return SetBoolResponse(False, "任务已在进行中")
        else:
            # 停止任务并重置
            self.stop_robot()
            self.current_state = self.STATE_IDLE
            rospy.loginfo("对准任务已重置")
            return SetBoolResponse(True, "对准任务已重置")
    
    def stop_robot(self):
        """发送停止指令"""
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)
    
    def scan_callback(self, scan_msg):
        """处理激光雷达数据，检测前方的板子"""
        if self.current_state == self.STATE_IDLE:
            return
        
        # 检测前方的垂直板子
        self.is_board_found, self.board_center_x_m, self.board_lateral_error_m, self.board_angle_error_deg = \
            self._find_obstacle_board_with_signed_angle(
                scan_msg,
                target_angle_deg=0.0,        # 扫描中心: 正前方
                scan_range_deg=self.DETECTION_SCAN_RANGE_DEG,
                alignment_mode='PERPENDICULAR',
                min_dist_m=self.DETECTION_MIN_DIST_M,
                max_dist_m=self.DETECTION_MAX_DIST_M,
                min_length_m=self.MIN_LENGTH_M,
                max_length_m=self.MAX_LENGTH_M,
                angle_tol_deg=self.DETECTION_ANGLE_TOL_DEG
            )
    
    def control_loop(self, event):
        """控制循环，根据当前状态执行相应动作"""
        if self.current_state == self.STATE_IDLE:
            return
        
        twist_msg = Twist()
        
        if self.current_state == self.STATE_ALIGNING_YAW:
            # 第一步：正对板子
            if not self.is_board_found:
                # 未找到板子，原地旋转搜索
                rospy.loginfo_throttle(1, "正在搜索板子...")
                twist_msg.angular.z = self.ROTATION_SPEED_RAD
            else:
                # 找到板子，根据角度误差旋转
                if abs(self.board_angle_error_deg) <= self.ALIGN_ANGLE_TOL_DEG:
                    # 角度已对准，进入下一阶段
                    rospy.loginfo("板子角度已对准 (误差: %.2f度)，开始横向对准", self.board_angle_error_deg)
                    self.current_state = self.STATE_ALIGNING_LATERAL
                    self.stop_robot()  # 停止当前动作，确保平稳过渡
                    return
                else:
                    # 根据误差方向旋转
                    twist_msg.angular.z = np.sign(self.board_angle_error_deg) * self.ROTATION_SPEED_RAD
                    rospy.loginfo_throttle(1, "正在对准板子角度，当前误差: %.2f度", self.board_angle_error_deg)
        
        elif self.current_state == self.STATE_ALIGNING_LATERAL:
            # 第二步：平移到中心
            if not self.is_board_found:
                # 未找到板子，停止并等待
                rospy.loginfo_throttle(1, "横向对准过程中丢失板子，停止等待...")
                self.stop_robot()
                return
            
            # 计算横向误差
            if abs(self.board_lateral_error_m) <= self.ALIGN_LATERAL_TOL_M:
                # 横向位置已对准，进入下一阶段
                rospy.loginfo("横向位置已对准 (误差: %.2fm)，开始调整距离", self.board_lateral_error_m)
                self.current_state = self.STATE_ALIGNING_LONGITUDINAL
                self.stop_robot()  # 停止当前动作，确保平稳过渡
                return
            else:
                # 根据横向误差平移
                twist_msg.linear.y = np.sign(self.board_lateral_error_m) * self.LATERAL_SPEED_M_S
                rospy.loginfo_throttle(1, "正在横向对准，当前误差: %.2fm", self.board_lateral_error_m)
        
        elif self.current_state == self.STATE_ALIGNING_LONGITUDINAL:
            # 第三步：前后移动到目标距离
            if not self.is_board_found:
                # 未找到板子，停止并等待
                rospy.loginfo_throttle(1, "距离调整过程中丢失板子，停止等待...")
                self.stop_robot()
                return
            
            # 计算距离误差 (考虑激光雷达的安装偏移)
            actual_distance = self.board_center_x_m + self.LIDAR_X_OFFSET_M
            distance_error = actual_distance - self.TARGET_DISTANCE_M
            
            if abs(distance_error) <= self.ALIGN_DISTANCE_TOL_M:
                # 距离已达标，任务完成
                rospy.loginfo("距离已调整到目标值 (%.2fm)，任务完成", actual_distance)
                self.current_state = self.STATE_DONE
                self.stop_robot()
                return
            else:
                # 根据距离误差前进或后退
                twist_msg.linear.x = np.sign(distance_error) * self.LINEAR_SPEED_M_S
                rospy.loginfo_throttle(1, "正在调整距离，当前: %.2fm，目标: %.2fm，误差: %.2fm", 
                                      actual_distance, self.TARGET_DISTANCE_M, distance_error)
        
        elif self.current_state == self.STATE_DONE:
            # 任务完成，保持停止状态
            self.stop_robot()
            return
        
        # 发布速度指令
        self.cmd_vel_pub.publish(twist_msg)
    
    def _find_obstacle_board_with_signed_angle(self, scan_msg, target_angle_deg, scan_range_deg, alignment_mode, 
                                      min_dist_m=0.25, max_dist_m=1.5, min_length_m=0.45, max_length_m=0.62, 
                                      angle_tol_deg=9.0):
        """
        检测指定方向的板子，并返回带符号的角度偏差
        
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
            
            if len(points) < self.BOARD_DETECT_MIN_CLUSTER_PTS:
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
                    
                    if distance <= self.BOARD_DETECT_CLUSTER_TOL_M:
                        current_cluster.append(point)
                    else:
                        # 距离太远，开始新聚类
                        if len(current_cluster) >= self.BOARD_DETECT_MIN_CLUSTER_PTS:
                            clusters.append(current_cluster)
                        current_cluster = [point]
            
            # 不要忘记最后一个聚类
            if len(current_cluster) >= self.BOARD_DETECT_MIN_CLUSTER_PTS:
                clusters.append(current_cluster)
            
            # 3. 聚类验证和角度检测
            for cluster in clusters:
                if len(cluster) < self.BOARD_DETECT_MIN_CLUSTER_PTS:
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
                    
                    angle_deg = np.rad2deg(angle_rad)  # 不取绝对值，保留符号
                
                # 根据对齐模式进行判断
                if alignment_mode == 'PERPENDICULAR':
                    # 计算带符号的偏差
                    if angle_deg > 0:  # 板子向左倾斜
                        deviation = angle_deg - 90
                    else:  # 板子向右倾斜
                        deviation = angle_deg + 90
                        
                    if abs(deviation) <= angle_tol_deg:
                        # 找到了一个垂直的板子
                        center_x_m = np.mean(cluster_array[:, 0])  # 前向距离（X轴）
                        lateral_error_m = np.mean(cluster_array[:, 1])  # 横向偏差（Y轴）
                        
                        # 为日志记录计算base_link坐标
                        center_x_base_link = center_x_m + self.LIDAR_X_OFFSET_M
                        rospy.loginfo_throttle(2, "检测到垂直板子: 中心点(机器人坐标系 x=%.2f, y=%.2f)m, 长度=%.2fm, 角度偏差=%.1f度", 
                                            center_x_base_link, lateral_error_m, length, deviation)
                        return (True, center_x_m, lateral_error_m, deviation)
                        
                elif alignment_mode == 'PARALLEL':
                    deviation = angle_deg  # 平行时，角度应接近0度，保留符号
                    if abs(deviation) <= angle_tol_deg:
                        # 找到了一个平行的板子
                        center_x_m = np.mean(cluster_array[:, 0])  # 前向距离（X轴）
                        lateral_error_m = np.mean(cluster_array[:, 1])  # 横向偏差（Y轴）
                        
                        # 为日志记录计算base_link坐标
                        center_x_base_link = center_x_m + self.LIDAR_X_OFFSET_M
                        rospy.loginfo_throttle(2, "检测到平行板子: 中心点(机器人坐标系 x=%.2f, y=%.2f)m, 长度=%.2fm, 角度=%.1f度", 
                                            center_x_base_link, lateral_error_m, length, angle_deg)
                        return (True, center_x_m, lateral_error_m, deviation)
            
            return (False, 0.0, 0.0, 999.0)
            
        except Exception as e:
            rospy.logwarn_throttle(5, "板子检测出错: %s", str(e))
            return (False, 0.0, 0.0, 999.0)

if __name__ == '__main__':
    try:
        # 创建并运行节点
        node = BoardAligner()
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("板子对准节点已关闭。")
