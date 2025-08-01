#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray

from threading import Lock
import time
import math
'''
可视化
rqt_image_view /line_follower/debug_image

启动巡线算法
rosservice call /follow_line/run "data: true"

停止巡线算法
rosservice call /follow_line/run "data: false"
'''
# --- 参数配置区 ---
# 有限状态机（FSM）状态定义
FOLLOW_RIGHT = 0          # 状态一：沿右墙巡线
ALIGN_WITH_ENTRANCE_BOARD = 1 # 状态二：旋转直到平行入口板
ADJUST_LATERAL_POSITION = 2 # 状态三：横向调整位置

# 状态名称映射（用于日志输出）
STATE_NAMES = {
    FOLLOW_RIGHT: "FOLLOW_RIGHT",
    ALIGN_WITH_ENTRANCE_BOARD: "ROTATE_TO_PARALLEL",
    ADJUST_LATERAL_POSITION: "ADJUST_LATERAL_POSITION"
}
# ROS话题参数
IMAGE_TOPIC = "/usb_cam/image_raw"
DEBUG_IMAGE_TOPIC = "/line_follower/debug_image"  # 新增：调试图像发布话题
# 高斯模糊参数
GAUSSIAN_KERNEL_SIZE = (5, 5)  # 高斯核大小
GAUSSIAN_SIGMA_X = 0  # 标准差，0表示根据核大小自动计算
# Canny边缘检测参数
CANNY_LOW_THRESHOLD = 50
CANNY_HIGH_THRESHOLD = 150
# 图像翻转参数
PERFORM_HORIZONTAL_FLIP = True  # 是否执行水平翻转
# 起始点寻找参数
START_POINT_SCAN_STEP = 10  # 向上扫描的步长（像素）
HORIZONTAL_SEARCH_OFFSET = -20 # 水平搜索起始点的偏移量(相对于中心, 负为左, 正为右)
START_POINT_SEARCH_MIN_Y = 120 # 允许寻找起始点的最低Y坐标(从顶部0开始算)
# 胡萝卜点参数
LOOKAHEAD_DISTANCE = 10  # 胡萝卜点与基准点的距离（像素）
PRINT_HZ = 4  # 打印error的频率（次/秒）
# 路径规划参数
CENTER_LINE_OFFSET = -47  # 从右边线向左偏移的像素数
# PID控制器参数
Kp = 0.3  # 比例系数
Ki = 0.0   # 积分系数
Kd = 0.1   # 微分系数
# 速度控制参数
LINEAR_SPEED = 0.1  # 前进速度 (m/s)
ERROR_DEADZONE_PIXELS = 15  # 误差死区（像素），低于此值则认为方向正确
STEERING_TO_ANGULAR_VEL_RATIO = 0.02  # 转向角到角速度的转换系数
MAX_ANGULAR_SPEED_DEG = 15.0  # 最大角速度（度/秒）

# 逆透视变换矩阵（从鸟瞰图坐标到原始图像坐标的映射）
INVERSE_PERSPECTIVE_MATRIX = np.array([
    [-3.365493,  2.608984, -357.317062],
    [-0.049261,  1.302389, -874.095796],
    [ 0.000029,  0.007556,   -4.205510]
], dtype=np.float32)
# 形态学操作的卷积核
DILATE_KERNEL = np.ones((15, 15), np.uint8)  # 较大的核用于膨胀，强力连接断点
ERODE_KERNEL = np.ones((7, 7), np.uint8)   # 较小的核用于腐蚀，精细去除噪点
# 在IPM鸟瞰图上进行分析的ROI区域参数
IPM_ROI_Y = 240  # ROI起始Y坐标
IPM_ROI_H = 240  # ROI高度
IPM_ROI_X = 0    # ROI起始X坐标
IPM_ROI_W = 640  # ROI宽度

# 特殊区域检测参数
NORMAL_AREA_HEIGHT_FROM_BOTTOM = 50  # 从ROI底部算起，被视为"常规"的区域高度（像素）
CONSECUTIVE_FRAMES_FOR_DETECTION = 3  # 连续可疑帧数，达到此值则确认进入

# ==============================================================================
# 状态二: ALIGN_WITH_ENTRANCE_BOARD (与左侧入口板平行)
# ==============================================================================
# --- 行为参数 ---
ALIGNMENT_ROTATION_SPEED_DEG = 7.0      # 旋转对齐时的角速度 (度/秒)
BOARD_DETECT_ANGLE_TOL_DEG = 9.0        # 角度容忍度(越小要求越精确)

# --- 检测参数 ---
ALIGN_TARGET_ANGLE_DEG = 90.0           # 扫描中心: 左侧 (90度)
ALIGN_SCAN_RANGE_DEG = 80.0             # 扫描范围: 中心±40度
ALIGN_MIN_DIST_M = 0.2                  # 最小检测距离
ALIGN_MAX_DIST_M = 3.0                  # 最大检测距离
ALIGN_MIN_LENGTH_M = 1.4                # 板子最小长度
ALIGN_MAX_LENGTH_M = 1.6                # 板子最大长度

# ==============================================================================
# 状态三: ADJUST_LATERAL_POSITION (与左侧板保持距离)
# ==============================================================================
# --- 行为参数 ---
ADJUST_TARGET_LATERAL_DIST_M = 2.0      # 与左侧板的目标横向距离 (米)
ADJUST_LATERAL_SPEED_M_S = 0.1          # 横向平移速度 (米/秒)
ADJUST_LATERAL_POS_TOL_M = 0.05         # 横向位置容差 (米)

# --- 检测参数 ---
ADJUST_TARGET_ANGLE_DEG = 90.0          # 扫描中心: 左侧 (90度)
ADJUST_SCAN_RANGE_DEG = 80.0            # 扫描范围: 中心±40度
ADJUST_MIN_DIST_M = 0.2                 # 最小检测距离
ADJUST_MAX_DIST_M = 3.0                 # 最大检测距离
ADJUST_MIN_LENGTH_M = 1.4               # 板子最小长度 (米)
ADJUST_MAX_LENGTH_M = 1.6               # 板子最大长度 (米)
ADJUST_BOARD_ANGLE_TOL_DEG = 9.0        # 角度容忍度

# ==============================================================================
# 全局激光雷达参数 (适用于所有状态)
# ==============================================================================
LIDAR_TOPIC = "/scan"                   # 激光雷达话题名称
BOARD_DETECT_CLUSTER_TOL_M = 0.05       # 聚类时，点与点之间的最大距离
BOARD_DETECT_MIN_CLUSTER_PTS = 5        # 一个有效聚类最少的点数


# 定义沿墙走的搜索模式（Follow The Wall）
# 顺时针搜索，用于沿着右侧赛道内边界行走
FTW_SEEDS_RIGHT = [
    (-1, 0),    # 左
    (-1, -1),   # 左上
    (0, -1),    # 上
    (1, -1),    # 右上
    (1, 0),     # 右
    (1, 1),     # 右下
    (0, 1),     # 下
    (-1, 1)     # 左下
]

def follow_the_wall(image, start_point, seeds):
    """
    使用沿墙走(Follow The Wall)算法跟踪边界
    
    参数:
    image: 二值图像
    start_point: 起始点坐标 (x, y)
    seeds: 八邻域搜索顺序
    
    返回:
    boundary_points: 边界点列表
    """
    boundary_points = []
    current_point = start_point
    h, w = image.shape[:2]
    
    for _ in range(400):
        boundary_points.append(current_point)
        candidates = []
        
        for i in range(8):
            dx_a, dy_a = seeds[i]
            A_x = current_point[0] + dx_a
            A_y = current_point[1] + dy_a
            
            dx_b, dy_b = seeds[(i + 1) % 8]
            B_x = current_point[0] + dx_b
            B_y = current_point[1] + dy_b
            
            if (0 <= A_x < w and 0 <= A_y < h and 
                0 <= B_x < w and 0 <= B_y < h):
                if image[A_y, A_x] == 0 and image[B_y, B_x] == 255:
                    candidates.append((A_x, A_y))
        
        if not candidates:
            break
        
        next_point = min(candidates, key=lambda p: p[1])
        current_point = next_point
    
    return boundary_points

def extract_final_border(image_height, raw_points):
    """
    从原始轮廓点集中提取每行一个点的最终边线
    """
    final_border = np.full(image_height, -1, dtype=int)
    found_rows = set()
    
    for x, y in raw_points:
        if y not in found_rows:
            final_border[y] = x
            found_rows.add(y)
    
    return final_border



class LineFollowerNode:
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
        # 初始化运行状态
        self.is_running = False
        
        # 初始化FSM状态
        self.current_state = FOLLOW_RIGHT
        
        # 初始化PID内部状态跟踪变量
        self.was_in_deadzone = None # 用于跟踪上一帧是否在PID死区内
        
        # 初始化用于存储处理结果的变量 (线程同步)
        self.data_lock = Lock()
        self.latest_vision_error = 0.0
        self.is_line_found = False
        self.line_y_position = 0  # 用于状态转换判断
        self.latest_debug_image = np.zeros((IPM_ROI_H, IPM_ROI_W, 3), dtype=np.uint8)
        self.is_board_aligned = False  # 用于标记是否已与板子平行
        self.is_left_board_found = False  # 用于标记是否找到左侧板子
        self.latest_lateral_error_m = 0.0  # 与左侧板子的当前距离
        self.last_valid_lateral_twist_y = 0.0  # 上一次有效的横向速度指令
        
        # 初始化特殊区域检测相关的状态变量
        self.consecutive_special_frames = 0
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 初始化PID和打印相关的状态变量
        self.integral = 0.0
        self.last_error = 0.0
        self.last_print_time = time.time()
        
        # 将最大角速度从度转换为弧度
        self.max_angular_speed_rad = np.deg2rad(MAX_ANGULAR_SPEED_DEG)
        
        # 将对齐旋转速度从度转换为弧度
        self.alignment_rotation_speed_rad = np.deg2rad(ALIGNMENT_ROTATION_SPEED_DEG)
        
        # 计算正向透视变换矩阵
        try:
            self.forward_perspective_matrix = np.linalg.inv(INVERSE_PERSPECTIVE_MATRIX)
        except np.linalg.LinAlgError:
            rospy.logerr("错误: 提供的矩阵是奇异矩阵, 无法求逆。请检查矩阵参数。")
            raise
        
        # 创建图像订阅者
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
        # 创建激光雷达订阅者
        self.scan_sub = rospy.Subscriber(LIDAR_TOPIC, LaserScan, self.scan_callback)
        # 创建调试图像发布者
        self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC, Image, queue_size=1)
        # 创建速度指令发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 创建一个用于在RViz中可视化雷达聚类的发布者
        self.clusters_pub = rospy.Publisher('/line_follower/lidar_clusters', MarkerArray, queue_size=10)
        # 创建一个用于在RViz中可视化调试信息的发布者
        self.debug_markers_pub = rospy.Publisher('/line_follower/debug_markers', MarkerArray, queue_size=10)
        
        # 创建运行状态控制服务
        self.run_service = rospy.Service('/follow_line/run', SetBool, self.handle_set_running)
        
        # 创建一个30Hz的主控制循环
        self.main_loop_timer = rospy.Timer(rospy.Duration(1.0/30.0), self.main_control_loop)
        
        rospy.loginfo("已创建图像订阅者和调试图像发布者，等待图像数据...")
        rospy.loginfo("当前状态: FOLLOW_RIGHT")

    def stop(self):
        """发布停止指令"""
        rospy.loginfo("发送停止指令...")
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def handle_set_running(self, request):
        """
        处理运行状态切换请求
        """
        self.is_running = request.data
        if not self.is_running:
            self.stop()
        response = SetBoolResponse()
        response.success = True
        response.message = "Running state set to: {}".format(self.is_running)
        return response
        
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
        tuple: (是否找到符合条件的板子, 中心点X坐标, 中心点Y坐标)
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
                return (False, 0.0, 0.0)
            
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
                        
                        rospy.loginfo_throttle(2, "检测到垂直板子: 中心点(x=%.2f, y=%.2f)m, 长度=%.2fm, 角度偏差=%.1f度", 
                                             center_x_m, lateral_error_m, length, deviation)
                        return (True, center_x_m, lateral_error_m)
                        
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
                        
                        rospy.loginfo_throttle(2, "检测到平行板子: 中心点(x=%.2f, y=%.2f)m, 长度=%.2fm, 角度=%.1f度", 
                                             center_x_m, lateral_error_m, length, angle_deg)
                        return (True, center_x_m, lateral_error_m)
            
            return (False, 0.0, 0.0)
            
        except Exception as e:
            rospy.logwarn_throttle(5, "板子检测出错: %s", str(e))
            return (False, 0.0, 0.0)
    
    def scan_callback(self, msg):
        """
        处理激光雷达数据，根据当前状态检测相应的板子
        """
        # 安全地读取当前状态
        with self.data_lock:
            current_state = self.current_state
        
        if current_state == ALIGN_WITH_ENTRANCE_BOARD:
            # 左侧入口板检测（状态二）
            board_found, board_center_x, board_center_y = self._find_board(
                msg, 
                ALIGN_TARGET_ANGLE_DEG,
                ALIGN_SCAN_RANGE_DEG,
                'PARALLEL',
                ALIGN_MIN_DIST_M,
                ALIGN_MAX_DIST_M,
                ALIGN_MIN_LENGTH_M,
                ALIGN_MAX_LENGTH_M,
                BOARD_DETECT_ANGLE_TOL_DEG
            )
            
            # 更新共享状态
            with self.data_lock:
                self.is_board_aligned = board_found
                
        elif current_state == ADJUST_LATERAL_POSITION:
            # 左侧板检测（状态三）
            board_found, board_center_x, board_center_y = self._find_board(
                msg,
                ADJUST_TARGET_ANGLE_DEG,
                ADJUST_SCAN_RANGE_DEG,
                'PARALLEL',
                ADJUST_MIN_DIST_M,
                ADJUST_MAX_DIST_M,
                ADJUST_MIN_LENGTH_M,
                ADJUST_MAX_LENGTH_M,
                ADJUST_BOARD_ANGLE_TOL_DEG
            )
            
            # 更新共享状态
            with self.data_lock:
                self.is_left_board_found = board_found
                self.latest_lateral_error_m = board_center_y

    def image_callback(self, data):
        """
        这个回调函数是图像处理专家。
        它完成所有视觉计算，并将最终结果存储起来供主循环使用。
        """
        # --- 1. 图像预处理 ---
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("图像回调中转换错误: %s", str(e))
            return
        
        if PERFORM_HORIZONTAL_FLIP:
            frame = cv2.flip(frame, 1)
        
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_frame = cv2.GaussianBlur(gray_frame, GAUSSIAN_KERNEL_SIZE, GAUSSIAN_SIGMA_X)
        canny_edges = cv2.Canny(blurred_frame, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)
        height, width = frame.shape[:2]
        ipm_frame = cv2.warpPerspective(canny_edges, self.forward_perspective_matrix, (width, height))
        dilated_frame = cv2.dilate(ipm_frame, DILATE_KERNEL)
        morphed_full_ipm = cv2.erode(dilated_frame, ERODE_KERNEL)
        final_roi_frame = morphed_full_ipm[IPM_ROI_Y:IPM_ROI_Y + IPM_ROI_H, IPM_ROI_X:IPM_ROI_X + IPM_ROI_W]
        _, binary_roi_frame = cv2.threshold(final_roi_frame, 5, 255, cv2.THRESH_BINARY)
        roi_display = cv2.cvtColor(binary_roi_frame, cv2.COLOR_GRAY2BGR)
        roi_h, roi_w = binary_roi_frame.shape[:2]

        # --- 2. 使用原有的单边线逻辑 ---
        error = 0.0
        is_line_found = False
        line_y_position = 0
        
        start_search_x = (roi_w // 2) + HORIZONTAL_SEARCH_OFFSET
        right_start_point = None
        current_scan_y = None
        
        # 从底部开始，每隔START_POINT_SCAN_STEP个像素向上扫描，寻找右边线起始点
        # 限制最高搜索位置到START_POINT_SEARCH_MIN_Y
        for y in range(roi_h - 1, START_POINT_SEARCH_MIN_Y, -START_POINT_SCAN_STEP):
            # 从中心向右扫描寻找右边线的内侧起始点
            for x in range(start_search_x, roi_w - 1):
                if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x + 1] == 255:
                    right_start_point = (x + 1, y)
                    current_scan_y = y
                    break
            
            if right_start_point is not None:
                break

        if right_start_point:
            is_line_found = True
            line_y_position = right_start_point[1]
            
            # 计算误差的逻辑
            points = follow_the_wall(binary_roi_frame, right_start_point, FTW_SEEDS_RIGHT)
            if points:
                final_border = extract_final_border(roi_h, points)
                if final_border is not None:
                    base_y = right_start_point[1]
                    anchor_y = max(0, base_y - LOOKAHEAD_DISTANCE)
                    roi_points = []
                    for y_idx, x_val in enumerate(final_border):
                        if anchor_y <= y_idx <= base_y and x_val != -1:
                            center_x_path = x_val + CENTER_LINE_OFFSET
                            if 0 <= center_x_path < roi_w:
                                roi_points.append((center_x_path, y_idx))
                                # 绘制区域内的中心线点（青色）
                                cv2.circle(roi_display, (center_x_path, y_idx), 2, (255, 255, 0), -1)
                    
                    if roi_points:
                        avg_x = sum(p[0] for p in roi_points) / len(roi_points)
                        error = avg_x - (roi_w // 2)
                    
                    # 绘制右侧边线
                    for point in points:
                        cv2.circle(roi_display, point, 1, (0, 255, 255), -1)
                    
                    # 找到并绘制胡萝卜点
                    if final_border[anchor_y] != -1:
                        carrot_x = final_border[anchor_y] + CENTER_LINE_OFFSET
                        if 0 <= carrot_x < roi_w:
                            cv2.drawMarker(roi_display, (carrot_x, anchor_y), 
                                         (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        
        # 可视化：画出扫描线和找到的起始点
        if current_scan_y is not None:
            cv2.line(roi_display, (0, current_scan_y), (roi_w, current_scan_y), (255, 0, 0), 1)
        
        if is_line_found:
            cv2.circle(roi_display, right_start_point, 5, (0, 0, 255), -1)

        # --- 3. 将计算结果安全地存入实例变量 ---
        with self.data_lock:
            self.is_line_found = is_line_found
            self.line_y_position = line_y_position
            self.latest_vision_error = error
            self.latest_debug_image = roi_display.copy()

    def main_control_loop(self, timer_event):
        """
        这个函数是总指挥，是状态机的家。
        它只负责决策和发布指令。
        """
        if not self.is_running:
            return

        # --- 1. 从实例变量中安全地读取最新数据 ---
        with self.data_lock:
            is_line_found = self.is_line_found
            vision_error = self.latest_vision_error
            line_y = self.line_y_position
            debug_image = self.latest_debug_image.copy()
            is_board_aligned = self.is_board_aligned

        # --- 2. 状态机决策与执行 ---
        twist_msg = Twist()
        
        # 状态转换逻辑
        if self.current_state == FOLLOW_RIGHT:
            # 如果边线出现在图像上部（远离机器人），则认为是特殊区域
            if is_line_found and line_y < (IPM_ROI_H - NORMAL_AREA_HEIGHT_FROM_BOTTOM):
                self.consecutive_special_frames += 1
            else:
                # 如果条件不满足，则重置计数器
                self.consecutive_special_frames = 0
            
            # 如果连续N帧都满足条件，则执行状态转换
            if self.consecutive_special_frames >= CONSECUTIVE_FRAMES_FOR_DETECTION:
                rospy.loginfo("状态转换: FOLLOW_RIGHT -> ALIGN_WITH_ENTRANCE_BOARD")
                self.stop() # 立即停车
                self.current_state = ALIGN_WITH_ENTRANCE_BOARD
                # 关键：立即发布停车指令并结束本次循环，避免执行旧状态的逻辑
                self.cmd_vel_pub.publish(twist_msg)
                return
        
        # 状态执行逻辑
        if self.current_state == FOLLOW_RIGHT:
            # PID巡线逻辑
            if is_line_found:
                self._execute_line_following_logic_in_main_loop(vision_error, twist_msg)
            else:
                # 丢线则停止
                self.stop()
        
        elif self.current_state == ALIGN_WITH_ENTRANCE_BOARD:
            if is_board_aligned:
                # 如果已对齐，则停止并转换到下一个状态
                rospy.loginfo("状态转换: ALIGN_WITH_ENTRANCE_BOARD -> ADJUST_LATERAL_POSITION")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.current_state = ADJUST_LATERAL_POSITION
                # 发布停止指令并结束本次循环
                self.cmd_vel_pub.publish(twist_msg)
                return
            else:
                # 如果未对齐，则向左旋转
                rospy.loginfo_throttle(1, "状态: %s | 未检测到平行入口板，向左旋转...", STATE_NAMES[self.current_state])
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.alignment_rotation_speed_rad
        
        elif self.current_state == ADJUST_LATERAL_POSITION:
            # 从实例变量中安全地读取左侧板子的检测结果
            with self.data_lock:
                is_left_board_found = self.is_left_board_found
                latest_lateral_error_m = self.latest_lateral_error_m
            
            # 确保twist_msg的前进和旋转速度为零
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
            if not is_left_board_found:
                # 如果没有找到左侧板子，则继续使用上一次的有效横向速度指令
                rospy.loginfo_throttle(1, "状态: %s | 未检测到左侧板子，继续上一次的横向移动...", STATE_NAMES[self.current_state])
                twist_msg.linear.y = self.last_valid_lateral_twist_y
            else:
                # 如果找到了左侧板子，则计算距离误差并控制横向速度
                dist_error = latest_lateral_error_m - ADJUST_TARGET_LATERAL_DIST_M
                
                # 判断是否在容差范围内
                if abs(dist_error) <= ADJUST_LATERAL_POS_TOL_M:
                    # 已达到目标位置，停止
                    rospy.loginfo_throttle(1, "状态: %s | 已达到目标位置，停止 (当前距离: %.2fm, 目标距离: %.2fm)", 
                                         STATE_NAMES[self.current_state], latest_lateral_error_m, ADJUST_TARGET_LATERAL_DIST_M)
                    twist_msg.linear.y = 0.0
                else:
                    # 未达到目标位置，计算横向速度
                    # 如果dist_error为正，说明当前距离大于目标距离，需要向左移动（正方向）
                    # 如果dist_error为负，说明当前距离小于目标距离，需要向右移动（负方向）
                    twist_msg.linear.y = np.sign(dist_error) * ADJUST_LATERAL_SPEED_M_S
                    rospy.loginfo_throttle(1, "状态: %s | 调整位置中 (当前距离: %.2fm, 目标距离: %.2fm, 误差: %.2fm)", 
                                         STATE_NAMES[self.current_state], latest_lateral_error_m, ADJUST_TARGET_LATERAL_DIST_M, dist_error)
                
                # 保存当前的横向速度指令，供目标丢失时使用
                self.last_valid_lateral_twist_y = twist_msg.linear.y
        
        # 发布最终确定的指令
        self.cmd_vel_pub.publish(twist_msg)

        # --- 3. 发布调试图像 ---
        try:
            debug_img_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_img_msg)
        except CvBridgeError as e:
            rospy.logerr("主循环中调试图像发布错误: %s", str(e))

    def _execute_line_following_logic_in_main_loop(self, vision_error, twist_msg):
        """
        在主循环中执行PID巡线逻辑的简化版本
        """
        # 检查是否在死区内外发生切换，如果是则刹车
        is_in_deadzone = abs(vision_error) <= ERROR_DEADZONE_PIXELS
        if self.was_in_deadzone is not None and self.was_in_deadzone != is_in_deadzone:
            rospy.loginfo("状态: %s | 切换行驶模式(直行/转向)，刹车...", STATE_NAMES[self.current_state])
            self.stop()
        self.was_in_deadzone = is_in_deadzone

        # PID控制逻辑
        if abs(vision_error) > ERROR_DEADZONE_PIXELS:
            # 状态：原地旋转以修正方向
            twist_msg.linear.x = 0.0
            
            # 计算PID控制器的输出
            p_term = Kp * vision_error
            self.integral += vision_error
            i_term = Ki * self.integral
            derivative = vision_error - self.last_error
            d_term = Kd * derivative
            self.last_error = vision_error
            steering_angle = p_term + i_term + d_term
            
            # 计算角速度并进行限幅
            angular_z_rad = -1 * steering_angle * STEERING_TO_ANGULAR_VEL_RATIO
            twist_msg.angular.z = np.clip(angular_z_rad, -self.max_angular_speed_rad, self.max_angular_speed_rad)
        
        else:
            # 状态：方向正确，直线前进
            twist_msg.linear.x = LINEAR_SPEED
            twist_msg.angular.z = 0.0
            # 重置PID积分项和last_error
            self.integral = 0.0
            self.last_error = 0.0
        
        # 按指定频率打印error、线速度和角速度
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0 / PRINT_HZ:
            final_angular_deg = np.rad2deg(twist_msg.angular.z)
            rospy.loginfo("状态: %s | Error: %7.2f | Linear_x: %.2f | Angular_z: %7.2f deg/s", 
                        STATE_NAMES[self.current_state], vision_error, twist_msg.linear.x, final_angular_deg)
            self.last_print_time = current_time

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('line_follower_node', anonymous=True)
        
        # 创建并运行节点
        node = LineFollowerNode()
        
        # 注册关闭钩子
        rospy.on_shutdown(node.stop)
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("节点已关闭。")