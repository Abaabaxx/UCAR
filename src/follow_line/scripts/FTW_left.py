#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from threading import Lock
import math
import time
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
FOLLOW_LEFT = 0          # 状态一：沿左墙巡线
STRAIGHT_TRANSITION = 1   # 状态二：直行过渡
ROTATE_ALIGNMENT = 2      # 状态三：原地转向对准
LIDAR_CRUISE = 3 # 状态四：雷达锁板巡航(带避障)
AVOIDANCE_MANEUVER = 4    # 状态五：执行避障机动
FOLLOW_TO_FINISH = 5     # 状态六：最终冲刺巡线
FINAL_STOP = 6           # 状态七：任务结束并停止

# 状态名称映射（用于日志输出）
STATE_NAMES = {
    FOLLOW_LEFT: "FOLLOW_LEFT",
    STRAIGHT_TRANSITION: "STRAIGHT_TRANSITION",
    ROTATE_ALIGNMENT: "ROTATE_ALIGNMENT",
    LIDAR_CRUISE: "LIDAR_CRUISE",
    AVOIDANCE_MANEUVER: "AVOIDANCE_MANEUVER",
    FOLLOW_TO_FINISH: "FOLLOW_TO_FINISH",
    FINAL_STOP: "FINAL_STOP"
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
HORIZONTAL_SEARCH_OFFSET = 20 # 水平搜索起始点的偏移量(相对于中心, 负为左, 正为右)
START_POINT_SEARCH_MIN_Y = 120 # 允许寻找起始点的最低Y坐标(从顶部0开始算)
# 胡萝卜点参数
LOOKAHEAD_DISTANCE = 10  # 胡萝卜点与基准点的距离（像素）
PRINT_HZ = 4  # 打印error的频率（次/秒）
# 路径规划参数
CENTER_LINE_OFFSET = 47  # 从左边线向右偏移的像素数
# PID控制器参数
Kp = 0.3  # 比例系数
Ki = 0.0   # 积分系数
Kd = 0.1   # 微分系数
# 速度控制参数
LINEAR_SPEED = 0.1  # 前进速度 (m/s)
ERROR_DEADZONE_PIXELS = 15  # 误差死区（像素），低于此值则认为方向正确
STEERING_TO_ANGULAR_VEL_RATIO = 0.02  # 转向角到角速度的转换系数
MAX_ANGULAR_SPEED_DEG = 15.0  # 最大角速度（度/秒）
# 原地转向对准状态参数
ROTATE_ALIGNMENT_SPEED_DEG = 7.0 # 固定的原地左转角速度 (度/秒, 正值为左转)
ROTATE_ALIGNMENT_ERROR_THRESHOLD = 15 # 退出转向状态的像素误差阈值
LINE_SEARCH_ROTATION_SPEED_DEG = 7.0 # 丢线后原地向左旋转搜索的速度 (度/秒)
# 激光雷达板子垂直度检测参数 (用于ROTATE_ALIGNMENT状态)
BOARD_DETECT_ANGLE_DEG = 90.0      # 扫描前方 +/- 这么多度
BOARD_DETECT_MIN_DIST_M = 0.26      # 考虑的最小距离
BOARD_DETECT_MAX_DIST_M = 0.9      # 考虑的最大距离
BOARD_DETECT_CLUSTER_TOL_M = 0.05  # 聚类时，点与点之间的最大距离
BOARD_DETECT_MIN_CLUSTER_PTS = 5   # 一个有效聚类最少的点数
BOARD_DETECT_MIN_LENGTH_M = 0.36    # 聚成的线段最小长度
BOARD_DETECT_MAX_LENGTH_M = 0.66    # 聚成的线段最大长度
BOARD_DETECT_ANGLE_TOL_DEG = 12.0  # 与水平线的最大角度容忍度(越小越垂直)
# 激光雷达避障参数
LIDAR_TOPIC = "/scan"                                  # 激光雷达话题名称
AVOIDANCE_ANGLE_DEG = 40.0                             # 监控的前方角度范围（正负各20度）
AVOIDANCE_DISTANCE_M = 0.4                             # 触发避障的距离阈值（米）
AVOIDANCE_POINT_THRESHOLD = 20                         # 触发避障的点数阈值
# 避障机动参数
ODOM_TOPIC = "/odom"                                   # 里程计话题
AVOIDANCE_STRAFE_DISTANCE_M = 0.5                      # 避障-平移距离 (米)
AVOIDANCE_FORWARD_DISTANCE_M = 0.58                     # 避障-前进距离 (米)
AVOIDANCE_STRAFE_SPEED_MPS = 0.15                       # 避障-平移速度 (米/秒)
AVOIDANCE_FORWARD_SPEED_MPS = 0.15                      # 避障-前进速度 (米/秒)
# 雷达锁板巡航(LIDAR_CRUISE)状态参数
LIDAR_CRUISE_FORWARD_SPEED_MPS = 0.1   # 直行速度 (m/s)
LIDAR_CRUISE_STRAFE_SPEED_MPS = 0.08  # 平移速度 (m/s)
LIDAR_CRUISE_ANGULAR_SPEED_DEG = 7.0   # 旋转速度 (度/秒)
LIDAR_CRUISE_ANGLE_TOL_DEG = 10.0      # 角度容忍度/死区 (度)
LIDAR_CRUISE_LATERAL_TOL_M = 0.05      # 横向误差容忍度/死区 (米)
LIDAR_CRUISE_LOST_TARGET_THRESH_FRAMES = 15 # 连续丢失目标的容忍帧数 (15帧 @ 30Hz ≈ 0.5秒)
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

# 停车区域检测参数 (用于FOLLOW_TO_FINISH状态)
STOP_ZONE_ROI_HEIGHT_PX = 3        # 从图像底部向上计算的窗口高度
STOP_ZONE_ROI_WIDTH_PX = 30       # 窗口宽度
STOP_ZONE_WHITE_PIXEL_THRESH = 0.60  # 窗口中白色像素的百分比阈值
STOP_ZONE_CONSECUTIVE_FRAMES = 3     # 连续满足条件的帧数

# 定义沿墙走的搜索模式（Follow The Wall）
# 逆时针搜索，用于沿着左侧赛道内边界行走
FTW_SEEDS_LEFT = [
    (0, 1),     # 下
    (1, 1),     # 右下
    (1, 0),     # 右
    (1, -1),    # 右上
    (0, -1),    # 上
    (-1, -1),   # 左上
    (-1, 0),    # 左
    (-1, 1)     # 左下
]

# 顺时针搜索，用于沿着右侧赛道内边界行走 (逻辑来自 FTW_right.py)
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
    def __init__(self):
        # 初始化运行状态
        self.is_running = False
        
        # 初始化FSM状态
        self.current_state = FOLLOW_LEFT
        
        # 初始化状态机控制标志
        self.realign_cycle_completed = False
        
        # 初始化PID内部状态跟踪变量
        self.was_in_deadzone = None # 用于跟踪上一帧是否在PID死区内
        
        # 初始化避障标志位
        self.obstacle_detected = False # 避障标志位
        
        # 初始化里程计和避障机动相关的状态变量
        self.latest_odom_pose = None         # 存储当前里程计姿态
        self.maneuver_initial_pose = None # 存储机动动作开始时的姿态
        self.maneuver_step = 0           # 标记机动动作的步骤 (0:左移, 1:前进, 2:右移)
        
        # 初始化用于存储处理结果的变量 (线程同步)
        self.data_lock = Lock()
        self.latest_vision_error = 0.0
        self.is_line_found = False
        self.line_y_position = 0 # 用于状态转换判断
        self.latest_debug_image = np.zeros((IPM_ROI_H, IPM_ROI_W, 3), dtype=np.uint8)
        
        # 初始化板子检测相关状态变量
        self.board_detected_info = (False, 0.0, 0.0) # (是否找到, 角度误差deg, 横向误差m)
        self.board_lost_frames = 0                         # 连续丢失板子目标的帧数计数器
        self.last_known_board_info = (False, 0.0, 0.0)     # 缓存上一次成功检测到的板子信息
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 初始化PID和打印相关的状态变量
        self.integral = 0.0
        self.last_error = 0.0
        self.last_print_time = time.time()
        
        # 初始化特殊区域检测相关的状态变量
        self.consecutive_special_frames = 0
        
        # 初始化停车区域检测相关的状态变量
        self.is_stop_zone_detected = False   # 标记单帧是否检测到停车区
        self.consecutive_stop_frames = 0     # 连续检测到停车区的帧数
        
        # 将最大角速度从度转换为弧度
        self.max_angular_speed_rad = np.deg2rad(MAX_ANGULAR_SPEED_DEG)
        
        # 将原地转向角速度从度转换为弧度
        self.rotate_alignment_speed_rad = np.deg2rad(ROTATE_ALIGNMENT_SPEED_DEG)
        
        # 将丢线搜索角速度从度转换为弧度
        self.line_search_rotation_speed_rad = np.deg2rad(LINE_SEARCH_ROTATION_SPEED_DEG)
        
        # 将雷达锁板巡航角速度从度转换为弧度
        self.lidar_cruise_angular_speed_rad = np.deg2rad(LIDAR_CRUISE_ANGULAR_SPEED_DEG)
        
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
        # 创建里程计订阅者
        self.odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback)
        # 创建调试图像发布者
        self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC, Image, queue_size=1)
        # 创建速度指令发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 创建一个用于在RViz中可视化雷达聚类的发布者
        self.clusters_pub = rospy.Publisher('/line_follower/lidar_clusters', MarkerArray, queue_size=10)
        
        # 创建一个用于在RViz中可视化调试信息（如中心点、法向量）的发布者
        self.debug_markers_pub = rospy.Publisher('/line_follower/debug_markers', MarkerArray, queue_size=10)
        
        # 创建运行状态控制服务
        self.run_service = rospy.Service('/follow_line/run', SetBool, self.handle_set_running)
        
        # 创建一个30Hz的主控制循环
        self.main_loop_timer = rospy.Timer(rospy.Duration(1.0/30.0), self.main_control_loop)
        
        rospy.loginfo("已创建图像订阅者和调试图像发布者，等待图像数据...")
        rospy.loginfo("当前状态: FOLLOW_LEFT")

    def _process_image_for_centerline(self, binary_roi_frame, roi_display):
        """
        专门用于FOLLOW_TO_FINISH状态的双边线处理函数
        
        参数:
        binary_roi_frame: 二值化ROI图像
        roi_display: 用于绘制调试信息的彩色图像
        
        返回:
        error: 计算出的误差
        is_line_found: 是否找到线
        roi_display: 更新后的调试图像
        """
        roi_h, roi_w = binary_roi_frame.shape[:2]
        
        # 寻找双起点 - 从下向上扫描，直到同时找到左右起点
        left_start_point = None
        right_start_point = None
        current_scan_y = None
        
        for y in range(roi_h - 1, START_POINT_SEARCH_MIN_Y, -START_POINT_SCAN_STEP):
            # 寻找左起点 (使用FTW_left.py现有逻辑)
            start_search_x_left = (roi_w // 2) + HORIZONTAL_SEARCH_OFFSET
            for x in range(start_search_x_left, 0, -1):
                if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x - 1] == 255:
                    left_start_point = (x - 1, y)
                    break
            
            # 寻找右起点 (使用FTW_right.py的逻辑)
            start_search_x_right = (roi_w // 2) - HORIZONTAL_SEARCH_OFFSET  # 相反偏移
            for x in range(start_search_x_right, roi_w - 1):
                if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x + 1] == 255:
                    right_start_point = (x + 1, y)
                    break
            
            # 只有同时找到左右起点才认为有效
            if left_start_point is not None and right_start_point is not None:
                current_scan_y = y
                break
            else:
                # 重置，继续向上寻找
                left_start_point = None
                right_start_point = None
        
        # 如果没有找到双起点，返回无效结果
        if left_start_point is None or right_start_point is None:
            return 0.0, False, roi_display
        
        # 执行双边爬线
        left_points = follow_the_wall(binary_roi_frame, left_start_point, FTW_SEEDS_LEFT)
        right_points = follow_the_wall(binary_roi_frame, right_start_point, FTW_SEEDS_RIGHT)
        
        if not left_points or not right_points:
            return 0.0, False, roi_display
        
        # 提取边线
        left_border = extract_final_border(roi_h, left_points)
        right_border = extract_final_border(roi_h, right_points)
        
        if left_border is None or right_border is None:
            return 0.0, False, roi_display
        
        # 计算中线并复用胡萝卜点逻辑
        base_y = left_start_point[1]  # 使用左起点的y坐标作为基准
        anchor_y = max(0, base_y - LOOKAHEAD_DISTANCE)
        
        center_points = []
        for y in range(anchor_y, base_y + 1):
            if left_border[y] != -1 and right_border[y] != -1:
                center_x = (left_border[y] + right_border[y]) // 2
                if 0 <= center_x < roi_w:
                    center_points.append((center_x, y))
                    # 绘制中线点（绿色）
                    cv2.circle(roi_display, (center_x, y), 2, (0, 255, 0), -1)
        
        # 计算误差
        error = 0.0
        if center_points:
            avg_x = sum(p[0] for p in center_points) / len(center_points)
            error = avg_x - (roi_w // 2)
        
        # 绘制调试信息
        # 绘制扫描线
        if current_scan_y is not None:
            cv2.line(roi_display, (0, current_scan_y), (roi_w, current_scan_y), (255, 0, 0), 1)
        
        # 绘制起点
        cv2.circle(roi_display, left_start_point, 5, (0, 0, 255), -1)  # 红色
        cv2.circle(roi_display, right_start_point, 5, (255, 0, 0), -1)  # 蓝色
        
        # 绘制左右边线
        for point in left_points:
            cv2.circle(roi_display, point, 1, (0, 255, 255), -1)  # 黄色
        for point in right_points:
            cv2.circle(roi_display, point, 1, (255, 255, 0), -1)  # 青色
        
        # 绘制胡萝卜点（如果中线在anchor_y有点的话）
        if center_points:
            # 找到anchor_y行的中心点
            for center_x, y in center_points:
                if y == anchor_y:
                    cv2.drawMarker(roi_display, (center_x, anchor_y), 
                                 (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                    break
        
        return error, True, roi_display

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

    def _check_for_perpendicular_board(self, scan_msg):
        """
        检测激光雷达前方是否存在垂直的板子。
        借鉴find_board.py的聚类和拟合思想，但简化为轻量级实时检测。
        
        返回: tuple (bool, float, float) - (是否检测到垂直板子, 角度误差deg, 横向误差m)
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
            
            # 1. 数据筛选：只考虑前方指定角度和距离范围内的点
            angle_rad_range = np.deg2rad(BOARD_DETECT_ANGLE_DEG)
            center_angle = 0.0  # 正前方
            
            # 计算角度索引范围
            center_index = int((center_angle - scan_msg.angle_min) / scan_msg.angle_increment)
            angle_index_range = int(angle_rad_range / scan_msg.angle_increment)
            start_index = max(0, center_index - angle_index_range)
            end_index = min(len(scan_msg.ranges), center_index + angle_index_range)
            
            # 提取有效点的坐标
            points = []
            for i in range(start_index, end_index):
                distance = scan_msg.ranges[i]
                if BOARD_DETECT_MIN_DIST_M <= distance <= BOARD_DETECT_MAX_DIST_M:
                    angle = scan_msg.angle_min + i * scan_msg.angle_increment
                    x = distance * np.cos(angle)
                    y = distance * np.sin(angle)
                    points.append((x, y))
            
            if len(points) < BOARD_DETECT_MIN_CLUSTER_PTS:
                self.debug_markers_pub.publish(debug_marker_array)
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
            
            # --- [开始] 可视化所有找到的聚类 (学习自 3_show_roi_clusters.py) ---
            marker_array = MarkerArray()

            # 1. 创建一个特殊的Marker用于清除上一帧的所有标记
            # 这是一个健壮的实践，确保RViz中不残留旧的可视化
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
                
                if not (BOARD_DETECT_MIN_LENGTH_M <= length <= BOARD_DETECT_MAX_LENGTH_M):
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
                
                # 检查是否接近垂直（正对机器人前进方向）
                # 垂直线的角度应该接近90度
                angle_from_vertical = abs(angle_deg - 90)
                
                if angle_from_vertical <= BOARD_DETECT_ANGLE_TOL_DEG:
                    # 计算板子中心点的完整坐标
                    center_x_m = np.mean(cluster_array[:, 0])  # 前向距离（X轴）
                    lateral_error_m = np.mean(cluster_array[:, 1])  # 横向偏差（Y轴）
                    
                    # --- [开始] 可视化有效聚类的中心点和法向量 ---
            
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
                    if x_std > y_std: # 拟合 y = mx + c
                        slope = coeffs[0]
                        # 法向量方向 (-slope, 1)
                        normal_vector = np.array([-slope, 1.0])
                    else: # 拟合 x = my + c
                        slope = coeffs[0]
                        # 法向量方向 (1, -slope)
                        normal_vector = np.array([1.0, -slope])

                    # (学习点) 检查并确保法线指向外侧 (远离雷达原点)
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
                    
                    # --- [结束] 可视化代码 ---
                    
                    rospy.loginfo_throttle(2, "检测到垂直板子: 中心点(x=%.2f, y=%.2f)m, 长度=%.2fm, 角度偏差=%.1f度", 
                                         center_x_m, lateral_error_m, length, angle_from_vertical)
                    self.debug_markers_pub.publish(debug_marker_array)
                    return (True, angle_from_vertical, lateral_error_m)
            
            self.debug_markers_pub.publish(debug_marker_array)
            return (False, 0.0, 0.0)
            
        except Exception as e:
            rospy.logwarn_throttle(5, "板子检测出错: %s", str(e))
            self.debug_markers_pub.publish(debug_marker_array)
            return (False, 0.0, 0.0)

    def scan_callback(self, msg):
        """
        处理激光雷达数据，判断前方是否存在障碍物。
        """
        try:
            # 1. 计算要检查的激光雷达数据点的索引范围
            # 计算0度（正前方）的索引
            center_index = int((0.0 - msg.angle_min) / msg.angle_increment)
            
            # 计算角度偏移对应的索引数量
            angle_rad = np.deg2rad(AVOIDANCE_ANGLE_DEG)
            index_offset = int(angle_rad / msg.angle_increment)
            
            # 确定扫描的起始和结束索引
            start_index = center_index - index_offset
            end_index = center_index + index_offset
            
            # 2. 遍历指定范围内的点，统计满足条件的障碍物点数
            obstacle_points_count = 0
            for i in range(start_index, end_index):
                distance = msg.ranges[i]
                # 检查距离是否在有效且危险的范围内 (忽略0和inf)
                if 0 < distance < AVOIDANCE_DISTANCE_M:
                    obstacle_points_count += 1
            
            # 3. 更新障碍物检测标志
            if obstacle_points_count > AVOIDANCE_POINT_THRESHOLD:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
            
            # 4. 更新板子垂直度检测标志
            with self.data_lock:
                self.board_detected_info = self._check_for_perpendicular_board(msg)

        except Exception as e:
            rospy.logwarn_throttle(1.0, "scan_callback中发生错误: %s", str(e))
            self.obstacle_detected = False
            with self.data_lock:
                self.board_detected_info = (False, 0.0, 0.0)

    def odom_callback(self, msg):
        """
        处理里程计数据，更新当前机器人姿态。
        """
        self.latest_odom_pose = msg.pose.pose

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

        # --- 2. 根据当前状态选择处理模式 ---
        # 安全地读取当前状态
        current_state_snapshot = self.current_state
        
        error = 0.0
        is_line_found = False
        line_y_position = 0
        
        if current_state_snapshot == FOLLOW_TO_FINISH:
            # 使用双边线中线拟合逻辑
            error, is_line_found, roi_display = self._process_image_for_centerline(binary_roi_frame, roi_display)
            # 对于line_y_position，我们可以使用一个虚拟值，因为FOLLOW_TO_FINISH状态不需要它来判断状态转换
            line_y_position = roi_h // 2  # 使用中间位置作为占位符
        else:
            # 使用原有的单边线逻辑
            start_search_x = (roi_w // 2) + HORIZONTAL_SEARCH_OFFSET
            left_start_point = None
            current_scan_y = None
            
            # 从底部开始，每隔START_POINT_SCAN_STEP个像素向上扫描，寻找左边线起始点
            # 限制最高搜索位置到START_POINT_SEARCH_MIN_Y
            for y in range(roi_h - 1, START_POINT_SEARCH_MIN_Y, -START_POINT_SCAN_STEP):
                # 从中心向左扫描寻找左边线的内侧起始点
                for x in range(start_search_x, 0, -1):
                    if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x - 1] == 255:
                        left_start_point = (x - 1, y)
                        current_scan_y = y
                        break
                
                if left_start_point is not None:
                    break

            if left_start_point:
                is_line_found = True
                line_y_position = left_start_point[1]
                
                # 计算误差的逻辑
                points = follow_the_wall(binary_roi_frame, left_start_point, FTW_SEEDS_LEFT)
                if points:
                    final_border = extract_final_border(roi_h, points)
                    if final_border is not None:
                        base_y = left_start_point[1]
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
                        
                        # 绘制左侧边线
                        for point in points:
                            cv2.circle(roi_display, point, 1, (0, 255, 255), -1)
                        
                        # 找到并绘制胡萝卜点
                        if final_border[anchor_y] != -1:
                            carrot_x = final_border[anchor_y] + CENTER_LINE_OFFSET
                            if 0 <= carrot_x < roi_w:
                                cv2.drawMarker(roi_display, (carrot_x, anchor_y), 
                                             (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            
            # 可视化：画出扫描线和找到的起始点（仅单边线模式）
            if current_scan_y is not None:
                cv2.line(roi_display, (0, current_scan_y), (roi_w, current_scan_y), (255, 0, 0), 1)
            
            if is_line_found:
                cv2.circle(roi_display, left_start_point, 5, (0, 0, 255), -1)

        # --- 3. 停车区域检测 ---
        # 计算检测窗口坐标（底部中心）
        stop_window_x_start = (roi_w // 2) - (STOP_ZONE_ROI_WIDTH_PX // 2)
        stop_window_x_end = stop_window_x_start + STOP_ZONE_ROI_WIDTH_PX
        stop_window_y_start = roi_h - STOP_ZONE_ROI_HEIGHT_PX
        stop_window_y_end = roi_h
        
        # 确保窗口坐标在图像边界内
        stop_window_x_start = max(0, stop_window_x_start)
        stop_window_x_end = min(roi_w, stop_window_x_end)
        stop_window_y_start = max(0, stop_window_y_start)
        
        # 提取检测窗口
        stop_detection_window = binary_roi_frame[stop_window_y_start:stop_window_y_end, 
                                               stop_window_x_start:stop_window_x_end]
        
        # 计算白色像素数量和百分比
        white_pixel_count = cv2.countNonZero(stop_detection_window)
        total_pixels = stop_detection_window.shape[0] * stop_detection_window.shape[1]
        white_pixel_ratio = white_pixel_count / total_pixels if total_pixels > 0 else 0.0
        
        # 判断是否满足停车条件
        stop_zone_detected = white_pixel_ratio >= STOP_ZONE_WHITE_PIXEL_THRESH
        
        # 在调试图像上绘制检测窗口
        cv2.rectangle(roi_display, (stop_window_x_start, stop_window_y_start), 
                     (stop_window_x_end, stop_window_y_end), (0, 0, 255), 2)

        # --- 4. 将计算结果安全地存入实例变量 ---
        with self.data_lock:
            self.is_line_found = is_line_found
            self.line_y_position = line_y_position
            self.latest_vision_error = error
            self.is_stop_zone_detected = stop_zone_detected
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
        
        obstacle_detected = self.obstacle_detected
        current_pose = self.latest_odom_pose

        # --- 2. 状态机决策与执行 ---
        twist_msg = Twist()

        # 处理AVOIDANCE_MANEUVER状态 (独立于视觉)
        if self.current_state == AVOIDANCE_MANEUVER:
            if current_pose is None:
                rospy.logwarn_throttle(1.0, "主循环：正在等待里程计数据...")
                return

            # 如果是新步骤的开始，记录初始姿态
            if self.maneuver_initial_pose is None:
                self.maneuver_initial_pose = current_pose

            # 计算从本步骤开始时已移动的距离
            initial_p = self.maneuver_initial_pose.position
            current_p = current_pose.position
            distance_moved = math.sqrt(
                math.pow(current_p.x - initial_p.x, 2) + 
                math.pow(current_p.y - initial_p.y, 2)
            )

            # 根据当前步骤执行相应动作
            if self.maneuver_step == 0: # 步骤0: 向左平移
                rospy.loginfo_throttle(1.0, "避障步骤0: 向左平移... (%.2f / %.2f m)", distance_moved, AVOIDANCE_STRAFE_DISTANCE_M)
                if distance_moved < AVOIDANCE_STRAFE_DISTANCE_M:
                    twist_msg.linear.y = AVOIDANCE_STRAFE_SPEED_MPS
                else:
                    self.stop()
                    rospy.loginfo("向左平移完成。")
                    self.maneuver_step = 1
                    self.maneuver_initial_pose = None # 重置，为下一步做准备
            
            elif self.maneuver_step == 1: # 步骤1: 向前直行
                rospy.loginfo_throttle(1.0, "避障步骤1: 向前直行... (%.2f / %.2f m)", distance_moved, AVOIDANCE_FORWARD_DISTANCE_M)
                if distance_moved < AVOIDANCE_FORWARD_DISTANCE_M:
                    twist_msg.linear.x = AVOIDANCE_FORWARD_SPEED_MPS
                else:
                    self.stop()
                    rospy.loginfo("向前直行完成。")
                    self.maneuver_step = 2
                    self.maneuver_initial_pose = None # 重置

            elif self.maneuver_step == 2: # 步骤2: 向右平移
                rospy.loginfo_throttle(1.0, "避障步骤2: 向右平移... (%.2f / %.2f m)", distance_moved, AVOIDANCE_STRAFE_DISTANCE_M)
                if distance_moved < AVOIDANCE_STRAFE_DISTANCE_M:
                    twist_msg.linear.y = -AVOIDANCE_STRAFE_SPEED_MPS # 负号表示向右
                else:
                    self.stop()
                    rospy.loginfo("向右平移完成。避障机动结束。")
                    # 机动完成，进入最终冲刺巡线状态
                    self.current_state = FOLLOW_TO_FINISH
            
            self.cmd_vel_pub.publish(twist_msg)
            
            # 创建避障状态的调试图像
            maneuver_display = np.zeros((IPM_ROI_H, IPM_ROI_W, 3), dtype=np.uint8)
            cv2.putText(maneuver_display, "AVOIDANCE MANEUVER - STEP: {}".format(self.maneuver_step), 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(maneuver_display, "Distance: {:.2f}/{:.2f}m".format(distance_moved, 
                       AVOIDANCE_STRAFE_DISTANCE_M if self.maneuver_step != 1 else AVOIDANCE_FORWARD_DISTANCE_M), 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            debug_image = maneuver_display
            
        # 处理需要视觉的状态
        else:
            # 状态转换逻辑
            if self.current_state == FOLLOW_LEFT:
                if is_line_found and not self.realign_cycle_completed and line_y < (IPM_ROI_H - NORMAL_AREA_HEIGHT_FROM_BOTTOM):
                    self.consecutive_special_frames += 1
                else:
                    self.consecutive_special_frames = 0
                
                if self.consecutive_special_frames >= CONSECUTIVE_FRAMES_FOR_DETECTION:
                    rospy.loginfo("状态转换: FOLLOW_LEFT -> STRAIGHT_TRANSITION")
                    self.stop()
                    self.was_in_deadzone = None
                    self.current_state = STRAIGHT_TRANSITION
                    return
            
            elif self.current_state == STRAIGHT_TRANSITION:
                if is_line_found and line_y > (IPM_ROI_H - 30):
                    rospy.loginfo("直行完成，刹车并准备转向...")
                    self.stop()
                    self.current_state = ROTATE_ALIGNMENT
                    return
            
            elif self.current_state == ROTATE_ALIGNMENT:
                # 雷达条件：从 self.board_detected_info 读取
                with self.data_lock:
                    lidar_condition_met = self.board_detected_info[0]

                rospy.loginfo_throttle(1, "状态: ROTATE_ALIGNMENT | 等待雷达确认垂直: %s",
                                     str(lidar_condition_met))

                # 只依赖雷达条件判断
                if lidar_condition_met:
                    rospy.loginfo("状态转换: 雷达确认垂直，转向完成。")
                    self.realign_cycle_completed = True
                    self.current_state = LIDAR_CRUISE
                    self.stop()
                    return
            
            elif self.current_state == LIDAR_CRUISE:
                if obstacle_detected:
                    rospy.loginfo("状态转换: LIDAR_CRUISE -> AVOIDANCE_MANEUVER")
                    self.stop()
                    self.current_state = AVOIDANCE_MANEUVER
                    self.maneuver_step = 0
                    self.maneuver_initial_pose = None
                    return
            
            elif self.current_state == FOLLOW_TO_FINISH:
                # 读取停车区检测结果
                with self.data_lock:
                    stop_zone_detected = self.is_stop_zone_detected
                
                if stop_zone_detected:
                    self.consecutive_stop_frames += 1
                else:
                    self.consecutive_stop_frames = 0
                
                if self.consecutive_stop_frames >= STOP_ZONE_CONSECUTIVE_FRAMES:
                    rospy.loginfo("状态转换: FOLLOW_TO_FINISH -> FINAL_STOP")
                    self.stop()
                    self.current_state = FINAL_STOP
                    return

            # 状态执行逻辑 - 首先，根据当前状态确定默认的twist_msg (假设总能找到线)
            if self.current_state == FOLLOW_LEFT or self.current_state == FOLLOW_TO_FINISH:
                # PID巡线逻辑
                if is_line_found:
                    self._execute_line_following_logic_in_main_loop(vision_error, twist_msg)
                
            elif self.current_state == STRAIGHT_TRANSITION:
                if is_line_found:
                    rospy.loginfo_throttle(1, "状态: %s | 正在直行...", STATE_NAMES[self.current_state])
                    twist_msg.linear.x = LINEAR_SPEED
                    twist_msg.angular.z = 0.0
                
            elif self.current_state == ROTATE_ALIGNMENT:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.rotate_alignment_speed_rad
            
            elif self.current_state == LIDAR_CRUISE:
                # 雷达锁板巡航逻辑：带容错帧的状态记忆控制
                with self.data_lock:
                    is_board_found, angle_error_deg, lateral_error_m = self.board_detected_info

                # 决策要使用的板子信息（当前帧或缓存）
                active_board_info = None

                if is_board_found:
                    # 如果当前帧找到了，重置计数器，更新缓存
                    self.board_lost_frames = 0
                    self.last_known_board_info = (is_board_found, angle_error_deg, lateral_error_m)
                    active_board_info = self.last_known_board_info
                else:
                    # 如果当前帧没找到，增加计数器
                    self.board_lost_frames += 1
                    rospy.logwarn_throttle(0.2, "状态: LIDAR_CRUISE | 目标暂时丢失(%d/%d帧)，使用缓存数据...", 
                                         self.board_lost_frames, LIDAR_CRUISE_LOST_TARGET_THRESH_FRAMES)
                    
                    # 检查是否超过容忍阈值
                    if self.board_lost_frames <= LIDAR_CRUISE_LOST_TARGET_THRESH_FRAMES:
                        # 未超时，使用上一次的有效数据继续执行
                        active_board_info = self.last_known_board_info
                    else:
                        # 已超时，目标被认为真正丢失，停止所有动作
                        rospy.logwarn_throttle(1, "状态: LIDAR_CRUISE | 警告: 连续丢失板子目标超过阈值，停止所有动作")
                        # active_board_info 保持为 None

                # 根据最终确定的active_board_info来执行动作
                if active_board_info and active_board_info[0]:
                    _, active_angle_error, active_lateral_error = active_board_info
                    
                    # 第一优先级：角度修正
                    if abs(active_angle_error) > LIDAR_CRUISE_ANGLE_TOL_DEG:
                        twist_msg.linear.x = 0.0
                        twist_msg.linear.y = 0.0
                        twist_msg.angular.z = self.lidar_cruise_angular_speed_rad if active_angle_error > 0 else -self.lidar_cruise_angular_speed_rad
                        rospy.loginfo_throttle(1, "状态: LIDAR_CRUISE | 角度修正: %.1f度", active_angle_error)
                    # 第二优先级：横向位置修正 (已修复方向)
                    elif abs(active_lateral_error) > LIDAR_CRUISE_LATERAL_TOL_M:
                        twist_msg.linear.x = 0.0
                        twist_msg.angular.z = 0.0
                        # 横向误差(板子y坐标)为正表示板子在左侧，需向左平移(y速度为正)
                        twist_msg.linear.y = LIDAR_CRUISE_STRAFE_SPEED_MPS if active_lateral_error > 0 else -LIDAR_CRUISE_STRAFE_SPEED_MPS
                        rospy.loginfo_throttle(1, "状态: LIDAR_CRUISE | 横向修正: %.2fm", active_lateral_error)
                    # 默认：直行
                    else:
                        twist_msg.linear.x = LIDAR_CRUISE_FORWARD_SPEED_MPS
                        twist_msg.linear.y = 0.0
                        twist_msg.angular.z = 0.0
                        rospy.loginfo_throttle(2, "状态: LIDAR_CRUISE | 直行中，角度误差: %.1f度, 横向误差: %.2fm", 
                                             active_angle_error, active_lateral_error)
                else:
                    # 如果 active_board_info 为 None 或无效，则停车
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = 0.0
            
            elif self.current_state == FINAL_STOP:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
            
            # 然后，作为最后一步，检查是否丢失了线，并覆盖上面的指令
            if not is_line_found:
                # 只有在需要巡线的状态下才旋转搜索
                if self.current_state in [FOLLOW_LEFT, FOLLOW_TO_FINISH, STRAIGHT_TRANSITION]:
                    rospy.loginfo_throttle(1, "状态: %s | 丢线，开始原地旋转搜索...", STATE_NAMES[self.current_state])
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = self.line_search_rotation_speed_rad
            
            # 最后发布最终确定的指令
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