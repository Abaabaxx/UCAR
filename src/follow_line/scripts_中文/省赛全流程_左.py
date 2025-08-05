#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

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
FOLLOW_LEFT = 0          # 状态一：沿左墙巡线
STRAIGHT_TRANSITION = 1     # 新增状态：直行过渡
ROTATE_TO_FACE_EXIT_BOARD = 2 # 状态二：旋转正对出口板
FOLLOW_LEFT_WITH_AVOIDANCE = 3 # 状态三：带避障巡线
ALIGN_WITH_OBSTACLE_BOARD = 4 # 状态四：对准前方的障碍物板
AVOIDANCE_MANEUVER = 5    # 状态五：执行避障机动
FOLLOW_TO_FINISH = 6      # 状态六：最终冲刺巡线
FINAL_STOP = 7           # 状态七：任务结束并停止

# 状态名称映射（用于日志输出）
STATE_NAMES = {
    FOLLOW_LEFT: "FOLLOW_LEFT",
    STRAIGHT_TRANSITION: "STRAIGHT_TRANSITION",
    ROTATE_TO_FACE_EXIT_BOARD: "ROTATE_TO_FACE_EXIT_BOARD",
    FOLLOW_LEFT_WITH_AVOIDANCE: "FOLLOW_LEFT_WITH_AVOIDANCE",
    ALIGN_WITH_OBSTACLE_BOARD: "ALIGN_WITH_OBSTACLE_BOARD",
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
CENTER_LINE_OFFSET = 50  # 从左边线向右偏移的像素数
# 速度控制参数
LINEAR_SPEED = 0.1  # 前进速度 (m/s)
ERROR_DEADZONE_PIXELS = 15  # 误差死区（像素），低于此值则认为方向正确
LINE_FOLLOWING_ANGULAR_SPEED_DEG = 10.0 # 巡线时的固定转向角速度 (度/秒)
ALIGNMENT_ROTATION_SPEED_DEG = 10.0      # 旋转对齐时的角速度 (度/秒)

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
STRAIGHT_TRANSITION_EXIT_FROM_BOTTOM_PX = 38 # 从ROI底部算起，小于此像素距离则退出直行

# 激光雷达避障参数
LIDAR_TOPIC = "/scan"                                  # 激光雷达话题名称
AVOIDANCE_ANGLE_DEG = 40.0                             # 监控的前方角度范围（正负各20度）
AVOIDANCE_DISTANCE_M = 0.4                             # 触发避障的距离阈值（米）
AVOIDANCE_POINT_THRESHOLD = 10                         # 触发避障的点数阈值

# 避障机动参数
ODOM_TOPIC = "/odom"                                   # 里程计话题
AVOIDANCE_STRAFE_OUT_DISTANCE_M = 0.5                  # 避障-向外平移距离 (米)
AVOIDANCE_STRAFE_IN_DISTANCE_M = 0.45                   # 避障-向内平移距离 (米)
AVOIDANCE_FORWARD_DISTANCE_M = 0.58                     # 避障-前进距离 (米)
AVOIDANCE_STRAFE_SPEED_MPS = 0.15                       # 避障-平移速度 (米/秒)
AVOIDANCE_FORWARD_SPEED_MPS = 0.15                      # 避障-前进速度 (米/秒)

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



# ==============================================================================
# 状态六: ROTATE_TO_FACE_EXIT_BOARD (旋转正对出口板)
# ==============================================================================
# --- 检测参数 ---
EXIT_TARGET_ANGLE_DEG = 0.0           # 扫描中心: 正前方 (0度)
EXIT_SCAN_RANGE_DEG = 100.0           # 扫描范围: 中心±50度
EXIT_MIN_DIST_M = 0.2                 # 最小检测距离
EXIT_MAX_DIST_M = 1.5                 # 最大检测距离
EXIT_MIN_LENGTH_M = 0.37               # 板子最小长度 (米)
EXIT_MAX_LENGTH_M = 0.63               # 板子最大长度 (米)
EXIT_ANGLE_TOL_DEG = 2.0              # 正对出口板时的角度容忍度 (度)

# ==============================================================================
# 状态六-A: ALIGN_WITH_OBSTACLE_BOARD (对准前方的障碍物板)
# ==============================================================================
ALIGN_OBSTACLE_TARGET_ANGLE_DEG = 0.0      # 扫描中心: 正前方 (0度)
ALIGN_OBSTACLE_SCAN_RANGE_DEG = 120.0      # 扫描范围: 中心±60度
ALIGN_OBSTACLE_MIN_DIST_M = 0.2            # 最小检测距离 (米)
ALIGN_OBSTACLE_MAX_DIST_M = 0.5            # 最大检测距离 (米)
ALIGN_OBSTACLE_MIN_LENGTH_M = 0.37         # 板子最小长度 (米)
ALIGN_OBSTACLE_MAX_LENGTH_M = 0.63         # 板子最大长度 (米)
ALIGN_OBSTACLE_OBSERVE_ANGLE_TOL_DEG = 20.0 # 锁定板子的角度容忍度 (度)
ALIGN_OBSTACLE_ALIGN_ANGLE_TOL_DEG = 3.0   # 完成对准的角度容忍度 (度)

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
        
        # 初始化PID内部状态跟踪变量
        self.was_in_deadzone = None # 用于跟踪上一帧是否在PID死区内
        
        # 初始化用于存储处理结果的变量 (线程同步)
        self.data_lock = Lock()
        self.latest_vision_error = 0.0
        self.is_line_found = False
        self.line_y_position = 0  # 用于状态转换判断
        self.latest_debug_image = np.zeros((IPM_ROI_H, IPM_ROI_W, 3), dtype=np.uint8)
        self.is_exit_board_faced = False    # 状态六：是否已正对出口板
        
        # 初始化状态机控制标志
        self.realign_cycle_completed = False
        
        # 初始化避障标志位
        self.obstacle_detected = False # 避障标志位
        self.is_obstacle_board_locked = False # 障碍物板锁定标志
        self.obstacle_board_angle_error_deg = 0.0 # 障碍物板角度误差
        
        # 初始化里程计和避障机动相关的状态变量
        self.latest_odom_pose = None         # 存储当前里程计姿态
        self.maneuver_initial_pose = None    # 存储机动动作开始时的姿态
        self.maneuver_step = 0               # 标记机动动作的步骤 (0:右移, 1:前进, 2:左移)
        
        # 初始化停车区域检测相关的状态变量
        self.is_stop_zone_detected = False   # 标记单帧是否检测到停车区
        self.consecutive_stop_frames = 0     # 连续检测到停车区的帧数
        
        # 初始化特殊区域检测相关的状态变量
        self.consecutive_special_frames = 0
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 初始化打印相关的状态变量
        self.last_print_time = time.time()
        
        # 将角速度从度转换为弧度
        self.alignment_rotation_speed_rad = np.deg2rad(ALIGNMENT_ROTATION_SPEED_DEG)
        self.line_following_angular_speed_rad = np.deg2rad(LINE_FOLLOWING_ANGULAR_SPEED_DEG)
        
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
                    left_start_point = (x, y)
                    break
            
            # 寻找右起点 (使用FTW_right.py的逻辑)
            start_search_x_right = (roi_w // 2) - HORIZONTAL_SEARCH_OFFSET  # 相反偏移
            for x in range(start_search_x_right, roi_w - 1):
                if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x + 1] == 255:
                    right_start_point = (x, y)
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
        
    def _find_obstacle_board_with_signed_angle(self, scan_msg, target_angle_deg, scan_range_deg, alignment_mode, 
                                      min_dist_m=0.25, max_dist_m=1.5, min_length_m=0.45, max_length_m=0.62, 
                                      angle_tol_deg=9.0):
        """
        与 _find_board 函数类似，但返回带符号的角度偏差，用于确定板子的偏转方向。
        正值表示板子向左偏，负值表示板子向右偏。
        
        参数和返回值与 _find_board 相同，但角度偏差带有符号。
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
                        center_x_base_link = center_x_m + LIDAR_X_OFFSET_M
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
                        center_x_base_link = center_x_m + LIDAR_X_OFFSET_M
                        rospy.loginfo_throttle(2, "检测到平行板子: 中心点(机器人坐标系 x=%.2f, y=%.2f)m, 长度=%.2fm, 角度=%.1f度", 
                                            center_x_base_link, lateral_error_m, length, angle_deg)
                        return (True, center_x_m, lateral_error_m, deviation)
            
            return (False, 0.0, 0.0, 999.0)
            
        except Exception as e:
            rospy.logwarn_throttle(5, "板子检测出错: %s", str(e))
            return (False, 0.0, 0.0, 999.0)

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
                        
                                                # 为日志记录计算base_link坐标
                        center_x_base_link = center_x_m + LIDAR_X_OFFSET_M
                        rospy.loginfo_throttle(2, "检测到垂直板子: 中心点(机器人坐标系 x=%.2f, y=%.2f)m, 长度=%.2fm, 角度偏差=%.1f度", 
                                             center_x_base_link, lateral_error_m, length, deviation)
                        return (True, center_x_m, lateral_error_m, deviation)
                        
                elif alignment_mode == 'PARALLEL':
                    deviation = angle_deg  # 平行时，角度应接近0度
                    if deviation <= angle_tol_deg:
                        # 找到了一个平行的板子
                        center_x_m = np.mean(cluster_array[:, 0])  # 前向距离（X轴）
                        lateral_error_m = np.mean(cluster_array[:, 1])  # 横向偏差（Y轴）
                        
                        # 为日志记录计算base_link坐标
                        center_x_base_link = center_x_m + LIDAR_X_OFFSET_M
                        rospy.loginfo_throttle(2, "检测到平行板子: 中心点(机器人坐标系 x=%.2f, y=%.2f)m, 长度=%.2fm, 角度=%.1f度", 
                                             center_x_base_link, lateral_error_m, length, angle_deg)
                        return (True, center_x_m, lateral_error_m, deviation)
            
            return (False, 0.0, 0.0, 999.0)
            
        except Exception as e:
            rospy.logwarn_throttle(5, "板子检测出错: %s", str(e))
            return (False, 0.0, 0.0, 999.0)
    
    def odom_callback(self, msg):
        """
        处理里程计数据，更新当前机器人姿态。
        """
        self.latest_odom_pose = msg.pose.pose
    
    def scan_callback(self, msg):
        """
        处理激光雷达数据，根据当前状态检测相应的板子，以及前方障碍物
        """
        # 1. 安全地读取当前状态
        with self.data_lock:
            current_state = self.current_state
        
        if current_state == ROTATE_TO_FACE_EXIT_BOARD:
            # 寻找正前方的垂直板子
            board_found, board_center_x, board_center_y, board_angle_dev = self._find_board(
                msg,
                EXIT_TARGET_ANGLE_DEG,
                EXIT_SCAN_RANGE_DEG,
                'PERPENDICULAR',
                EXIT_MIN_DIST_M,
                EXIT_MAX_DIST_M,
                EXIT_MIN_LENGTH_M,
                EXIT_MAX_LENGTH_M,
                EXIT_ANGLE_TOL_DEG  # 使用状态五的专属角度阈值
            )
            
            # 更新共享状态
            with self.data_lock:
                self.is_exit_board_faced = board_found
                
        elif current_state == ALIGN_WITH_OBSTACLE_BOARD:
            # 寻找正前方的垂直板子（障碍物）
            board_found, board_center_x, board_center_y, signed_angle_error = self._find_obstacle_board_with_signed_angle(
                msg,
                ALIGN_OBSTACLE_TARGET_ANGLE_DEG,
                ALIGN_OBSTACLE_SCAN_RANGE_DEG,
                'PERPENDICULAR',
                ALIGN_OBSTACLE_MIN_DIST_M,
                ALIGN_OBSTACLE_MAX_DIST_M,
                ALIGN_OBSTACLE_MIN_LENGTH_M,
                ALIGN_OBSTACLE_MAX_LENGTH_M,
                ALIGN_OBSTACLE_OBSERVE_ANGLE_TOL_DEG # 使用宽容的"观察"阈值来发现和锁定板子
            )
            
            # 更新共享状态
            with self.data_lock:
                self.is_obstacle_board_locked = board_found
                if board_found:
                    self.obstacle_board_angle_error_deg = signed_angle_error
                else:
                    # 如果没找到，重置误差
                    self.obstacle_board_angle_error_deg = 0.0
                
        elif current_state == FOLLOW_LEFT_WITH_AVOIDANCE:
            # 检测前方障碍物
            try:
                # 计算0度（正前方）的索引
                center_index = int((0.0 - msg.angle_min) / msg.angle_increment)
                
                # 计算角度偏移对应的索引数量
                angle_rad = np.deg2rad(AVOIDANCE_ANGLE_DEG)
                index_offset = int(angle_rad / msg.angle_increment)
                
                # 确定扫描的起始和结束索引
                start_index = center_index - index_offset
                end_index = center_index + index_offset
                
                # 遍历指定范围内的点，统计满足条件的障碍物点数
                obstacle_points_count = 0
                for i in range(start_index, end_index):
                    distance = msg.ranges[i]
                    # 检查距离是否在有效且危险的范围内 (忽略0和inf)
                    if 0 < distance < AVOIDANCE_DISTANCE_M:
                        obstacle_points_count += 1
                
                # 更新障碍物检测标志
                if obstacle_points_count > AVOIDANCE_POINT_THRESHOLD:
                    self.obstacle_detected = True
                else:
                    self.obstacle_detected = False
            except Exception as e:
                rospy.logwarn_throttle(1.0, "障碍物检测出错: %s", str(e))
                self.obstacle_detected = False
        else:
            # 在其他所有状态下，确保障碍物检测标志被禁用
            self.obstacle_detected = False

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
            # 使用单边线逻辑 - 左侧巡线 (寻找内边界: 黑->白)
            start_search_x = (roi_w // 2) + HORIZONTAL_SEARCH_OFFSET
            left_start_point = None
            current_scan_y = None
            
            # 从底部开始，每隔START_POINT_SCAN_STEP个像素向上扫描，寻找左边线起始点
            # 限制最高搜索位置到START_POINT_SEARCH_MIN_Y
            for y in range(roi_h - 1, START_POINT_SEARCH_MIN_Y, -START_POINT_SCAN_STEP):
                # 从中心向左扫描寻找左边线的内侧起始点
                for x in range(start_search_x, 0, -1):
                    # 【核心修改点】将条件从 (白,黑) 改为 (黑,白)
                    if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x - 1] == 255:
                        # 【关键修正点】起始点应该是那个黑色的像素点
                        left_start_point = (x, y) 
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
            
            # 可视化：画出扫描线和找到的起始点
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
            is_exit_board_faced = self.is_exit_board_faced
            is_stop_zone_detected = self.is_stop_zone_detected
        
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
            if self.maneuver_step == 0: # 步骤0: 向右平移
                rospy.loginfo_throttle(1.0, "避障步骤0: 向右平移... (%.2f / %.2f m)", distance_moved, AVOIDANCE_STRAFE_OUT_DISTANCE_M)
                if distance_moved < AVOIDANCE_STRAFE_OUT_DISTANCE_M:
                    twist_msg.linear.y = -AVOIDANCE_STRAFE_SPEED_MPS
                else:
                    self.stop()
                    rospy.loginfo("向右平移完成。")
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
            
            elif self.maneuver_step == 2: # 步骤2: 向左平移
                rospy.loginfo_throttle(1.0, "避障步骤2: 向左平移... (%.2f / %.2f m)", distance_moved, AVOIDANCE_STRAFE_IN_DISTANCE_M)
                if distance_moved < AVOIDANCE_STRAFE_IN_DISTANCE_M:
                    twist_msg.linear.y = AVOIDANCE_STRAFE_SPEED_MPS # 正号表示向左
                else:
                    self.stop()
                    rospy.loginfo("向左平移完成。避障机动结束。")
                    # 机动完成，进入最终冲刺巡线状态
                    self.current_state = FOLLOW_TO_FINISH
            
            self.cmd_vel_pub.publish(twist_msg)
            
            # 创建避障状态的调试图像
            maneuver_display = np.zeros((IPM_ROI_H, IPM_ROI_W, 3), dtype=np.uint8)
            cv2.putText(maneuver_display, "AVOIDANCE MANEUVER - STEP: {}".format(self.maneuver_step), 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 根据步骤动态选择目标距离
            target_distance = 0.0
            if self.maneuver_step == 0:
                target_distance = AVOIDANCE_STRAFE_OUT_DISTANCE_M
            elif self.maneuver_step == 1:
                target_distance = AVOIDANCE_FORWARD_DISTANCE_M
            elif self.maneuver_step == 2:
                target_distance = AVOIDANCE_STRAFE_IN_DISTANCE_M

            cv2.putText(maneuver_display, "Distance: {:.2f}/{:.2f}m".format(distance_moved, target_distance), 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            debug_image = maneuver_display
            
            # 发布调试图像并返回，不执行后续逻辑
            try:
                debug_img_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_image_pub.publish(debug_img_msg)
            except CvBridgeError as e:
                rospy.logerr("主循环中调试图像发布错误: %s", str(e))
            return
        
        # 状态转换逻辑
        if self.current_state == FOLLOW_LEFT:
            # 如果边线出现在图像上部（远离机器人），则认为是特殊区域
            if is_line_found and line_y < (IPM_ROI_H - NORMAL_AREA_HEIGHT_FROM_BOTTOM):
                self.consecutive_special_frames += 1
            else:
                # 如果条件不满足，则重置计数器
                self.consecutive_special_frames = 0
            
            # 如果连续N帧都满足条件，则执行状态转换
            if self.consecutive_special_frames >= CONSECUTIVE_FRAMES_FOR_DETECTION:
                rospy.loginfo("状态转换: FOLLOW_LEFT -> STRAIGHT_TRANSITION")
                self.stop() # 立即停车
                self.current_state = STRAIGHT_TRANSITION
                # 关键：立即发布停车指令并结束本次循环，避免执行旧状态的逻辑
                self.cmd_vel_pub.publish(twist_msg)
                return
        
        elif self.current_state == STRAIGHT_TRANSITION:
            if is_line_found and line_y >= (IPM_ROI_H - STRAIGHT_TRANSITION_EXIT_FROM_BOTTOM_PX):
                rospy.loginfo("状态转换: STRAIGHT_TRANSITION -> ROTATE_TO_FACE_EXIT_BOARD")
                self.stop() # 立即停车
                self.current_state = ROTATE_TO_FACE_EXIT_BOARD
                # 关键：立即发布停车指令并结束本次循环，避免执行旧状态的逻辑
                self.cmd_vel_pub.publish(twist_msg)
                return
        
        # 状态执行逻辑
        if self.current_state == FOLLOW_LEFT:
            # PID巡线逻辑
            if is_line_found:
                self._execute_line_following_logic_in_main_loop(vision_error, twist_msg)
            else:
                # 丢线则停止
                self.stop()
        
        elif self.current_state == STRAIGHT_TRANSITION:
            # 直行过渡
            rospy.loginfo_throttle(1, "状态: %s | 直行过渡中...", STATE_NAMES[self.current_state])
            twist_msg.linear.x = LINEAR_SPEED
            twist_msg.angular.z = 0.0
        
        elif self.current_state == ROTATE_TO_FACE_EXIT_BOARD:
            # 从实例变量中安全地读取出口板子的检测结果
            with self.data_lock:
                is_exit_board_faced = self.is_exit_board_faced
            
            # 确保twist_msg的线速度为零
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            
            if is_exit_board_faced:
                # 如果已正对出口板，则进入带避障巡线状态
                rospy.loginfo("状态转换: ROTATE_TO_FACE_EXIT_BOARD -> FOLLOW_LEFT_WITH_AVOIDANCE")
                self.stop()  # 立即停车
                
                # 设置状态标志
                self.current_state = FOLLOW_LEFT_WITH_AVOIDANCE
                
                # 立即发布停止指令并结束本次循环
                self.cmd_vel_pub.publish(Twist())
                return
            else:
                # 如果未找到正对的出口板，则向左旋转
                rospy.loginfo_throttle(1, "状态: %s | 向左旋转寻找出口板...", STATE_NAMES[self.current_state])
                twist_msg.angular.z = self.alignment_rotation_speed_rad  # 正号表示向左旋转
                
        elif self.current_state == FOLLOW_LEFT_WITH_AVOIDANCE:
            if obstacle_detected:
                rospy.loginfo("状态转换: FOLLOW_LEFT_WITH_AVOIDANCE -> ALIGN_WITH_OBSTACLE_BOARD")
                self.stop()
                self.current_state = ALIGN_WITH_OBSTACLE_BOARD
                # 重置对准状态变量
                with self.data_lock:
                    self.is_obstacle_board_locked = False
                    self.obstacle_board_angle_error_deg = 0.0
                return
            
            # PID巡线逻辑
            if is_line_found:
                self._execute_line_following_logic_in_main_loop(vision_error, twist_msg)
            else:
                # 丢线则停止
                self.stop()
        
        elif self.current_state == ALIGN_WITH_OBSTACLE_BOARD:
            # 从实例变量中安全地读取障碍物板的检测结果
            with self.data_lock:
                is_obstacle_board_locked = self.is_obstacle_board_locked
                obstacle_board_angle_error_deg = self.obstacle_board_angle_error_deg
            
            # 确保twist_msg的线速度为零
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            
            if not is_obstacle_board_locked:
                # 阶段A: 未锁定目标，执行默认搜索旋转 (向左)
                rospy.loginfo_throttle(1, "状态: %s | 未锁定目标板，向左搜索...", STATE_NAMES[self.current_state])
                twist_msg.angular.z = self.alignment_rotation_speed_rad
            else:
                # 阶段B: 已锁定目标，开始对准
                if abs(obstacle_board_angle_error_deg) <= ALIGN_OBSTACLE_ALIGN_ANGLE_TOL_DEG:
                    # 对准完成，进入避障机动状态
                    rospy.loginfo("状态转换: ALIGN_WITH_OBSTACLE_BOARD -> AVOIDANCE_MANEUVER")
                    self.stop() # 立即停车
                    self.current_state = AVOIDANCE_MANEUVER
                    self.maneuver_step = 0
                    self.maneuver_initial_pose = None
                    return
                else:
                    # 未对准，根据误差符号进行智能旋转
                    rospy.loginfo_throttle(1, "状态: %s | 已锁定目标，对准中... 角度误差: %.2f度", 
                                         STATE_NAMES[self.current_state], obstacle_board_angle_error_deg)
                    # 误差为正，板子在左，应向左转(角速度为正)
                    # 误差为负，板子在右，应向右转(角速度为负)
                    twist_msg.angular.z = np.sign(obstacle_board_angle_error_deg) * self.alignment_rotation_speed_rad
                
        elif self.current_state == FOLLOW_TO_FINISH:
            # 读取停车区检测结果
            if is_stop_zone_detected:
                self.consecutive_stop_frames += 1
            else:
                self.consecutive_stop_frames = 0
            
            if self.consecutive_stop_frames >= STOP_ZONE_CONSECUTIVE_FRAMES:
                rospy.loginfo("状态转换: FOLLOW_TO_FINISH -> FINAL_STOP")
                self.stop()
                self.current_state = FINAL_STOP
                return
                
            # PID巡线逻辑
            if is_line_found:
                self._execute_line_following_logic_in_main_loop(vision_error, twist_msg)
            else:
                # 丢线则停止
                self.stop()
                
        elif self.current_state == FINAL_STOP:
            # 任务完成，停止
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
            # 重置状态标志并结束任务
            with self.data_lock:
                self.is_running = False # 任务完成，彻底停止主循环
                
            # 立即发布停止指令并结束本次循环
            self.cmd_vel_pub.publish(Twist())
            return
        
        # 全局丢线处理：如果丢线，则对于所有需要巡线的状态，都执行原地旋转搜索
        if not is_line_found:
            # 只有在需要巡线的状态下才旋转搜索
            if self.current_state in [FOLLOW_LEFT, STRAIGHT_TRANSITION, FOLLOW_LEFT_WITH_AVOIDANCE, FOLLOW_TO_FINISH]:
                rospy.loginfo_throttle(1, "状态: %s | 丢线，开始原地旋转搜索...", STATE_NAMES[self.current_state])
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.alignment_rotation_speed_rad  # 正号表示向左旋转
        
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
            twist_msg.angular.z = -np.sign(vision_error) * self.line_following_angular_speed_rad
        
        else:
            # 状态：方向正确，直线前进
            twist_msg.linear.x = LINEAR_SPEED
            twist_msg.angular.z = 0.0
        
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
