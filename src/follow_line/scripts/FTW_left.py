#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
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
FOLLOW_LEFT_WITH_AVOIDANCE = 3 # 状态四：带避障巡线

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
# 激光雷达避障参数
LIDAR_TOPIC = "/scan"                                  # 激光雷达话题名称
AVOIDANCE_ANGLE_DEG = 20.0                             # 监控的前方角度范围（正负各20度）
AVOIDANCE_DISTANCE_M = 0.4                             # 触发避障的距离阈值（米）
AVOIDANCE_POINT_THRESHOLD = 20                         # 触发避障的点数阈值
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

# 定义沿墙走的搜索模式（Follow The Wall）
# 逆时针搜索，用于沿着左侧赛道内边界行走
FTW_SEEDS = [
    (0, 1),     # 下
    (1, 1),     # 右下
    (1, 0),     # 右
    (1, -1),    # 右上
    (0, -1),    # 上
    (-1, -1),   # 左上
    (-1, 0),    # 左
    (-1, 1)     # 左下
]

def follow_the_wall(image, start_point):
    """
    使用沿墙走(Follow The Wall)算法跟踪边界
    
    参数:
    image: 二值图像
    start_point: 起始点坐标 (x, y)
    
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
            dx_a, dy_a = FTW_SEEDS[i]
            A_x = current_point[0] + dx_a
            A_y = current_point[1] + dy_a
            
            dx_b, dy_b = FTW_SEEDS[(i + 1) % 8]
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
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 初始化PID和打印相关的状态变量
        self.integral = 0.0
        self.last_error = 0.0
        self.last_print_time = time.time()
        
        # 初始化特殊区域检测相关的状态变量
        self.consecutive_special_frames = 0
        
        # 将最大角速度从度转换为弧度
        self.max_angular_speed_rad = np.deg2rad(MAX_ANGULAR_SPEED_DEG)
        
        # 将原地转向角速度从度转换为弧度
        self.rotate_alignment_speed_rad = np.deg2rad(ROTATE_ALIGNMENT_SPEED_DEG)
        
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
        
        # 创建运行状态控制服务
        self.run_service = rospy.Service('/follow_line/run', SetBool, self.handle_set_running)
        
        rospy.loginfo("已创建图像订阅者和调试图像发布者，等待图像数据...")
        rospy.loginfo("当前状态: FOLLOW_LEFT")

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

        except Exception as e:
            rospy.logwarn_throttle(1.0, "scan_callback中发生错误: %s", str(e))
            self.obstacle_detected = False

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 检查是否处于运行状态
            if not self.is_running:
                # 如果未运行，仍然发布原始图像用于调试
                try:
                    debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.debug_image_pub.publish(debug_img_msg)
                except CvBridgeError as e:
                    rospy.logerr("调试图像转换或发布错误: %s", str(e))
                return
                
        except CvBridgeError as e:
            rospy.logerr("图像转换错误: %s", str(e))
            return
            
        # 执行水平翻转（如果启用）
        if PERFORM_HORIZONTAL_FLIP:
            frame = cv2.flip(frame, 1)
        
        # 将原始帧转换为灰度图
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 对灰度图应用高斯模糊
        blurred_frame = cv2.GaussianBlur(gray_frame, GAUSSIAN_KERNEL_SIZE, GAUSSIAN_SIGMA_X)
        
        # 应用Canny边缘检测
        canny_edges = cv2.Canny(blurred_frame, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)
        
        # 应用逆透视变换（使用正向变换矩阵生成鸟瞰图）
        height, width = frame.shape[:2]
        ipm_frame = cv2.warpPerspective(canny_edges, self.forward_perspective_matrix, (width, height))
        
        # 对完整鸟瞰图应用形态学闭运算（先膨胀后腐蚀）
        dilated_frame = cv2.dilate(ipm_frame, DILATE_KERNEL)
        morphed_full_ipm = cv2.erode(dilated_frame, ERODE_KERNEL)
        
        # 从处理后的鸟瞰图中截取ROI区域
        final_roi_frame = morphed_full_ipm[IPM_ROI_Y:IPM_ROI_Y + IPM_ROI_H, IPM_ROI_X:IPM_ROI_X + IPM_ROI_W]
        
        # 对ROI区域进行二值化处理
        _, binary_roi_frame = cv2.threshold(final_roi_frame, 5, 255, cv2.THRESH_BINARY)
        
        # 为了在上面画图，我们先将ROI二值图转换为BGR彩色图
        roi_display = cv2.cvtColor(binary_roi_frame, cv2.COLOR_GRAY2BGR)

        # 获取ROI的尺寸
        roi_h, roi_w = binary_roi_frame.shape[:2]
        
        # 初始化中心点和起始点坐标
        start_search_x = (roi_w // 2) + HORIZONTAL_SEARCH_OFFSET
        left_start_point = None
        current_scan_y = None

        # 从底部开始，每隔START_POINT_SCAN_STEP个像素向上扫描，寻找左边线起始点
        for y in range(roi_h - 1, 0, -START_POINT_SCAN_STEP):
            # 从中心向左扫描寻找左边线的内侧起始点
            for x in range(start_search_x, 0, -1):
                if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x - 1] == 255:
                    left_start_point = (x - 1, y)
                    current_scan_y = y
                    break
            
            if left_start_point is not None:
                break
        
        # 可视化：画出扫描线和找到的起始点
        if current_scan_y is not None:
            cv2.line(roi_display, (0, current_scan_y), (roi_w, current_scan_y), (255, 0, 0), 1)
        
        if left_start_point:
            # 在起始点画一个红色的圆
            cv2.circle(roi_display, left_start_point, 5, (0, 0, 255), -1)
            
            # 根据当前状态执行相应的逻辑
            if self.current_state == FOLLOW_LEFT:
                # --- 状态一：沿左墙巡线 ---
                # 1. 检查状态转换条件（即检测到特殊区域）
                start_y = left_start_point[1]
                trigger_y_threshold = roi_h - NORMAL_AREA_HEIGHT_FROM_BOTTOM
                if start_y < trigger_y_threshold:
                    self.consecutive_special_frames += 1
                else:
                    self.consecutive_special_frames = 0

                if not self.realign_cycle_completed and self.consecutive_special_frames >= CONSECUTIVE_FRAMES_FOR_DETECTION:
                    rospy.loginfo("状态转换: FOLLOW_LEFT -> STRAIGHT_TRANSITION, 刹车...")
                    self.stop()
                    self.was_in_deadzone = None # 重置内部状态跟踪器
                    self.current_state = STRAIGHT_TRANSITION
                    # 注意：此处删除了原有的速度发布指令，将运动控制权完全交给下一帧
                else:
                    # 2. 如果不转换，执行本状态的行为（PID巡左墙）
                    self._execute_line_following_logic(binary_roi_frame, left_start_point, roi_h, roi_w, roi_display)

            elif self.current_state == STRAIGHT_TRANSITION:
                # --- 状态二：直行过渡 ---
                # 1. 执行本状态的行为（保持直行）
                rospy.loginfo_throttle(1, "当前状态: STRAIGHT_TRANSITION, 正在直行...")
                twist_msg = Twist()
                twist_msg.linear.x = LINEAR_SPEED
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)

                # 2. 检查状态转换条件（左侧边线是否靠近底部）
                transition_y_threshold = roi_h - 30
                if left_start_point[1] > transition_y_threshold:
                    rospy.loginfo("直行完成，刹车并准备转向...")
                    self.stop() # <--- 新增刹车指令
                    self.current_state = ROTATE_ALIGNMENT
                
                # 可视化：绘制左侧边线
                points = follow_the_wall(binary_roi_frame, left_start_point)
                if points:
                    for point in points:
                        cv2.circle(roi_display, point, 1, (0, 255, 255), -1)
                    
                    # 在图像上显示当前Y坐标和阈值
                    cv2.putText(roi_display, "Y: {0}, Threshold: {1}".format(left_start_point[1], transition_y_threshold), 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            elif self.current_state == ROTATE_ALIGNMENT:
                # --- 状态三：原地转向对准 ---
                # 1. 在此状态下，我们必须持续计算误差，以判断何时完成转向
                points = follow_the_wall(binary_roi_frame, left_start_point)
                error = 0.0
                error_calculated = False

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
                            error_calculated = True
                
                # 2. 检查是否满足退出条件 (误差足够小)
                if error_calculated and abs(error) < ROTATE_ALIGNMENT_ERROR_THRESHOLD:
                    # 对准完成，切换到最终的带避障巡线状态
                    rospy.loginfo("状态转换: ROTATE_ALIGNMENT -> FOLLOW_LEFT_WITH_AVOIDANCE")
                    self.realign_cycle_completed = True
                    self.current_state = FOLLOW_LEFT_WITH_AVOIDANCE
                    self.stop() # 立即停止，让下一帧的逻辑接管运动
                else:
                    # 未对准或未找到边线，执行或继续执行固定向左旋转
                    rospy.loginfo_throttle(1, "当前状态: ROTATE_ALIGNMENT, 正在向左旋转... Error: %.2f", error)
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = self.rotate_alignment_speed_rad
                    self.cmd_vel_pub.publish(twist_msg)
                
                # 可视化：绘制左侧边线
                if points:
                    for point in points:
                        cv2.circle(roi_display, point, 1, (0, 255, 255), -1)
                    
                    # 在图像上显示当前Y坐标和误差
                    cv2.putText(roi_display, "Y: {0}, Error: {1:.2f}".format(left_start_point[1], error), 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            elif self.current_state == FOLLOW_LEFT_WITH_AVOIDANCE:
                # --- 状态四：带避障巡线 ---
                # 1. 首先检查是否检测到障碍物
                if self.obstacle_detected:
                    rospy.loginfo_throttle(1.0, "检测到障碍物，执行避障停止！")
                    self.stop()
                    # 在图像上显示避障状态
                    cv2.putText(roi_display, "OBSTACLE DETECTED! STOPPING!", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    return  # 跳过后续的巡线逻辑
                
                # 2. 如果没有障碍物，执行与FOLLOW_LEFT相同的巡线逻辑
                self._execute_line_following_logic(binary_roi_frame, left_start_point, roi_h, roi_w, roi_display)
        else:
            # 如果没有找到起始点，发送停止指令
            self.stop()
        
        # 发布调试图像
        try:
            debug_img_msg = self.bridge.cv2_to_imgmsg(roi_display, "bgr8")
            self.debug_image_pub.publish(debug_img_msg)
        except CvBridgeError as e:
            rospy.logerr("调试图像转换或发布错误: %s", str(e))

    def _execute_line_following_logic(self, binary_roi_frame, left_start_point, roi_h, roi_w, roi_display):
        """
        执行PID巡线的核心逻辑。
        
        参数:
        binary_roi_frame: 二值化后的ROI图像
        left_start_point: 左边线起始点坐标
        roi_h: ROI高度
        roi_w: ROI宽度
        roi_display: 用于可视化的彩色图像
        """
        # 使用沿墙走算法寻找左边界
        points = follow_the_wall(binary_roi_frame, left_start_point)
        
        # 提取最终的左边线
        final_border = None
        if points:
            final_border = extract_final_border(roi_h, points)
        
        # 如果成功提取到左边线，计算error
        if final_border is not None:
            # 确定基准点和锚点行
            base_y = left_start_point[1]
            anchor_y = max(0, base_y - LOOKAHEAD_DISTANCE)
            
            # 收集目标区域内的点
            roi_points = []
            for y, x in enumerate(final_border):
                if anchor_y <= y <= base_y and x != -1:
                    # 计算中心线点
                    center_x_path = x + CENTER_LINE_OFFSET
                    if 0 <= center_x_path < roi_w:
                        roi_points.append((center_x_path, y))
                        # 绘制区域内的中心线点（青色）
                        cv2.circle(roi_display, (center_x_path, y), 2, (255, 255, 0), -1)
            
            # 计算error
            error = 0.0
            if roi_points:
                avg_x = sum(p[0] for p in roi_points) / len(roi_points)
                error = avg_x - (roi_w // 2)
                
                # 【新增刹车逻辑】检查是否在死区内外发生切换，如果是则刹车
                is_in_deadzone = abs(error) <= ERROR_DEADZONE_PIXELS
                if self.was_in_deadzone is not None and self.was_in_deadzone != is_in_deadzone:
                    rospy.loginfo("状态 %s: 切换行驶模式(直行/转向)，刹车...", str(self.current_state))
                    self.stop()
                self.was_in_deadzone = is_in_deadzone # 更新上一帧的状态
                
                # --- 原有的PID控制逻辑 ---
                # 初始化最终速度变量
                final_linear_x = 0.0
                final_angular_z_rad = 0.0

                # 检查误差是否超出死区
                if abs(error) > ERROR_DEADZONE_PIXELS:
                    # 状态：原地旋转以修正方向
                    final_linear_x = 0.0
                    
                    # 计算PID控制器的输出
                    p_term = Kp * error
                    self.integral += error
                    i_term = Ki * self.integral
                    derivative = error - self.last_error
                    d_term = Kd * derivative
                    self.last_error = error
                    steering_angle = p_term + i_term + d_term
                    
                    # 计算角速度并进行限幅
                    angular_z_rad = -1 * steering_angle * STEERING_TO_ANGULAR_VEL_RATIO
                    final_angular_z_rad = np.clip(angular_z_rad, -self.max_angular_speed_rad, self.max_angular_speed_rad)
                
                else:
                    # 状态：方向正确，直线前进
                    final_linear_x = LINEAR_SPEED
                    final_angular_z_rad = 0.0
                    # 重置PID积分项和last_error
                    self.integral = 0.0
                    self.last_error = 0.0
                
                # 创建并发布速度指令
                twist_msg = Twist()
                twist_msg.linear.x = final_linear_x
                twist_msg.angular.z = final_angular_z_rad
                self.cmd_vel_pub.publish(twist_msg)
                
                # 将最终角速度转换为度/秒用于打印
                final_angular_deg = np.rad2deg(final_angular_z_rad)
                
                # 找到并绘制胡萝卜点
                if final_border[anchor_y] != -1:
                    carrot_x = final_border[anchor_y] + CENTER_LINE_OFFSET
                    if 0 <= carrot_x < roi_w:
                        cv2.drawMarker(roi_display, (carrot_x, anchor_y), 
                                     (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            
                # 按指定频率打印error、线速度和角速度
                current_time = time.time()
                if current_time - self.last_print_time >= 1.0 / PRINT_HZ:
                    rospy.loginfo("状态: %s | Error: %7.2f | Linear_x: %.2f | Angular_z: %7.2f deg/s", 
                                str(self.current_state), error, final_linear_x, final_angular_deg)
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