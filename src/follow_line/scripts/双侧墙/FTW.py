#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist
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
STATE_FOLLOW_RIGHT = 0          # 状态一：沿右墙巡线
STATE_TRANSITION_STRAIGHT = 1   # 状态二：直行过渡
STATE_FINAL_STOP = 2            # 状态三：最终停止

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
# 胡萝卜点参数
LOOKAHEAD_DISTANCE = 10  # 胡萝卜点与基准点的距离（像素）
PRINT_HZ = 4  # 打印error的频率（次/秒）
# 路径规划参数
CENTER_LINE_OFFSET_BASE = 47  # 基础偏移像素数（正值）
# PID控制器参数
Kp = 0.6  # 比例系数
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
NORMAL_AREA_HEIGHT_FROM_BOTTOM = 30  # 从ROI底部算起，被视为"常规"的区域高度（像素）
CONSECUTIVE_FRAMES_FOR_DETECTION = 3  # 连续可疑帧数，达到此值则确认进入

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
    def __init__(self):
        # 初始化运行状态
        self.is_running = False
        
        # 初始化FSM状态
        self.current_state = STATE_FOLLOW_RIGHT
        
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
        
        # 计算正向透视变换矩阵
        try:
            self.forward_perspective_matrix = np.linalg.inv(INVERSE_PERSPECTIVE_MATRIX)
        except np.linalg.LinAlgError:
            rospy.logerr("错误: 提供的矩阵是奇异矩阵, 无法求逆。请检查矩阵参数。")
            raise
        
        # 创建图像订阅者
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback)
        # 创建调试图像发布者
        self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC, Image, queue_size=1)
        # 创建速度指令发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 创建运行状态控制服务
        self.run_service = rospy.Service('/follow_line/run', SetBool, self.handle_set_running)
        
        rospy.loginfo("已创建图像订阅者和调试图像发布者，等待图像数据...")
        rospy.loginfo("当前状态: 沿右墙巡线")

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
            
        # 如果已进入最终停止状态，则持续发送停止指令并终止后续处理
        if self.current_state == STATE_FINAL_STOP:
            self.stop()
            try:
                debug_img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.debug_image_pub.publish(debug_img_msg)
            except CvBridgeError as e:
                rospy.logerr("调试图像转换或发布错误: %s", str(e))
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
        center_x = roi_w // 2
        start_point = None
        current_scan_y = None

        # 根据当前状态决定扫描哪一侧的墙
        scan_for_left_wall = (self.current_state == STATE_TRANSITION_STRAIGHT)

        # 从底部开始，每隔START_POINT_SCAN_STEP个像素向上扫描，寻找边线起始点
        for y in range(roi_h - 1, 0, -START_POINT_SCAN_STEP):
            if not scan_for_left_wall:  # 扫描右墙
                # 从中心向右扫描寻找右边线的内侧起始点
                for x in range(center_x, roi_w - 1):
                    if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x + 1] == 255:
                        start_point = (x + 1, y)
                        current_scan_y = y
                        break
            else:  # 扫描左墙
                # 从中心向左扫描寻找左边线的内侧起始点
                for x in range(center_x, 0, -1):
                    if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x - 1] == 255:
                        start_point = (x - 1, y)
                        current_scan_y = y
                        break
            
            if start_point is not None:
                break
        
        # 可视化：画出扫描线和找到的起始点
        if current_scan_y is not None:
            cv2.line(roi_display, (0, current_scan_y), (roi_w, current_scan_y), (255, 0, 0), 1)
        
        if start_point:
            # 在起始点画一个红色的圆
            cv2.circle(roi_display, start_point, 5, (0, 0, 255), -1)
            
            # 根据当前状态执行相应的逻辑
            if self.current_state == STATE_FOLLOW_RIGHT:
                # --- 状态一：沿右墙巡线 ---
                # 1. 检查状态转换条件（即检测到特殊区域）
                start_y = start_point[1]
                trigger_y_threshold = roi_h - NORMAL_AREA_HEIGHT_FROM_BOTTOM
                if start_y < trigger_y_threshold:
                    self.consecutive_special_frames += 1
                else:
                    self.consecutive_special_frames = 0

                if self.consecutive_special_frames >= CONSECUTIVE_FRAMES_FOR_DETECTION:
                    rospy.loginfo("状态转换: FOLLOW_RIGHT -> TRANSITION_STRAIGHT")
                    self.current_state = STATE_TRANSITION_STRAIGHT
                    # 立即进入直行状态，发布直行指令
                    twist_msg = Twist()
                    twist_msg.linear.x = LINEAR_SPEED
                    twist_msg.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist_msg)
                else:
                    # 2. 如果不转换，执行本状态的行为（PID巡右墙）
                    # 使用沿墙走算法寻找右边界
                    points = follow_the_wall(binary_roi_frame, start_point, FTW_SEEDS_RIGHT)
                    
                    # 提取最终的右边线
                    final_border = None
                    if points:
                        final_border = extract_final_border(roi_h, points)
                    
                    # 如果成功提取到右边线，计算error
                    if final_border is not None:
                        # 确定基准点和锚点行
                        base_y = start_point[1]
                        anchor_y = max(0, base_y - LOOKAHEAD_DISTANCE)
                        
                        # 收集目标区域内的点
                        roi_points = []
                        for y, x in enumerate(final_border):
                            if anchor_y <= y <= base_y and x != -1:
                                # 计算中心线点
                                center_x = x - CENTER_LINE_OFFSET_BASE
                                if 0 <= center_x < roi_w:
                                    roi_points.append((center_x, y))
                                    # 绘制区域内的中心线点（青色）
                                    cv2.circle(roi_display, (center_x, y), 2, (255, 255, 0), -1)
                        
                        # 计算error
                        error = 0.0
                        if roi_points:
                            avg_x = sum(p[0] for p in roi_points) / len(roi_points)
                            error = avg_x - (roi_w // 2)
                            
                            # --- 新的状态机逻辑 ---
                            # 初始化最终速度变量
                            final_linear_x = 0.0
                            final_angular_z_rad = 0.0

                            # 检查误差是否超出死区
                            if abs(error) > ERROR_DEADZONE_PIXELS:
                                # 状态：原地旋转以修正方向
                                # 线速度为0，只计算角速度
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
                                # 角速度为0，只给定线速度
                                final_linear_x = LINEAR_SPEED
                                final_angular_z_rad = 0.0
                                # 重置PID积分项和last_error，为下一次需要转向时做准备
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
                                carrot_x = final_border[anchor_y] - CENTER_LINE_OFFSET_BASE
                                if 0 <= carrot_x < roi_w:
                                    cv2.drawMarker(roi_display, (carrot_x, anchor_y), 
                                                 (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                        
                            # 按指定频率打印error、线速度和角速度
                            current_time = time.time()
                            if current_time - self.last_print_time >= 1.0 / PRINT_HZ:
                                rospy.loginfo("状态: FOLLOW_RIGHT | Error: %7.2f | Linear_x: %.2f | Angular_z: %7.2f deg/s", 
                                            error, final_linear_x, final_angular_deg)
                                self.last_print_time = current_time

            elif self.current_state == STATE_TRANSITION_STRAIGHT:
                # --- 状态二：直行过渡 ---
                # 1. 执行本状态的行为（保持直行）
                rospy.loginfo_throttle(1, "当前状态: TRANSITION_STRAIGHT, 正在直行...")
                twist_msg = Twist()
                twist_msg.linear.x = LINEAR_SPEED
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)

                # 2. 检查状态转换条件（左侧边线是否靠近底部）
                transition_y_threshold = roi_h - 10
                if start_point[1] > transition_y_threshold:
                    rospy.loginfo("状态转换: TRANSITION_STRAIGHT -> FINAL_STOP")
                    self.current_state = STATE_FINAL_STOP
                    self.stop()  # 立即停止
                
                # 可视化：绘制左侧边线
                points = follow_the_wall(binary_roi_frame, start_point, FTW_SEEDS_LEFT)
                if points:
                    for point in points:
                        cv2.circle(roi_display, point, 1, (0, 255, 255), -1)
                    
                    # 在图像上显示当前Y坐标和阈值
                    cv2.putText(roi_display, "Y: {0}, Threshold: {1}".format(start_point[1], transition_y_threshold), 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            # 如果没有找到起始点，发送停止指令
            self.stop()
        
        # 发布调试图像
        try:
            debug_img_msg = self.bridge.cv2_to_imgmsg(roi_display, "bgr8")
            self.debug_image_pub.publish(debug_img_msg)
        except CvBridgeError as e:
            rospy.logerr("调试图像转换或发布错误: %s", str(e))

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