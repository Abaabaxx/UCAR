#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse
from collections import Counter
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
LOOKAHEAD_DISTANCE = 40  # 胡萝卜点与基准点的距离（像素）
PRINT_HZ = 4  # 打印error的频率（次/秒）
# 路径规划参数
CENTER_LINE_OFFSET = -47  # 从右边线向左偏移的像素数
# PID控制器参数
Kp = 1  # 比例系数
Ki = 0.0   # 积分系数
Kd = 0.5   # 微分系数
# 速度控制参数
LINEAR_SPEED = 0.12  # 前进速度 (m/s)
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

# 定义沿墙走的搜索模式（Follow The Wall）
FTW_SEEDS = [
    (-1, 0),    # 左
    (-1, -1),   # 左上
    (0, -1),    # 上
    (1, -1),    # 右上
    (1, 0),     # 右
    (1, 1),     # 右下
    (0, 1),     # 下
    (-1, 1)     # 左下
]

def follow_the_wall(image, start_point):
    """
    使用沿墙走(Follow The Wall)算法跟踪右侧边界
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
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 初始化PID和打印相关的状态变量
        self.integral = 0.0
        self.last_error = 0.0
        self.last_print_time = time.time()
        
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
        
        # 创建运行状态控制服务
        self.run_service = rospy.Service('/follow_line/run', SetBool, self.handle_set_running)
        
        rospy.loginfo("已创建图像订阅者和调试图像发布者，等待图像数据...")

    def calculate_overlap_rate(self, planned_x_coords):
        """
        计算路径中重合点的比例，并直接返回这个比率
        """
        total_points_in_path = len(planned_x_coords)
        
        # 如果路径点太少，则返回0.0表示无重合
        if total_points_in_path < 20:
            return 0.0
        
        x_counts = Counter(planned_x_coords)
        
        num_overlapping_points = 0
        for count in x_counts.values():
            if count > 1:
                num_overlapping_points += count
                
        overlap_rate = float(num_overlapping_points) / total_points_in_path
        
        return overlap_rate

    def handle_set_running(self, request):
        """
        处理运行状态切换请求
        """
        self.is_running = request.data
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
        right_start_point = None
        current_scan_y = None

        # 从底部开始，每隔START_POINT_SCAN_STEP个像素向上扫描，寻找右边线起始点
        for y in range(roi_h - 1, 0, -START_POINT_SCAN_STEP):
            for x in range(center_x, roi_w - 1):
                if binary_roi_frame[y, x] == 0 and binary_roi_frame[y, x + 1] == 255:
                    right_start_point = (x + 1, y)
                    current_scan_y = y
                    break
            
            if right_start_point is not None:
                break
        
        # 可视化：画出扫描线和找到的起始点
        if current_scan_y is not None:
            cv2.line(roi_display, (0, current_scan_y), (roi_w, current_scan_y), (255, 0, 0), 1)
        
        if right_start_point:
            # 在右起始点画一个红色的圆
            cv2.circle(roi_display, right_start_point, 5, (0, 0, 255), -1)
            
            # 使用沿墙走算法寻找右边界
            right_points = follow_the_wall(binary_roi_frame, right_start_point)
            
            # 提取最终的右边线
            final_right_border = None
            if right_points:
                final_right_border = extract_final_border(roi_h, right_points)
            
            # 如果成功提取到右边线，计算error
            if final_right_border is not None:
                # --- 1. 生成唯一的、正确的“规划路径”点集 (40像素范围) ---
                # 确定基准点和锚点行
                base_y = right_start_point[1]
                anchor_y = max(0, base_y - LOOKAHEAD_DISTANCE)

                # 收集目标区域内的点
                planned_path_points = [] # 使用一个更清晰的命名
                for y, x in enumerate(final_right_border):
                    if anchor_y <= y <= base_y and x != -1:
                        center_x = x + CENTER_LINE_OFFSET
                        if 0 <= center_x < roi_w:
                            planned_path_points.append((center_x, y))
                            cv2.circle(roi_display, (center_x, y), 2, (255, 255, 0), -1)

                # --- 2. 基于“规划路径”进行所有分析 ---
                # 初始化所有将要计算的变量
                error = 0.0
                final_linear_x = 0.0
                final_angular_deg = 0.0
                overlap_rate = 0.0 # 初始化重合率

                if planned_path_points:
                    # --- 2a. 计算重合率 ---
                    # 从规划路径点集中提取X坐标
                    planned_x_coords = [p[0] for p in planned_path_points]
                    overlap_rate = self.calculate_overlap_rate(planned_x_coords)
                    
                    # --- 2b. 计算Error ---
                    avg_x = sum(planned_x_coords) / len(planned_path_points)
                    error = avg_x - (roi_w // 2)
                    
                    # --- 2c. 执行PID停-转-走逻辑 ---
                    final_angular_z_rad = 0.0
                    if abs(error) > ERROR_DEADZONE_PIXELS:
                        final_linear_x = 0.0
                        p_term = Kp * error
                        self.integral += error
                        i_term = Ki * self.integral
                        derivative = error - self.last_error
                        d_term = Kd * derivative
                        self.last_error = error
                        steering_angle = p_term + i_term + d_term
                        angular_z_rad = -1 * steering_angle * STEERING_TO_ANGULAR_VEL_RATIO
                        final_angular_z_rad = np.clip(angular_z_rad, -self.max_angular_speed_rad, self.max_angular_speed_rad)
                    else:
                        final_linear_x = LINEAR_SPEED
                        final_angular_z_rad = 0.0
                        self.integral, self.last_error = 0.0, 0.0
                    
                    final_angular_deg = np.rad2deg(final_angular_z_rad)
                    
                    # --- 2d. 绘制胡萝卜点 ---
                    if final_right_border[anchor_y] != -1:
                        carrot_x = final_right_border[anchor_y] + CENTER_LINE_OFFSET
                        if 0 <= carrot_x < roi_w:
                            cv2.drawMarker(roi_display, (carrot_x, anchor_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

                # --- 3. 统一日志输出 ---
                current_time = time.time()
                if current_time - self.last_print_time >= 1.0 / PRINT_HZ:
                    rospy.loginfo("Error: %7.2f | Linear_x: %.2f | Angular_z: %7.2f deg/s | Overlap: %.2f", 
                                error, final_linear_x, final_angular_deg, overlap_rate)
                    self.last_print_time = current_time
        
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
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("节点已关闭。") 