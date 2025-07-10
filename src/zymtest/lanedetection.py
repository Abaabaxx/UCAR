#!/usr/bin/env python
# -*- coding: utf-8 -*- #
import rospy
import sys
import os
import math
import time
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped,PoseStamped,_Point,Pose, PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import Int32
import roslib
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import threading
import cv2
#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
#from pathlib import Path 
#from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage,LaserScan
#import roslaunch
#from std_srvs.srv import Trigger
import tf
#import moveit_commander
#from std_srvs.srv import Empty
import random
from pydub import AudioSegment  #播放模块
from pydub.playback import play  #播放模块
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes,ObjectCount
# from radar3 import ObstacleFilter
import os
from std_srvs.srv import Empty

# 相机内参矩阵，用于校正图像畸变[7](@ref)
# 修改参数将影响畸变校正精度，需与相机标定参数匹配
intrinsicMat = np.array([[489.3828, 0.8764, 297.5558],
                         [0, 489.8446, 230.0774],
                         [0, 0, 1]])

# 透视变换点配置（影响鸟瞰图转换效果）[6](@ref)
src_pts = np.float32([[114, 225], [10 , 310], [638, 310], [563, 225]])  # 源图像坐标
dst_pts = np.float32([[70, 0], [70, 480], [570, 480], [570, 0]])        # 目标鸟瞰图坐标

distortionCoe = np.array([-0.4119,0.1709,0,0.0011, 0.018])

# PID控制历史数据（影响控制平滑性）
c0, c1 = 0, 0              # 车道线拟合参数
prev_center_offset = 0      # 中心偏移量历史值
prev_angle = 0              # 角度历史值

class LaneDetector:
    def __init__(self):
        # 初始化CvBridge，用于ROS图像和OpenCV图像的转换
        self.bridge = CvBridge()
        
        # PID控制器参数
        # 这些参数直接影响车辆的控制响应
        self.kp_angle = 0.5  # 降低比例系数，减少过度修正
        self.kd_angle = 0.1  # 增加微分系数，提高稳定性
        self.ki_angle = 0.01  # 大幅降低积分系数，防止积分饱和
        
        # 新增PID速度控制参数
        self.kp_speed = 0.01     # 速度比例项
        self.ki_speed = 0.002    # 速度积分项
        self.kd_speed = 0.005    # 速度微分项
        
        # 检测参数
        self.peak_thresh = 15    # 峰值阈值: 增大会减少噪声检测，但可能丢失弱车道线；减小会增加检测灵敏度，但可能引入噪声
        self.image_width = 640   # 图像宽度
        self.image_center = 320  # 图像中心点的x坐标
        
        # 麦克纳姆轮速度参数
        self.default_linear_x = 0.25   # 默认前进速度
        self.min_linear_x = 0.2      # 最小前进速度
        self.max_linear_x = 0.4       # 最大前进速度
        self.max_linear_y = 0.3       # 最大横向速度: 增大会允许更快的横向修正，减小会使横向移动更平缓安全
        self.max_angular_z = 0.5      # 降低最大角速度，提高稳定性
        
        # 控制参数
        self.lateral_control_thresh = 40  # 横向控制阈值
        self.angle_control_thresh = 10    # 角度控制阈值
        self.end_dist_thresh = 5         # 终点判断阈值
        
        # PID控制辅助参数
        self.deadzone = 15           # 增大死区，忽略微小偏移
        self.integral_limit = 50     # 减小积分限幅，防止积分累积过大
        self.error_threshold = 40    # 增大积分分离阈值
        self.soft_limit_lower = -0.3 # 减小软限幅范围
        self.soft_limit_upper = 0.3  # 减小软限幅范围
        self.filter_coefficient = 0.7 # 降低滤波系数，增加平滑性
        self.last_filtered_error = 0
        self.integral_error = 0.0
        self.last_error = 0.0
        
        # 添加直行补偿参数
        self.straight_bias = -0.05  # 左侧偏移补偿，修正向右偏转问题
        
        # 图像处理参数
        self.line_threshold = 210    # 线阈值
        self.range_y = 3             # 垂直分割比例
        self.center_p = 5            # 中心线扩展参数
        self.debug_output = True    # 调试输出
        
        # ROS 节点设置
        rospy.init_node('lane_detection', anonymous=True)
        # 发布器：发布速度指令
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 订阅器：接收相机图像
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
        # 用于可视化的图像发布器
        self.image_pub = rospy.Publisher("/lane_detection/processed_image", Image, queue_size=1)
        
        # 初始化Twist消息对象
        self.twist = Twist()
        rospy.loginfo("麦克纳姆轮车道追踪器已初始化")

    def image_callback(self, data):
        """
        相机图像回调函数
        参数:
            data: ROS图像消息
        """
        try:
            # 将ROS图像转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # 调用车道检测函数
            self.lane_detection(cv_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge错误: {0}".format(e))

    def birdView(self, img, M):
        """
        透视变换，将图像转换为鸟瞰图
        参数:
            img: 输入图像
            M: 透视变换矩阵
        返回:
            img_warped: 变换后的鸟瞰图
        
        调整M矩阵会改变透视变换效果，影响车道线检测的准确性
        """
        img_sz = (img.shape[1], img.shape[0])
        img_warped = cv2.warpPerspective(img, M, img_sz, flags=cv2.INTER_LINEAR)
        return img_warped

    def perspective_transform(self, src_pts, dst_pts):
        """
        计算透视变换矩阵
        参数:
            src_pts: 源图像中的点
            dst_pts: 目标图像中的点
        返回:
            字典，包含正向(M)和反向(Minv)变换矩阵
        
        调整src_pts和dst_pts会改变透视变换效果，影响鸟瞰图的质量和车道线检测
        """
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        Minv = cv2.getPerspectiveTransform(dst_pts, src_pts)
        return {'M': M, 'Minv': Minv}

    def find_centroid(self, image, peak_thresh, window):
        """
        在指定窗口内查找车道线中心点
        参数:
            image: 二值化图像
            peak_thresh: 峰值阈值，用于过滤噪声
            window: 窗口定义，包含x0,y0,width,height
        返回:
            (centroid, peak_intensity, hotpixels_cnt): 中心点坐标，峰值强度，热点数量
        
        增大peak_thresh会减少噪声检测，但可能忽略弱车道线
        减小peak_thresh会增加检测灵敏度，但可能引入更多噪声
        """
        # 提取窗口内的图像区域
        mask_window = image[int(window['y0']-window['height']):int(window['y0']),
                           int(window['x0']):int(window['x0']+window['width'])]
        # 计算水平方向上像素和的直方图
        histogram = np.sum(mask_window, axis=0)
        
        if np.max(histogram) > 0:
            # 找到直方图中的最大值位置作为中心点
            centroid = np.argmax(histogram)
            peak_intensity = histogram[centroid]
            hotpixels_cnt = np.sum(histogram)
            
            # 如果峰值强度低于阈值，则使用窗口中心作为中心点
            if peak_intensity <= peak_thresh:
                centroid = int(round(window['x0']+window['width']/2))
                peak_intensity = 0
            else:
                # 调整中心点坐标到原图坐标系
                centroid = int(round(centroid+window['x0']))
                
            return (centroid, peak_intensity, hotpixels_cnt)
        else:
            # 如果直方图全为0，返回窗口中心
            return (int(window['x0']+window['width']/2), 0, 0)

    def find_starter_centroids(self, image, y0, peak_thresh):
        """
        查找图像中车道线的起始中心点
        参数:
            image: 二值化图像
            y0: 窗口底部y坐标
            peak_thresh: 峰值阈值
        返回:
            字典，包含中心点坐标和峰值强度
        
        修改y0会改变检测窗口的位置，影响中心点的检测结果
        """
        # 定义初始窗口，宽度为整个图像宽度，高度为图像高度的1/5
        window = {'x0': 0, 'y0': y0, 'width': image.shape[1], 'height': int(image.shape[0]/5)}
        
        # 获取中心点
        centroid, peak_intensity, _ = self.find_centroid(image, peak_thresh, window)
        
        # 如果未找到明显的峰值，尝试使用整个图像高度作为窗口
        if peak_intensity < peak_thresh:
            window['height'] = image.shape[0]
            centroid, peak_intensity, _ = self.find_centroid(image, peak_thresh, window)
            
        return {'centroid': centroid, 'intensity': peak_intensity}

    def process_image(self, img):
        """
        处理图像，提取车道线
        """
        # 1. 图像预处理 - 校正畸变
        corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
        
        # 2. 转换为灰度图并应用高斯模糊
        gray_ex = cv2.cvtColor(corr_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray_ex, (5, 5), 0)  # 高斯核大小为5x5
        
        # 3. 阈值分割提取白色车道线
        # 使用二值化操作提取亮度高的区域（白色车道线）
        # 阈值200是提取白色区域的亮度阈值，可根据光照条件调整
        # 增大阈值：只检测更亮的区域，减少噪声但可能丢失部分车道线
        # 减小阈值：检测更多的白色区域，但可能引入更多噪声
        _, binary_white = cv2.threshold(blurred, self.line_threshold, 255, cv2.THRESH_BINARY)
        
        # 4. Canny边缘检测
        # 边缘检测用于识别车道线边缘
        # 第二个参数(50)为低阈值，第三个参数(150)为高阈值
        # 增大阈值：检测更显著的边缘，降低噪声，但可能丢失一些车道线边缘
        # 减小阈值：检测更多边缘，但会引入更多噪声
        canny_edges = cv2.Canny(blurred, 100, 150)
        
        # 5. 合并结果，只保留白色车道线
        # 使用位操作合并白色区域和边缘检测结果
        combined_output = cv2.bitwise_or(binary_white, canny_edges)
        cv2.imwrite('img1.jpg',combined_output)
        
        # 6. 透视变换（鸟瞰图）
        # 将图像转换为俯视图，便于车道线分析
        transform_matrix = self.perspective_transform(src_pts, dst_pts)
        warped_image = self.birdView(combined_output*1.0, transform_matrix['M'])
        # cv2.imwrite('img2.jpg',warped_image)
        
        # 7. 应用形态学操作增强车道线
        # 膨胀操作：增加白色区域，连接断开的车道线
        # 增大核大小：连接更远的断点，但会使车道线变粗
        # 减小核大小：精确度更高，但对断点的连接能力降低
        kernel_dilate = np.ones((15, 15), np.uint8)
        warped_image = cv2.dilate(warped_image, kernel_dilate)
        
        # 腐蚀操作：减少白色区域，去除噪点
        # 增大核大小：去除更多噪点，但可能使车道线变细或断开
        # 减小核大小：保留更多细节，但噪点去除效果减弱
        kernel_erode = np.ones((7, 7), np.uint8)
        warped_image = cv2.erode(warped_image, kernel_erode)
        cv2.imwrite('img2.jpg',warped_image)
        
        # 8. 霍夫线变换检测直线
        # 将处理后的图像转换为CV_8U类型
        HoughLine_image = np.array(warped_image, np.uint8)
        
        # 霍夫线变换参数说明：
        # minLineLength：最小线段长度，小于此长度的线段将被忽略
        # maxLineGap：允许的最大间隙，小于此值的间隙会被视为同一条线
        # 增大阈值(70)：只检测更明显的线，减少噪声，但可能丢失一些车道线
        # 减小阈值：检测更多可能的线，但会引入噪声
        lines = cv2.HoughLinesP(HoughLine_image, 1, np.pi/180, 70, 
                              minLineLength=80, maxLineGap=70)
        
        # 9. 裁剪底部区域以便更好地分析
        # 去除图像底部可能的噪声，专注于分析更远处的车道
        bottom_crop = -40
        cropped_warped = warped_image[0:bottom_crop, :]
        
        # 10. 计算车道线中心点
        # 在图像底部查找车道线的中心位置
        centroid_bottom = self.find_starter_centroids(cropped_warped, 
                                                     y0=cropped_warped.shape[0], 
                                                     peak_thresh=self.peak_thresh)
        
        # 在图像顶部查找车道线的中心位置
        centroid_top = self.find_starter_centroids(cropped_warped, 
                                                 y0=cropped_warped.shape[0]//5, 
                                                 peak_thresh=self.peak_thresh)
        
        # 11. 计算车道偏移和中心偏移
        # lane_offset: 车道倾斜程度，用于判断转弯
        # 大于0：车道左倾，应该向右转
        # 小于0：车道右倾，应该向左转
        lane_offset = centroid_top['centroid'] - centroid_bottom['centroid']
        return lane_offset
        print("<<<<lane_offset:",lane_offset)
        
        # 12. 创建Twist消息，用于发布速度指令
        # twist = Twist()
        
        # # 仅根据lane_offset控制速度
        # # 线性速度x根据偏移量减小，偏移越大速度越慢
        # linear_x = self.max_linear_x - min(abs(lane_offset) / 100.0, 1.0) * (self.max_linear_x - self.min_linear_x)
        # # 横向速度y与lane_offset成比例，偏移为正向右，负向左
        # linear_y = -0.01 * lane_offset  # 系数可根据实际调整
        # # 角速度z与lane_offset成比例，偏移为正向右转，负向左转
        # angular_z = -0.01 * lane_offset  # 系数可根据实际调整
        
        # # 限制最大最小速度，确保安全
        # linear_x = max(min(linear_x, self.max_linear_x), self.min_linear_x)
        # linear_y = max(min(linear_y, self.max_linear_y), -self.max_linear_y)
        # angular_z = max(min(angular_z, self.max_angular_z), -self.max_angular_z)
        
        # # 设置Twist消息
        # twist.linear.x = linear_x
        # twist.linear.y = linear_y
        # twist.angular.z = angular_z
        
        # # 发布Twist消息
        # self.vel_pub.publish(twist)
        
        # # 日志记录
        # rospy.loginfo('简化控制 - 车道偏移: %f, 前进: %f, 横向: %f, 旋转: %f', lane_offset, linear_x, linear_y, angular_z)
        
        # # 可视化（可选）
        # if self.image_pub.get_num_connections() > 0:
        #     try:
        #         vis_img = corr_img.copy()
        #         cv2.polylines(vis_img, [np.int32(src_pts)], True, (0, 255, 0), 2)
        #         self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis_img, "bgr8"))
        #     except CvBridgeError as e:
        #         rospy.logerr("CvBridge错误: {0}".format(e))

    def follow_line(self, lane_offset):
        """
        使用PID控制跟随车道线
        """
        # 应用直行补偿，抵消向右偏转趋势
        lane_offset = lane_offset + self.straight_bias * self.image_width
        
        # 应用滤波
        filtered_error = self.filter_coefficient * lane_offset + (1 - self.filter_coefficient) * self.last_filtered_error
        self.last_filtered_error = filtered_error
        
        # 应用死区
        if abs(filtered_error) < self.deadzone:
            filtered_error = 0
        else:
            filtered_error = filtered_error - self.deadzone if filtered_error > 0 else filtered_error + self.deadzone
        
        # 自适应PID参数
        self.adapt_pid_parameters(filtered_error)
        
        # 积分分离
        if abs(filtered_error) < self.error_threshold:
            self.integral_error += filtered_error
        else:
            self.integral_error = 0
        
        # 积分限幅
        self.integral_error = max(-self.integral_limit, min(self.integral_error, self.integral_limit))
        
        # 计算微分
        derivative = filtered_error - self.last_error
        
        # 计算转向角
        steering_angle = (self.kp_angle * filtered_error + 
                         self.ki_angle * self.integral_error + 
                         self.kd_angle * derivative)
        
        # 软限幅
        steering_angle = max(self.soft_limit_lower, min(steering_angle, self.soft_limit_upper))
        
        # 根据转向角调整线性速度
        linear_speed = self.max_linear_x - min(abs(lane_offset) / 100.0, 1.0) * (self.max_linear_x - self.min_linear_x)
        linear_speed = max(self.min_linear_x, min(linear_speed, self.max_linear_x))
        
        # 设置Twist消息
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = -steering_angle
        
        # 限制角速度
        twist.angular.z = max(-self.max_angular_z, min(twist.angular.z, self.max_angular_z))
        
        # 发布控制命令
        self.vel_pub.publish(twist)
        
        # 记录当前误差为下一次的历史误差
        self.last_error = filtered_error
        
        if self.debug_output:
            direction = "直行" if abs(twist.angular.z) < 0.3 else ("左转" if twist.angular.z > 0.3 else "右转")
            print("线速度: %.2f m/s, 角速度: %.2f rad/s, 检测到: %s" % (twist.linear.x, twist.angular.z, direction))
        
        return twist

    def adapt_pid_parameters(self, error):
        """
        自适应调整PID参数
        """
        if abs(error) > self.error_threshold * 2:
            self.kp_angle = 0.015  # 增大比例系数
            self.ki_angle = 0.0025  # 减小积分系数
            self.kd_angle = 0.04   # 增大微分系数
        else:
            self.kp_angle = 0.01
            self.ki_angle = 0.005
            self.kd_angle = 0.02
    
    def stop(self):
        """
        停止车辆
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.vel_pub.publish(twist)
        if self.debug_output:
            print("停止")
        # exit(0)  # 在现实环境中可能需要移除此行

    def lane_detection(self, img):
        """
        主车道检测函数
        """
        # 处理图像，获取车道线信息
        lane_offset = self.process_image(img)
        
        # 如果检测到停止条件，则停车
        # if is_stop:
        #     self.stop()
        #     return
        
        # 使用PID控制跟随车道线
        self.follow_line(lane_offset)
        
        # # 发布处理后的图像用于可视化
        # if self.image_pub.get_num_connections() > 0:
        #     try:
        #         self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
        #     except CvBridgeError as e:
        #         rospy.logerr("CvBridge错误: {0}".format(e))

if __name__ == '__main__':
    try:
        detector = LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
