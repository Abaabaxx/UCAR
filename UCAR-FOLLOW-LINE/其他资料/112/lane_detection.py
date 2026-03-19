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
#import cv2
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
from radar3 import ObstacleFilter
import os
from std_srvs.srv import Empty

# 相机内参矩阵，用于校正图像畸变
# 修改这些参数会影响图像校正效果，应根据实际相机标定结果设置
intrinsicMat = np.array([[489.3828, 0.8764, 297.5558],
                         [0, 489.8446, 230.0774],
                         [0, 0, 1]])
# 相机畸变系数，用于校正图像畸变
distortionCoe = np.array([-0.4119, 0.1709, 0, 0.0011, 0.018])

# 透视变换点，定义了从原始图像到鸟瞰图的变换
# src_pts: 原始图像中的四个点（通常是车道线区域的四个角点）
# 修改这些点会改变透视变换效果，影响车道线检测准确性
src_pts = np.float32([[159, 380], [1, 460], [637, 449], [505, 380]])
# dst_pts: 变换后图像中对应的四个点
dst_pts = np.float32([[70, 0], [70, 480], [570, 480], [570, 0]])

# 全局变量，用于存储和跟踪PID控制中的历史数据
c0, c1 = 0, 0  # 车道线拟合参数
prev_center_offset = 0  # 上一帧的中心偏移量
prev_angle = 0  # 上一帧的角度
prev_lateral_offset = 0  # 上一帧的横向偏移量

class LaneDetector:
    def __init__(self):
        # 初始化CvBridge，用于ROS图像和OpenCV图像的转换
        self.bridge = CvBridge()
        
        # PID控制器参数
        # 这些参数直接影响车辆的控制响应
        self.kp_angle = 0.05      # 角度比例项: 增大会使转向更敏感，减小会使转向更平缓
        self.kd_angle = 0.02      # 角度微分项: 增大会抑制振荡但可能导致响应迟钝，减小会使响应更快但可能增加振荡
        self.kp_offset = 0.03     # 偏移比例项: 增大会使车辆更快回到中心，减小会使车辆更平稳但偏移修正较慢
        self.kd_offset = 0.01     # 偏移微分项: 增大会抑制横向振荡，减小会使横向响应更快但可能导致振荡
        self.kp_lateral = 0.04    # 横向移动比例项: 增大会使横向移动更敏感，减小会使横向移动更平缓
        self.kd_lateral = 0.02    # 横向移动微分项: 增大会抑制横向振荡，减小会使横向响应更快但可能导致振荡
        
        # 检测参数
        self.peak_thresh = 15    # 峰值阈值: 增大会减少噪声检测，但可能丢失弱车道线；减小会增加检测灵敏度，但可能引入噪声
        self.image_width = 640   # 图像宽度
        self.image_center = 320  # 图像中心点的x坐标
        
        # 麦克纳姆轮速度参数
        self.default_linear_x = 0.3   # 默认前进速度: 增大会使车辆行驶更快，减小会使车辆行驶更慢
        self.min_linear_x = 0.1       # 最小前进速度: 设置安全最低速度，增大会在修正时保持更高速度
        self.max_linear_x = 0.5       # 最大前进速度: 设置安全最高速度，减小会限制车辆最高速度提高安全性
        self.max_linear_y = 0.3       # 最大横向速度: 增大会允许更快的横向修正，减小会使横向移动更平缓安全
        self.max_angular_z = 0.8      # 最大角速度: 增大会允许更快的转向，减小会限制转向速度提高稳定性
        
        # 控制参数
        self.lateral_control_thresh = 40  # 横向控制阈值: 增大会减少横向控制的触发，减小会增加横向控制的使用频率
        self.angle_control_thresh = 10    # 角度控制阈值: 增大会减少旋转控制的触发，减小会增加旋转控制的使用频率
        
        # ROS 节点设置
        rospy.init_node('lane_detection', anonymous=True)
        # 发布器：发布速度指令
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 订阅器：接收相机图像
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
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

    def lane_detection(self, img):
        global c0, c1, prev_center_offset, prev_angle, prev_lateral_offset
        
        # 1. 图像预处理 - 校正畸变
        # 使用相机内参和畸变参数校正图像
        corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
        
        # 2. 转换为灰度图并应用高斯模糊
        # 灰度图减少计算量，高斯模糊减少噪点
        gray_ex = cv2.cvtColor(corr_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray_ex, (5, 5), 0)  # 高斯核大小为5x5
        
        # 3. 阈值分割提取白色车道线
        # 使用二值化操作提取亮度高的区域（白色车道线）
        # 阈值200是提取白色区域的亮度阈值，可根据光照条件调整
        # 增大阈值：只检测更亮的区域，减少噪声但可能丢失部分车道线
        # 减小阈值：检测更多的白色区域，但可能引入更多噪声
        _, binary_white = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
        
        # 4. Canny边缘检测
        # 边缘检测用于识别车道线边缘
        # 第二个参数(50)为低阈值，第三个参数(150)为高阈值
        # 增大阈值：检测更显著的边缘，降低噪声，但可能丢失一些车道线边缘
        # 减小阈值：检测更多边缘，但会引入更多噪声
        canny_edges = cv2.Canny(blurred, 50, 150)
        
        # 5. 合并结果，只保留白色车道线
        # 使用位操作合并白色区域和边缘检测结果
        combined_output = cv2.bitwise_or(binary_white, canny_edges)
        
        # 6. 透视变换（鸟瞰图）
        # 将图像转换为俯视图，便于车道线分析
        transform_matrix = self.perspective_transform(src_pts, dst_pts)
        warped_image = self.birdView(combined_output*1.0, transform_matrix['M'])
        
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
        
        # lane_center: 车道中心位置
        # center_offset: 车辆相对车道中心的偏移量
        # 大于0：车辆位于车道中心右侧，应该向左移动
        # 小于0：车辆位于车道中心左侧，应该向右移动
        lane_center = (centroid_top['centroid'] + centroid_bottom['centroid']) / 2
        center_offset = lane_center - self.image_center
        
        # 12. 创建Twist消息，用于发布速度指令
        twist = Twist()
        
        # 13. 初始化麦克纳姆轮控制值
        linear_x = self.default_linear_x  # 前进速度
        linear_y = 0.0                    # 横向速度（麦克纳姆轮特有）
        angular_z = 0.0                   # 旋转速度
        
        # 14. 车道检测逻辑
        # 判断是否检测到了清晰的车道线
        # 条件1: 车道倾斜度小于阈值(80)
        # 条件2和3: 顶部和底部的车道线检测强度足够高
        if math.fabs(lane_offset) < 80 and centroid_top['intensity'] > 0 and centroid_bottom['intensity'] > 0:
            # 15. 双车道检测情况
            rospy.loginfo('双车道检测 - 车道偏移: %f, 中心偏移: %f', lane_offset, center_offset)
            
            # 16. 麦克纳姆轮控制策略 - 使用横向移动和旋转组合
            # 计算角度控制分量（用于旋转）
            # kp_angle：角度比例系数，控制旋转响应强度
            # 增大：使车辆转向更敏感，但可能导致振荡
            # 减小：使车辆转向更平稳，但响应更慢
            # kd_angle：角度微分系数，抑制振荡
            # 增大：减少振荡，但可能导致响应迟钝
            # 减小：响应更快，但可能增加振荡
            angular_component = self.kp_angle * (-lane_offset/100.0) + self.kd_angle * ((-lane_offset/100.0) - prev_angle)
            prev_angle = -lane_offset/100.0
            
            # 17. 计算横向控制分量（用于横向移动）
            # kp_lateral：横向比例系数，控制横向移动强度
            # kd_lateral：横向微分系数，抑制横向振荡
            lateral_component = self.kp_lateral * (-center_offset/100.0) + self.kd_lateral * ((-center_offset/100.0) - prev_lateral_offset)
            prev_lateral_offset = -center_offset/100.0
            
            # 18. 麦克纳姆轮能力：分离横向移动和旋转
            # 当横向偏移超过阈值时，主要使用横向移动进行修正
            if abs(center_offset) > self.lateral_control_thresh:
                # lateral_control_thresh：横向控制阈值
                # 增大：减少横向移动的触发频率，使用更多旋转控制
                # 减小：增加横向移动的使用，减少旋转控制
                linear_y = lateral_component
                # 根据偏移程度减小前进速度，提高安全性
                linear_x = max(self.min_linear_x, self.default_linear_x - 0.1 * abs(center_offset) / 100.0)
            
            # 19. 当角度偏移超过阈值时，使用旋转修正
            if abs(lane_offset) > self.angle_control_thresh:
                # angle_control_thresh：角度控制阈值
                # 增大：减少旋转控制的触发，使车辆控制更平稳但可能跟踪不够准确
                # 减小：增加旋转控制的频率，提高跟踪准确性但可能导致振荡
                angular_z = angular_component
            
            # 20. 组合控制：横向移动 + 轻微旋转以保持对齐
            # 当横向和角度偏移都较小时，使用轻微的组合控制
            if abs(center_offset) <= self.lateral_control_thresh and abs(lane_offset) <= self.angle_control_thresh:
                # 保持正常前进速度
                linear_x = self.default_linear_x
                # 使用减弱的横向和旋转控制
                linear_y = lateral_component * 0.5  # 减小横向修正
                angular_z = angular_component * 0.5  # 减小旋转修正
        else:
            # 21. 只有一条车道线或未明确检测到车道线的情况
            if lines is not None:
                # 22. 通过所有检测到的线拟合直线
                all_x = []
                all_y = []
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        all_x.extend([x1, x2])
                        all_y.extend([y1, y2])
                
                # 23. 如果有足够的点，拟合直线
                if len(all_x) >= 2:
                    # 使用多项式拟合计算直线参数
                    # c1：直线斜率
                    # c0：直线截距
                    c1, c0 = np.polyfit(all_x, all_y, 1)
                    
                    # 24. 计算线角度和消失点
                    # line_angle：直线与水平线的夹角（度）
                    # 正值：向右倾斜，需要左转
                    # 负值：向左倾斜，需要右转
                    line_angle = math.atan(c1) * 180 / math.pi
                    
                    # vanishing_point_x：消失点的x坐标
                    # > 320：消失点在右侧，车道偏左
                    # < 320：消失点在左侧，车道偏右
                    vanishing_point_x = (480-c0)/c1 if c1 != 0 else 320
                    
                    # 25. 计算横向偏移距离
                    lateral_offset = vanishing_point_x - 320
                    
                    rospy.loginfo('线角度: %f 度, 消失点: %f, 横向偏移: %f', line_angle, vanishing_point_x, lateral_offset)
                    
                    # 26. 麦克纳姆轮特殊控制模式
                    # 当横向偏移显著时，使用横向移动能力
                    if abs(lateral_offset) > self.lateral_control_thresh:
                        # 计算横向控制量
                        lateral_component = -self.kp_lateral * (lateral_offset/100.0)
                        linear_y = lateral_component
                        # 根据线角度调整前进速度
                        linear_x = max(self.min_linear_x, self.default_linear_x - 0.05 * abs(line_angle) / 20.0)
                        rospy.loginfo('麦克纳姆横向修正: %f', linear_y)
                    
                    # 27. 角度控制
                    # 使用PID控制计算旋转速度
                    angle_correction = self.kp_angle * (-line_angle/45.0) + self.kd_angle * ((-line_angle/45.0) - prev_angle)
                    prev_angle = -line_angle/45.0
                    angular_z = angle_correction
                    
                    # 28. 根据消失点位置记录日志
                    if vanishing_point_x < 320:  # 左侧
                        rospy.loginfo('左车道 - 麦克纳姆调整 - 横向: %f, 角速度: %f', linear_y, angular_z)
                    elif vanishing_point_x > 320:  # 右侧
                        rospy.loginfo('右车道 - 麦克纳姆调整 - 横向: %f, 角速度: %f', linear_y, angular_z)
                    
                    # 29. 安全检查：检查极端斜率
                    # 当线角度很大时，表示可能需要急转弯
                    if abs(line_angle) > 60:
                        # 减速并增加修正力度
                        linear_x = self.min_linear_x
                        angular_z = angular_z * 1.5
                else:
                    # 30. 当线拟合失败时的回退策略
                    linear_x = self.min_linear_x
                    angular_z = 0.0
                    linear_y = 0.0
            else:
                # 31. 未检测到线时的安全措施
                linear_x = self.min_linear_x
                angular_z = 0.0
                linear_y = 0.0
        
        # 32. 限制最大速度，确保安全
        # 设置前进速度上下限
        linear_x = max(min(linear_x, self.max_linear_x), self.min_linear_x)
        # 设置横向速度上下限
        linear_y = max(min(linear_y, self.max_linear_y), -self.max_linear_y)
        # 设置旋转速度上下限
        angular_z = max(min(angular_z, self.max_angular_z), -self.max_angular_z)
        
        # 33. 设置Twist消息 - 麦克纳姆轮的全向能力
        twist.linear.x = linear_x   # 前进/后退
        twist.linear.y = linear_y   # 左/右横向移动（麦克纳姆轮特有）
        twist.angular.z = angular_z # 旋转
        
        # 34. 发布Twist消息
        self.vel_pub.publish(twist)
        
        # 35. 日志记录
        rospy.loginfo('麦克纳姆轮控制 - 前进: %f, 横向: %f, 旋转: %f', linear_x, linear_y, angular_z)
        
        # 36. 可视化（可选）
        if self.image_pub.get_num_connections() > 0:
            try:
                # 在图像上绘制检测结果并发布
                vis_img = corr_img.copy()
                
                # 绘制鸟瞰图转换区域
                cv2.polylines(vis_img, [np.int32(src_pts)], True, (0, 255, 0), 2)
                
                # 绘制检测到的线（如果有）
                if lines is not None:
                    # 将检测到的线投影回原始图像
                    for line in lines:
                        for x1, y1, x2, y2 in line:
                            # 在原始图像上绘制霍夫线
                            pts = np.array([[x1, y1], [x2, y2]], np.float32)
                            pts = cv2.perspectiveTransform(pts.reshape(1, 2, 2), transform_matrix['Minv']).reshape(2, 2)
                            cv2.line(vis_img, (int(pts[0][0]), int(pts[0][1])), 
                                   (int(pts[1][0]), int(pts[1][1])), (0, 0, 255), 2)
                
                # 绘制控制信息
                cv2.putText(vis_img, f"X: {linear_x:.2f}, Y: {linear_y:.2f}, R: {angular_z:.2f}", 
                          (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # 转换为ROS图像并发布
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis_img, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr("CvBridge错误: {0}".format(e))

if __name__ == '__main__':
    try:
        detector = LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
