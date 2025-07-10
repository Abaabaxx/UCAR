#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
"""
使用说明：

1. 查看二维码原始内容：
   rostopic echo /QR/result
   # 显示识别到的二维码原始文本

2. 查看解析后的任务类型：
   rostopic echo /QR/task_type
   # 显示解析得到的任务类型（fruits/vegetables/desserts/unknown）

3. 查看完整的日志信息：
   # 使用 rqt_console 或 直接查看终端输出
   # 包含识别结果和解析过程的详细信息
   
注意：确保先启动：
1. USB摄像头节点：
   roslaunch usb_cam usb_cam-test.launch
2. 二维码识别节点：
   rosrun your_package QR_detect.py

作者：abaabaxxx
创建时间：2025-04-30 21:20:31
"""

# 新增: 任务类型常量类
class TaskType:
    FRUITS = "fruits"
    VEGETABLES = "vegetables"
    DESSERTS = "desserts"
    UNKNOWN = "unknown"

# 新增: 重构为面向对象形式的QR码检测器类
class QRCodeDetector:
    def __init__(self):
        rospy.init_node('QR_detect')
        
        self.result_pub = rospy.Publisher('/QR/result', String, queue_size=1)
        # 新增: 任务类型发布者
        self.task_pub = rospy.Publisher('/QR/task_type', String, queue_size=1)
        
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        self.bridge = CvBridge()
        
        # 新增: 初始化关键词列表
        self.init_keywords()
        
        rospy.loginfo('二维码识别节点已启动')
    
    # 新增: 关键词初始化方法
    def init_keywords(self):
        self.fruit_keywords = ['fruit', 'fruits', '水果']
        self.vegetable_keywords = ['vegetable', 'vegetables', '蔬菜']
        self.dessert_keywords = ['dessert', 'desserts', 'sweet', 'sweets', '甜品', '甜点']
    
    def image_callback(self, ros_image):
        cv2_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        cv2_image = cv2.flip(cv2_image, 1)
        
        self.detect_qrcode(cv2_image)
    
    def detect_qrcode(self, cv2_image):
        qr_codes = decode(cv2_image)
        
        for qr in qr_codes:
            qr_text = qr.data.decode('utf-8')
            rospy.loginfo('二维码识别结果为：%s', qr_text)
            
            self.result_pub.publish(qr_text)
            
            # 新增: 解析任务类型并发布
            task_type = self.parse_task_type(qr_text)
            self.task_pub.publish(task_type)
    
    # 新增: 从状态机移植的任务类型解析方法
    def parse_task_type(self, qr_content):
        content_lower = qr_content.lower()
        
        if any(keyword in content_lower for keyword in self.fruit_keywords):
            task_type = TaskType.FRUITS
            rospy.loginfo("任务类型设置为: 水果 (原始内容: %s)", qr_content)
        elif any(keyword in content_lower for keyword in self.vegetable_keywords):
            task_type = TaskType.VEGETABLES
            rospy.loginfo("任务类型设置为: 蔬菜 (原始内容: %s)", qr_content)
        elif any(keyword in content_lower for keyword in self.dessert_keywords):
            task_type = TaskType.DESSERTS
            rospy.loginfo("任务类型设置为: 甜品 (原始内容: %s)", qr_content)
        else:
            task_type = TaskType.UNKNOWN
            rospy.logwarn("未知的任务类型: %s", qr_content)
        
        return task_type

if __name__ == '__main__':
    try:
        # 修改: 使用类实例而不是直接运行
        detector = QRCodeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass