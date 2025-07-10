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
   # 仅在任务类型变化时输出日志信息
   
注意：确保先启动：
1. USB摄像头节点：
   roslaunch usb_cam usb_cam-test.launch
2. 二维码识别节点：
   rosrun your_package QR_detect.py

更新说明：
- 优化日志输出：仅在任务类型改变时输出日志
- 话题持续发布最新识别结果
- 缓存最新的识别结果

作者：abaabaxxx
创建时间：2025-05-01 14:31:42
"""

class TaskType:
    FRUITS = "fruits"
    VEGETABLES = "vegetables"
    DESSERTS = "desserts"
    UNKNOWN = "unknown"

class QRCodeDetector:
    def __init__(self):
        rospy.init_node('QR_detect')
        
        self.result_pub = rospy.Publisher('/QR/result', String, queue_size=1)
        self.task_pub = rospy.Publisher('/QR/task_type', String, queue_size=1)
        
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        self.bridge = CvBridge()
        
        # 缓存最新的识别结果
        self.latest_qr_text = ""
        self.latest_task_type = ""
        # 记录上一次发布的任务类型（仅用于日志输出控制）
        self.last_logged_task_type = ""
        
        # 初始化关键词列表
        self.init_keywords()
        
        rospy.loginfo('二维码识别节点已启动')
    
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
            self.latest_qr_text = qr.data.decode('utf-8')
            self.latest_task_type = self.parse_task_type(self.latest_qr_text)
            
            # 持续发布话题
            self.result_pub.publish(self.latest_qr_text)
            self.task_pub.publish(self.latest_task_type)
            
            # 只在任务类型改变时输出日志
            if self.latest_task_type != self.last_logged_task_type:
                rospy.loginfo('二维码识别结果为：%s', self.latest_qr_text)
                rospy.loginfo('任务类型变更为: %s (原始内容: %s)', 
                             self.get_task_type_name(self.latest_task_type), 
                             self.latest_qr_text)
                self.last_logged_task_type = self.latest_task_type
    
    def parse_task_type(self, qr_content):
        content_lower = qr_content.lower()
        
        if any(keyword in content_lower for keyword in self.fruit_keywords):
            return TaskType.FRUITS
        elif any(keyword in content_lower for keyword in self.vegetable_keywords):
            return TaskType.VEGETABLES
        elif any(keyword in content_lower for keyword in self.dessert_keywords):
            return TaskType.DESSERTS
        else:
            return TaskType.UNKNOWN
    
    def get_task_type_name(self, task_type):
        task_names = {
            TaskType.FRUITS: "水果",
            TaskType.VEGETABLES: "蔬菜",
            TaskType.DESSERTS: "甜品",
            TaskType.UNKNOWN: "未知"
        }
        return task_names.get(task_type, "未知")

if __name__ == '__main__':
    try:
        detector = QRCodeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass