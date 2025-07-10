#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # 导入CvBridge模块，用于在ROS图像消息和OpenCV图像之间进行格式转换
import numpy as np
from pyzbar.pyzbar import decode  # 导入pyzbar模块，用于二维码识别

""" 
Note:
    二维码识别节点
    订阅：/usb_cam/image_raw
    发布：/QR/result, String
"""

# 对图像进行二维码识别并向 /QR/result 发布二维码识别结果
def detect_qrcode(cv2_image):

    # 二维码识别 with pyzbar
    for i in decode(cv2_image):
        res = i.data.decode('utf-8')
        rospy.loginfo('二维码识别结果为：%s', res)
        pub.publish(res)  # 发布二维码识别结果

# 摄像头图像回调函数，处理接收到的 ROS 图像消息并进行二维码识别。
def image_callback(ros_image):
    global pub
    bridge = CvBridge()  # 创建CvBridge图像格式转化工具的实例对象
    cv2_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")  # 将ROS图像消息转换为OpenCV格式的图像
    # 由于摄像头拍摄以后是镜像的，所以需要将图像进行翻转，否则无法识别二维码。
    cv2_image = cv2.flip(cv2_image,1)
    
    detect_qrcode(cv2_image)


if __name__ == '__main__':
    rospy.init_node('QR_detect')  # 初始化节点，注册QR_detect二维码识别节点
    rospy.loginfo('二维码识别节点已启动')
    # 记得修改订阅的话题名称！！！
    pub = rospy.Publisher('/QR/result', String, queue_size=1)  # 创建发布二维码识别结果的发布者
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)  # 创建订阅摄像头图像的订阅者
# 它会阻塞主线程，并处理所有接收到的 ROS 
    rospy.spin() #保持节点运行，直到节点被关闭。它会阻塞主线程，并处理所有接收到的 ROS 消息，例如订阅话题的回调函数
