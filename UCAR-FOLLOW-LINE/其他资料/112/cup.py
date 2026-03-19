#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import deque
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
from laser_test.msg import laser_control
from sensor_msgs.msg import LaserScan
# import jetson.inference
# import jetson.utils
import argparse
import sys
from jetauto_interfaces.msg import ObjectsInfo
import math
import os
import time
global x0 
x0 = 1
global peak_thresh
peak_thresh = 50 
global n 
n = 0
global laser_cmd
laser_cmd = 0
global msg 
msg = AckermannDriveStamped()
global flag1 
flag1 = 0
global flag2 
flag2 = 0
global flag3 
flag3 = 0
global c0,c1
global findfrontObject
findfrontObject = 0
global findrearObject
findrearObject = 0
global rightsign
rightsign = 0
global nearist
nearist = 0
global u,v,bu,bv
intrinsicMat = np.array([[489.3828, 0.8764, 297.5558],
                            [0, 489.8446, 230.0774],
                            [0, 0, 1]])
distortionCoe = np.array([-0.4119,0.1709,0,0.0011, 0.018])

global flag_laser
flag_laser = 0

# src_pts = np.float32([[128,378],[1,435],[639,435],[488,378]])#[220,306],[1,435],[639,435],[451,306]   [[132,358],[1,435],[639,435],[501,353] [160,334],[38,378],[628,378],[502,334] 
# src_pts = np.float32([[203,386],[1,435],[639,435],[463,386]])
src_pts = np.float32([[159,380],[1,460],[637,449],[505,380]])
dst_pts = np.float32([[70,0],[70,480],[570,480],[570,0]])
showMe = 0
    
def light_detection(origin_img):
    light_cmd=0
    hsv=cv2.cvtColor(origin_img,cv2.COLOR_BGR2HSV) 
    hsv1 = hsv.copy()
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    #red_lower = np.array([0,95,230])        #这两个是阈值，红灯的上界和下界
    #red_upper = np.array([5,255,255])
#
    red_lower = np.array([0,127,127])        #这两个是阈值，红灯的上界和下界
    red_upper = np.array([20,255,255])
    red_mask = cv2.inRange(hsv,red_lower,red_upper)
    red_mask = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
    #red_target = cv2.bitwise_and(hsv,hsv,mask = red_mask)
    #red_target = cv2.erode(red_target,element)
    #red_target = cv2.dilate(red_target,element)
    red_gray = cv2.cvtColor(red_mask,cv2.COLOR_BGR2GRAY)
    r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)
    r_gray2 = cv2.Canny(r_binary, 100, 200) 
    r = r_gray2[:,:] == 255
    count_red = len(r_gray2[r])
    if count_red>1500:
        redLight = 1
    else:
        redLight = 0

    print(">>>>>>>>>red ",count_red)

    green_lower = np.array([40,80,80])    
    green_upper = np.array([80,255,255])
    green_mask = cv2.inRange(hsv1,green_lower,green_upper)
    green_mask = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
    #green_target = cv2.bitwise_and(hsv1,hsv1,mask = green_mask)
    #green_target = cv2.erode(green_target,element)
    #green_target = cv2.dilate(green_target,element)
    green_gray = cv2.cvtColor(green_mask,cv2.COLOR_BGR2GRAY)
    g_ret,g_binary = cv2.threshold(green_mask,100,255,cv2.THRESH_BINARY)
    g_gray2 = cv2.Canny(g_binary, 100, 200)       
    g = g_gray2[:,:] == 255
    count_green = len(g_gray2[g])
    if count_green>1500:
       greenLight = 1
    else:
        greenLight = 0
    if (redLight ==1) and (greenLight == 0) :
        light_cmd = 0
    if greenLight == 1 :
        light_cmd = 1
    print(">>>>>>>>>green ",count_green)   
    print(light_cmd)

    return light_cmd

def birdView(img,M):

    img_sz = (img.shape[1],img.shape[0])
    img_warped = cv2.warpPerspective(img,M,img_sz,flags = cv2.INTER_LINEAR)
    return img_warped

def perspective_transform(src_pts,dst_pts):
    M = cv2.getPerspectiveTransform(src_pts,dst_pts)
    Minv = cv2.getPerspectiveTransform(dst_pts,src_pts)
    return {'M':M,'Minv':Minv}

def find_centroid(image,peak_thresh,window,showMe):
   
    mask_window = image[int(window['y0']-window['height']):int(window['y0']),
                        int(window['x0']):int(window['x0']+window['width'])]
    histogram = np.sum(mask_window,axis=0)
    centroid = np.argmax(histogram)
    hotpixels_cnt = np.sum(histogram)
    peak_intensity = histogram[centroid]
    if peak_intensity<=peak_thresh:
        centroid = int(round(window['x0']+window['width']/2))
        peak_intensity = 0
    else:
        centroid = int(round(centroid+window['x0']))
    return (centroid,peak_intensity,hotpixels_cnt)

def find_starter_centroids(image,y0,peak_thresh,showMe):

    window = {'x0':0,'y0':y0,'width':image.shape[1],'height':image.shape[0]/5}
    # get centroid
    centroid , peak_intensity,_ = find_centroid(image,peak_thresh,window,showMe)
    if peak_intensity<peak_thresh:
        window['height'] = image.shape[0]
        centroid,peak_intensity,_ = find_centroid(image,peak_thresh,window,showMe)
    return {'centroid':centroid,'intensity':peak_intensity}

def lane_detection(img):
    # Undistort image using camera calibration parameters
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    
    # Convert to grayscale for edge detection
    gray_ex = cv2.cvtColor(corr_img, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise before edge detection
    blurred = cv2.GaussianBlur(gray_ex, (5, 5), 0)
    
    # Apply Canny edge detection with optimized parameters
    combined_output = cv2.Canny(blurred, 50, 150)
    
    # Transform to bird's eye view for better lane analysis
    transform_matrix = perspective_transform(src_pts, dst_pts)
    warped_image = birdView(combined_output*1.0, transform_matrix['M'])

    # Enhance lane markings with morphological operations
    kernel_dilate = np.ones((15, 15), np.uint8)
    kernel_erode = np.ones((7, 7), np.uint8)
    warped_image = cv2.dilate(warped_image, kernel_dilate)
    warped_image = cv2.erode(warped_image, kernel_erode)
    
    # Create enhanced version for Hough line detection
    HoughLine_image = np.array(warped_image, np.uint8)
    
    # Detect lines using Hough transform with improved parameters
    lines = cv2.HoughLinesP(HoughLine_image, 1, np.pi/180, 80, minLineLength=100, maxLineGap=50)
    
    # Crop bottom part for better road analysis
    bottom_crop = -40
    warped_image = warped_image[0:bottom_crop, :]
    
    # Analyze left and right regions separately
    rightlim_image = warped_image[360:480, 400:640]
    leftlim_image = warped_image[360:480, 0:240]
    center_image = warped_image[360:480, 240:400]
    
    # Find lane centroids
    peak_thresh = 10
    showMe = 1
    
    centroid_starter_top = find_starter_centroids(warped_image, y0=warped_image.shape[0],
                                               peak_thresh=peak_thresh, showMe=showMe)
    centroid_starter_bottom = find_starter_centroids(warped_image, y0=warped_image.shape[0]/5,
                                               peak_thresh=peak_thresh, showMe=showMe)
    
    # Mecanum wheel movement parameters
    # Create a custom message for mecanum wheels (normally you'd need a custom msg type)
    # For simulation purposes, we'll adapt the existing message
    # In a real implementation, you would use a 4-wheel control message
    
    # Default values
    vx = -20  # Forward/backward speed
    vy = 0    # Lateral speed (for mecanum wheels)
    vr = 0    # Rotational speed
    
    # Calculate lane offset
    lane_offset = centroid_starter_top['centroid'] - centroid_starter_bottom['centroid']
    lane_center = (centroid_starter_top['centroid'] + centroid_starter_bottom['centroid']) / 2
    image_center = warped_image.shape[1] / 2
    center_offset = lane_center - image_center
    
    # Both lanes detected case
    if math.fabs(lane_offset) < 80:
        print('Both lanes detected - lane offset:', lane_offset)
        
        # PID-like control for mecanum wheels
        if abs(center_offset) > 30:
            # For significant deviation, use omnidirectional capability
            # Adjust lateral speed based on center offset
            vy = -center_offset / 30.0
            vx = -15  # Reduced forward speed during correction
            
            # Add small rotation to align with lane direction
            vr = -lane_offset / 80.0
            
            # Map to ackermann message format (in actual implementation, use proper mecanum control)
            msg.drive.speed = vx
            msg.drive.steering_angle = vr * 30  # Scale rotation to steering angle
        else:
            # Small deviation - mostly forward movement with minor correction
            msg.drive.speed = -20
            msg.drive.steering_angle = center_offset / 30.0
    else:
        # Only one lane or no clear lanes detected
        global c0, c1
        
        # Use detected lines to estimate lane direction
        if lines is not None:
            # Improve line fitting by considering all detected lines
            all_x = []
            all_y = []
            for line in lines:
                for x1, y1, x2, y2 in line:
                    all_x.extend([x1, x2])
                    all_y.extend([y1, y2])
            
            # Fit line to all points if enough points available
            if len(all_x) >= 2:
                c1, c0 = np.polyfit(all_x, all_y, 1)
                
                # Improved parameters for mecanum control
                k1 = 15  # Reduced from 20 for smoother control
                k2 = 0.04  # Increased from 0.03 for better responsiveness
                
                # Calculate mecanum control values based on detected line
                line_angle = math.atan(c1) * 180 / math.pi
                vanishing_point_x = (480-c0)/c1 if c1 != 0 else 320
                
                rospy.logwarn('Line angle: %s degrees, Vanishing point: %s', line_angle, vanishing_point_x)
                
                # Mecanum-specific control strategy
                if vanishing_point_x < 320:  # Left side
                    # Calculate lateral and rotational components
                    lateral_component = -k2 * (320 - vanishing_point_x)
                    rotational_component = -k1 * line_angle / 45.0
                    
                    vx = -20
                    vy = lateral_component
                    vr = rotational_component
                    
                    # Map to ackermann for compatibility
                    msg.drive.speed = vx
                    msg.drive.steering_angle = rotational_component
                    print('Left Lane - Mecanum adjusted')
                    
                elif vanishing_point_x > 320:  # Right side
                    # Calculate lateral and rotational components
                    lateral_component = k2 * (vanishing_point_x - 320)
                    rotational_component = k1 * line_angle / 45.0
                    
                    vx = -20
                    vy = lateral_component
                    vr = rotational_component
                    
                    # Map to ackermann for compatibility
                    msg.drive.speed = vx
                    msg.drive.steering_angle = rotational_component
                    print('Right Lane - Mecanum adjusted')
                
                # Additional safety: check for extreme slope
                if abs(line_angle) > 60:
                    # Very steep angle indicates potential problem
                    # Slow down and increase correction
                    msg.drive.speed = -10
                    msg.drive.steering_angle = rotational_component * 1.5
            else:
                # Fallback when line fitting fails
                msg.drive.speed = -15
                msg.drive.steering_angle = 0
        else:
            # No lines detected - safety measure
            msg.drive.speed = -10
            msg.drive.steering_angle = 0
    
    # For a real implementation with mecanum wheels, you would do:
    # mecanum_msg = MecanumDriveMsg()
    # mecanum_msg.vx = vx
    # mecanum_msg.vy = vy
    # mecanum_msg.vr = vr
    # mecanum_pub.publish(mecanum_msg)
    
    # For compatibility with existing code, publish standard ackermann message
    rospy.loginfo('Mecanum equivalent - vx: %s, vy: %s, vr: %s', vx, vy, vr)
    rospy.logerr('Ackermann cmd - speed: %s, angle: %s', msg.drive.speed, msg.drive.steering_angle)
    pub.publish(msg)
    
def blue_pixel_detection(img):

    cross_verify = 0
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # _, thresh_img = cv2.threshold(gray, thresh=200, maxval=255, type=cv2.THRESH_BINARY)
    # mask = cv2.bitwise_not(thresh_img)
    # masked_img = cv2.bitwise_and(img, img, mask=mask)
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    hsv=cv2.cvtColor(corr_img,cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv',hsv)
#提取蓝色区域
    blue_lower=np.array([100,50,130])#100 50  150
    # blue_lower=np.array([100,50,190])
    # blue_upper=np.array([124,255,255])
    #blue_lower=np.array([100,44,151])
    blue_upper=np.array([124,255,250])
    blue_mask=cv2.inRange(hsv,blue_lower,blue_upper)
   
#模糊
    blue_blurred=cv2.blur(blue_mask,(9,9))
    #cv2.imshow('blurred',blue_mask)
    cv2.waitKey(25)

#二值化
    ret,binary=cv2.threshold(blue_blurred,100,255,cv2.THRESH_BINARY)
    transform_matrix = perspective_transform(src_pts,dst_pts)
    blue_warped_image = birdView(binary*1.0,transform_matrix['M'])
    # cv2.imshow('blurred binary',blue_warped_image)
    cv2.waitKey(25)
    blue_point = cv2.countNonZero(blue_warped_image)
    return blue_point

def blue_pixel_detection_lane(img):

    cross_verify = 0
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # _, thresh_img = cv2.threshold(gray, thresh=200, maxval=255, type=cv2.THRESH_BINARY)
    # mask = cv2.bitwise_not(thresh_img)
    # masked_img = cv2.bitwise_and(img, img, mask=mask)
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    hsv=cv2.cvtColor(corr_img,cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv',hsv)
#提取蓝色区域
    blue_lower=np.array([100,50,130])#100 50  150
    # blue_lower=np.array([100,50,190])
    # blue_upper=np.array([124,255,255])
    #blue_lower=np.array([100,44,151])
    blue_upper=np.array([124,255,250])
    blue_mask=cv2.inRange(hsv,blue_lower,blue_upper)
   
#模糊
    blue_blurred=cv2.blur(blue_mask,(9,9))
    #cv2.imshow('blurred',blue_mask)
    cv2.waitKey(25)

#二值化
    ret,binary=cv2.threshold(blue_blurred,100,255,cv2.THRESH_BINARY)
    transform_matrix = perspective_transform(src_pts,dst_pts)
    blue_warped_image = birdView(binary*1.0,transform_matrix['M'])
    
    img1 = blue_warped_image[360:480,:]
    # cv2.imshow('blurred binary',blue_mask[360:480,:])
    cv2.waitKey(25)
    blue_point = cv2.countNonZero(img1)
    return blue_point

def traffic_sign_detection(img):
    global flag1
    global findfrontObject,findrearObject
    global flag2
    global sign
    global time1
    global u,v,bu,bv,bsign
    global bbluecount
    global rightsign
    global nearist
    threshold =4500#2600
    blue_point = blue_pixel_detection_lane(img)
    
    print('The number of blue pixels!!!',blue_point)
    if findfrontObject == 'parking':
        print("enter if")                 
        # print("rightsign---------",rightsign)
        # msg.drive.steering_angle =0
        # msg.drive.speed = 0
        # pub.publish(msg) 
        # rospy.sleep(2)
        # if nearist <= 1080:
        #     msg.drive.steering_angle =0
        #     msg.drive.speed = -20
        #     pub.publish(msg) 
        #     rospy.sleep(1.5)      
        msg.drive.steering_angle =-50
        msg.drive.speed = 20
        pub.publish(msg) 
        rospy.sleep(7)      
        msg.drive.steering_angle =50
        msg.drive.speed = 20
        pub.publish(msg) 
        rospy.sleep(5)  
        msg.drive.steering_angle =-20
        msg.drive.speed =-20
        pub.publish(msg) 
        rospy.sleep(3)                          
        # msg.drive.speed = 20
        # msg.drive.steering_angle =0
        # pub.publish(msg)
        # time.sleep(4)
        # msg.drive.speed = 20
        # msg.drive.steering_angle =0
        # pub.publish(msg) 
        # time.sleep(15)
        # msg.drive.speed = -20
        # msg.drive.steering_angle =0
        # pub.publish(msg)
        # time.sleep(8) 
        # if rightsign <= 0.7:
        #     msg.drive.steering_angle =0
        #     msg.drive.speed = 0
        #     pub.publish(msg) 
    else:
        if (blue_point < threshold) & (flag1 == 0):
            lane_detection(img)  
        else:
            flag1 = 1
            
            print('Parking line is detected!!!',blue_point)

            if (flag2 == 0):
                msg.drive.speed = 0
                # msg.drive.steering_angle = 0
                pub.publish(msg)
                rospy.sleep(3)
                sign = findfrontObject
                bsign = findrearObject
                flag2 = 1
                time1 = time.time()
            else:           
                
                print(sign)
                if(sign == 'left'):
                    msg.drive.speed = -20
                    msg.drive.steering_angle = 50
                    pub.publish(msg)
                    rospy.sleep(3)
                    msg.drive.steering_angle = 0
                    msg.drive.speed = -30
                    pub.publish(msg)
                    rospy.sleep(2)
                    msg.drive.steering_angle = 20
                    msg.drive.speed = -30
                    pub.publish(msg)
                    rospy.sleep(6)
                elif(sign == 'right'):
                    msg.drive.steering_angle = 0
                    msg.drive.speed = -30
                    pub.publish(msg)
                    rospy.sleep(4)
                    msg.drive.steering_angle = -20
                    msg.drive.speed = -30
                    pub.publish(msg)
                    rospy.sleep(4)
                elif(sign == 'straight'):
                    msg.drive.speed = -20
                    msg.drive.steering_angle = 50
                    pub.publish(msg)
                    rospy.sleep(1)
                    msg.drive.steering_angle = 0
                    msg.drive.speed = -30
                    pub.publish(msg)
                    rospy.sleep(4)
                    msg.drive.steering_angle = 7
                    msg.drive.speed = -30
                    pub.publish(msg)
                    rospy.sleep(4)
                elif(sign == 'uturn'):
                    msg.drive.steering_angle = 0
                    msg.drive.speed = -30
                    pub.publish(msg)
                    rospy.sleep(1.5)
                    msg.drive.steering_angle = 100
                    msg.drive.speed = -40
                    pub.publish(msg)
                    rospy.sleep(4)
                    msg.drive.steering_angle = -100
                    msg.drive.speed = 40
                    pub.publish(msg)
                    rospy.sleep(2)
                    msg.drive.steering_angle = 100
                    msg.drive.speed = -40
                    pub.publish(msg)
                    rospy.sleep(4)
                    
                while(sign == 'parking'):#前停
                    global u
                    msg.drive.steering_angle = -(u-320)/10
                    msg.drive.speed = -20
                    pub.publish(msg)
                    # rospy.sleep(2)
                    while(blue_pixel_detection(img) >=150000):
                        # print(">>>",blue_pixel_detection(img))
                        msg.drive.steering_angle =0
                        msg.drive.speed = 0
                        pub.publish(msg)
                    
                while (bsign=='parking'):
                    global bu
                    msg.drive.steering_angle = (bu-320)/10
                    msg.drive.speed = 20
                    pub.publish(msg)
                    
                    while(bbluecount >=150000):
                        msg.drive.steering_angle =0
                        msg.drive.speed = 0
                        pub.publish(msg)
                    # time2 = time.time()
                    # if (time2 - time1>3) :
                flag1 = 0
                flag2 = 0
            
def front_camera_callback(data):  
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    global n
    #print(laser_cmd)
    if(n>=1):
        if laser_cmd == 0:
            traffic_sign_detection(img)
    else:
        # if light_detection(img):
        if (1):
            rospy.sleep(3)
            n=n+1
        else:
            msg.drive.speed = 0
            pub.publish(msg)
            n = n
    
def laser_callback(data):
    global laser_cmd
    #laser_cmd = 0
    #if flag_laser ==1 :
    laser_cmd = data.laser_control

def rear_camera_callback(data):   
    global bbluecount
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    bbluecount = blue_pixel_detection(img)
        
def frontBoundingBoxCallBack(BdBdata):
    global findfrontObject,u,v
    if len(BdBdata.objects)==0:
        findfrontObject = 0
    else:
        for dat in BdBdata.objects:
            findfrontObject = dat.class_name
            u = (dat.box[0] + dat.box[2])/2
            v = (dat.box[1] + dat.box[3])/2

def rearBoundingBoxCallBack(BdBdata):
    global findrearObject,bu,bv,flag1
    if len(BdBdata.objects)==0:
        findrearObject = 0
    else:
        for dat in BdBdata.objects:
            findrearObject = dat.class_name
            bu = (dat.box[0] + dat.box[2])/2
            bv = (dat.box[1] + dat.box[3])/2
# def sideparking():
#     global rightsign
#     while rightsign > 1:
#         print("enter while")
#         msg.drive.speed = -20
#         msg.drive.steering_angle =0
#         pub.publish(msg)
#         rospy.sleep(3)
#         msg.drive.speed = 20
#         msg.drive.steering_angle =0
#         pub.publish(msg) 
#         rospy.sleep(5)
#         msg.drive.speed = -20
#         msg.drive.steering_angle =0
#         pub.publish(msg)
#         rospy.sleep(2) 
#         if rightsign <= 1:
#             msg.drive.steering_angle =0
#             msg.drive.speed = 0
#             pub.publish(msg)         
#             break

    
def laser_callback1(data):
    global rightsign
    rightsign = data.ranges[1080]
    nearist = np.argmin(data.ranges)
    # print("near---------",nearist)

def detector():
    global pub
    rospy.init_node('camera_cmd', anonymous=False)
    rospy.Subscriber("/usb_cam_2/image", Image, front_camera_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/usb_cam_1/image", Image, rear_camera_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/yolov5_2/object_detect', ObjectsInfo, frontBoundingBoxCallBack, queue_size=1)
    rospy.Subscriber('/yolov5/object_detect', ObjectsInfo, rearBoundingBoxCallBack, queue_size=1)
    rospy.Subscriber("/laser_control", laser_control, laser_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, laser_callback1)  
    pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
   
    detector()
#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import deque
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
from laser_test.msg import laser_control
# import jetson.inference
# import jetson.utils
import argparse
import sys
from jetauto_interfaces.msg import ObjectsInfo
import math
import os
import time
global x0 
x0 = 1
global peak_thresh
peak_thresh = 50 
global n 
n = 0
global laser_cmd
laser_cmd = 0
global msg 
msg = AckermannDriveStamped()
global flag1 
flag1 = 0
global flag2 
flag2 = 0
global flag3 
flag3 = 0
global c0,c1
global findfrontObject
findfrontObject = 0
global findrearObject
findrearObject = 0
global u,v,bu,bv
intrinsicMat = np.array([[489.3828, 0.8764, 297.5558],
                            [0, 489.8446, 230.0774],
                            [0, 0, 1]])
distortionCoe = np.array([-0.4119,0.1709,0,0.0011, 0.018])

global flag_laser
flag_laser = 0

# src_pts = np.float32([[128,378],[1,435],[639,435],[488,378]])#[220,306],[1,435],[639,435],[451,306]   [[132,358],[1,435],[639,435],[501,353] [160,334],[38,378],[628,378],[502,334] 
# src_pts = np.float32([[203,386],[1,435],[639,435],[463,386]])
src_pts = np.float32([[159,380],[1,460],[637,449],[505,380]])
dst_pts = np.float32([[70,0],[70,480],[570,480],[570,0]])
showMe = 0
    
def light_detection(origin_img):
    light_cmd=0
    hsv=cv2.cvtColor(origin_img,cv2.COLOR_BGR2HSV) 
    hsv1 = hsv.copy()
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    #red_lower = np.array([0,95,230])        #这两个是阈值，红灯的上界和下界
    #red_upper = np.array([5,255,255])
#
    red_lower = np.array([0,127,127])        #这两个是阈值，红灯的上界和下界
    red_upper = np.array([20,255,255])
    red_mask = cv2.inRange(hsv,red_lower,red_upper)
    red_mask = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
    #red_target = cv2.bitwise_and(hsv,hsv,mask = red_mask)
    #red_target = cv2.erode(red_target,element)
    #red_target = cv2.dilate(red_target,element)
    red_gray = cv2.cvtColor(red_mask,cv2.COLOR_BGR2GRAY)
    r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)
    r_gray2 = cv2.Canny(r_binary, 100, 200) 
    r = r_gray2[:,:] == 255
    count_red = len(r_gray2[r])
    if count_red>1500:
        redLight = 1
    else:
        redLight = 0

    print(">>>>>>>>>red ",count_red)

    green_lower = np.array([40,80,80])    
    green_upper = np.array([80,255,255])
    green_mask = cv2.inRange(hsv1,green_lower,green_upper)
    green_mask = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
    #green_target = cv2.bitwise_and(hsv1,hsv1,mask = green_mask)
    #green_target = cv2.erode(green_target,element)
    #green_target = cv2.dilate(green_target,element)
    green_gray = cv2.cvtColor(green_mask,cv2.COLOR_BGR2GRAY)
    g_ret,g_binary = cv2.threshold(green_mask,100,255,cv2.THRESH_BINARY)
    g_gray2 = cv2.Canny(g_binary, 100, 200)       
    g = g_gray2[:,:] == 255
    count_green = len(g_gray2[g])
    if count_green>1500:
       greenLight = 1
    else:
        greenLight = 0
    if (redLight ==1) and (greenLight == 0) :
        light_cmd = 0
    if greenLight == 1 :
        light_cmd = 1
    print(">>>>>>>>>green ",count_green)   
    print(light_cmd)

    return light_cmd

def birdView(img,M):

    img_sz = (img.shape[1],img.shape[0])
    img_warped = cv2.warpPerspective(img,M,img_sz,flags = cv2.INTER_LINEAR)
    return img_warped

def perspective_transform(src_pts,dst_pts):
    M = cv2.getPerspectiveTransform(src_pts,dst_pts)
    Minv = cv2.getPerspectiveTransform(dst_pts,src_pts)
    return {'M':M,'Minv':Minv}

def find_centroid(image,peak_thresh,window,showMe):
   
    mask_window = image[int(window['y0']-window['height']):int(window['y0']),
                        int(window['x0']):int(window['x0']+window['width'])]
    histogram = np.sum(mask_window,axis=0)
    centroid = np.argmax(histogram)
    hotpixels_cnt = np.sum(histogram)
    peak_intensity = histogram[centroid]
    if peak_intensity<=peak_thresh:
        centroid = int(round(window['x0']+window['width']/2))
        peak_intensity = 0
    else:
        centroid = int(round(centroid+window['x0']))
    return (centroid,peak_intensity,hotpixels_cnt)

def find_starter_centroids(image,y0,peak_thresh,showMe):

    window = {'x0':0,'y0':y0,'width':image.shape[1],'height':image.shape[0]/5}
    # get centroid
    centroid , peak_intensity,_ = find_centroid(image,peak_thresh,window,showMe)
    if peak_intensity<peak_thresh:
        window['height'] = image.shape[0]
        centroid,peak_intensity,_ = find_centroid(image,peak_thresh,window,showMe)
    return {'centroid':centroid,'intensity':peak_intensity}

def lane_detection(img):
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    # cv2.imwrite('corr1.jpg', corr_img)
    # cv2.imshow('corr1',corr_img)
    # cv2.waitKey(1000)
    gray_ex = cv2.cvtColor(corr_img,cv2.COLOR_RGB2GRAY)
    
    combined_output = cv2.Canny(gray_ex, 50, 100) #100, 200 75,200 400,800
    
    cleaned = combined_output
    # cv2.imwrite('corr2.jpg', cleaned)
    # cv2.imshow('2',cleaned)
    # cv2.waitKey(10)

    transform_matrix = perspective_transform(src_pts,dst_pts)
    warped_image = birdView(cleaned*1.0,transform_matrix['M'])
    # cv2.imwrite('corr3.jpg', warped_image)

    warped_image = cv2.dilate(warped_image, np.ones((15,15), np.uint8))
    warped_image = cv2.erode(warped_image, np.ones((7,7), np.uint8))
    # cv2.imwrite('corr4.jpg', warped_image)
    # cv2.imshow('1',warped_image)
    cv2.waitKey(25)

    HoughLine_image = np.array(warped_image,np.uint8)
    lines = cv2.HoughLinesP(HoughLine_image,1,np.pi/180,100,100,100,50)
    if lines is not None :
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(HoughLine_image,(x1,y1),(x2,y2),(255,0,0),1)

    bottom_crop = -40
    warped_image = warped_image[0:bottom_crop,:]
    # cv2.imwrite('corr5.jpg', warped_image)
    rightlim_image = warped_image[360:480, 400:640]
    leftlim_image = warped_image[360:480,0:240]
    
    peak_thresh = 10
    showMe = 1


    centroid_starter_top = find_starter_centroids(warped_image,y0=warped_image.shape[0],
                                               peak_thresh=peak_thresh,showMe=showMe)
    centroid_starter_bottom = find_starter_centroids(warped_image,y0=warped_image.shape[0]/5,
                                               peak_thresh=peak_thresh,showMe=showMe)
    # rospy.logwarn('offset!!!!!!!!!!!!!! %s %s %s',centroid_starter_top['centroid'],centroid_starter_bottom['centroid'],math.fabs(centroid_starter_top['centroid']-centroid_starter_bottom['centroid']))
    
            
    msg.drive.speed = -20
    msg.drive.steering_angle = 0
    # print(np.sum(rightlim_image == 255),np.sum(leftlim_image == 255))
    if  math.fabs(centroid_starter_top['centroid']-centroid_starter_bottom['centroid'])<80:             
    # if  math.fabs(centroid_starter_top['centroid']-centroid_starter_bottom['centroid'])<500:             
        print('both of the lanes are detected')
        offset=centroid_starter_top['centroid']-centroid_starter_bottom['centroid']
        # Vehicle_PID.update(offset)
        # msg.drive.steering_angle = Vehicle_PID.output###正负未测试
        
        # if np.sum(rightlim_image == 255)<=400 and np.sum(leftlim_image == 255)>400:
        #     msg.drive.steering_angle = -5
        # elif np.sum(rightlim_image == 255)>400 and np.sum(leftlim_image == 255)<=400:
        #     msg.drive.steering_angle = 5
        if abs(offset)>30:
            msg.drive.steering_angle = (abs(offset)/ offset) *1
            msg.drive.speed = -10
    
    
    else :
        global c0,c1 
        if lines is not None:
            for i in range(warped_image.shape[1]/5):
                px = [x1,x2]
                py = [y1,y2]
                c1,c0 = np.polyfit(px, py, 1)
        
        k1 =20 #60#50   k1,k2 需要调参
        k2 = 0.03#0.02  0.035
        rospy.logwarn('>>>>>>>>>>>>>>>> %s %s',c0,c1)

        if (480-c0)/c1<320:
            msg.drive.steering_angle =  -abs(1/c1*k1 - k2*(480- c0)/c1 )

            print('Left Lane')
                                 
        elif (480-c0)/c1>320:
            msg.drive.steering_angle =  abs(1/c1*k1 + k2*(640-(480-c0)/c1))  
            print('Right Lane')

    rospy.logerr('angle>>>>>>>>>>> %s',msg.drive.steering_angle)
    pub.publish(msg)
    
def blue_pixel_detection(img):

    cross_verify = 0
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # _, thresh_img = cv2.threshold(gray, thresh=200, maxval=255, type=cv2.THRESH_BINARY)
    # mask = cv2.bitwise_not(thresh_img)
    # masked_img = cv2.bitwise_and(img, img, mask=mask)
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    hsv=cv2.cvtColor(corr_img,cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv',hsv)
#提取蓝色区域
    blue_lower=np.array([100,50,130])#100 50  150
    # blue_lower=np.array([100,50,190])
    # blue_upper=np.array([124,255,255])
    #blue_lower=np.array([100,44,151])
    blue_upper=np.array([124,255,250])
    blue_mask=cv2.inRange(hsv,blue_lower,blue_upper)
   
#模糊
    blue_blurred=cv2.blur(blue_mask,(9,9))
    #cv2.imshow('blurred',blue_mask)
    cv2.waitKey(25)

#二值化
    ret,binary=cv2.threshold(blue_blurred,100,255,cv2.THRESH_BINARY)
    transform_matrix = perspective_transform(src_pts,dst_pts)
    blue_warped_image = birdView(binary*1.0,transform_matrix['M'])
    # cv2.imshow('blurred binary',blue_warped_image)
    cv2.waitKey(25)
    blue_point = cv2.countNonZero(blue_warped_image)
    return blue_point

def blue_pixel_detection_lane(img):

    cross_verify = 0
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # _, thresh_img = cv2.threshold(gray, thresh=200, maxval=255, type=cv2.THRESH_BINARY)
    # mask = cv2.bitwise_not(thresh_img)
    # masked_img = cv2.bitwise_and(img, img, mask=mask)
    corr_img = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
    hsv=cv2.cvtColor(corr_img,cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv',hsv)
#提取蓝色区域
    blue_lower=np.array([100,50,130])#100 50  150
    # blue_lower=np.array([100,50,190])
    # blue_upper=np.array([124,255,255])
    #blue_lower=np.array([100,44,151])
    blue_upper=np.array([124,255,250])
    blue_mask=cv2.inRange(hsv,blue_lower,blue_upper)
   
#模糊
    blue_blurred=cv2.blur(blue_mask,(9,9))
    #cv2.imshow('blurred',blue_mask)
    cv2.waitKey(25)

#二值化
    ret,binary=cv2.threshold(blue_blurred,100,255,cv2.THRESH_BINARY)
    transform_matrix = perspective_transform(src_pts,dst_pts)
    blue_warped_image = birdView(binary*1.0,transform_matrix['M'])
    
    img1 = blue_warped_image[360:480,:]
    # cv2.imshow('blurred binary',blue_mask[360:480,:])
    cv2.waitKey(25)
    blue_point = cv2.countNonZero(img1)
    return blue_point

def traffic_sign_detection(img):
    global flag1
    global findfrontObject,findrearObject
    global flag2
    global sign
    global time1
    global u,v,bu,bv,bsign
    global bbluecount
    threshold =4500#2600
    blue_point = blue_pixel_detection_lane(img)
    
    print('The number of blue pixels!!!',blue_point)
    if (blue_point > threshold) & (flag1 == 0) :
       lane_detection(img)  
    else:
        flag1 = 1
        
        print('Parking line is detected!!!',blue_point)

        if (flag2 == 0):
            msg.drive.speed = 0
            # msg.drive.steering_angle = 0
            pub.publish(msg)
            rospy.sleep(3)
            sign = findfrontObject
            bsign = findrearObject
            flag2 = 1
            time1 = time.time()
        else:           
            
            print(sign)
            if(sign == 'left'):
                msg.drive.speed = -20
                msg.drive.steering_angle = 50
                pub.publish(msg)
                rospy.sleep(3)
                msg.drive.steering_angle = 0
                msg.drive.speed = -30
                pub.publish(msg)
                rospy.sleep(2)
                msg.drive.steering_angle = 20
                msg.drive.speed = -30
                pub.publish(msg)
                rospy.sleep(6)
            elif(sign == 'right'):
                msg.drive.steering_angle = 0
                msg.drive.speed = -30
                pub.publish(msg)
                rospy.sleep(4)
                msg.drive.steering_angle = -20
                msg.drive.speed = -30
                pub.publish(msg)
                rospy.sleep(4)
            elif(sign == 'straight'):
                msg.drive.speed = -20
                msg.drive.steering_angle = 50
                pub.publish(msg)
                rospy.sleep(1)
                msg.drive.steering_angle = 0
                msg.drive.speed = -30
                pub.publish(msg)
                rospy.sleep(4)
                msg.drive.steering_angle = 7
                msg.drive.speed = -30
                pub.publish(msg)
                rospy.sleep(4)
            elif(sign == 'uturn'):
                msg.drive.steering_angle = 0
                msg.drive.speed = -30
                pub.publish(msg)
                rospy.sleep(1.5)
                msg.drive.steering_angle = 100
                msg.drive.speed = -40
                pub.publish(msg)
                rospy.sleep(4)
                msg.drive.steering_angle = -100
                msg.drive.speed = 40
                pub.publish(msg)
                rospy.sleep(2)
                msg.drive.steering_angle = 100
                msg.drive.speed = -40
                pub.publish(msg)
                rospy.sleep(4)
                
            while(sign == 'parking'):#前停
                global u
                msg.drive.steering_angle = -(u-320)/10
                msg.drive.speed = -20
                pub.publish(msg)
                # rospy.sleep(2)
                while(blue_pixel_detection(img) >=150000):
                    # print(">>>",blue_pixel_detection(img))
                    msg.drive.steering_angle =0
                    msg.drive.speed = 0
                    pub.publish(msg)
                
            while (bsign=='parking'):
                global bu
                msg.drive.steering_angle = (bu-320)/10
                msg.drive.speed = 20
                pub.publish(msg)
                
                while(bbluecount >=150000):
                    msg.drive.steering_angle =0
                    msg.drive.speed = 0
                    pub.publish(msg)
                # time2 = time.time()
                # if (time2 - time1>3) :
            flag1 = 0
            flag2 = 0
		
def front_camera_callback(data):  
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    global n
    #print(laser_cmd)
    if(n>=1):
        if laser_cmd == 0:
            traffic_sign_detection(img)
    else:
        # if light_detection(img):
        if (1):
            rospy.sleep(3)
            n=n+1
        else:
            msg.drive.speed = 0
            pub.publish(msg)
            n = n
    
def laser_callback(data):
    global laser_cmd
    #laser_cmd = 0
    #if flag_laser ==1 :
    laser_cmd = data.laser_control

def rear_camera_callback(data):   
    global bbluecount
    img = CvBridge().imgmsg_to_cv2(data, "bgr8")
    bbluecount = blue_pixel_detection(img)
        
def frontBoundingBoxCallBack(BdBdata):
    global findfrontObject,u,v
    if len(BdBdata.objects)==0:
        findfrontObject = 0
    else:
        for dat in BdBdata.objects:
            findfrontObject = dat.class_name
            u = (dat.box[0] + dat.box[2])/2
            v = (dat.box[1] + dat.box[3])/2

def rearBoundingBoxCallBack(BdBdata):
    global findrearObject,bu,bv,flag1
    if len(BdBdata.objects)==0:
        findrearObject = 0
    else:
        for dat in BdBdata.objects:
            findrearObject = dat.class_name
            bu = (dat.box[0] + dat.box[2])/2
            bv = (dat.box[1] + dat.box[3])/2

def detector():
    global pub
    rospy.init_node('camera_cmd', anonymous=False)
    rospy.Subscriber("/usb_cam_2/image", Image, front_camera_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/usb_cam_1/image", Image, rear_camera_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/yolov5_2/object_detect', ObjectsInfo, frontBoundingBoxCallBack, queue_size=1)
    rospy.Subscriber('/yolov5/object_detect', ObjectsInfo, rearBoundingBoxCallBack, queue_size=1)
    rospy.Subscriber("/laser_control", laser_control, laser_callback, queue_size=1)
    pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
   
    detector()
