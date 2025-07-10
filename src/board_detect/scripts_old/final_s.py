#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
#from cv_bridge import CvBridge
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

detect_need=False
detect_ok=False
person_num=0
spontoon=0
bulletproof=0
head=0 
teargas=0
parking_ok=False

task_stage=0 # 0为未启动  1为到恐怖分子识别点

board_aligned=False

detect_stage=0
board_nav=[]

right_dist=0

duizheng_need=True

gx = 320.0
gy = 240.0
u = 0.001
v = 0.001
las_switch = False
tol = 40

def rotation_fast(angle):
    rate = rospy.Rate(10)
    PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel = Twist()
    angle_rad = math.radians(angle)
    duration = abs(angle_rad) / (0.1 * 1.0) 
    duration = int(duration)
    for _ in range(duration):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = -1.0
        PUB.publish(vel)
        rate.sleep()
    for _ in range(10):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        PUB.publish(vel)
        rate.sleep()

def rotate(rotation_angle, rotation_speed = 2.5):
    """
    rotation_angle 正值左转，负值右转
    """
    rotation_cmd = Twist()
    if rotation_angle < 0:
        rotation_speed = -rotation_speed
    rotation_cmd.angular.z = rotation_speed
    # 计算旋转所需的时间
    rotation_duration = abs(rotation_angle) * 1.0 / abs(rotation_speed)
    end_time = rospy.Time.now() + rospy.Duration(rotation_duration)
    while rospy.Time.now() < end_time:
    # while True:
        self.cmd_vel_pub.publish(rotation_cmd)
        rospy.sleep(0.1)  # 控制发送频率

    #  if rotation_angle < 0:
        # rotation_speed = -rotation_speed
    # # 旋转机器人
    # current_theta = abs((self.odom_ori_w * rotation_speed / abs(rotation_speed) + rotation_angle * 1.0 / 360 + 1) % 2 - 1)
    # pre_num = 1
    # while True:
    #     ori_w = self.odom_ori_w
    #     ori_z = self.odom_ori_z
    #     rospy.loginfo("{}, {}, {}, {}".format(current_theta, ori_w, ori_z, abs(abs(ori_w) - current_theta)))
    #     # if pre_num < abs(abs(ori_w ) - current_theta):
    #         # break
    #     pre_num = abs(abs(ori_w) - current_theta)
    #     self.cmd_vel_pub.publish(rotation_cmd)
    #     rospy.sleep(0.1)

    # 停止机器人
    rotation_cmd.angular.z = 0.0
    self.cmd_vel_pub.publish(rotation_cmd)
      


def rotation_cw(angle,speed=2.0):
    rate = rospy.Rate(10)
    PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel = Twist()
    if(angle<0):
        speed=-speed
    angle_rad = math.radians(angle)
    duration = abs(angle_rad) / (0.1 * speed) 
    duration = int(duration)
    for _ in range(duration):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = -speed
        PUB.publish(vel)
        rate.sleep()
    for _ in range(10):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        PUB.publish(vel)
        rate.sleep()

    # self.cmd_vel_pub.publish(rotation_cmd)

    # duration = int(duration)
    # for _ in range(duration):
    #     vel.linear.x = 0
    #     vel.linear.y = 0
    #     vel.linear.z = 0
    #     vel.angular.x = 0
    #     vel.angular.y = 0
    #     vel.angular.z = rotation_speed
    #     PUB.publish(vel)
    #     rate.sleep()
    # for _ in range(10):
    #     vel.linear.x = 0
    #     vel.linear.y = 0
    #     vel.linear.z = 0
    #     vel.angular.x = 0
    #     vel.angular.y = 0
    #     vel.angular.z = 0
    #     PUB.publish(vel)
    #     rate.sleep()
def rotation_acw(angle,speed=2.0):
    rate = rospy.Rate(10)
    PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel = Twist()
    if(angle<0):
        speed=-speed
    angle_rad = math.radians(angle)
    duration = abs(angle_rad) / (0.1 * speed) 
    duration = int(duration)
    for _ in range(duration):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = speed
        PUB.publish(vel)
        rate.sleep()
    for _ in range(10):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        PUB.publish(vel)
        rate.sleep()

    # self.cmd_vel_pub.publish(rotation_cmd)

    # duration = int(duration)
    # for _ in range(duration):
    #     vel.linear.x = 0
    #     vel.linear.y = 0
    #     vel.linear.z = 0
    #     vel.angular.x = 0
    #     vel.angular.y = 0
    #     vel.angular.z = rotation_speed
    #     PUB.publish(vel)
    #     rate.sleep()
    # for _ in range(10):
    #     vel.linear.x = 0
    #     vel.linear.y = 0
    #     vel.linear.z = 0
    #     vel.angular.x = 0
    #     vel.angular.y = 0
    #     vel.angular.z = 0
    #     PUB.publish(vel)
    #     rate.sleep()


def rotation(angle,speed=2.0):
    rate = rospy.Rate(10)
    PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel = Twist()
    if(angle<0):
        speed=-speed
    angle_rad = math.radians(angle)
    duration = abs(angle_rad) / (0.1 * speed) 
    duration = int(duration)
    for _ in range(duration):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = speed
        PUB.publish(vel)
        rate.sleep()
    for _ in range(10):
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        PUB.publish(vel)
        rate.sleep()

    # self.cmd_vel_pub.publish(rotation_cmd)

    # duration = int(duration)
    # for _ in range(duration):
    #     vel.linear.x = 0
    #     vel.linear.y = 0
    #     vel.linear.z = 0
    #     vel.angular.x = 0
    #     vel.angular.y = 0
    #     vel.angular.z = rotation_speed
    #     PUB.publish(vel)
    #     rate.sleep()
    # for _ in range(10):
    #     vel.linear.x = 0
    #     vel.linear.y = 0
    #     vel.linear.z = 0
    #     vel.angular.x = 0
    #     vel.angular.y = 0
    #     vel.angular.z = 0
    #     PUB.publish(vel)
    #     rate.sleep()
class duizheng:
    global board_aligned
    def __init__(self):
        global board_aligned

        board_aligned=False
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.center_threshold = 0.1  
        self.angular_speed = 0.1     

    def scan_callback(self, data):
        ranges = data.ranges
        front_ranges = ranges[len(ranges)//3 : 2*len(ranges)//3]
        min_distance = min(front_ranges)
        min_index = front_ranges.index(min_distance)
        self.align_to_center(min_index, len(front_ranges) // 2)

    def align_to_center(self, min_index, center_index):
        global board_aligned

        twist = Twist()
        if abs(min_index - center_index) > self.center_threshold:
            twist.angular.z = -self.angular_speed if min_index > center_index else self.angular_speed
            board_aligned=False
        else:
            twist.angular.z=0
            board_aligned=True
        self.cmd_vel_pub.publish(twist)

def move2(distance) :
    v_pub2 = rospy.Publisher("cmd_vel", Twist, queue_size=10)   
    rate = rospy.Rate(100) 
    goal_distance = distance   
    vel = Twist()  
    vel.linear.x = 0.2 * (goal_distance / abs(goal_distance))
    vel.linear.y = 0.0
    vel.angular.z = 0.0    
    linear_duration = abs(goal_distance / vel.linear.x)
    ticks = int(linear_duration * 100)  
    for j in range(ticks):  
        v_pub2.publish(vel)
        rate.sleep()  
    vel.linear.x = 0.0
    vel.angular.z = 0.0
    for i in range(10):  
        v_pub2.publish(vel)  
        rate.sleep()  
    return
def move_l(distance):
    v_pub2 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(100)
    goal_distance = distance
    vel = Twist()
    vel.linear.y = 0.2
    vel.angular.z = 0.0
    linear_duration = goal_distance / vel.linear.y
    ticks = int(linear_duration * 100)
    for j in range(ticks):
        v_pub2.publish(vel)
        rate.sleep()
    vel.linear.y = 0
    vel.angular.z = 0
    for i in range(10):
        v_pub2.publish(vel)
        rate.sleep()
def move_r(distance):
    v_pub2 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(100)
    goal_distance = distance
    vel = Twist()
    vel.linear.y = -0.2
    vel.angular.z = 0.0
    linear_duration = goal_distance / abs(vel.linear.y)
    ticks = int(linear_duration * 100)
    for j in range(ticks):
        v_pub2.publish(vel)
        rate.sleep()
    vel.linear.y = 0
    vel.angular.z = 0
    for i in range(10):
        v_pub2.publish(vel)
        rate.sleep()

def client(x,y,z,w):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w
    rospy.loginfo(">>>>>>>>>>>>>>>>>>>>Sending goal")
    move_base.send_goal(goal)
    finished_within_time = move_base.wait_for_result(rospy.Duration(300))
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.loginfo(">>>>>>>>>>>>>>>>>>>>Timed out achieving goal")
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo(">>>>>>>>>>>>>>>>>>>>Goal succeeded!")
            rospy.sleep(2)
        else:
            rospy.loginfo(">>>>>>>>>>>>>>>>>>>>Goal failed ")

def thread_job1():
    rospy.spin()

def ready():
    rospy.Subscriber("/mic/awake/angle",Int32,dovoice,queue_size=10)
# 
    while v_msg == 0.0:
        rospy.loginfo(">>>>>>>>>>>>>>>>>>>>ready to nav?")
        if v_msg:
            break
    add_thread1 = threading.Thread(target=thread_job1)
    add_thread1.start()
    return

def dovoice(msg):
    global v_msg
    rospy.loginfo(">>>>>>>>>>>>>>>>>>>>say something to wake up")
    v_msg = msg.data
    rospy.loginfo(">>>>>>>>>>>>>>>>>>>>%f",v_msg)

def yolo_proc(msg):
    global spontoon,teargas,bulletproof,detect_need,person_num,detect_ok,detect_stage,u,v
    
    for item in msg.bounding_boxes:
        if item.Class!='person' and item.Class!='firstaid':
            u = (item.xmin + item.xmax) / 2
            v = (item.ymin + item.ymax) / 2

    if detect_need:
        temp=0
        temp_2=0
        for item in msg.bounding_boxes:
            # u = (item.xmin + item.xmax) / 2
            # v = (item.ymin + item.ymax) / 2
            if item.Class=='person':
                temp=temp+1

            elif item.Class=='spontoon':
                spontoon=1
                print(spontoon)

            elif item.Class=='teargas':
                teargas=1
                print(teargas)

            elif item.Class=='bulletproof':
                bulletproof=1
                print(bulletproof)

        print(temp)
        if detect_stage==0:
            person_num=max(temp,temp_2)
        detect_need=False
        detect_ok=True

def parking_obj():
    global spontoon,teargas,bulletproof,detect_need,person_num,detect_ok,parking_ok

    
    if person_num==1:
        if spontoon==1 and parking_ok:

            rospy.loginfo("【物资获取停车】准备播报语音")
            sound = AudioSegment.from_file("/home/ucar/ucar_ws/src/xwctest/voice/spontoon.mp3", format="mp3")
            play(sound)
            # move2(-0.2)
    elif person_num==2:
        if bulletproof==1 and parking_ok:
            rospy.loginfo("【物资获取停车】准备播报语音")
            sound = AudioSegment.from_file("/home/ucar/ucar_ws/src/xwctest/voice/bulletproof.mp3", format="mp3")
            play(sound)
            # move2(-0.2)

    elif person_num==3:
        if teargas==1 and parking_ok:
            rospy.loginfo("【物资获取停车】准备播报语音")
            sound = AudioSegment.from_file("/home/ucar/ucar_ws/src/xwctest/voice/teargas.mp3", format="mp3")
            play(sound)
            # move2(-0.2)


def search_m1():
# 雷达找板方式搜索，这是读点部分，需要先用radar2/radar3存点的数据
    
    global board_nav
    global las_switch
    global spontoon,teargas,bulletproof,detect_need,person_num,detect_ok,detect_stage,detect_need,parking_ok
    back_need=False

    rospy.loginfo("==========================开始读取识别板位置【M1】")

    with open('/home/ucar/ucar_ws/src/xwctest/src/board.txt','r') as temp_file:
        templines = temp_file.readlines()
    for line in templines:
        line_data = line.strip().split(',')
        board_nav.append([float(num) for num in line_data])

    for i in board_nav:
        rospy.loginfo("【M1】导航至坐标（%f,%f,%f,%f）",i[0],i[1],i[2],i[3])
        counter=0
        detect_need=False

        client(i[0],i[1],i[2],i[3])
        detect_ok=False
        detect_need=True

        while detect_ok is not True:
            rospy.sleep(0.1)
            counter=counter+1
            if (counter>=6):
                break
        if detect_ok is not True:
            rotation_acw(30,0.7)
            if detect_ok is True:
                move_l(0.2)
        while detect_ok is not True:
            rospy.sleep(0.1)
            counter=counter+1
            if (counter>=6):
                break
        if detect_ok is not True:
            rotation_cw(60,0.7)
            if detect_ok is True:
                move_r(0.2)

        rospy.loginfo("【M1】识别情况：恐怖分子%d，警棍%d，防弹衣%d，催泪瓦斯%d",person_num,spontoon,bulletproof,teargas)

        while detect_ok is not True:
            rospy.sleep(0.1)
            counter=counter+1
            if(counter>6):
                detect_need=False
                rospy.loginfo("没有发现物品")
                # back_need=True
                break
        
        if person_num==1 and spontoon==1:
            # aim()
            # move2(0.2)
            las_switch = True
            parking_ok=True
            rospy.loginfo("【M1】恐怖分子数1，警棍数1匹配成功！")
            break
        elif person_num==2 and bulletproof==1:
            # aim()

            # move2(0.2)
            las_switch = True
            parking_ok=True
            rospy.loginfo("【M1】恐怖分子数2，防弹衣1匹配成功！")
            break
        elif person_num==3 and teargas==1:
            # aim()

            # move2(0.2)
            las_switch = True
            parking_ok=True
            rospy.loginfo("【M1】恐怖分子数3，催泪瓦斯1匹配成功！")
            break

def search_m1_down():
# 雷达找板方式搜索，这是读点部分，需要先用radar2/radar3存点的数据
    
    global board_nav
    global las_switch
    global spontoon,teargas,bulletproof,detect_need,person_num,detect_ok,detect_stage,detect_need,parking_ok
    back_need=False

    rospy.loginfo("==========================开始读取识别板位置【M1_DOWN】")

    with open('/home/ucar/ucar_ws/src/xwctest/src/board_down.txt','r') as temp_file:
        templines = temp_file.readlines()
    for line in templines:
        line_data = line.strip().split(',')
        board_nav.append([float(num) for num in line_data])

    for i in board_nav:
        rospy.loginfo("【M1_D】导航至坐标（%f,%f,%f,%f）",i[0],i[1],i[2],i[3])
        counter=0
        detect_need=False

        client(i[0],i[1],i[2],i[3])
        detect_ok=False
        detect_need=True
        rospy.loginfo("【M1_D】识别情况：恐怖分子%d，警棍%d，防弹衣%d，催泪瓦斯%d",person_num,spontoon,bulletproof,teargas)

        while detect_ok is not True:
            rospy.sleep(0.1)
            counter=counter+1
            if(counter>6):
                rospy.loginfo("没有发现物品")
                # back_need=True
                break
        
        if person_num==1 and spontoon==1:
            # aim()
            # move2(0.2)
            las_switch = True
            parking_ok=True
            rospy.loginfo("【M1_D】恐怖分子数1，警棍数1匹配成功！")
            break
        elif person_num==2 and bulletproof==1:
            # aim()

            # move2(0.2)
            las_switch = True
            parking_ok=True
            rospy.loginfo("【M1_D】恐怖分子数2，防弹衣1匹配成功！")
            break
        elif person_num==3 and teargas==1:
            # aim()

            # move2(0.2)
            las_switch = True
            parking_ok=True
            rospy.loginfo("【M1_D】恐怖分子数3，催泪瓦斯1匹配成功！")
            break

def aim():
    global tol,u,v,gx,gy
    rate=rospy.Rate(100)
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    ymove = Twist()
    pausemove = Twist()
    pausemove.linear.x = 0.0
    pausemove.linear.y = 0.0
    # print(u,v,gx,gy)

    while abs(u-gx) > tol :
        ymove.linear.y = ysele(u,gx)
        ymove.linear.x = 0.0
        pub.publish(ymove)
        if abs(gx - u) <= tol :
            print("STOP")

            pub.publish(pausemove)
            break
        rate.sleep()
        if abs(gx - u) <= tol :
            print("STOP")

            pub.publish(pausemove)
            break
        # print(ymove.linear.y)
    if abs(gx - u) <= tol :
        pub.publish(pausemove)
        print("STOP")
    pub.publish(pausemove)

def ysele(a,b):
    global checkv
    if a>b :
        return -0.1
    elif a<b :
        return +0.1
    else:
        return 0.0

def radar_callback(msg):
    global position_status,duizheng_need,left_8,right_8,head,left_3,right_3,lasum
    global right_dist
    data = msg.ranges
    angle_increment = msg.angle_increment
    lasum = sum(1 for dist in data[357:399] if dist > 2.0)
    # print(lasum)
    def get_lidar(data, angle, angle_increment, radius=5):
        from math import pi
        angle = angle / 360.0 
        index = int(angle * (2*pi) / angle_increment)
        deg = data[index]   
        # print(index)
        behind_data = data[index-radius if index-radius>0 else 0 : index+radius if index+radius < len(data) else len(data)]
        

        return deg
    right_dist=get_lidar(data,270,angle_increment)
    left_8 = get_lidar(data,193,angle_increment)
    # left_3 = get_lidar(data,183,angle_increment)
    # right_3 = get_lidar(data,177,angle_increment)
    right_8 = get_lidar(data,167,angle_increment)
    head = get_lidar(data,180,angle_increment)

    global las_switch
    if las_switch == True:   
        move2(get_lidar(data,180,angle_increment) - 0.40)    
        las_switch = False

# def ramp_duizheng():
#     global duizheng_need
#     rospy.loginfo("左侧13°:%.1f, 右侧13°: %.1f，前方：%.1f",left_8,right_8,head)
#     while duizheng_need:
#         if head>1.0 or head==0 or math.isinf(head):
#             if (abs(left_8-right_8)/(0.5*(left_8+right_8))<0.1):
#                 rospy.loginfo("已对正！")
#                 duizheng_need=False
#             else:
#                 if(left_8-right_8>=0.05) or math.isinf(left_8):
#                     rospy.loginfo("当前位置偏右")
#                     move_l(0.03)
#                     if (abs(left_8-right_8)/(0.5*(left_8+right_8))<0.1):
#                         duizheng_need=False

#                         break
                    
#                 elif(right_8-left_8>=0.05) or math.isinf(right_8):
#                     rospy.loginfo("当前位置偏左")
#                     move_r(0.03)
#                     if (abs(left_8-right_8)/(0.5*(left_8+right_8))<0.1):
#                         duizheng_need=False

#                         break

def ramp_duizheng():
    global lasum
    while lasum>=32:
        move_l(0.3)
        move_r(0.3)
        # print(lasum)
        if lasum >= 32:
            pauseMove()
            break

def pauseMove():
    rate = rospy.Rate(10)
    stop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pause = Twist()
    pause.linear.x = 0.0
    pause.linear.y = 0.0
    pause.angular.z = 0.0
    for k in range(10):
        stop_pub.publish(pause)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('xwcnavdemo', anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,yolo_proc)
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.Subscriber('/scan', LaserScan, radar_callback)

    move_base.wait_for_server(rospy.Duration(4.0))
    rospy.loginfo(">>>>>>>>>>>>>>>>>>>>Connected to move base server")

    v_msg = 0.0
    ready()
    if task_stage ==0:
        rospy.loginfo("STAGE%d",task_stage)
        rospy.sleep(2)
        rospy.loginfo(">>>>>>>>>>>>>>>>>>>>ready to nav!")
        
        client(0.44,0.1,0.707,0.707)
        # move2(0.2)
        # client(0.46,0.09,0.707,0.707) #第一个过渡点
        # client(-14.8,-13.8,0.1,0.0)#Bx
        client(1.0,1.282,0,1.0)#Bx
        rospy.sleep(0.5)
        task_stage = 1
    if task_stage ==1: 
        rospy.loginfo("STAGE%d",task_stage)

        rospy.loginfo("==========================开始识别恐怖分子")
        detect_need=True
        while detect_ok is not True:
            rospy.sleep(0.1)
        rospy.loginfo("==========================识别到恐怖分子%d个",person_num)
        rospy.loginfo(">>>>>>>>>>>>>>>>>>>>point has been gotten")
        if(person_num==1):   
            sound = AudioSegment.from_file("/home/ucar/ucar_ws/src/xwctest/voice/person1.mp3", format="mp3")
            play(sound)

        elif (person_num==2):    
            sound = AudioSegment.from_file("/home/ucar/ucar_ws/src/xwctest/voice/person2.mp3", format="mp3")
            play(sound)

        elif (person_num==3):    
            sound = AudioSegment.from_file("/home/ucar/ucar_ws/src/xwctest/voice/person3.mp3", format="mp3")
            play(sound)
        task_stage=2

    # client(-13.8,-12.2,0.1,0.0)#Ax
    # client(0.0,0.0,0.1,1.0)#Ax
    if task_stage==2:
        rospy.loginfo("STAGE%d",task_stage)
        rotation_cw(240)#掉头

        client(0.40,0.1,-0.707,0.707)#Bx

        # rotation_cw(120)


        # client(0.0,0.0,1.0,0.1)#Ax
        # rotation(200)#掉头->A-x
        # move2(2.0)
        # client(-11.5,-12.2,0.0,0.1)#过渡点-x
        # client(-2.41,0.0,0.0,0.1)#过渡点-x
        # client(0.09,0.003,1,0)
        # client(0,0,1,0)
        client(0.09,0.003,1,0)

        move_r(0.10)
        client(-2.3,0.05,1,0)

        # client(-2.5,0,1,0)

        task_stage=3
    # client(-11.5,-10.6,0.707,0.707)#急救包点-y
    if task_stage==3:
        rospy.loginfo("STAGE%d",task_stage)
        rotation_acw(200,0.7)
        #client(-2.55,-1.83,-0.744,0.66)#急救包点-y
        client(-2.45,-1.5,-0.744,0.66)
        if(head > 0.3):
            print(head)
            move2(head-0.45)
        sound = AudioSegment.from_file("/home/ucar/ucar_ws/src/xwctest/voice/firstaid.mp3", format="mp3")
        play(sound)
        rospy.loginfo("==========================已取到急救包")
        rotation(200)#掉头->急救包点y
    # client(-11.5,-12.2,-0.707,0.707)#过渡点y
        detect_stage=1

        # client(-2.56,0.03,0.707,0.707)#过渡点y
        client(-2.3,0.05,1,0)

        ret =1 
        ret = os.system('rosrun xwctest radar3.py')
        while ret !=0:
            time.sleep(1)
        search_m1()  #雷达搜点方式1，认为都能搜到，而且点都在地图内侧
        parking_obj()
        if parking_ok is not True:
        
    # client(-11.8,-13.2,-0.707,0.707)#搜索点y
    #client(-1.8,-1.0,0.707,0.707)#搜索点y
            # client(-1.84,0.841,0.735,0.677)#搜索点y
            # client(-1.84,0.841,0.735,0.677)#搜索点y
            client(-2.43,0.05,0.735,0.677)




        # rotation(360)
        task_stage=4
    if task_stage==4:
        rospy.loginfo("STAGE%d",task_stage)

        # if parking_ok is not True:
        
        #     ret = os.system('rosrun xwctest radar3.py')
        #     print(ret)
        # # thread1= threading.Thread(target=ObstacleFilter)
        # # thread1.start()
        # # thread1.join()

        #     # time.sleep(1)
        # # rospy.sleep(5)
        #     search_m1()  #雷达搜点方式1，认为都能搜到，而且点都在地图内侧
        #     parking_obj()

        if parking_ok is not True:
            ret1 =1
            ret1 = os.system('rosrun xwctest radar3.py')
            # time.sleep(5)
            while 1:
                time.sleep(3)
                if ret1 ==0:
                    break
            # while os.path.exists("/home/ucar/ucar_ws/src/xwctest/src/board.txt"):
            #     time.sleep(1)

            search_m1()
            parking_obj()
            if parking_ok is not True:
                ret2 =1 
                ret2 = os.system('rosrun xwctest radar3.py')
                # time.sleep(5)
                while 1:
                    time.sleep(3)
                    if ret2 ==0 :
                        break
                # while os.path.exists("/home/ucar/ucar_ws/src/xwctest/src/board.txt"):
                #     time.sleep(1)
                search_m1()
                parking_obj()


# 下半平面逻辑：
        if parking_ok is not True:
            rospy.loginfo("上半平面没有找到所有物品，正在尝试前往下半平面")
            client(-2.5,0,-0.707,0.707)  #搜索点

            ret4 =1
            ret4 = os.system('rosrun xwctest radar3_down.py')
            while 1:
                time.sleep(3)
                if ret4 ==0:
                    break
            search_m1_down()
            parking_obj()


            if parking_ok is not True:
                ret5 =1
                ret5 = os.system('rosrun xwctest radar3_down.py')
                while 1:
                    time.sleep(3)
                    if ret5 ==0:
                        break
                search_m1_down()
                parking_obj()
                
        if parking_ok is not True:
            client(-2.6,-1.83,-0.744,0.66)#急救包点-y
            ret6 =1
            ret6 = os.system('rosrun xwctest radar3_down.py')
            # time.sleep(5)
            while 1:
                time.sleep(3)
                if ret6 ==0:
                    break
            search_m1_down()
            parking_obj()

        task_stage=5

    if task_stage==5:
        rospy.loginfo("STAGE%d",task_stage)

        rotation(200)#掉头->搜索点-y
        # client(-11.5,-12.2,0.1,0.0)#过渡点x
        client(-2.5,0,0,1)  #过度点
        rotation_acw(200,0.5)#掉头->急救包点y

        # client(-2.0,0.07,0,1) #过渡点
        client(-1.95,0.10,0,1)
        # print(move_comp)
        # if move_comp<0.1:
        #     move_l(move_comp)
        # duizheng_need = True
        # ramp_duizheng()

        move_r(0.15)
        if (head is not None):
            if(head<=1.5):
                move_l(0.20)
                rotation_cw(15) 
        # client(-2.41,0.0,0.1,1.0)#过渡点x
        task_stage=6

    # client(-13.8,-12.2,0.1,0.0)#Ax
    if task_stage==6:
        rospy.loginfo("STAGE%d",task_stage)

        client(0.02,0.12,0.01,1.0)#Ax
        task_stage=7
    # client(-15.8,-12.2,0.1,0.0)#Cx
    if task_stage==7:
        # rotation_acw(380)
        # move2(0.4)
        rospy.loginfo("STAGE%d",task_stage)
        # client(0.765,-0.44,0.01,1.0)#Cx
        rotation_acw(100)

        las_switch=True

        # client(2.0,0.0,0.1,1.0)#Cx


        ret7 = os.system('rosrun xwctest line_s.py')
        
        while 1:
            time.sleep(1)
            if ret6 ==0:
                break


        sound = AudioSegment.from_file("/home/ucar/ucar_ws/src/xwctest/voice/finished.mp3", format="mp3")
        play(sound)
        rospy.loginfo("已完成人质营救工作，请快速增派支援进行人质救援!")

