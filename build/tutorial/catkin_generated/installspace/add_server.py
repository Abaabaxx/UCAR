#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy # 导入ROS Python库

# 从 tutorial 功能包的 srv 服务通信的子模块中，导入AddTwoInts、AddTwoIntsResponse、AddTwoIntsRequest 这三个类
# 该 srv 子模块位于 lby_ws/devel/lib/python2.7/dist-packages/tutorial/srv/_AddTwoInts.py中，是catkin_make构建时，message_generation自动生成的 python 模块
from tutorial.srv import AddTwoInts, AddTwoIntsResponse, AddTwoIntsRequest

# 从 tutorial 功能包的 srv 服务通信的子模块中，导入所有类
# from tutorial.srv import *

# 处理服务请求的回调函数
def add_server_callback(req):
    sum = req.a + req.b
    rospy.loginfo("收到请求: %d + %d = %d" % (req.a, req.b, sum))
    # 将结果传入类 AddTwoIntsResponse() 中，创造一个服务通信的响应的实例并返回
    return AddTwoIntsResponse(sum)

def add_server():
    # 初始化ROS节点
    rospy.init_node('add_server')
    
    # 创建服务端
    # 给 rospy.Service 这个类传入三个参数创建一个 服务端实例server
    # 参数1：服务端实例s的名称
    # 参数2：服务消息的类型（.srv文件）
    # 参数3：服务端接受到请求后调用的回调函数
    server = rospy.Service('add_server', AddTwoInts, add_server_callback)
    
    rospy.loginfo("准备好进行加法运算。")
    
    # 保持节点运行
    rospy.spin()

if __name__ == "__main__":
    try:
        add_server()
    except rospy.ROSInterruptException:
        pass