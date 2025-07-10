#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy

# 从 tutorial 功能包的 srv 服务通信的子模块中，导入AddTwoInts、AddTwoIntsResponse、AddTwoIntsRequest 这三个类
# 该 srv 子模块位于 lby_ws/devel/lib/python2.7/dist-packages/tutorial/srv/_AddTwoInts.py中，是catkin_make构建时，message_generation自动生成的 python 模块
from tutorial.srv import AddTwoInts, AddTwoIntsResponse, AddTwoIntsRequest

# 从 tutorial 功能包的 srv 服务通信的子模块中，导入所有类
# from tutorial.srv import *


def add_client():
    # 初始化ROS节点
    rospy.init_node('add_client')
    
    # 等待服务可用
    # 参数：等待的服务端的名称（需要与服务端创建服务时使用的名称一致）
    # 防止服务端还没有创建好，客户端就开始调用服务，导致报错
    rospy.wait_for_service('add_server')
    
    try:
        # 创建客户端
        # 给 rospy.ServiceProxy 这个类传入二个参数创建一个 客户端实例client
        # 参数1：客户端实例client的名称
        # 参数2：服务消息的类型（.srv文件）
        client = rospy.ServiceProxy('add_server', AddTwoInts)
        
        # 给AddTwoIntsRequest()类 传入a和b的参数，创建请求消息数据的实例
        req = AddTwoIntsRequest(10, 20)

        # 发送请求并等待响应
        # 方式一：直接传递参数
        # 调用创建的客户端实例client,传入两个参数作为服务消息的请求的参数
        # 返回值是服务端返回的响应消息的实例 AddTwoIntsResponse
        # resp1 = client(10, 20)
        # rospy.loginfo("方式一 - 请求: 10 + 20 = %d" % resp1.sum)
        
        # 方式二：使用请求对象
        # 调用创建的客户端实例client,直接传入req(服务消息的AddTwoIntsRequest请求消息的实例)作为服务消息的请求的参数
        # 返回值是服务端返回的响应消息的实例 AddTwoIntsResponse
        resp2 = client(req)
        rospy.loginfo("方式二 - 请求: %d + %d = %d" % (req.a, req.b, resp2.sum))
        
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s" % e)

if __name__ == "__main__":
    try:
        add_client()
    except rospy.ROSInterruptException:
        pass