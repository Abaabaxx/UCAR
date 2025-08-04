#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 这个脚本是为Python 2.7编写的

import rospy
import random
import sys
# 导入我们刚刚定义的服务类型
# 请确保您的包名是 ucar2pc，如果不是请修改
from ucar2pc.srv import FindItem, FindItemRequest

def call_simulation_service():
    # 定义要调用的服务名称
    service_name = '/find_item_simulation'
    
    rospy.loginfo('[小车端] 正在等待名为 \'%s\' 的服务启动...' % service_name)
    
    # rospy.wait_for_service 会阻塞程序，直到服务可用，非常重要！
    rospy.wait_for_service(service_name)
    
    try:
        # 创建一个服务的句柄，就像一个可以直接调用的函数
        find_item_service = rospy.ServiceProxy(service_name, FindItem)
        
        # 随机选择一个要寻找的物品
        items_to_find = ['水果', '甜品', '蔬菜']
        chosen_item = random.choice(items_to_find)
        
        rospy.loginfo('[小车端] 准备请求电脑开始仿真，寻找目标: \'%s\'' % chosen_item)
        
        # 调用服务，并传递请求参数。
        request = FindItemRequest(item_to_find=chosen_item)
        response = find_item_service(request)
        
        # 打印从电脑返回的响应结果
        rospy.loginfo('------------------------------------------')
        rospy.loginfo('[小车端] 收到电脑的响应！')
        rospy.loginfo('         目标 \'%s\' 位于: %s' % (chosen_item, response.room_location))
        rospy.loginfo('------------------------------------------')
        
    except rospy.ServiceException, e:  # Python 2.7 style exception syntax
        rospy.logerr('[小车端] 服务调用失败: %s' % e)

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('simulation_client_node')
    call_simulation_service()