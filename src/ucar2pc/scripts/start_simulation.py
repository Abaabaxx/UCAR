#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 这个脚本是为Python 2.7编写的

import rospy
import random
import sys
# 导入我们定义的服务类型
from ucar2pc.srv import FindItem, FindItemRequest

def call_simulation_service():
    # 定义要调用的服务名称
    service_name = '/find_item_simulation'
    
    rospy.loginfo('[小车端] 正在等待名为 \'%s\' 的服务启动...' % service_name)
    rospy.wait_for_service(service_name)
    
    try:
        find_item_service = rospy.ServiceProxy(service_name, FindItem)
        
        # 【修改点1】定义要发送的英文标识符
        items_to_find = ['fruits', 'vegetables', 'desserts']
        chosen_item = random.choice(items_to_find)
        
        # 打印日志时，我们可以把它翻译成中文，方便人类阅读
        log_display_names = {'fruits': '水果', 'vegetables': '蔬菜', 'desserts': '甜品'}
        rospy.loginfo('[小车端] 准备请求电脑开始仿真，寻找目标: %s (发送: %s)' % (log_display_names.get(chosen_item, '未知'), chosen_item))
        
        # 【修改点2】发送的请求中，只包含纯英文标识符
        request = FindItemRequest(item_to_find=chosen_item)
        response = find_item_service(request)
        
        # 打印从电脑返回的响应结果
        rospy.loginfo('------------------------------------------')
        rospy.loginfo('[小车端] 收到电脑的响应！')
        # 【修改点3】收到的也是纯英文标识符，打印时可以翻译
        rospy.loginfo('         目标 \'%s\' 位于: %s' % (log_display_names.get(chosen_item, '未知'), response.room_location))
        rospy.loginfo('------------------------------------------')
        
    except rospy.ServiceException, e:
        rospy.logerr('[小车端] 服务调用失败: %s' % e)

if __name__ == '__main__':
    rospy.init_node('simulation_client_node')
    call_simulation_service()