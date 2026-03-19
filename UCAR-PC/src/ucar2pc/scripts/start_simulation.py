#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 这个脚本可以在Python 2.7或Python 3环境下运行

import roslibpy
import random
import sys
import time

def call_simulation_service(client):
    # 定义要调用的服务名称
    service_name = '/start_state_machine'
    
    print('[小车端] 准备调用电脑端状态机服务 \'%s\'' % service_name)
    
    try:
        # 创建服务客户端，指向电脑端的状态机服务
        find_item_service = roslibpy.Service(client, service_name, 'ucar2pc/FindItem')
        
        # 1. 修改发送的请求为固定的'fruits'
        chosen_item = 'fruits'
        
        # 打印日志时，我们可以把它翻译成中文，方便人类阅读
        log_display_names = {'fruits': '水果', 'vegetables': '蔬菜', 'desserts': '甜品'}
        print('[小车端] 准备请求状态机开始任务，寻找目标: %s (发送: %s)' % (log_display_names.get(chosen_item, '未知'), chosen_item))
        
        # 发送的请求中，只包含纯英文标识符
        request = roslibpy.ServiceRequest({'item_to_find': chosen_item})
        
        # 调用服务并等待结果（阻塞式）
        print('[小车端] 正在发送请求并等待结果...')
        response = find_item_service.call(request)
        
        # 2. 兼容并打印新的双字段响应
        print('------------------------------------------')
        print('[小车端] 收到状态机的响应！')
        print('         - 目标类别: %s' % log_display_names.get(chosen_item, '未知'))
        # 使用.get()方法安全地访问字典键值
        print('         - 所在房间: %s' % response.get('room_location', '未知'))
        print('         - 找到物品: %s' % response.get('found_item_name', '未找到'))
        print('------------------------------------------')
        
    except Exception as e:
        print('[小车端] 服务调用失败: %s' % e)

def main():
    # --- 请在这里修改为你电脑的IP地址 ---
    PC_IP = '192.168.68.206'  # 请确保IP地址正确！
    # -----------------------------------------
    
    # 创建roslibpy客户端连接到电脑端的rosbridge
    client = roslibpy.Ros(host=PC_IP, port=9090)
    
    try:
        # 启动客户端连接
        client.run()
        
        # 等待连接建立
        while not client.is_connected:
            print('[小车端] 正在连接到电脑端的rosbridge...')
            time.sleep(1)
            
        print('[小车端] 连接成功！')
        
        # 调用服务
        call_simulation_service(client)
        
    except KeyboardInterrupt:
        print('[小车端] 用户中断程序')
    except Exception as e:
        print('[小车端] 发生错误: %s' % e)
    finally:
        # 确保关闭连接
        if client.is_connected:
            client.terminate()
        print('[小车端] 程序已退出')

if __name__ == '__main__':
    main()