#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from __future__ import division
import json
import time
import threading
import websocket
import sys
import uuid

class TaskRequester:
    def __init__(self, websocket_url="ws://192.168.68.52:9090"):
        """初始化rosbridge连接和通信接口"""
        self.websocket_url = websocket_url
        self.ws = None
        self.connected = False
        self.lock = threading.Lock()
        self.sequence_number = None  # 存储接收到的序列号
        print("TaskRequester initialized")
    
    def on_open(self, ws):
        """WebSocket连接建立时的回调"""
        print("Connected to rosbridge server")
        self.connected = True
        
        # 订阅结果话题
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/object_recognition_result",
            "type": "std_msgs/String"
        }
        ws.send(json.dumps(subscribe_msg))
        print("Subscribed to /object_recognition_result topic")
    
    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            if data.get("topic") == "/object_recognition_result":
                # 获取消息内容
                result_data = data.get("msg", {}).get("data", "")
                print("Received result data:", result_data)
                
                # 将结果存储为序列号
                with self.lock:
                    self.sequence_number = result_data
                print("Updated sequence number to:", self.sequence_number)
                
            elif data.get("topic") == "/object_recognition_request":
                if not self.task_running:
                    self.task_running = True
                    request_data = data.get("msg", {}).get("data", "")
                    print("Received request_data:", request_data)
                    thread = threading.Thread(target=self.run_mission, args=(request_data,))
                    thread.daemon = True
                    thread.start()
                else:
                    print("Task already running, ignoring new request")
        except Exception as e:
            print("Error processing message: {0}".format(e))

    
    def on_error(self, ws, error):
        """WebSocket错误时的回调"""
        print("WebSocket error: {0}".format(error))
        self.connected = False
    
    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket连接关闭时的回调"""
        print("Connection to rosbridge server closed")
        self.connected = False
    
    def connect(self):
        """连接到rosbridge服务器"""
        try:
            print("Connecting to rosbridge server at {0}".format(self.websocket_url))
            self.ws = websocket.WebSocketApp(
                self.websocket_url,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close
            )
            
            # 在后台线程中运行WebSocket
            ws_thread = threading.Thread(target=self.ws.run_forever)
            # Python 2.x 兼容的方式设置守护线程
            ws_thread.daemon = True
            ws_thread.start()
            
            # 等待连接建立
            timeout = 5  # 缩短等待时间到5秒
            while not self.connected and timeout > 0:
                time.sleep(0.1)
                timeout -= 0.1
            
            return self.connected
            
        except Exception as e:
            print("Connection failed: {0}".format(e))
            return False
    
    def publish_request(self, object_name):
        if not self.connected:
            print("Not connected to rosbridge server")
            return False

        try:
                        # 先注册话题类型
            # 先准备要发送的对象字典
            request_obj = {
                "object_name": object_name,  # 你之前的object_name，比如 "fruit"
                "request_id": 1  # 你可以自定义请求ID
            }

            # 将字典转换成JSON字符串
            json_str = json.dumps(request_obj)

            # 广播话题
            advertise_msg = {
                "op": "advertise",
                "topic": "/object_recognition_request",
                "type": "std_msgs/String"
            }
            self.ws.send(json.dumps(advertise_msg))
            print("Advertised /object_recognition_request topic")

            # 发送真正的消息，msg.data是JSON字符串
            publish_msg = {
                "op": "publish",
                "topic": "/object_recognition_request",
                "msg": {
                    "data": json_str
                }
            }
            self.ws.send(json.dumps(publish_msg))
            print("Published message on /object_recognition_request:", json_str)


            return True

        except Exception as e:
            print("Failed to publish message: {0}".format(e))
            return False
    def publish_request(self, object_name):
        if not self.connected:
            print("Not connected to rosbridge server")
            return False

        try:
            # 生成唯一的请求ID
            unique_request_id = str(uuid.uuid4())

            # 先准备要发送的对象字典
            request_obj = {
                "object_name": object_name,  # 比如 "fruit"
                "request_id": unique_request_id  # 每次调用都会生成新的唯一ID
            }

            # 将字典转换成JSON字符串
            json_str = json.dumps(request_obj)

            # 广播话题
            advertise_msg = {
                "op": "advertise",
                "topic": "/object_recognition_request",
                "type": "std_msgs/String"
            }
            self.ws.send(json.dumps(advertise_msg))
            print("Advertised /object_recognition_request topic")

            # 发送真正的消息，msg.data是JSON字符串
            publish_msg = {
                "op": "publish",
                "topic": "/object_recognition_request",
                "msg": {
                    "data": json_str
                }
            }
            self.ws.send(json.dumps(publish_msg))
            print("Published message on /object_recognition_request:", json_str)

            return True

        except Exception as e:
            print("Failed to publish message: {0}".format(e))
            return False
    
    def disconnect(self):
        """断开与rosbridge服务器的连接"""
        if self.ws:
            self.ws.close()
        self.connected = False
        print("Disconnected from rosbridge server")

if __name__ == '__main__':
    try:
        # 创建TaskRequester实例，使用默认的rosbridge地址
        requester = TaskRequester()
        
        # 连接到rosbridge服务器
        if not requester.connect():
            print("Failed to connect to rosbridge server")
            sys.exit(1)
        
        # 从命令行参数获取要查找的对象，或使用默认值
        if len(sys.argv) > 1:
            target_object = sys.argv[1]
        else:
            target_object = "fruit"
        print("Starting task requester for object: '{0}'".format(target_object))
        
        # 发送识别请求
        requester.publish_request(target_object)
        
        # 保持程序运行以接收结果
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nRequester node interrupted by user")
            
    except Exception as e:
        print("Requester node error: {0}".format(e))
    finally:
        if 'requester' in locals():
            requester.disconnect()
        print("Requester node shut down.")