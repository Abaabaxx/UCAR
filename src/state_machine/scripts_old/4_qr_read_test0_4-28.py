#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

"""
修改说明：
1. 添加了二维码内容解析功能，但造成状态机代码臃肿
2. 使用阻塞式延时（rospy.sleep），影响系统响应
3. 缺少双唤醒保护机制

优化建议：
1. 将二维码解析逻辑迁移至二维码识别节点
2. 使用定时器替代阻塞式延时
3. 添加双唤醒保护机制

作者：abaabaxxx
创建时间：2025-04-28 18:13:42
"""

class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_QR1 = 1
    ROTATE_TO_QR2 = 2
    NAVIGATE_TO_QR_AREA = 3
    WAIT_FOR_QR_RESULT = 4  # 新增: 等待二维码识别结果状态
    ERROR = 99

class Event(object):
    VOICE_CMD_WAKEUP = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2
    QR_RESULT_VALID = 3      # 新增: 有效的二维码结果事件
    PERCEPTION_TIMEOUT = 4   # 新增: 感知超时事件

# 新增：任务类型的类
class TaskType(object):
    FRUITS = "fruits"         # 水果任务
    VEGETABLES = "vegetables" # 蔬菜任务
    DESSERTS = "desserts"     # 甜品任务
    UNKNOWN = "unknown"       # 未知任务

class RobotStateMachine(object):
    def __init__(self):
        rospy.init_node('robot_state_machine')
        
        self.is_awake = False
        self.last_awake_angle = None
        self.current_state = RobotState.IDLE
        self.navigation_active = False
        
        # 新增: 二维码相关变量
        self.qr_perception_timeout = 5.0  # 5秒感知超时
        self.qr_timer = None              # 二维码感知超时定时器
        self.task_type = TaskType.UNKNOWN # 任务类型
        self.qr_content = ""              # 二维码内容
        
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        self.task_pub = rospy.Publisher('/robot/task_type', String, queue_size=1)  # 新增: 发布任务类型
        
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        self.awake_sub = rospy.Subscriber('/mic/awake/angle', Int32, self.awake_callback, queue_size=1)
        
        self.qr_sub = rospy.Subscriber('/QR/result', String, self.qr_callback, queue_size=1)
        
        self.locations = {
            'qr1': self.create_pose(
                2.07, 
                0.50, 
                0.2588,
                0.9659
            ),
            'qr2': self.create_pose(
                2.07, 
                0.50, 
                0.9659,
                0.2588
            ),
            'qr_area': self.create_pose(
                1.25, 
                0.75, 
                -1.0,
                0.0
            )
        }
        
        rospy.loginfo("状态机初始化完成")
        rospy.loginfo("等待语音唤醒...")
        
        self.publish_state()
    
    def create_pose(self, x, y, z_orientation, w_orientation):
        pose = PoseStamped()
        pose.header.frame_id = "map"       
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z_orientation
        pose.pose.orientation.w = w_orientation
        
        return pose 
    
    def state_name(self, state):
        for attr in dir(RobotState):
            if not attr.startswith('__') and getattr(RobotState, attr) == state:
                return attr
        return "UNKNOWN"
    
    def event_name(self, event):
        for attr in dir(Event):
            if not attr.startswith('__') and getattr(Event, attr) == event:
                return attr
        return "UNKNOWN"
    
    def publish_state(self):
        state_name = self.state_name(self.current_state)
        self.state_pub.publish("State: " + state_name)
        rospy.loginfo("当前状态: %s", state_name)
    
    # 新增: 发布任务类型的方法
    def publish_task_type(self):
        self.task_pub.publish(self.task_type)
        rospy.loginfo("任务类型: %s", self.task_type)
    
    def transition(self, new_state):
        old_state_name = self.state_name(self.current_state)
        new_state_name = self.state_name(new_state)
        rospy.loginfo("状态转换: %s -> %s", old_state_name, new_state_name)
        
        self.current_state = new_state
        self.publish_state()
        self.execute_state_actions()
    
    def execute_state_actions(self):
        if self.current_state == RobotState.IDLE:
            rospy.loginfo("0-IDLE 空闲状态，等待唤醒...")
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR1:
            rospy.loginfo("1-导航至QR1")
            self.send_nav_goal('qr1')
            self.navigation_active = True
            
        elif self.current_state == RobotState.ROTATE_TO_QR2:
            rospy.loginfo("2-原地旋转至QR2")
            self.send_nav_goal('qr2')
            self.navigation_active = True
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR_AREA:
            rospy.loginfo("3-导航至二维码区QR")
            self.send_nav_goal('qr_area')
            self.navigation_active = True
            
        # 新增: 等待二维码识别结果状态的执行动作    
        elif self.current_state == RobotState.WAIT_FOR_QR_RESULT:  # 新增: 等待二维码识别结果状态的执行动作
            rospy.loginfo("4-等待二维码识别结果")
            # 启动感知超时定时器
            self.start_perception_timer()
            
        elif self.current_state == RobotState.ERROR:
            rospy.logerr("99-错误状态，停止所有活动")
            self.stop_all_activities()
    
    def handle_event(self, event):
        event_name = self.event_name(event)
        state_name = self.state_name(self.current_state)
        rospy.loginfo("在状态 %s 收到事件 %s", state_name, event_name)
        
        # 状态转换逻辑
        if self.current_state == RobotState.IDLE and event == Event.VOICE_CMD_WAKEUP:
            self.transition(RobotState.NAVIGATE_TO_QR1)
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR1:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.ROTATE_TO_QR2)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.ROTATE_TO_QR2:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.NAVIGATE_TO_QR_AREA)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAVIGATE_TO_QR_AREA:
            if event == Event.NAV_DONE_SUCCESS:
                rospy.loginfo("导航至二维码区域完成")
                # 新增: 导航到二维码区域成功后，转换到等待二维码识别结果状态
                self.transition(RobotState.WAIT_FOR_QR_RESULT)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        # 新增: 等待二维码识别结果状态的事件处理逻辑
        elif self.current_state == RobotState.WAIT_FOR_QR_RESULT:
            if event == Event.QR_RESULT_VALID:
                # 收到有效的二维码结果，停止感知超时定时器
                self.stop_perception_timer()
                rospy.loginfo("收到有效的二维码结果: %s", self.qr_content)
                self.parse_qr_content()  # 解析二维码内容，确定任务类型
                self.publish_task_type()  # 发布任务类型
                # 根据Mermaid图，保持在当前状态
            elif event == Event.PERCEPTION_TIMEOUT:
                # 感知超时，转换到错误状态
                rospy.logerr("二维码识别超时")
                self.transition(RobotState.ERROR)
    
    def awake_callback(self, msg):
        self.last_awake_angle = msg.data
        rospy.loginfo("收到唤醒角度: %d度", self.last_awake_angle)

        if self.current_state == RobotState.IDLE and not self.is_awake:
            self.is_awake = True
            rospy.loginfo("Ucar被唤醒，等待0.3秒让系统准备...")
            rospy.sleep(0.3)  # 添加0.3秒延时
            rospy.loginfo("系统准备完成，开始状态转换...")
            self.handle_event(Event.VOICE_CMD_WAKEUP)
    
    # 新增: 二维码识别结果回调函数
    def qr_callback(self, msg):
        if self.current_state == RobotState.WAIT_FOR_QR_RESULT:
            self.qr_content = msg.data
            if self.qr_content and len(self.qr_content) > 0:
                rospy.loginfo("接收到二维码内容: %s", self.qr_content)
                self.handle_event(Event.QR_RESULT_VALID)
            else:
                rospy.logwarn("接收到空的二维码内容")
    
    # 新增: 解析二维码内容，确定任务类型
    def parse_qr_content(self):
        # 转为小写用于不区分大小写的比较
        content_lower = self.qr_content.lower()
        
        # 水果类型关键词
        fruit_keywords = ['fruit', 'fruits']
        # 蔬菜类型关键词
        vegetable_keywords = ['vegetable', 'vegetables']
        # 甜品类型关键词 (包括dessert和sweet)
        dessert_keywords = ['dessert', 'desserts', 'sweet', 'sweets']
        
        # 判断内容属于哪种任务类型
        if any(keyword in content_lower for keyword in fruit_keywords):
            self.task_type = TaskType.FRUITS
            rospy.loginfo("任务类型设置为: 水果 (原始内容: %s)", self.qr_content)
        elif any(keyword in content_lower for keyword in vegetable_keywords):
            self.task_type = TaskType.VEGETABLES
            rospy.loginfo("任务类型设置为: 蔬菜 (原始内容: %s)", self.qr_content)
        elif any(keyword in content_lower for keyword in dessert_keywords):
            self.task_type = TaskType.DESSERTS
            rospy.loginfo("任务类型设置为: 甜品 (原始内容: %s)", self.qr_content)
        else:
            self.task_type = TaskType.UNKNOWN
            rospy.logwarn("未知的任务类型: %s", self.qr_content)
        
        # 发布任务类型
        self.publish_task_type()
    
    # 新增: 启动感知超时定时器方法
    def start_perception_timer(self):
        # 如果已有定时器在运行，先停止它
        self.stop_perception_timer()
        
        rospy.loginfo("启动二维码感知超时定时器: %.1f秒", self.qr_perception_timeout)
        self.qr_timer = rospy.Timer(rospy.Duration(self.qr_perception_timeout), 
                                    self.perception_timeout_callback,
                                    oneshot=True)
    
    # 新增: 停止感知超时定时器方法
    def stop_perception_timer(self):
        if self.qr_timer:
            self.qr_timer.shutdown()
            self.qr_timer = None
            rospy.loginfo("停止二维码感知超时定时器")
    
    # 新增: 感知超时回调函数
    def perception_timeout_callback(self, event):
        rospy.logwarn("二维码感知超时")
        self.handle_event(Event.PERCEPTION_TIMEOUT)
    
    def send_nav_goal(self, location_name):
        if location_name in self.locations:
            goal = MoveBaseGoal()
            goal.target_pose = self.locations[location_name]
            goal.target_pose.header.stamp = rospy.Time.now()
            
            self.move_base_client.send_goal(goal, done_cb=self.navigation_done_callback)
            rospy.loginfo("导航至: %s", location_name)
        else:
            rospy.logerr("未知位置: %s", location_name)
            self.handle_event(Event.NAV_DONE_FAILURE)
    
    def navigation_done_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            current_state_name = self.state_name(self.current_state)
            if self.current_state in [RobotState.NAVIGATE_TO_QR1, RobotState.ROTATE_TO_QR2]:
                rospy.loginfo("导航成功，等待0.3秒让AMCL校准...")
                rospy.sleep(0.3) 
                rospy.loginfo("AMCL校准等待完成，继续状态转换...")
            self.handle_event(Event.NAV_DONE_SUCCESS)
        else:
            self.handle_event(Event.NAV_DONE_FAILURE)
        self.navigation_active = False
    
    def stop_all_activities(self):
        if self.navigation_active:
            self.move_base_client.cancel_all_goals()
            self.navigation_active = False
        
        # 新增: 在停止所有活动时关闭感知超时定时器
        self.stop_perception_timer()
        
        rospy.loginfo("已停止所有活动")

if __name__ == '__main__':
    try:
        robot_sm = RobotStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass