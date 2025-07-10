#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_QR1 = 1  # 修改: 更新状态名称为导航至QR1
    ROTATE_TO_QR2 = 2    # 新增: 添加原地旋转至QR2状态
    NAVIGATE_TO_QR_AREA = 3  # 新增: 添加导航至二维码区QR状态
    ERROR = 99

class Event(object):
    VOICE_CMD_WAKEUP = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2

class RobotStateMachine(object):
    def __init__(self):
        rospy.init_node('robot_state_machine')
        
        self.is_awake = False
        self.last_awake_angle = None
        self.current_state = RobotState.IDLE
        self.navigation_active = False
        
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        
        # 新增: 创建真实的导航客户端替代模拟
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        self.awake_sub = rospy.Subscriber('/mic/awake/angle', Int32, self.awake_callback, queue_size=1)
        
        # 修改: 更新导航位置字典，添加QR1和QR2位置
        self.locations = {
            # QR1位置
            'qr1': self.create_pose(
                2.07, 
                0.50, 
                0.2588,
                0.9659
            ),
            # QR2位置（与QR1位置相同，但朝向不同）
            'qr2': self.create_pose(
                2.07, 
                0.50, 
                0.9659,
                0.2588
            ),
            # 更新二维码区域坐标
            'qr_area': self.create_pose(
                1.25, 
                0.75, 
                -1.0,
                0.0
            )
        }
        
        rospy.loginfo("状态机初始化完成（功能版本）")
        rospy.loginfo("等待语音唤醒...")
        
        self.publish_state()
    
    # 新增: 创建导航位置的位姿
    def create_pose(self, x, y, z_orientation, w_orientation):
        pose = PoseStamped()
        pose.header.frame_id = "map"        # 使用 map 坐标系
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 使用完整的四元数表示朝向
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
    
    def transition(self, new_state):
        old_state_name = self.state_name(self.current_state)
        new_state_name = self.state_name(new_state)
        rospy.loginfo("状态转换: %s -> %s", old_state_name, new_state_name)
        
        self.current_state = new_state
        self.publish_state()
        self.execute_state_actions()
    
    def execute_state_actions(self):
        if self.current_state == RobotState.IDLE:
            rospy.loginfo("0-IDLE 空闲状态，等待唤醒...")  # 修改: 更新日志信息
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR1:  # 修改: 原NAVIGATE_TO_QR改为NAVIGATE_TO_QR1
            rospy.loginfo("1-导航至QR1")  # 修改: 添加状态描述日志
            self.send_nav_goal('qr1')  # 修改: 导航目标改为qr1
            self.navigation_active = True
            
        elif self.current_state == RobotState.ROTATE_TO_QR2:  # 新增: 添加旋转到QR2的状态处理
            rospy.loginfo("2-原地旋转至QR2")
            self.send_nav_goal('qr2')
            self.navigation_active = True
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR_AREA:  # 新增: 添加导航到二维码区域的状态处理
            rospy.loginfo("3-导航至二维码区QR")
            self.send_nav_goal('qr_area')
            self.navigation_active = True
            
        elif self.current_state == RobotState.ERROR:
            rospy.logerr("99-错误状态，停止所有活动")  # 修改: 更新错误状态日志
            self.stop_all_activities()
    
    def handle_event(self, event):
        event_name = self.event_name(event)
        state_name = self.state_name(self.current_state)
        rospy.loginfo("在状态 %s 收到事件 %s", state_name, event_name)
        
        # 修改: 更新状态转换逻辑以匹配新的状态图
        if self.current_state == RobotState.IDLE and event == Event.VOICE_CMD_WAKEUP:
            self.transition(RobotState.NAVIGATE_TO_QR1)  # 修改: 唤醒后转到导航至QR1状态
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR1:  # 修改: 原NAVIGATE_TO_QR改为NAVIGATE_TO_QR1
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.ROTATE_TO_QR2)  # 修改: 导航成功后转到旋转至QR2状态
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.ROTATE_TO_QR2:  # 新增: 添加旋转至QR2状态的事件处理
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.NAVIGATE_TO_QR_AREA)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAVIGATE_TO_QR_AREA:  # 新增: 添加导航至二维码区状态的事件处理
            if event == Event.NAV_DONE_SUCCESS:
                rospy.loginfo("导航至二维码区域完成")
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
    
    def awake_callback(self, msg):
        self.last_awake_angle = msg.data
        rospy.loginfo("收到唤醒角度: %d度", self.last_awake_angle)

        if self.current_state == RobotState.IDLE and not self.is_awake:
            self.is_awake = True
            # rospy.loginfo("Ucar被唤醒，准备开始状态转换...")
            rospy.loginfo("Ucar被唤醒，等待1秒后开始状态转换...")
            rospy.sleep(1.0) # 添加1秒延时 
            self.handle_event(Event.VOICE_CMD_WAKEUP)
    
    # 新增: 实际发送导航目标的方法
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
    
    # 新增: 实际导航完成的回调函数
    def navigation_done_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.handle_event(Event.NAV_DONE_SUCCESS)
        else:
            self.handle_event(Event.NAV_DONE_FAILURE)
        self.navigation_active = False
    
    # 新增: 实际停止所有活动的方法
    def stop_all_activities(self):
        if self.navigation_active:
            self.move_base_client.cancel_all_goals()
            self.navigation_active = False
        
        rospy.loginfo("已停止所有活动")

if __name__ == '__main__':
    try:
        robot_sm = RobotStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass