#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
"""
修改说明：
1. 添加了服务通信启动功能
2. 保留原有的语音唤醒功能
3. 统一使用 START_CMD 事件

使用方法：
1. 启动状态机：
   方式1 - 服务调用：rosservice call /start_state_machine "{}"
   方式2 - 语音唤醒：通过语音唤醒
2. 重置状态机：
   rosservice call /reset_state_machine "{}"

作者：abaabaxxx
创建时间：2025-04-29 13:39:47
"""
class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_QR1 = 1
    ROTATE_TO_QR2 = 2
    NAVIGATE_TO_QR_AREA = 3
    ERROR = 99

class Event(object):
    START_CMD = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2

class RobotStateMachine(object):
    def __init__(self):
        rospy.init_node('robot_state_machine')
        
        # 修改：添加定时器成员变量
        self.delay_timer = None
        
        self.setup()
        self.init_ros_comm()
        self.init_locations()
        
        rospy.loginfo("状态机初始化完成（支持语音唤醒和服务调用）")
        rospy.loginfo("等待语音唤醒或服务调用...")
        
        self.publish_state()

    def setup(self):
        self.is_awake = False
        self.last_awake_angle = None
        self.current_state = RobotState.IDLE
        self.navigation_active = False

    def init_ros_comm(self):
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        self.awake_sub = rospy.Subscriber('/mic/awake/angle', Int32, self.awake_callback, queue_size=1)
        
        self.reset_service = rospy.Service('reset_state_machine', Trigger, self.reset_callback)
        self.start_service = rospy.Service('start_state_machine', Trigger, self.start_callback)

    def init_locations(self):
        self.locations = {
            'qr1': self.create_pose(2.07, 0.60, 0.3007, 0.9537),
            'qr2': self.create_pose(2.07, 0.60, 0.9747, 0.2239),
            'qr_area': self.create_pose(1.25, 0.75, -1.0, 0.0)
        }

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
            
        elif self.current_state == RobotState.ERROR:
            rospy.logerr("99-错误状态，停止所有活动")
            self.stop_all_activities()

    def handle_event(self, event):
        event_name = self.event_name(event)
        state_name = self.state_name(self.current_state)
        rospy.loginfo("在状态 %s 收到事件 %s", state_name, event_name)
        
        if self.current_state == RobotState.IDLE and event == Event.START_CMD:
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
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)

    # 修改：使用Timer替代sleep
    def awake_callback(self, msg):
        self.last_awake_angle = msg.data
        rospy.loginfo("收到唤醒角度: %d度", self.last_awake_angle)

        if self.current_state == RobotState.IDLE and not self.is_awake:
            self.is_awake = True
            rospy.loginfo("通过语音唤醒，启动1秒定时器...")
            rospy.Timer(rospy.Duration(1.0), 
                       lambda event: self.handle_event(Event.START_CMD), 
                       oneshot=True)

    def start_callback(self, req):
        if self.current_state == RobotState.IDLE:
            rospy.loginfo("通过服务调用启动，开始状态转换...")
            self.handle_event(Event.START_CMD)
            return TriggerResponse(
                success=True,
                message=str("State machine started successfully")
            )
        else:
            return TriggerResponse(
                success=False,
                message="State machine is not in IDLE state, cannot start"
            )

    def reset_callback(self, req):
        rospy.loginfo("收到重置服务请求，准备重置状态机...")
        try:
            self.stop_all_activities()
            self.setup()
            self.publish_state()
            rospy.loginfo("状态机重置完成，等待新的启动...")
            return TriggerResponse(
                success=True,
                message="State machine reset successfully"
            )
        except Exception as e:
            rospy.logerr("重置状态机时出错: %s" % str(e))
            return TriggerResponse(
                success=False,
                message="Reset failed: %s" % str(e)
            )

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

    # 修改：使用Timer实现延迟导航成功回调
    def navigation_done_callback(self, status, result):
        self.navigation_active = False
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("导航成功，启动0.5秒定时器...")
            rospy.Timer(rospy.Duration(0.5), 
                       lambda event: self.delayed_nav_success(), 
                       oneshot=True)
        else:
            self.handle_event(Event.NAV_DONE_FAILURE)

    # 修改：添加延迟导航成功处理方法
    def delayed_nav_success(self):
        rospy.loginfo("0.5秒定时结束，执行状态转换...")
        self.handle_event(Event.NAV_DONE_SUCCESS)

    # 修改：添加定时器清理
    def stop_all_activities(self):
        if self.navigation_active:
            self.move_base_client.cancel_all_goals()
            self.navigation_active = False
        self.is_awake = False
        rospy.loginfo("已停止所有活动")

if __name__ == '__main__':
    try:
        robot_sm = RobotStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass