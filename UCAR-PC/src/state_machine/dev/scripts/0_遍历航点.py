#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger, TriggerResponse

"""
遍历航点状态机
功能：控制机器人依次导航到6个预设点位，导航失败时执行原地旋转脱困

使用方法：
1. 启动状态机：
   rosservice call /start_state_machine "{}"
2. 重置状态机：
   rosservice call /reset_state_machine "{}"

作者：Claude 3.7 Sonnet
创建时间：2024-08-02
"""

# 定义导航超时常量（单位：秒）
NAV_TIMEOUT_SECONDS = 30.0

class RobotState(object):
    IDLE = 0
    NAV_TO_C1 = 1
    NAV_TO_C2 = 2
    NAV_TO_B1 = 3
    NAV_TO_A1 = 4
    NAV_TO_MIDPOINT = 5
    NAV_TO_END = 6
    COMPLETED = 7
    RESCUE_ROTATION = 8

class Event(object):
    START_CMD = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2
    RESCUE_DONE = 3

class RobotStateMachine(object):
    def __init__(self):
        rospy.init_node('waypoint_follower_state_machine')
        
        # 记录进入救援状态前的导航状态
        self.previous_nav_state = None
        
        # 初始化导航超时定时器
        self.nav_timeout_timer = None
        
        # 初始化状态机
        self.setup()
        self.init_ros_comm()
        self.init_locations()
        
        rospy.loginfo("航点遍历状态机初始化完成")
        rospy.loginfo("等待服务调用启动...")
        
        self.publish_state()

    def setup(self):
        self.current_state = RobotState.IDLE
        self.navigation_active = False

    def init_ros_comm(self):
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        self.reset_service = rospy.Service('reset_state_machine', Trigger, self.reset_callback)
        self.start_service = rospy.Service('start_state_machine', Trigger, self.start_callback)

    def init_locations(self):
        # 从bash.txt提取的6个导航点坐标
        self.waypoints = {
            RobotState.NAV_TO_C1: self.create_pose(3.520860195159912, 0.6933646202087402, 0.5552674883213985, 0.8316718201389569),
            RobotState.NAV_TO_C2: self.create_pose(3.7584171295166016, 2.073887825012207, 0.36341517937667966, 0.9316272899602156),
            RobotState.NAV_TO_B1: self.create_pose(2.8548004627227783, 1.0420169830322266, 0.8185138937393395, 0.5744867324452892),
            RobotState.NAV_TO_A1: self.create_pose(1.0426336526870728, 0.8178969025611877, 0.798000045307492, 0.6026573883138252),
            RobotState.NAV_TO_MIDPOINT: self.create_pose(4.238326549530029, 0.9244804382324219, -0.6074919171092937, 0.7943258592333974),
            RobotState.NAV_TO_END: self.create_pose(0.1385047435760498, 0.03395068645477295, -0.005471294456532419, 0.9999850323564697)
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
            rospy.loginfo("0-IDLE 空闲状态，等待启动...")
            
        elif self.current_state == RobotState.NAV_TO_C1:
            rospy.loginfo("1-导航至C1房间观测点1")
            self.send_nav_goal(RobotState.NAV_TO_C1)
            
        elif self.current_state == RobotState.NAV_TO_C2:
            rospy.loginfo("2-导航至C2房间观测点2")
            self.send_nav_goal(RobotState.NAV_TO_C2)
            
        elif self.current_state == RobotState.NAV_TO_B1:
            rospy.loginfo("3-导航至B1房间观测点")
            self.send_nav_goal(RobotState.NAV_TO_B1)
            
        elif self.current_state == RobotState.NAV_TO_A1:
            rospy.loginfo("4-导航至A1房间观测点")
            self.send_nav_goal(RobotState.NAV_TO_A1)
            
        elif self.current_state == RobotState.NAV_TO_MIDPOINT:
            rospy.loginfo("5-导航至返程中间点")
            self.send_nav_goal(RobotState.NAV_TO_MIDPOINT)
            
        elif self.current_state == RobotState.NAV_TO_END:
            rospy.loginfo("6-导航至终点")
            self.send_nav_goal(RobotState.NAV_TO_END)
            
        elif self.current_state == RobotState.COMPLETED:
            rospy.loginfo("7-任务完成！所有航点已成功导航")
            
        elif self.current_state == RobotState.RESCUE_ROTATION:
            rospy.loginfo("8-旋转脱困：开始执行2秒旋转...")
            self.perform_rescue_rotation()

    def handle_event(self, event):
        event_name = self.event_name(event)
        state_name = self.state_name(self.current_state)
        rospy.loginfo("在状态 %s 收到事件 %s", state_name, event_name)
        
        if self.current_state == RobotState.IDLE and event == Event.START_CMD:
            self.transition(RobotState.NAV_TO_C1)
            
        elif self.current_state == RobotState.NAV_TO_C1:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.NAV_TO_C2)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.NAV_TO_C2:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.NAV_TO_B1)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.NAV_TO_B1:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.NAV_TO_A1)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.NAV_TO_A1:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.NAV_TO_MIDPOINT)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.NAV_TO_MIDPOINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.NAV_TO_END)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.NAV_TO_END:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.COMPLETED)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.RESCUE_ROTATION:
            if event == Event.RESCUE_DONE:
                if self.previous_nav_state is not None:
                    self.transition(self.previous_nav_state)

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

    def send_nav_goal(self, waypoint_state):
        if waypoint_state in self.waypoints:
            goal = MoveBaseGoal()
            goal.target_pose = self.waypoints[waypoint_state]
            goal.target_pose.header.stamp = rospy.Time.now()
            
            self.move_base_client.send_goal(goal, done_cb=self.navigation_done_callback)
            rospy.loginfo("导航至: %s", self.state_name(waypoint_state))
            self.navigation_active = True
            
            # 关闭旧的超时定时器（如果存在）
            if self.nav_timeout_timer:
                self.nav_timeout_timer.shutdown()
                
            # 创建新的超时定时器
            self.nav_timeout_timer = rospy.Timer(
                rospy.Duration(NAV_TIMEOUT_SECONDS),
                self.navigation_timeout_callback,
                oneshot=True
            )
        else:
            rospy.logerr("未知航点: %s", self.state_name(waypoint_state))
            self.handle_event(Event.NAV_DONE_FAILURE)

    def navigation_timeout_callback(self, event):
        """导航超时回调函数"""
        rospy.logwarn("导航超时（%s秒）！取消导航目标...", NAV_TIMEOUT_SECONDS)
        self.move_base_client.cancel_all_goals()
        # 注意：取消目标会自动触发navigation_done_callback，
        # 因此不需要在这里手动触发NAV_DONE_FAILURE事件

    def navigation_done_callback(self, status, result):
        # 首先关闭超时定时器
        if self.nav_timeout_timer:
            self.nav_timeout_timer.shutdown()
            
        self.navigation_active = False
        
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("导航成功！")
            rospy.Timer(rospy.Duration(0.5), 
                      lambda event: self.handle_event(Event.NAV_DONE_SUCCESS), 
                      oneshot=True)
        else:
            rospy.logwarn("导航失败，状态码: %d，准备进入旋转救援...", status)
            self.handle_event(Event.NAV_DONE_FAILURE)

    def perform_rescue_rotation(self):
        """执行旋转救援动作"""
        # 创建旋转速度指令
        twist = Twist()
        twist.angular.z = 2.0  # 2 rad/s
        
        # 发布旋转速度
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("开始旋转...")
        
        # 启动定时器，2秒后停止旋转
        rospy.Timer(rospy.Duration(2.0), 
                   lambda event: self.stop_rotation_and_recover(), 
                   oneshot=True)

    def stop_rotation_and_recover(self):
        """停止旋转并触发恢复事件"""
        # 发布零速度指令停止旋转
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("旋转完成，准备重新导航...")
        
        # 触发救援完成事件
        self.handle_event(Event.RESCUE_DONE)

    def stop_all_activities(self):
        if self.navigation_active:
            self.move_base_client.cancel_all_goals()
            # 关闭超时定时器
            if self.nav_timeout_timer:
                self.nav_timeout_timer.shutdown()
            self.navigation_active = False
        # 确保机器人停止运动
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("已停止所有活动")

if __name__ == '__main__':
    try:
        robot_sm = RobotStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass