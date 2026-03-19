#!/usr/bin/env python
# -*- coding: utf-8 -*-


# 命令行启动：rosservice call /go_board_detect "{}"

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

# 状态常量定义
class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_UP_POINT = 1
    SEARCH_UP_BOARD = 2
    ERROR = 99

# 事件常量定义
class Event(object):
    START_CMD = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2

# 主类框架
class RobotStateMachine(object):
    def __init__(self):
        rospy.init_node('board_detect_state_machine', anonymous=True)
        self.setup()
        self.init_ros_comm()
        self.init_locations()
        rospy.loginfo("状态机初始化完成，等待服务调用...")
        self.publish_state()

    # 成员变量设置
    def setup(self):
        self.current_state = RobotState.IDLE
        self.navigation_active = False
        self.locations = {}

    # ROS接口初始化
    def init_ros_comm(self):
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        self.start_service = rospy.Service('/go_board_detect', Trigger, self.start_callback)
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()

    # 初始化位置
    def init_locations(self):
        self.locations = {
            'up_point': self.create_pose(1.25, 3.50, 0.7071, 0.7071)
        }

    # 创建目标位置的函数
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

    # 发送导航目标
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

    # 导航完成回调函数
    def navigation_done_callback(self, status, result):
        self.navigation_active = False
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.delayed_nav_success()
        else:
            self.handle_event(Event.NAV_DONE_FAILURE)

    # 延迟导航成功处理
    def delayed_nav_success(self):
        rospy.Timer(rospy.Duration(0.5), 
                   lambda event: self.handle_event(Event.NAV_DONE_SUCCESS), 
                   oneshot=True)

    # 停止所有活动
    def stop_all_activities(self):
        if self.navigation_active:
            self.move_base_client.cancel_all_goals()
            self.navigation_active = False
        rospy.loginfo("已停止所有活动")

    # 工具方法：获取状态名称
    def state_name(self, state):
        for attr in dir(RobotState):
            if not attr.startswith('__') and getattr(RobotState, attr) == state:
                return attr
        return "UNKNOWN"

    # 工具方法：发布当前状态
    def publish_state(self):
        state_name = self.state_name(self.current_state)
        self.state_pub.publish("State: " + state_name)
        rospy.loginfo("当前状态: %s", state_name)

    # 核心方法：状态转换
    def transition(self, new_state):
        old_state_name = self.state_name(self.current_state)
        new_state_name = self.state_name(new_state)
        rospy.loginfo("状态转换: %s -> %s", old_state_name, new_state_name)
        
        self.current_state = new_state
        self.publish_state()
        self.execute_state_actions()

    # 核心方法：执行状态对应的动作
    def execute_state_actions(self):
        if self.current_state == RobotState.IDLE:
            rospy.loginfo("IDLE 空闲状态，等待启动...")
        elif self.current_state == RobotState.NAVIGATE_TO_UP_POINT:
            rospy.loginfo("开始导航至上平面位置...")
            self.send_nav_goal('up_point')
            self.navigation_active = True
        elif self.current_state == RobotState.SEARCH_UP_BOARD:
            rospy.loginfo("进入寻找上平面板子状态...")
        elif self.current_state == RobotState.ERROR:
            rospy.logerr("错误状态，停止所有活动")
            self.stop_all_activities()

    # 核心方法：事件处理
    def handle_event(self, event):
        if self.current_state == RobotState.IDLE and event == Event.START_CMD:
            self.transition(RobotState.NAVIGATE_TO_UP_POINT)
        elif self.current_state == RobotState.NAVIGATE_TO_UP_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.SEARCH_UP_BOARD)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)

    # 服务回调：启动状态机
    def start_callback(self, req):
        if self.current_state == RobotState.IDLE:
            rospy.loginfo("通过服务调用启动，开始状态转换...")
            self.handle_event(Event.START_CMD)
            return TriggerResponse(
                success=True,
                message="State machine started successfully"
            )
        else:
            return TriggerResponse(
                success=False,
                message="State machine is not in IDLE state, cannot start"
            )

# 程序入口
if __name__ == '__main__':
    try:
        robot_sm = RobotStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass