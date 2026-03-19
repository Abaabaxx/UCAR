#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

# 命令行启动：rosservice call /go_board_detect "{}"

# 状态常量定义
class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_UP_POINT = 1

# 事件常量定义
class Event(object):
    START_CMD = 0

# 主类框架
class RobotStateMachine(object):
    def __init__(self):
        rospy.init_node('board_detect_state_machine', anonymous=True)
        self.setup()
        self.init_ros_comm()
        rospy.loginfo("状态机初始化完成，等待服务调用...")
        self.publish_state()

    # 成员变量设置
    def setup(self):
        self.current_state = RobotState.IDLE

    # ROS接口初始化
    def init_ros_comm(self):
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        self.start_service = rospy.Service('/go_board_detect', Trigger, self.start_callback)

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
            rospy.loginfo("开始模拟导航")
            rospy.Timer(rospy.Duration(2.0), 
                       lambda event: rospy.loginfo("模拟导航成功"), 
                       oneshot=True)

    # 核心方法：事件处理
    def handle_event(self, event):
        if self.current_state == RobotState.IDLE and event == Event.START_CMD:
            self.transition(RobotState.NAVIGATE_TO_UP_POINT)

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