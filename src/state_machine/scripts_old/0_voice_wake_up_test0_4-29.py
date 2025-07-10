#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import actionlib
from std_srvs.srv import Trigger, TriggerResponse  # 使用标准的Trigger服务类型

# 该版本为话题启动小车的单唤醒的测试版本

# 新增：该版本添加了使用服务通信重置状态机状态的的功能
# 新增：使用以下指令重置状态机状态 rosservice call /reset_state_machine "{}"

# 定义机器人状态类
class RobotState(object):
    IDLE = 0                   # 空闲状态，机器人处于静止或等待状态
    NAVIGATE_TO_QR = 1         # 导航至二维码区域状态
    ERROR = 99                 # 错误状态，当出现问题时进入此状态

# 定义事件类
class Event(object):
    VOICE_CMD_WAKEUP = 0       # 语音唤醒命令事件
    NAV_DONE_SUCCESS = 1       # 导航成功完成事件
    NAV_DONE_FAILURE = 2       # 导航失败事件

# 机器人的状态机类
class RobotStateMachine(object):
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('robot_state_machine')
        
        # 首先调用setup函数进行状态变量初始化
        self.setup()
        
        # 初始化完成后，再创建发布者、订阅者和服务
        self.init_ros_comm()
        
        rospy.loginfo("状态机初始化完成（测试版本-仅状态转换）")
        rospy.loginfo("等待语音唤醒...")
        
        # 发布初始状态
        self.publish_state()
    
    def setup(self):
        """初始化/重置所有状态变量"""
        self.is_awake = False  # 初始化唤醒状态标志
        self.last_awake_angle = None  # 初始化最后唤醒角度
        self.current_state = RobotState.IDLE  # 初始化当前状态为空闲
        self.navigation_active = False  # 导航状态标志
        rospy.loginfo("状态机变量已初始化完成")
    
    def init_ros_comm(self):
        """初始化所有ROS通信相关的对象"""
        # 创建发布者
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        
        # 创建订阅者
        self.awake_sub = rospy.Subscriber('/mic/awake/angle', Int32, self.awake_callback, queue_size=1)
        
        # 创建服务
        self.reset_service = rospy.Service('reset_state_machine', Trigger, self.reset_callback)
        
        rospy.loginfo("ROS通信初始化完成")
    
    def state_name(self, state):
        """获取状态的字符串表示"""
        for attr in dir(RobotState):
            if not attr.startswith('__') and getattr(RobotState, attr) == state:
                return attr
        return "UNKNOWN"
    
    def event_name(self, event):
        """获取事件的字符串表示"""
        for attr in dir(Event):
            if not attr.startswith('__') and getattr(Event, attr) == event:
                return attr
        return "UNKNOWN"
    
    def publish_state(self):
        """发布当前状态信息"""
        state_name = self.state_name(self.current_state)
        self.state_pub.publish("State: " + state_name)
        rospy.loginfo("当前状态: %s", state_name)
    
    def transition(self, new_state):
        """状态转换处理"""
        old_state_name = self.state_name(self.current_state)
        new_state_name = self.state_name(new_state)
        rospy.loginfo("状态转换: %s -> %s", old_state_name, new_state_name)
        
        self.current_state = new_state
        self.publish_state()
        self.execute_state_actions()
    
    def execute_state_actions(self):
        """执行状态对应的动作（测试版本）"""
        if self.current_state == RobotState.IDLE:
            rospy.loginfo("空闲状态，等待唤醒...")
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR:
            rospy.loginfo("收到导航命令（测试版本）...")
            rospy.loginfo("模拟导航过程，3秒后自动完成...")
            self.navigation_active = True
            rospy.sleep(3.0)  # 延时3秒模拟导航过程
            self.navigation_done_callback(actionlib.GoalStatus.SUCCEEDED, None)
            
        elif self.current_state == RobotState.ERROR:
            rospy.logerr("进入错误状态")
            self.stop_all_activities()
    
    def handle_event(self, event):
        """事件处理"""
        event_name = self.event_name(event)
        state_name = self.state_name(self.current_state)
        rospy.loginfo("在状态 %s 收到事件 %s", state_name, event_name)
        
        if self.current_state == RobotState.IDLE and event == Event.VOICE_CMD_WAKEUP:
            self.transition(RobotState.NAVIGATE_TO_QR)
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR:
            if event == Event.NAV_DONE_SUCCESS:
                rospy.loginfo("导航成功完成（测试版本）")
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
    
    def awake_callback(self, msg):
        """处理唤醒角度消息的回调函数"""
        self.last_awake_angle = msg.data
        rospy.loginfo("收到唤醒角度: %d度", self.last_awake_angle)

        if self.current_state == RobotState.IDLE and not self.is_awake:
            self.is_awake = True
            # rospy.loginfo("Ucar被唤醒，准备开始状态转换...")
            rospy.loginfo("Ucar被唤醒，等待1秒后开始状态转换...")
            rospy.sleep(1.0) # 添加1秒延时   
            self.handle_event(Event.VOICE_CMD_WAKEUP)
    
    def navigation_done_callback(self, status, result):
        """导航完成回调（测试版本）"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("模拟导航成功完成")
            self.handle_event(Event.NAV_DONE_SUCCESS)
        else:
            rospy.loginfo("模拟导航失败")
            self.handle_event(Event.NAV_DONE_FAILURE)
        self.navigation_active = False
    
    def reset_callback(self, req):
        """重置服务的回调函数"""
        rospy.loginfo("收到重置服务请求，准备重置状态机...")
        try:
            # 先停止所有活动
            self.stop_all_activities()
            # 然后重置状态
            self.setup()
            # 最后发布新状态
            self.publish_state()
            rospy.loginfo("状态机重置完成，等待新的唤醒...")
            return TriggerResponse(
                success=True,
                message="状态机已成功重置"
            )
        except Exception as e:
            rospy.logerr("重置状态机时出错: %s" % str(e))  # 使用 % 格式化，避免 f-string 在某些Python版本中的兼容性问题
            return TriggerResponse(
                success=False,
                message="重置失败: %s" % str(e)
            )

    def stop_all_activities(self):
        """停止所有活动（测试版本）"""
        self.navigation_active = False
        rospy.loginfo("已停止所有活动（测试版本）")

if __name__ == '__main__':
    try:
        robot_sm = RobotStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass