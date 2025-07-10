#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import threading
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

# 使用简单的类定义代替Python 3的Enum（兼容Python 2.7）
class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_QR = 1
    WAIT_QR_RESULT = 2
    ANNOUNCE_TASK = 3
    NAVIGATE_TO_PICKUP = 4
    ROTATE_AND_SEARCH = 5
    NAVIGATE_TO_ITEM = 6
    ADJUST_POSTURE = 7
    ANNOUNCE_PICKUP = 8
    NAVIGATE_TO_SIM_WAIT = 9
    WAIT_FOR_STOP = 10
    TRIGGER_SIMULATION = 11
    ANNOUNCE_SIM_RESULT = 12
    NAVIGATE_TO_TRAFFIC_LIGHT = 13
    WAIT_TRAFFIC_LIGHT = 14
    DECIDE_PATH = 15
    NAVIGATE_TO_FINISH = 16
    WAIT_FOR_FINAL_STOP = 17
    CALCULATE_COST = 18
    ANNOUNCE_FINAL_RESULT = 19
    FINISHED = 20
    ERROR = 99

class Event(object):
    VOICE_CMD_WAKEUP = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2
    QR_RESULT_VALID = 3
    PERCEPTION_TIMEOUT = 4
    SPEAK_DONE = 5
    YOLO_RESULT_FOUND = 6
    ADJUST_SUCCESS = 7
    ADJUST_TIMEOUT = 8
    CAR_STOPPED = 9
    STOP_WAIT_TIMEOUT = 10
    SIM_RESULT_SUCCESS = 11
    SIM_TIMEOUT = 12
    SIM_RESULT_FAILURE = 13
    TRAFFIC_LIGHT_VALID = 14
    DECISION_FAILURE = 15
    CAR_STOPPED_MAIN = 16
    CAR_STOPPED_UNDEFINED = 17
    CALC_DONE = 18

class RobotStateMachine(object):
    def __init__(self):
        rospy.init_node('robot_state_machine')
        
        # 状态和变量
        self.current_state = RobotState.IDLE
        self.navigation_active = False
        self.task_type = None
        self.physical_item_picked = None
        self.simulated_item = None
        self.simulated_room = None
        self.traffic_light_result = None
        self.chosen_path_entry = None
        self.cost = 0
        self.change = 0
        
        # 定时器
        self.timer = None
        
        # 发布者
        self.speech_pub = rospy.Publisher('/robot/speech', String, queue_size=10)
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        
        # Action客户端
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        # 订阅者
        rospy.Subscriber('/voice_command', String, self.voice_command_callback)
        rospy.Subscriber('/qr_code_result', String, self.qr_code_callback)
        rospy.Subscriber('/yolo_detection', String, self.yolo_detection_callback)
        rospy.Subscriber('/adjust_status', String, self.adjust_status_callback)
        rospy.Subscriber('/robot_stopped', String, self.robot_stopped_callback)
        rospy.Subscriber('/simulation_result', String, self.simulation_result_callback)
        rospy.Subscriber('/traffic_light_result', String, self.traffic_light_callback)
        rospy.Subscriber('/speech_status', String, self.speech_status_callback)
        
        # 预定义位置
        self.locations = {
            'qr_area': self.create_pose(1.0, 1.0, 0.0),
            'pickup_area': self.create_pose(2.0, 2.0, 0.0),
            'sim_wait_area': self.create_pose(3.0, 3.0, 0.0),
            'traffic_light_area': self.create_pose(4.0, 4.0, 0.0),
            'finish_area': self.create_pose(5.0, 5.0, 0.0)
        }
        
        rospy.loginfo("机器人状态机初始化完成，等待唤醒...")
        self.publish_state()
    
    def create_pose(self, x, y, theta):
        """创建导航目标点的位姿"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = theta
        return pose
    
    def state_name(self, state):
        """获取状态的名称"""
        for attr in dir(RobotState):
            if not attr.startswith('__') and getattr(RobotState, attr) == state:
                return attr
        return "UNKNOWN"
    
    def event_name(self, event):
        """获取事件的名称"""
        for attr in dir(Event):
            if not attr.startswith('__') and getattr(Event, attr) == event:
                return attr
        return "UNKNOWN"
    
    def publish_state(self):
        """发布当前状态"""
        state_name = self.state_name(self.current_state)
        self.state_pub.publish("State: " + state_name)
    
    def transition(self, new_state):
        """状态转换函数"""
        old_state_name = self.state_name(self.current_state)
        new_state_name = self.state_name(new_state)
        rospy.loginfo("状态转换: %s -> %s", old_state_name, new_state_name)
        self.current_state = new_state
        self.publish_state()
        
        # 取消之前的定时器
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        
        # 执行新状态的动作
        self.execute_state_actions()
    
    def execute_state_actions(self):
        """执行当前状态对应的动作"""
        if self.current_state == RobotState.IDLE:
            rospy.loginfo("空闲状态，等待唤醒...")
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR:
            self.send_nav_goal('qr_area')
            self.navigation_active = True
            
        elif self.current_state == RobotState.WAIT_QR_RESULT:
            self.start_timeout(30, Event.PERCEPTION_TIMEOUT)
            
        elif self.current_state == RobotState.ANNOUNCE_TASK:
            speak_text = "任务获取成功，任务类型: %s" % self.task_type
            self.speak(speak_text)
            
        elif self.current_state == RobotState.NAVIGATE_TO_PICKUP:
            self.send_nav_goal('pickup_area')
            self.navigation_active = True
            
        elif self.current_state == RobotState.ROTATE_AND_SEARCH:
            self.start_rotation()
            self.start_timeout(60, Event.PERCEPTION_TIMEOUT)
            
        elif self.current_state == RobotState.NAVIGATE_TO_ITEM:
            self.stop_rotation()
            # 在实际应用中，这里应该使用物品的真实位置
            self.send_nav_goal('pickup_area')  # 简化版本直接使用预定义位置
            self.navigation_active = True
            
        elif self.current_state == RobotState.ADJUST_POSTURE:
            self.start_posture_adjustment()
            self.start_timeout(15, Event.ADJUST_TIMEOUT)
            
        elif self.current_state == RobotState.ANNOUNCE_PICKUP:
            speak_text = "已成功拾取物品: %s" % self.physical_item_picked
            self.speak(speak_text)
            
        elif self.current_state == RobotState.NAVIGATE_TO_SIM_WAIT:
            self.send_nav_goal('sim_wait_area')
            self.navigation_active = True
            
        elif self.current_state == RobotState.WAIT_FOR_STOP:
            self.start_timeout(10, Event.STOP_WAIT_TIMEOUT)
            
        elif self.current_state == RobotState.TRIGGER_SIMULATION:
            self.trigger_simulation()
            self.start_timeout(180, Event.SIM_TIMEOUT)
            
        elif self.current_state == RobotState.ANNOUNCE_SIM_RESULT:
            speak_text = "仿真完成，物品 %s 位于 %s" % (self.simulated_item, self.simulated_room)
            self.speak(speak_text)
            
        elif self.current_state == RobotState.NAVIGATE_TO_TRAFFIC_LIGHT:
            self.send_nav_goal('traffic_light_area')
            self.navigation_active = True
            
        elif self.current_state == RobotState.WAIT_TRAFFIC_LIGHT:
            self.start_timeout(30, Event.PERCEPTION_TIMEOUT)
            
        elif self.current_state == RobotState.DECIDE_PATH:
            self.chosen_path_entry = self.make_path_decision()
            speak_text = "选择通过 %s 路口" % self.chosen_path_entry
            self.speak(speak_text)
            
        elif self.current_state == RobotState.NAVIGATE_TO_FINISH:
            self.send_nav_goal('finish_area')
            self.navigation_active = True
            
        elif self.current_state == RobotState.WAIT_FOR_FINAL_STOP:
            self.start_timeout(5, Event.STOP_WAIT_TIMEOUT)
            
        elif self.current_state == RobotState.CALCULATE_COST:
            self.cost, self.change = self.calculate_cost_and_change()
            rospy.loginfo("计算结果: 成本=%d, 找零=%d", self.cost, self.change)
            self.handle_event(Event.CALC_DONE)
            
        elif self.current_state == RobotState.ANNOUNCE_FINAL_RESULT:
            speak_text = "任务完成，总成本: %d，找零: %d" % (self.cost, self.change)
            self.speak(speak_text)
            # 播报完成后加5秒延迟
            rospy.Timer(rospy.Duration(5), self.final_delay_callback, oneshot=True)
            
        elif self.current_state == RobotState.FINISHED:
            rospy.loginfo("任务全部完成")
            
        elif self.current_state == RobotState.ERROR:
            rospy.logerr("进入错误状态，停止所有活动")
            self.stop_all_activities()
    
    def handle_event(self, event):
        """处理事件，根据当前状态和事件触发状态转换"""
        event_name = self.event_name(event)
        state_name = self.state_name(self.current_state)
        rospy.loginfo("事件触发: %s 在状态 %s", event_name, state_name)
        
        # 状态转换逻辑
        if self.current_state == RobotState.IDLE and event == Event.VOICE_CMD_WAKEUP:
            self.transition(RobotState.NAVIGATE_TO_QR)
            
        elif self.current_state == RobotState.NAVIGATE_TO_QR:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.WAIT_QR_RESULT)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.WAIT_QR_RESULT:
            if event == Event.QR_RESULT_VALID:
                self.transition(RobotState.ANNOUNCE_TASK)
            elif event == Event.PERCEPTION_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.ANNOUNCE_TASK and event == Event.SPEAK_DONE:
            self.transition(RobotState.NAVIGATE_TO_PICKUP)
            
        elif self.current_state == RobotState.NAVIGATE_TO_PICKUP:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.ROTATE_AND_SEARCH)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.ROTATE_AND_SEARCH:
            if event == Event.YOLO_RESULT_FOUND:
                self.transition(RobotState.NAVIGATE_TO_ITEM)
            elif event == Event.PERCEPTION_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAVIGATE_TO_ITEM:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.ADJUST_POSTURE)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.ADJUST_POSTURE:
            if event == Event.ADJUST_SUCCESS:
                self.transition(RobotState.ANNOUNCE_PICKUP)
            elif event == Event.ADJUST_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.ANNOUNCE_PICKUP and event == Event.SPEAK_DONE:
            self.transition(RobotState.NAVIGATE_TO_SIM_WAIT)
            
        elif self.current_state == RobotState.NAVIGATE_TO_SIM_WAIT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.WAIT_FOR_STOP)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.WAIT_FOR_STOP:
            if event == Event.CAR_STOPPED:
                self.transition(RobotState.TRIGGER_SIMULATION)
            elif event == Event.STOP_WAIT_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.TRIGGER_SIMULATION:
            if event == Event.SIM_RESULT_SUCCESS:
                self.transition(RobotState.ANNOUNCE_SIM_RESULT)
            elif event in [Event.SIM_TIMEOUT, Event.SIM_RESULT_FAILURE]:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.ANNOUNCE_SIM_RESULT and event == Event.SPEAK_DONE:
            self.transition(RobotState.NAVIGATE_TO_TRAFFIC_LIGHT)
            
        elif self.current_state == RobotState.NAVIGATE_TO_TRAFFIC_LIGHT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.WAIT_TRAFFIC_LIGHT)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.WAIT_TRAFFIC_LIGHT:
            if event == Event.TRAFFIC_LIGHT_VALID:
                self.transition(RobotState.DECIDE_PATH)
            elif event == Event.PERCEPTION_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.DECIDE_PATH:
            if event == Event.SPEAK_DONE:
                self.transition(RobotState.NAVIGATE_TO_FINISH)
            elif event == Event.DECISION_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAVIGATE_TO_FINISH:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.WAIT_FOR_FINAL_STOP)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.WAIT_FOR_FINAL_STOP:
            if event == Event.CAR_STOPPED_MAIN:
                self.transition(RobotState.CALCULATE_COST)
            elif event == Event.STOP_WAIT_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.CALCULATE_COST and event == Event.CALC_DONE:
            self.transition(RobotState.ANNOUNCE_FINAL_RESULT)
            
        elif self.current_state == RobotState.ANNOUNCE_FINAL_RESULT and event == Event.SPEAK_DONE:
            self.transition(RobotState.FINISHED)
    
    # --- 回调函数 ---
    
    def voice_command_callback(self, msg):
        """语音命令回调"""
        if msg.data == "wake_up" and self.current_state == RobotState.IDLE:
            self.handle_event(Event.VOICE_CMD_WAKEUP)
    
    def qr_code_callback(self, msg):
        """二维码识别结果回调"""
        if self.current_state == RobotState.WAIT_QR_RESULT:
            self.task_type = msg.data
            self.handle_event(Event.QR_RESULT_VALID)
    
    def yolo_detection_callback(self, msg):
        """YOLO检测结果回调"""
        if self.current_state == RobotState.ROTATE_AND_SEARCH:
            self.physical_item_picked = msg.data
            self.handle_event(Event.YOLO_RESULT_FOUND)
    
    def adjust_status_callback(self, msg):
        """姿态调整状态回调"""
        if self.current_state == RobotState.ADJUST_POSTURE and msg.data == "success":
            self.handle_event(Event.ADJUST_SUCCESS)
    
    def robot_stopped_callback(self, msg):
        """机器人停止状态回调"""
        if self.current_state == RobotState.WAIT_FOR_STOP and msg.data == "stopped":
            self.handle_event(Event.CAR_STOPPED)
        elif self.current_state == RobotState.WAIT_FOR_FINAL_STOP:
            if msg.data == "main_stopped":
                self.handle_event(Event.CAR_STOPPED_MAIN)
            elif msg.data == "undefined_stopped":
                self.handle_event(Event.CAR_STOPPED_UNDEFINED)
    
    def simulation_result_callback(self, msg):
        """仿真结果回调"""
        if self.current_state == RobotState.TRIGGER_SIMULATION:
            try:
                parts = msg.data.split("|")
                self.simulated_item = parts[0]
                self.simulated_room = parts[1]
                self.handle_event(Event.SIM_RESULT_SUCCESS)
            except:
                rospy.logerr("仿真结果解析失败: %s", msg.data)
                self.handle_event(Event.SIM_RESULT_FAILURE)
    
    def traffic_light_callback(self, msg):
        """交通灯识别结果回调"""
        if self.current_state == RobotState.WAIT_TRAFFIC_LIGHT:
            self.traffic_light_result = msg.data
            self.handle_event(Event.TRAFFIC_LIGHT_VALID)
    
    def speech_status_callback(self, msg):
        """语音播报完成回调"""
        if msg.data == "done" and self.current_state in [
            RobotState.ANNOUNCE_TASK, 
            RobotState.ANNOUNCE_PICKUP,
            RobotState.ANNOUNCE_SIM_RESULT,
            RobotState.DECIDE_PATH,
            RobotState.ANNOUNCE_FINAL_RESULT
        ]:
            self.handle_event(Event.SPEAK_DONE)
    
    def final_delay_callback(self, event):
        """最终延迟回调"""
        self.handle_event(Event.SPEAK_DONE)
    
    # --- 操作函数 ---
    
    def send_nav_goal(self, location_name):
        """发送导航目标"""
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
        """导航完成回调"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.handle_event(Event.NAV_DONE_SUCCESS)
        else:
            self.handle_event(Event.NAV_DONE_FAILURE)
        self.navigation_active = False
    
    def speak(self, text):
        """发送语音播报"""
        rospy.loginfo("语音播报: %s", text)
        self.speech_pub.publish(text)
    
    def start_timeout(self, seconds, timeout_event):
        """开始超时计时"""
        if self.timer is not None:
            self.timer.cancel()
        
        self.timer = threading.Timer(seconds, lambda: self.handle_event(timeout_event))
        self.timer.start()
        rospy.loginfo("开始 %d 秒超时计时", seconds)
    
    def start_rotation(self):
        """开始旋转搜索"""
        rospy.loginfo("开始旋转搜索物品")
        # 实现旋转命令 - 根据您的机器人平台实现
        # 例如: 可以发布速度命令到 /cmd_vel 话题
    
    def stop_rotation(self):
        """停止旋转"""
        rospy.loginfo("停止旋转")
        # 停止旋转命令实现
    
    def start_posture_adjustment(self):
        """开始姿态调整"""
        rospy.loginfo("开始姿态调整")
        # 姿态调整命令实现
    
    def trigger_simulation(self):
        """触发仿真过程"""
        rospy.loginfo("触发仿真")
        # 仿真触发实现
        # 例如: 调用仿真服务或发布触发消息
    
    def make_path_decision(self):
        """根据交通灯结果决定路径"""
        rospy.loginfo("根据交通灯 %s 决定路径", self.traffic_light_result)
        # 路径决策实现
        return "主路口"
    
    def calculate_cost_and_change(self):
        """计算成本和找零"""
        # 计算逻辑实现
        cost = 100  # 示例值
        change = 20  # 示例值
        return cost, change
    
    def stop_all_activities(self):
        """停止所有活动"""
        if self.navigation_active:
            self.move_base_client.cancel_all_goals()
            self.navigation_active = False
        
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        
        rospy.loginfo("已停止所有活动")

if __name__ == '__main__':
    try:
        robot_sm = RobotStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass