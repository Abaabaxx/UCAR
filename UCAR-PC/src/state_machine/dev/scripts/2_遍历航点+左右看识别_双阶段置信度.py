#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger, TriggerResponse
from darknet_ros_msgs.msg import BoundingBoxes  # 导入新的消息类型

"""
遍历航点状态机
功能：控制机器人依次导航到6个预设点位，导航失败时执行原地旋转脱困
      在C1、C2、B1和A1航点执行物体检测

使用方法：
1. 启动状态机：
   rosservice call /start_state_machine "{}"
2. 重置状态机：
   rosservice call /reset_state_machine "{}"

作者：Claude 3.7 Sonnet
创建时间：2024-08-02
修改时间：2025-08-03
"""

# 定义导航超时常量（单位：秒）
NAV_TIMEOUT_SECONDS = 10.0

# 定义左右看检测参数（可调节）
DETECTION_ANGULAR_SPEED = 0.8 
LOOK_LEFT_DURATION = 0.5  # 向左看的时间（秒）
LOOK_RIGHT_DURATION = 1.0  # 向右看的时间（秒）

# 定义置信度过滤阈值（可调节）
PRIMARY_CONFIDENCE_THRESHOLD = 0.9  # 第一阶段（左右看）的置信度阈值
SECONDARY_CONFIDENCE_THRESHOLD = 0.5  # 第二阶段（重试）的置信度阈值
RETRY_DETECTION_DURATION = 2.0  # 重试检测的持续时间（秒）

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
    # 新增检测状态
    DETECT_AT_C1 = 9
    DETECT_AT_C2 = 10
    DETECT_AT_B1 = 11
    DETECT_AT_A1 = 12

class Event(object):
    START_CMD = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2
    RESCUE_DONE = 3
    # 新增检测事件
    DETECT_DONE_FOUND = 4
    DETECT_DONE_NOT_FOUND = 5

class RobotStateMachine(object):
    def __init__(self):
        rospy.init_node('waypoint_follower_state_machine')
        
        # 记录进入救援状态前的导航状态
        self.previous_nav_state = None
        
        # 初始化导航超时定时器
        self.nav_timeout_timer = None
        
        # 初始化检测相关变量
        self.room_c_objects = None
        self.room_b_objects = None
        self.room_a_objects = None
        self.detected_objects_buffer = []
        self.detection_timer = None
        self.yolo_subscriber = None
        
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
        
        # 重置检测相关变量
        self.room_c_objects = None
        self.room_b_objects = None
        self.room_a_objects = None
        self.detected_objects_buffer = []
        self.is_retry_attempt = False  # 标记是否为重试检测
        self.best_detection = None  # 存储最佳检测结果
        if self.detection_timer:
            self.detection_timer.shutdown()
            self.detection_timer = None

    def init_ros_comm(self):
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        # 初始化YOLO检测订阅者
        self.yolo_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.yolo_callback)
        
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
            
        # 处理新的检测状态
        elif self.current_state == RobotState.DETECT_AT_C1:
            rospy.loginfo("9-开始在C1点检测物体...")
            self.is_retry_attempt = False  # 重置重试标记
            self.start_detection_phase()
            
        elif self.current_state == RobotState.DETECT_AT_C2:
            rospy.loginfo("10-开始在C2点检测物体...")
            self.is_retry_attempt = False  # 重置重试标记
            self.start_detection_phase()
            
        elif self.current_state == RobotState.DETECT_AT_B1:
            rospy.loginfo("11-开始在B1点检测物体...")
            self.is_retry_attempt = False  # 重置重试标记
            self.start_detection_phase()
            
        elif self.current_state == RobotState.DETECT_AT_A1:
            rospy.loginfo("12-开始在A1点检测物体...")
            self.is_retry_attempt = False  # 重置重试标记
            self.start_detection_phase()

    def handle_event(self, event):
        event_name = self.event_name(event)
        state_name = self.state_name(self.current_state)
        rospy.loginfo("在状态 %s 收到事件 %s", state_name, event_name)
        
        if self.current_state == RobotState.IDLE and event == Event.START_CMD:
            self.transition(RobotState.NAV_TO_C1)
            
        # 修改导航成功后的转换目标为检测状态
        elif self.current_state == RobotState.NAV_TO_C1:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.DETECT_AT_C1)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.NAV_TO_C2:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.DETECT_AT_C2)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.NAV_TO_B1:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.DETECT_AT_B1)
            elif event == Event.NAV_DONE_FAILURE:
                self.previous_nav_state = self.current_state
                self.transition(RobotState.RESCUE_ROTATION)
                
        elif self.current_state == RobotState.NAV_TO_A1:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.DETECT_AT_A1)
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
                    
        # 处理新增的检测状态事件
        elif self.current_state == RobotState.DETECT_AT_C1:
            if event == Event.DETECT_DONE_FOUND:
                rospy.loginfo("在C1点检测到物体，跳过C2点，直接前往B1点")
                self.transition(RobotState.NAV_TO_B1)
            elif event == Event.DETECT_DONE_NOT_FOUND:
                rospy.loginfo("在C1点未检测到物体，继续前往C2点")
                self.transition(RobotState.NAV_TO_C2)
                
        elif self.current_state == RobotState.DETECT_AT_C2:
            if event == Event.DETECT_DONE_FOUND:
                rospy.loginfo("在C2点检测到物体，继续前往B1点")
                self.transition(RobotState.NAV_TO_B1)
            elif event == Event.DETECT_DONE_NOT_FOUND:
                rospy.loginfo("在C2点未检测到物体，继续前往B1点")
                self.transition(RobotState.NAV_TO_B1)
                
        elif self.current_state == RobotState.DETECT_AT_B1:
            if event == Event.DETECT_DONE_FOUND:
                rospy.loginfo("在B1点检测到物体，继续前往A1点")
                self.transition(RobotState.NAV_TO_A1)
            elif event == Event.DETECT_DONE_NOT_FOUND:
                rospy.loginfo("在B1点未检测到物体，继续前往A1点")
                self.transition(RobotState.NAV_TO_A1)
                
        elif self.current_state == RobotState.DETECT_AT_A1:
            if event == Event.DETECT_DONE_FOUND:
                rospy.loginfo("在A1点检测到物体，继续前往中间点")
                self.transition(RobotState.NAV_TO_MIDPOINT)
            elif event == Event.DETECT_DONE_NOT_FOUND:
                rospy.loginfo("在A1点未检测到物体，继续前往中间点")
                self.transition(RobotState.NAV_TO_MIDPOINT)

    def yolo_callback(self, msg):
        """接收YOLO检测的结果，根据置信度过滤并追踪最佳检测结果"""
        # 确定当前应使用的置信度阈值
        current_threshold = SECONDARY_CONFIDENCE_THRESHOLD if self.is_retry_attempt else PRIMARY_CONFIDENCE_THRESHOLD
        
        # 遍历所有检测框，寻找符合阈值且置信度最高的
        for box in msg.bounding_boxes:
            # 如果置信度高于阈值
            if box.probability > current_threshold:
                # 如果还没有最佳检测，或者当前框的置信度更高
                if self.best_detection is None or box.probability > self.best_detection.probability:
                    self.best_detection = box
                    rospy.logdebug("更新最佳检测: %s (置信度: %.2f)", box.Class, box.probability)
        
        # 保存所有原始检测结果（用于调试）
        self.detected_objects_buffer = msg.bounding_boxes
        if self.detected_objects_buffer:
            rospy.logdebug("收到检测对象: %d 个", len(self.detected_objects_buffer))

    def start_detection_phase(self):
        """开始检测阶段，清空缓冲区并根据是否为重试决定检测方式"""
        self.detected_objects_buffer = []
        self.best_detection = None  # 重置最佳检测结果
        
        # 如果存在旧的定时器，关闭它
        if self.detection_timer:
            self.detection_timer.shutdown()
        
        if not self.is_retry_attempt:
            # 第一阶段：左右看检测（高置信度）
            rospy.loginfo("第一阶段检测 - 左右看 (置信度阈值: %.2f)", PRIMARY_CONFIDENCE_THRESHOLD)
            self.perform_look_around()
        else:
            # 第二阶段：静态重试检测（低置信度）
            rospy.loginfo("第二阶段检测 - 静态重试 (置信度阈值: %.2f)", SECONDARY_CONFIDENCE_THRESHOLD)
            # 创建定时器，RETRY_DETECTION_DURATION秒后结束检测
            self.detection_timer = rospy.Timer(
                rospy.Duration(RETRY_DETECTION_DURATION),
                self.finish_detection_phase,
                oneshot=True
            )

    def finish_detection_phase(self, timer_event=None):
        """检测阶段结束，处理检测结果并触发相应事件或进入重试阶段"""
        # 确定房间名称
        if self.current_state == RobotState.DETECT_AT_C1 or self.current_state == RobotState.DETECT_AT_C2:
            room_name = "C房间"
        elif self.current_state == RobotState.DETECT_AT_B1:
            room_name = "B房间"
        elif self.current_state == RobotState.DETECT_AT_A1:
            room_name = "A房间"
        else:
            room_name = "未知房间"
        
        # 检查是否找到了符合置信度要求的最佳检测结果
        if self.best_detection is not None:
            # 找到了符合要求的物体
            if self.current_state == RobotState.DETECT_AT_C1 or self.current_state == RobotState.DETECT_AT_C2:
                self.room_c_objects = [self.best_detection]
            elif self.current_state == RobotState.DETECT_AT_B1:
                self.room_b_objects = [self.best_detection]
            elif self.current_state == RobotState.DETECT_AT_A1:
                self.room_a_objects = [self.best_detection]
            
            rospy.loginfo("在%s检测到物体: %s (置信度: %.2f)", 
                        room_name, self.best_detection.Class, self.best_detection.probability)
            self.handle_event(Event.DETECT_DONE_FOUND)
        else:
            # 未找到符合要求的物体
            if not self.is_retry_attempt:
                # 第一阶段失败，进入重试阶段
                rospy.loginfo("在%s未检测到符合高置信度要求的物体，开始重试检测...", room_name)
                self.is_retry_attempt = True
                self.start_detection_phase()
            else:
                # 重试阶段也失败，彻底放弃该点检测
                rospy.loginfo("在%s重试检测也未找到符合要求的物体，放弃检测", room_name)
                self.handle_event(Event.DETECT_DONE_NOT_FOUND)
        
        # 清空检测缓冲区
        self.detected_objects_buffer = []

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

    def perform_look_around(self):
        """执行左右看检测动作，先向左旋转"""
        rospy.loginfo("开始左右看检测：先向左看...")
        
        # 创建旋转速度指令
        twist = Twist()
        twist.angular.z = DETECTION_ANGULAR_SPEED  # 向左旋转
        
        # 发布旋转速度
        self.cmd_vel_pub.publish(twist)
        
        # 启动定时器，LOOK_LEFT_DURATION秒后切换到向右看
        rospy.Timer(rospy.Duration(LOOK_LEFT_DURATION),
                  lambda event: self.switch_to_look_right(),
                  oneshot=True)
    
    def switch_to_look_right(self):
        """切换到向右看"""
        rospy.loginfo("切换到向右看...")
        
        # 创建旋转速度指令
        twist = Twist()
        twist.angular.z = -DETECTION_ANGULAR_SPEED  # 向右旋转
        
        # 发布旋转速度
        self.cmd_vel_pub.publish(twist)
        
        # 启动定时器，LOOK_RIGHT_DURATION秒后停止旋转并结束检测
        rospy.Timer(rospy.Duration(LOOK_RIGHT_DURATION),
                  lambda event: self.stop_look_around_and_finish_detection(),
                  oneshot=True)
    
    def stop_look_around_and_finish_detection(self):
        """停止旋转并结束检测"""
        rospy.loginfo("左右看结束，停止旋转...")
        
        # 发布零速度指令停止旋转
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # 结束检测并处理结果
        self.finish_detection_phase(None)
    
    def stop_all_activities(self):
        if self.navigation_active:
            self.move_base_client.cancel_all_goals()
            # 关闭超时定时器
            if self.nav_timeout_timer:
                self.nav_timeout_timer.shutdown()
            self.navigation_active = False
        
        # 关闭检测定时器
        if self.detection_timer:
            self.detection_timer.shutdown()
            
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