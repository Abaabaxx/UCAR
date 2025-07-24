#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
# 导入语音播报服务消息
from xf_mic_asr_offline.srv import VoiceCmd, VoiceCmdRequest

"""
统一状态机节点（前半段）- 实现二维码识别与任务获取流程

功能说明：
1. 实现了UnifiedStateMachine的前半段功能（状态0-6）
2. 添加了语音播报的超时处理机制
3. 添加了语音播报完成事件处理
4. 与新版二维码识别节点和语音播报节点兼容
5. 使用服务方式调用语音播报节点
6. 在导航到拣货准备区后结束，不进入后续状态

使用方法：
1. 启动状态机：
   方式1 - 服务调用：rosservice call /start_state_machine "{}"
   方式2 - 语音唤醒：通过语音唤醒
2. 重置状态机：
   rosservice call /reset_state_machine "{}"

状态机的本质原理：
在当前状态下执行当前状态对应的动作 execute_state_actions（该函数根据当前状态决定执行什么动作）
状态的切换依靠事件的触发（对应的回调函数），在回调函数中执行状态的切换
"""

#********************************* 常量定义 *********************************#
# 所有状态的类
class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_QR1 = 1
    ROTATE_TO_QR2 = 2
    NAVIGATE_TO_QR_AREA = 3
    WAIT_FOR_QR_RESULT = 4 
    SPEAK_TASK_TYPE = 5    
    NAV_TO_PICK_PREP_AREA = 6  # 关键桥梁状态：导航至拣货准备区
    ERROR = 99

# 所有事件的类
class Event(object):
    START_CMD = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2
    QR_RESULT_VALID = 3     
    PERCEPTION_TIMEOUT = 4   
    SPEAK_DONE = 5           # 语音播放完成事件
    SPEAK_TIMEOUT = 6        # 语音播放超时事件

# 领取到的任务类型的类
class TaskType(object):
    FRUITS = "fruits"
    VEGETABLES = "vegetables"
    DESSERTS = "desserts"
    UNKNOWN = "unknown"

#********************************* 主状态机类 *********************************#
# 主状态机
class UnifiedStateMachine(object):
    def __init__(self):
        rospy.init_node('unified_state_machine', anonymous=True)
        
        # 初始化成员变量
        self.delay_timer = None
        self.qr_timer = None
        self.speak_timer = None  # 语音播放超时定时器
        
        # 变量初始化 + ROS通信初始化 + 位置初始化
        self.setup()
        self.init_ros_comm()
        self.init_locations()
        
        rospy.loginfo("统一状态机（前半段）初始化完成（支持语音唤醒和服务调用，包含二维码处理与语音播报功能）")
        rospy.loginfo("等待语音唤醒或服务调用...")
        
        # 广播当前状态
        self.publish_state()

    #*********************** 初始化相关函数 ***********************#
    def setup(self):
        self.is_awake = False
        self.last_awake_angle = None
        self.current_state = RobotState.IDLE
        self.navigation_active = False
        
        # 二维码相关变量
        self.qr_perception_timeout = 5.0 
        self.task_type = TaskType.UNKNOWN
        # 用于防止重复处理任务的标志
        self.task_processed = False
        
        # 语音播放相关变量
        self.speak_timeout = 10.0  # 语音播放超时时间(秒)
        self.current_voice_cmd = None  # 当前播放的语音命令
        self.voice_service_called = False  # 标记语音服务是否已调用

    def init_ros_comm(self):
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        self.task_pub = rospy.Publisher('/robot/task_type', String, queue_size=1)
        
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        self.awake_sub = rospy.Subscriber('/mic/awake/angle', Int32, self.awake_callback, queue_size=1)
        
        self.task_type_sub = rospy.Subscriber('/QR/task_type', String, self.task_type_callback, queue_size=1)
        
        # 等待语音播报服务
        rospy.loginfo("等待语音播报服务...")
        try:
            rospy.wait_for_service('/robot/voice_cmd', timeout=5.0)
            rospy.loginfo("语音播报服务连接成功")
        except rospy.ROSException:
            rospy.logwarn("语音播报服务不可用，将在服务可用时自动连接")
        
        self.reset_service = rospy.Service('reset_state_machine', Trigger, self.reset_callback)
        self.start_service = rospy.Service('start_state_machine', Trigger, self.start_callback)

    def init_locations(self):
        self.locations = {
            'qr1': self.create_pose(2.07, 0.60, 0.3007, 0.9537),
            'qr2': self.create_pose(2.07, 0.60, 0.9747, 0.2239),
            'qr_area': self.create_pose(1.25, 0.75, -1.0, 0.0),
            'pick_prep_area': self.create_pose(0.5, 2.0, 0.7071, 0.7071),  # 桥梁位置：拣货准备区
        }

    # 创建目标位置的函数,辅助状态初始化
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

    #*********************** 工具函数 ***********************#
    # 将状态和事件的整数值转换为对应的字符串名称的实用工具函数
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

    # 广播当前状态
    def publish_state(self):
        state_name = self.state_name(self.current_state)
        self.state_pub.publish("State: " + state_name)
        rospy.loginfo("当前状态: %s", state_name)
    
    # 广播任务类型
    def publish_task_type(self):
        self.task_pub.publish(self.task_type)
        rospy.loginfo("任务类型: %s", self.task_type)

    #*********************** 状态转换与执行 ***********************#
    # 状态转化函数（如从状态0--->状态1）
    def transition(self, new_state):
        old_state_name = self.state_name(self.current_state)
        new_state_name = self.state_name(new_state)
        rospy.loginfo("状态转换: %s -> %s", old_state_name, new_state_name)
        
        self.current_state = new_state
        self.publish_state()
        self.execute_state_actions()

    # 执行状态对应的动作（如在状态1（导航到qr1）时，发布导航到qr1的指令（状态1对应的动作））
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
            
        # 等待二维码识别结果状态
        elif self.current_state == RobotState.WAIT_FOR_QR_RESULT:
            rospy.loginfo("4-等待二维码识别结果")
            # 重置任务处理标志
            self.task_processed = False
            self.start_perception_timer()
            
        # 播报任务类型状态
        elif self.current_state == RobotState.SPEAK_TASK_TYPE:
            rospy.loginfo("5-播报任务类型: %s", self.task_type)
            
            # 根据任务类型选择播报内容
            if self.task_type == TaskType.FRUITS:
                self.current_voice_cmd = "task_get_fruit"
            elif self.task_type == TaskType.VEGETABLES:
                self.current_voice_cmd = "task_get_veg"
            elif self.task_type == TaskType.DESSERTS:
                self.current_voice_cmd = "task_get_dessert"
            else:
                rospy.logwarn("未知任务类型，无法播报: %s", self.task_type)
                # 处理未知任务类型的情况
                self.transition(RobotState.ERROR)
                return
            
            self.voice_service_called = False  # 重置服务调用标志
            
            # 启动语音播放超时定时器
            self.start_speak_timer()
            
            # 发送语音播放命令(通过服务调用)
            self.call_voice_service()
            

            
        # 导航至拣货准备区状态（桥梁状态）
        elif self.current_state == RobotState.NAV_TO_PICK_PREP_AREA:
            rospy.loginfo("6-导航至拣货准备区（桥梁状态）")
            self.send_nav_goal('pick_prep_area')
            self.navigation_active = True
            
        elif self.current_state == RobotState.ERROR:
            rospy.logerr("99-错误状态，停止所有活动")
            self.stop_all_activities()

    # 事件处理函数，根据当前状态和接受到的事件进行状态转换（如在状态0空闲状态时，接收到事件1（语音唤醒），则执行状态转换函数，转换到下一个状态）
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
                # 导航到二维码区域成功后，转换到等待二维码识别结果状态
                self.transition(RobotState.WAIT_FOR_QR_RESULT)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        # 等待二维码识别结果状态的事件处理
        elif self.current_state == RobotState.WAIT_FOR_QR_RESULT:
            if event == Event.QR_RESULT_VALID:
                self.stop_perception_timer()
                rospy.loginfo("收到有效的二维码结果，任务类型: %s", self.task_type)
                self.publish_task_type()
                # 转换到播报任务类型状态
                self.transition(RobotState.SPEAK_TASK_TYPE)
            elif event == Event.PERCEPTION_TIMEOUT:
                rospy.logerr("二维码识别超时")
                self.transition(RobotState.ERROR)
                
        # 播报任务类型状态的事件处理
        elif self.current_state == RobotState.SPEAK_TASK_TYPE:
            if event == Event.SPEAK_DONE:
                rospy.loginfo("语音播报完成，准备导航至拣货准备区")
                # 转换到导航至拣货准备区状态
                self.transition(RobotState.NAV_TO_PICK_PREP_AREA)
            elif event == Event.SPEAK_TIMEOUT:
                rospy.logerr("语音播报超时")
                self.transition(RobotState.ERROR)
                
        # 导航至拣货准备区状态的事件处理
        elif self.current_state == RobotState.NAV_TO_PICK_PREP_AREA:
            if event == Event.NAV_DONE_SUCCESS:
                rospy.loginfo("成功抵达拣货准备区。前半段任务完成，保持当前状态。")
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)

    #*********************** 回调函数 ***********************#
    # 语音唤醒回调函数（语音唤醒后进入该回调函数，执行状态转换）
    def awake_callback(self, msg):
        self.last_awake_angle = msg.data
        rospy.loginfo("收到唤醒角度: %d度", self.last_awake_angle)

        if self.current_state == RobotState.IDLE and not self.is_awake:
            self.is_awake = True
            rospy.loginfo("通过语音唤醒，启动1秒定时器...")
            rospy.Timer(rospy.Duration(1.0), 
                       lambda event: self.handle_event(Event.START_CMD), 
                       oneshot=True)

    # 服务启动状态机的回调函数（服务调用后进入该回调函数，执行状态转换）
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

    # 领取二维码识别到的任务类型的回调函数（处于识别二维码领取任务状态时，接收到领取任务结果后，执行状态转换）
    def task_type_callback(self, msg):
        if self.current_state == RobotState.WAIT_FOR_QR_RESULT and not self.task_processed:
            self.task_type = msg.data
            if self.task_type != TaskType.UNKNOWN:
                rospy.loginfo("接收到有效的任务类型: %s", self.task_type)
                # 设置标记为已处理，防止重复处理同一任务
                self.task_processed = True
                self.handle_event(Event.QR_RESULT_VALID)
            else:
                rospy.logwarn("接收到未知任务类型")

    # 服务重置状态机的回调函数（服务调用后进入该回调函数，执行状态转换，初始化所有变量，回到初始空闲状态）
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

    # 导航完成回调函数（导航完成后进入该回调函数，执行状态转换）
    def navigation_done_callback(self, status, result):
        self.navigation_active = False
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("导航成功，启动0.5秒定时器...")
            rospy.Timer(rospy.Duration(0.5), 
                      lambda event: self.delayed_nav_success(), 
                      oneshot=True)
        else:
            self.handle_event(Event.NAV_DONE_FAILURE)

    #*********************** 语音播报相关功能 ***********************#
    # 调用语音播报服务（同步版本）
    def call_voice_service(self):
        if self.voice_service_called:
            return  # 防止重复调用
        
        # 检查服务是否可用
        try:
            rospy.loginfo("检查语音服务 '/robot/voice_cmd' 是否可用...")
            rospy.wait_for_service('/robot/voice_cmd', timeout=3.0)
            rospy.loginfo("语音服务可用")
        except rospy.ROSException:
            rospy.logerr("等待语音服务 '/robot/voice_cmd' 超时或不可用")
            self.handle_event(Event.SPEAK_TIMEOUT)
            return
        
        try:
            rospy.loginfo("准备调用语音播放服务: %s", self.current_voice_cmd)
            
            # 创建服务代理
            voice_client = rospy.ServiceProxy('/robot/voice_cmd', VoiceCmd)
            
            # 使用正确的请求消息类型
            request = VoiceCmdRequest()
            
            # 类型检查和赋值
            if not isinstance(self.current_voice_cmd, str):
                rospy.logwarn("voice_cmd 不是字符串类型，进行类型转换: %s", 
                            type(self.current_voice_cmd).__name__)
                request.voice_cmd = str(self.current_voice_cmd)
            else:
                request.voice_cmd = self.current_voice_cmd
            
            # 标记语音服务已调用
            self.voice_service_called = True
            
            # 启动超时定时器
            self.start_speak_timer()
            
            # 执行同步调用
            rospy.loginfo("开始调用语音服务，将阻塞直到完成...")
            response = voice_client(request)
            rospy.loginfo("语音服务调用完成")
            
            # 停止超时定时器（服务调用已完成）
            self.stop_speak_timer()
            
            # 处理服务响应
            if response.success:
                rospy.loginfo("语音播放成功: %s", response.message)
                # 触发语音播放完成事件
                rospy.Timer(rospy.Duration(0.1), 
                        lambda event: self.handle_event(Event.SPEAK_DONE), 
                        oneshot=True)
            else:
                rospy.logwarn("语音播放失败: %s", response.message)
                self.handle_event(Event.SPEAK_TIMEOUT)
                
        except rospy.ServiceException as e:
            rospy.logerr("调用语音服务失败: %s", str(e))
            self.stop_speak_timer()
            self.handle_event(Event.SPEAK_TIMEOUT)
        except Exception as e:
            # 捕获其他所有异常，包括序列化错误
            rospy.logerr("调用语音服务时发生未知错误: %s", str(e))
            import traceback
            traceback.print_exc()  # 打印详细堆栈跟踪
            self.stop_speak_timer()
            self.handle_event(Event.SPEAK_TIMEOUT)


    #*********************** 超时处理 ***********************#
    # 启动感知超时定时器
    def start_perception_timer(self):
        if self.qr_timer:
            self.qr_timer.shutdown()
            self.qr_timer = None
        
        rospy.loginfo("启动二维码感知超时定时器: %.1f秒", self.qr_perception_timeout)
        self.qr_timer = rospy.Timer(rospy.Duration(self.qr_perception_timeout), 
                                  lambda event: self.handle_event(Event.PERCEPTION_TIMEOUT), 
                                  oneshot=True)
    
    # 停止感知超时定时器
    def stop_perception_timer(self):
        if self.qr_timer:
            self.qr_timer.shutdown()
            self.qr_timer = None
            rospy.loginfo("停止二维码感知超时定时器")
            
    # 启动语音播放超时定时器
    def start_speak_timer(self):
        if self.speak_timer:
            self.speak_timer.shutdown()
            self.speak_timer = None
        
        rospy.loginfo("启动语音播放超时定时器: %.1f秒", self.speak_timeout)
        self.speak_timer = rospy.Timer(rospy.Duration(self.speak_timeout), 
                                     lambda event: self.handle_speak_timeout(), 
                                     oneshot=True)
    
    # 停止语音播放超时定时器
    def stop_speak_timer(self):
        if self.speak_timer:
            self.speak_timer.shutdown()
            self.speak_timer = None
            rospy.loginfo("停止语音播放超时定时器")
    
    # 处理语音播放超时
    def handle_speak_timeout(self):
        rospy.logwarn("语音播放超时")
        if self.current_state == RobotState.SPEAK_TASK_TYPE:
            self.handle_event(Event.SPEAK_TIMEOUT)

    #*********************** 导航相关功能 ***********************#
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

    # 延迟导航成功处理
    def delayed_nav_success(self):
        rospy.loginfo("0.5秒定时结束，执行状态转换...")
        self.handle_event(Event.NAV_DONE_SUCCESS)

    #*********************** 资源管理 ***********************#
    # 停止所有活动
    def stop_all_activities(self):
        if self.navigation_active:
            self.move_base_client.cancel_all_goals()
            self.navigation_active = False
        
        # 停止感知超时定时器
        if self.qr_timer:
            self.qr_timer.shutdown()
            self.qr_timer = None
            
        # 停止语音播放超时定时器
        if self.speak_timer:
            self.speak_timer.shutdown()
            self.speak_timer = None
        
        self.is_awake = False
        self.task_processed = False  # 重置任务处理标志
        self.current_voice_cmd = None  # 重置当前语音命令
        self.voice_service_called = False  # 重置语音服务调用标志
        rospy.loginfo("已停止所有活动")

#*********************** 主函数 ***********************#
if __name__ == '__main__':
    try:
        robot_sm = UnifiedStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass