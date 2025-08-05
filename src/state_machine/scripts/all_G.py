#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
统一状态机节点（完整版）

功能描述:
这是一个为ROS机器人比赛设计的、包含完整任务流程的统一状态机。
它将任务的全过程，从启动、接收任务到最终完成，整合在一个单一的逻辑流中。

核心流程包括：
1.  **任务获取**: 启动后，通过导航和旋转识别二维码，获取指定任务类型（如水果、甜点等）。
2.  **节点生命周期管理**: 在获取任务后，会显式关闭二维码识别节点(/qr_detect)以释放资源。
3.  **货物巡检与识别**: 导航至指定区域，通过多点巡航和YOLOv4进行目标货物识别。
4.  **仿真任务与红绿灯处理**: 在取货后，执行仿真任务并处理复杂的红绿灯路口逻辑。
5.  **任务收尾**: 在完成红绿灯导航后，会显式关闭YOLO节点(/darknet_ros)，并进入最终的巡线状态。

使用方法:
1.  启动状态机:
    方式1 (服务调用): rosservice call /start_state_machine "{}"
    方式2 (语音唤醒): 通过语音“小车小车”唤醒
2.  重置状态机:
    rosservice call /reset_state_machine "{}"

开发者测试说明:
可通过修改 UnifiedStateMachine.setup() 方法中的 self.current_state 初始值，
来从任意指定的状态启动测试，方便调试。
"""

import rospy
import rosnode
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Int32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
# 导入语音播报服务消息
from xf_mic_asr_offline.srv import VoiceCmd, VoiceCmdRequest
import subprocess
import os
import yaml
import glob
import threading
import roslibpy  # 新增
import time      # 新增
from darknet_ros_msgs.msg import BoundingBoxes  # YOLO检测结果消息类型

#********************************* 常量定义 *********************************#

# 任务类型定义
class TaskType(object):
    FRUITS = "fruits"
    VEGETABLES = "vegetables"
    DESSERTS = "desserts"
    UNKNOWN = "unknown"

# 事件类定义
class Event(object):
    # --- 通用事件 ---
    START_CMD = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2
    # --- 前半段特定事件 ---
    QR_RESULT_VALID = 3
    PERCEPTION_TIMEOUT = 4
    QR_NODE_SHUTDOWN_COMPLETE = 5
    QR_NODE_SHUTDOWN_TIMEOUT = 6
    # --- 通用事件 ---
    SPEAK_DONE = 7
    SPEAK_TIMEOUT = 8
    # --- 后半段特定事件 ---
    SEARCH_DONE_SUCCESS = 9
    SEARCH_DONE_FAILURE = 10
    GOODS_FOUND = 11
    PATROL_SEQUENCE_COMPLETED = 12
    SIMULATION_DONE = 13
    LIGHT_DETECTED_GREEN = 14
    LIGHT_DETECTED_RED = 15
    LIGHT_DETECT_TIMEOUT = 16
    # --- 任务收尾阶段事件 ---
    NODES_SHUTDOWN_COMPLETE = 17  # 替代原YOLO_SHUTDOWN_...
    NODES_SHUTDOWN_TIMEOUT = 18   # 替代原YOLO_SHUTDOWN_...
    LINE_FOLLOWING_DONE = 19      # 新增：巡线完成事件

# 统一状态类定义
class RobotState(object):
    # --- 前半段状态 ---
    IDLE = 0
    NAVIGATE_TO_QR1 = 1
    ROTATE_TO_QR2 = 2
    NAVIGATE_TO_QR_AREA = 3
    WAIT_FOR_QR_RESULT = 4
    SHUTDOWN_QR_NODE = 5
    SPEAK_TASK_TYPE = 6
    NAV_TO_PICK_PREP_AREA = 7
    # --- 后半段状态 ---
    NAVIGATE_TO_UP_POINT = 8
    SEARCH_UP_BOARD = 9
    NAVIGATE_TO_DOWN_POINT = 10
    SEARCH_DOWN_BOARD = 11
    PICK_UP_GOODS = 12
    SPEAK_GOODS = 13
    NAV_TO_SIMULATION = 14
    DO_SIMULATION_TASKS = 15
    SPEAK_ROOM = 16
    NAV_TO_TRAFFIC = 17
    NAV_TO_LANE1_OBSERVE_POINT = 18
    DETECTING_LANE1_LIGHT = 19
    SPEAK_LANE1_CLEAR = 20
    NAV_TO_LANE1_WAITING_POINT = 21
    NAV_TO_LANE2_OBSERVE_POINT = 22
    DETECTING_LANE2_LIGHT = 23
    SPEAK_LANE2_CLEAR = 24
    NAV_TO_LANE2_WAITING_POINT = 25
    # --- 任务收尾阶段（新） ---
    SHUTDOWN_FINAL_NODES = 26         # 替代 SHUTDOWN_YOLO_NODE
    RELAUNCH_CAMERA_FOR_LINE_FOLLOWING = 27
    LINE_FOLLOWING = 28
    SPEAK_FINAL = 29
    TASK_COMPLETE = 30
    # --- 系统状态 ---
    ERROR = 99

#********************************* 主状态机类 *********************************#

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
        
        rospy.loginfo("统一状态机初始化完成（支持语音唤醒和服务调用，包含完整流程）")
        rospy.loginfo("等待语音唤醒或服务调用...")
        
        # 广播当前状态
        self.publish_state()

    #*********************** 初始化相关函数 ***********************#
    def setup(self):
        # 前半段变量
        self.is_awake = False
        self.last_awake_angle = None
        self.current_state = RobotState.IDLE  # 开发者可修改此行进行测试
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
        
        # 节点关闭相关变量
        self.shutdown_check_timer = None
        self.shutdown_timeout_timer = None
        self.qr_shutdown_timeout_duration = 5.0
        # self.yolo_shutdown_check_timer = None # 将被新的通用节点关闭替代
        # self.yolo_shutdown_timeout_timer = None # 将被新的通用节点关闭替代
        # self.yolo_shutdown_timeout_duration = 5.0 # 将被新的通用节点关闭替代
        
        # 新增：通用节点关闭控制相关变量 (替代原yolo关闭变量)
        self.nodes_shutdown_check_timer = None
        self.nodes_shutdown_timeout_timer = None
        self.nodes_shutdown_timeout_duration = 10.0  # 关闭多个节点，超时时间设长一些

        # 后半段变量
        self.locations = {}
        self.board_detect_script_path = "/home/ucar/lby_ws/src/board_detect/scripts/board_detect_up.py"
        self.board_detect_down_script_path = "/home/ucar/lby_ws/src/board_detect/scripts/board_detect_down.py"
        self.search_timeout_duration = 25.0
        self.board_detect_process = None
        self.search_monitor_timer = None
        self.search_timeout_timer = None
        
        # 新增：巡线任务相关变量
        self.camera_process = None
        self.camera_node_name = "/usb_cam"  # 请根据实际情况确认摄像头节点名
        self.camera_line_following_launch_command = ['roslaunch', 'lby_usb_cam', 'usb_cam_noimg.launch']
        self.line_following_script_right = "/home/ucar/lby_ws/src/follow_line/scripts/shengsai/shengsai_right.py"
        self.line_following_script_left = "/home/ucar/lby_ws/src/follow_line/scripts/shengsai/shengsai_left.py"
        self.line_following_status_sub = None
        self.line_following_timeout_timer = None
        self.line_following_timeout = 300.0 # 巡线任务超时时间（5分钟）

        # 目标点目录路径
        self.goals_up_dir = "/home/ucar/lby_ws/src/board_detect/goals_up"
        self.goals_down_dir = "/home/ucar/lby_ws/src/board_detect/goals_down"
        
        # 货物分类、任务管理和置信度控制
        self.goods_categories = {
            'fruits': ['watermelon', 'banana', 'apple'],
            'desserts': ['milk', 'cake', 'cola'],
            'vegetables': ['pepper', 'potato', 'tomato']
        }
        self.yolo_confidence_threshold = 0.6 # YOLO识别的置信度阈值
        self.found_good_name = None   # 用于存储找到的货物名称
        
        # 新增：仿真任务相关变量
        self.PC_IP = '192.168.68.206'  # 请确保这是您电脑的正确IP地址
        self.simulation_found_item = None
        self.simulation_room_location = None
        
        # 新增：记录最终通过的车道，'lane1' 或 'lane2'
        self.final_lane_choice = None
        
        # 图像和过滤器参数
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        self.CENTER_ZONE_MARGIN = 30  # 中心安全区的边界 (从图像边缘向内)
        self.EDGE_ZONE_MIN_WIDTH = 200 # 边缘区域物体的最小宽度要求
        self.MIN_AREA_RATIO = 0.05     # 最小面积占图像百分比
        self.MAX_AREA_RATIO = 0.80     # 最大面积占图像百分比
        
        # 红绿灯检测相关变量
        self.light_detection_timeout = 3.0  # 红绿灯检测超时时间(秒)
        self.light_detection_timer = None
        self.light_detection_subscriber = None

    def init_ros_comm(self):
        # 状态发布和任务发布
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        self.task_pub = rospy.Publisher('/robot/task_type', String, queue_size=1)
        
        # 导航客户端
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        # 语音唤醒订阅
        self.awake_sub = rospy.Subscriber('/mic/awake/angle', Int32, self.awake_callback, queue_size=1)
        
        # 任务类型订阅
        self.task_type_sub = rospy.Subscriber('/QR/task_type', String, self.task_type_callback, queue_size=1)
        
        # 等待语音播报服务
        rospy.loginfo("等待语音播报服务...")
        try:
            rospy.wait_for_service('/robot/voice_cmd', timeout=5.0)
            rospy.loginfo("语音播报服务连接成功")
        except rospy.ROSException:
            rospy.logwarn("语音播报服务不可用，将在服务可用时自动连接")
        
        # 服务定义
        self.reset_service = rospy.Service('reset_state_machine', Trigger, self.reset_callback)
        self.start_service = rospy.Service('start_state_machine', Trigger, self.start_callback)

    def init_locations(self):
        self.locations = {
            # 前半段位置
            'qr1': self.create_pose(2.07, 0.60, 0.3007, 0.9537),
            'qr2': self.create_pose(2.07, 0.60, 0.9747, 0.2239),
            'qr_area': self.create_pose(1.25, 0.75, -1.0, 0.0),
            'pick_prep_area': self.create_pose(0.5, 2.0, 0.7071, 0.7071),  # 桥梁位置
            # 后半段位置
            'up_point': self.create_pose(1.25, 3.0, 0.7071, 0.7071),
            'down_point': self.create_pose(1.25, 4.10, -0.7071, 0.7071),
            'simulation_area': self.create_pose(1.25, 3.75, 0.7071, 0.7071),
            'lane1_observe_point': self.create_pose(3.25, 4.25, 0.7071, 0.7071),
            'lane2_observe_point': self.create_pose(4.25, 4.25, 0.7071, 0.7071),
            'lane1_waiting_point': self.create_pose(2.75, 3.75, -0.7071, 0.7071),
            'lane2_waiting_point': self.create_pose(4.75, 3.75, -0.7071, 0.7071)
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

    # 执行状态对应的动作
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
            
        # 关闭QR节点状态
        elif self.current_state == RobotState.SHUTDOWN_QR_NODE:
            rospy.loginfo("5-关闭QR节点")
            
            # 执行rosnode kill命令关闭QR节点
            try:
                # 使用subprocess执行rosnode kill命令
                subprocess.Popen(["rosnode", "kill", "/qr_detect"])
                rospy.loginfo("已发送关闭QR节点命令")
                
                # 启动定时检查节点是否已关闭
                self.shutdown_check_timer = rospy.Timer(
                    rospy.Duration(0.5),
                    self._check_qr_node_shutdown_status
                )
                
                # 启动超时定时器
                self.shutdown_timeout_timer = rospy.Timer(
                    rospy.Duration(self.qr_shutdown_timeout_duration),
                    self._handle_qr_node_shutdown_timeout,
                    oneshot=True
                )
            except Exception as e:
                rospy.logerr("关闭QR节点时出错: %s", str(e))
                self.handle_event(Event.QR_NODE_SHUTDOWN_TIMEOUT)
            
        # 播报任务类型状态
        elif self.current_state == RobotState.SPEAK_TASK_TYPE:
            rospy.loginfo("6-播报任务类型: %s", self.task_type)
            
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
            rospy.loginfo("7-导航至拣货准备区（桥梁状态）")
            self.send_nav_goal('pick_prep_area')
            self.navigation_active = True
            
        # --- 后半段状态动作 ---
        elif self.current_state == RobotState.NAVIGATE_TO_UP_POINT:
            rospy.loginfo("8-开始导航至上平面位置...")
            self.send_nav_goal('up_point')
            self.navigation_active = True
            
        elif self.current_state == RobotState.SEARCH_UP_BOARD:
            rospy.loginfo("9-搜索上平面板子")
            self.start_board_detection()
            
        elif self.current_state == RobotState.NAVIGATE_TO_DOWN_POINT:
            rospy.loginfo("10-开始导航至下平面位置...")
            self.send_nav_goal('down_point')
            self.navigation_active = True
            
        elif self.current_state == RobotState.SEARCH_DOWN_BOARD:
            rospy.loginfo("11-搜索下平面板子")
            self.start_down_board_detection()
            
        elif self.current_state == RobotState.PICK_UP_GOODS:
            rospy.loginfo("12-开始执行多点巡检...")
            self.execute_patrol_sequence()
            
        elif self.current_state == RobotState.SPEAK_GOODS:
            if self.found_good_name:
                self.current_voice_cmd = "get_" + self.found_good_name
                rospy.loginfo("13-状态[SPEAK_GOODS]: 准备播报找到的货物: %s", self.current_voice_cmd)
                self.voice_service_called = False
                self.call_voice_service()
            else:
                rospy.logerr("进入SPEAK_GOODS状态，但未指定货物名称")
                self.handle_event(Event.SPEAK_TIMEOUT)
                
        elif self.current_state == RobotState.NAV_TO_SIMULATION:
            rospy.loginfo("14-开始导航至最终目标点：仿真区...")
            self.send_nav_goal('simulation_area')
            self.navigation_active = True
            
        elif self.current_state == RobotState.DO_SIMULATION_TASKS:
            rospy.loginfo("15-开始执行仿真任务，调用电脑端服务...")
            # 调用仿真服务并根据结果触发事件
            success = self._call_simulation_service()
            if success:
                self.handle_event(Event.SIMULATION_DONE)
            else:
                rospy.logerr("仿真任务执行失败。")
                self.handle_event(Event.NAV_DONE_FAILURE) # 复用导航失败事件
            
        elif self.current_state == RobotState.SPEAK_ROOM:
            rospy.loginfo("16-状态[SPEAK_ROOM]: 准备播报仿真任务找到的房间...")
            if self.simulation_room_location and self.simulation_room_location != 'unknown_room':
                self.current_voice_cmd = self.simulation_room_location
                self.voice_service_called = False
                self.call_voice_service()
            else:
                rospy.logwarn("未记录有效的仿真房间位置，跳过播报。")
                rospy.Timer(rospy.Duration(0.1), lambda e: self.handle_event(Event.SPEAK_DONE), oneshot=True)

        elif self.current_state == RobotState.NAV_TO_TRAFFIC:
            rospy.loginfo("17-进入红绿灯区域状态，准备导航至第一车道观察点...")
            rospy.Timer(rospy.Duration(0.5), self._traffic_timer_callback, oneshot=True)
            
        elif self.current_state == RobotState.NAV_TO_LANE1_OBSERVE_POINT:
            rospy.loginfo("18-开始导航至第一车道观察点...")
            self.send_nav_goal('lane1_observe_point')
            self.navigation_active = True

        elif self.current_state == RobotState.DETECTING_LANE1_LIGHT:
            rospy.loginfo("19-开始检测第一车道红绿灯...")
            self.start_light_detection()
            
        elif self.current_state == RobotState.SPEAK_LANE1_CLEAR:
            rospy.loginfo("20-第一车道绿灯，准备播报通行信息...")
            self.current_voice_cmd = "way_1"
            self.voice_service_called = False
            self.call_voice_service()
            
        elif self.current_state == RobotState.NAV_TO_LANE1_WAITING_POINT:
            rospy.loginfo("21-开始导航至第一车道等待点...")
            self.send_nav_goal('lane1_waiting_point')
            self.navigation_active = True
            
        elif self.current_state == RobotState.NAV_TO_LANE2_OBSERVE_POINT:
            rospy.loginfo("22-开始导航至第二车道观察点...")
            self.send_nav_goal('lane2_observe_point')
            self.navigation_active = True
            
        elif self.current_state == RobotState.DETECTING_LANE2_LIGHT:
            rospy.loginfo("23-开始检测第二车道红绿灯...")
            self.start_light_detection()
            
        elif self.current_state == RobotState.SPEAK_LANE2_CLEAR:
            rospy.loginfo("24-第二车道绿灯，准备播报通行信息...")
            self.current_voice_cmd = "way_2"
            self.voice_service_called = False
            self.call_voice_service()
            
        elif self.current_state == RobotState.NAV_TO_LANE2_WAITING_POINT:
            rospy.loginfo("25-开始导航至第二车道等待点...")
            self.send_nav_goal('lane2_waiting_point')
            self.navigation_active = True
            
        elif self.current_state == RobotState.SHUTDOWN_FINAL_NODES:
            rospy.loginfo("26-正在尝试关闭最终节点: %s, /darknet_ros, /move_base, /amcl...", self.camera_node_name)
            subprocess.call(['rosnode', 'kill', self.camera_node_name])
            subprocess.call(['rosnode', 'kill', '/darknet_ros'])
            subprocess.call(['rosnode', 'kill', '/move_base'])
            subprocess.call(['rosnode', 'kill', '/amcl'])
            
            self.nodes_shutdown_check_timer = rospy.Timer(
                rospy.Duration(0.25),
                self._check_nodes_shutdown_status
            )
            
            self.nodes_shutdown_timeout_timer = rospy.Timer(
                rospy.Duration(self.nodes_shutdown_timeout_duration),
                self._handle_nodes_shutdown_timeout,
                oneshot=True
            )

        elif self.current_state == RobotState.RELAUNCH_CAMERA_FOR_LINE_FOLLOWING:
            rospy.loginfo("27-所有节点已关闭。等待2秒后为巡线任务重启摄像头...")
            rospy.sleep(2.0)
            try:
                rospy.loginfo("正在执行命令: %s", ' '.join(self.camera_line_following_launch_command))
                self.camera_process = subprocess.Popen(self.camera_line_following_launch_command)
                rospy.loginfo("摄像头启动命令已发送，进程ID: %s", self.camera_process.pid)
                rospy.sleep(3.0)
                self.transition(RobotState.LINE_FOLLOWING)
            except Exception as e:
                rospy.logerr("启动摄像头launch文件失败: %s", str(e))
                self.transition(RobotState.ERROR)

        elif self.current_state == RobotState.LINE_FOLLOWING:
            rospy.loginfo("28-进入最终巡线状态，根据选择的车道启动相应脚本...")
            script_to_run = None
            if self.final_lane_choice == 'lane1':
                rospy.loginfo("选择第一车道，启动右侧巡线脚本: %s", self.line_following_script_right)
                script_to_run = self.line_following_script_right
            elif self.final_lane_choice == 'lane2':
                rospy.loginfo("选择第二车道，启动左侧巡线脚本: %s", self.line_following_script_left)
                script_to_run = self.line_following_script_left
            else:
                rospy.logerr("无法确定最终车道选择，默认使用右侧巡线脚本。")
                script_to_run = self.line_following_script_right
            
            try:
                if not os.path.exists(script_to_run):
                    rospy.logerr("巡线脚本不存在: %s", script_to_run)
                    self.transition(RobotState.ERROR)
                    return

                rospy.loginfo("正在执行命令: python %s", script_to_run)
                subprocess.Popen(["python", script_to_run])
                rospy.loginfo("巡线脚本已启动。等待完成信号...")
                
                self.line_following_status_sub = rospy.Subscriber(
                    '/line_following_status', String, self._line_following_status_callback
                )
                self.line_following_timeout_timer = rospy.Timer(
                    rospy.Duration(self.line_following_timeout),
                    self._line_following_timeout_callback,
                    oneshot=True
                )
            except Exception as e:
                rospy.logerr("启动巡线脚本失败: %s", str(e))
                self.transition(RobotState.ERROR)

        elif self.current_state == RobotState.SPEAK_FINAL:
            rospy.loginfo("29-状态[SPEAK_FINAL]: 准备进行最终任务播报...")
            item1 = self.found_good_name
            item2 = self.simulation_found_item
            if item1 and item2:
                self.current_voice_cmd = "task_fin_{}_{}".format(item1, item2)
                rospy.loginfo("构造最终播报命令: %s", self.current_voice_cmd)
                self.voice_service_called = False
                self.call_voice_service()
            else:
                rospy.logerr("无法执行最终播报，缺少物品信息: 巡检物品='%s', 仿真物品='%s'", item1, item2)
                rospy.Timer(rospy.Duration(0.1), lambda e: self.transition(RobotState.TASK_COMPLETE), oneshot=True)

        elif self.current_state == RobotState.TASK_COMPLETE:
            rospy.loginfo("30-全部任务完成！机器人将停止所有活动。")
            self.stop_all_activities()
            
        elif self.current_state == RobotState.ERROR:
            rospy.logerr("99-错误状态，停止所有活动")
            self.stop_all_activities()

    # 事件处理函数，根据当前状态和接受到的事件进行状态转换
    def handle_event(self, event):
        event_name = self.event_name(event)
        state_name = self.state_name(self.current_state)
        rospy.loginfo("在状态 %s 收到事件 %s", state_name, event_name)
        
        # --- 前半段事件处理 ---
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
                self.transition(RobotState.WAIT_FOR_QR_RESULT)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.WAIT_FOR_QR_RESULT:
            if event == Event.QR_RESULT_VALID:
                self.stop_perception_timer()
                rospy.loginfo("收到有效的二维码结果，任务类型: %s", self.task_type)
                self.publish_task_type()
                self.transition(RobotState.SHUTDOWN_QR_NODE)
            elif event == Event.PERCEPTION_TIMEOUT:
                rospy.logerr("二维码识别超时")
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.SHUTDOWN_QR_NODE:
            if event == Event.QR_NODE_SHUTDOWN_COMPLETE:
                rospy.loginfo("QR节点已成功关闭，准备播报任务类型")
                self.transition(RobotState.SPEAK_TASK_TYPE)
            elif event == Event.QR_NODE_SHUTDOWN_TIMEOUT:
                rospy.logerr("QR节点关闭超时")
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.SPEAK_TASK_TYPE:
            if event == Event.SPEAK_DONE:
                rospy.loginfo("语音播报完成，准备导航至拣货准备区")
                self.transition(RobotState.NAV_TO_PICK_PREP_AREA)
            elif event == Event.SPEAK_TIMEOUT:
                rospy.logerr("语音播报超时")
                self.transition(RobotState.ERROR)
                
        # 关键桥梁状态的事件处理
        elif self.current_state == RobotState.NAV_TO_PICK_PREP_AREA:
            if event == Event.NAV_DONE_SUCCESS:
                rospy.loginfo("导航至拣货准备区完成，开始后半段流程")
                # 关键桥梁：转换到后半段起始状态
                self.transition(RobotState.NAVIGATE_TO_UP_POINT)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
        
        # --- 后半段事件处理 ---
        elif self.current_state == RobotState.NAVIGATE_TO_UP_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.SEARCH_UP_BOARD)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.SEARCH_UP_BOARD:
            if event == Event.SEARCH_DONE_SUCCESS:
                self.transition(RobotState.NAVIGATE_TO_DOWN_POINT)
            elif event == Event.SEARCH_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAVIGATE_TO_DOWN_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.SEARCH_DOWN_BOARD)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.SEARCH_DOWN_BOARD:
            if event == Event.SEARCH_DONE_SUCCESS:
                self.transition(RobotState.PICK_UP_GOODS)
            elif event == Event.SEARCH_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.PICK_UP_GOODS:
            if event == Event.GOODS_FOUND:
                self.transition(RobotState.SPEAK_GOODS)
            elif event == Event.PATROL_SEQUENCE_COMPLETED:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.SPEAK_GOODS:
            if event == Event.SPEAK_DONE:
                self.transition(RobotState.NAV_TO_SIMULATION)
            elif event == Event.SPEAK_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAV_TO_SIMULATION:
            if event == Event.NAV_DONE_SUCCESS:
                rospy.loginfo("成功到达仿真区，准备执行仿真任务。")
                self.transition(RobotState.DO_SIMULATION_TASKS)
            elif event == Event.NAV_DONE_FAILURE:
                rospy.logerr("导航至仿真区失败。")
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.DO_SIMULATION_TASKS and event == Event.SIMULATION_DONE:
            self.transition(RobotState.SPEAK_ROOM)
            
        elif self.current_state == RobotState.SPEAK_ROOM and event == Event.SPEAK_DONE:
            self.transition(RobotState.NAV_TO_TRAFFIC)
            
        elif self.current_state == RobotState.NAV_TO_TRAFFIC and event == Event.SPEAK_DONE:
            self.transition(RobotState.NAV_TO_LANE1_OBSERVE_POINT)
            
        elif self.current_state == RobotState.NAV_TO_LANE1_OBSERVE_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.DETECTING_LANE1_LIGHT)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)

        elif self.current_state == RobotState.DETECTING_LANE1_LIGHT:
            if event == Event.LIGHT_DETECTED_GREEN:
                self.transition(RobotState.SPEAK_LANE1_CLEAR)
            elif event == Event.LIGHT_DETECTED_RED:
                self.transition(RobotState.NAV_TO_LANE2_OBSERVE_POINT)
            elif event == Event.LIGHT_DETECT_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.SPEAK_LANE1_CLEAR:
            if event == Event.SPEAK_DONE:
                self.final_lane_choice = 'lane1'  # 记录选择的是第一车道
                self.transition(RobotState.NAV_TO_LANE1_WAITING_POINT)
            elif event == Event.SPEAK_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAV_TO_LANE1_WAITING_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.SHUTDOWN_FINAL_NODES) # 修改：进入关闭最终节点状态
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAV_TO_LANE2_OBSERVE_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.DETECTING_LANE2_LIGHT)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.DETECTING_LANE2_LIGHT:
            if event == Event.LIGHT_DETECTED_GREEN:
                self.transition(RobotState.SPEAK_LANE2_CLEAR)
            elif event == Event.LIGHT_DETECTED_RED or event == Event.LIGHT_DETECT_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.SPEAK_LANE2_CLEAR:
            if event == Event.SPEAK_DONE:
                self.final_lane_choice = 'lane2'  # 记录选择的是第二车道
                self.transition(RobotState.NAV_TO_LANE2_WAITING_POINT)
            elif event == Event.SPEAK_TIMEOUT:
                self.transition(RobotState.ERROR)
                
        elif self.current_state == RobotState.NAV_TO_LANE2_WAITING_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.SHUTDOWN_FINAL_NODES) # 修改：进入关闭最终节点状态
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)

        # --- 以下为新增和重构的收尾阶段事件处理 ---
        
        elif self.current_state == RobotState.SHUTDOWN_FINAL_NODES:
            if event == Event.NODES_SHUTDOWN_COMPLETE:
                self.transition(RobotState.RELAUNCH_CAMERA_FOR_LINE_FOLLOWING)
            elif event == Event.NODES_SHUTDOWN_TIMEOUT:
                self.transition(RobotState.ERROR)

        elif self.current_state == RobotState.LINE_FOLLOWING:
            if event == Event.LINE_FOLLOWING_DONE:
                self.transition(RobotState.SPEAK_FINAL)
            elif event == Event.NAV_DONE_FAILURE: # 复用事件表示超时
                self.transition(RobotState.ERROR)

        elif self.current_state == RobotState.SPEAK_FINAL:
            if event == Event.SPEAK_DONE:
                self.transition(RobotState.TASK_COMPLETE)
            elif event == Event.SPEAK_TIMEOUT:
                rospy.logwarn("最终播报超时，直接进入完成状态。")
                self.transition(RobotState.TASK_COMPLETE)

    #*********************** 回调函数 ***********************#
    # 语音唤醒回调函数
    def awake_callback(self, msg):
        self.last_awake_angle = msg.data
        rospy.loginfo("收到唤醒角度: %d度", self.last_awake_angle)

        if self.current_state == RobotState.IDLE and not self.is_awake:
            self.is_awake = True
            rospy.loginfo("通过语音唤醒，启动1秒定时器...")
            rospy.Timer(rospy.Duration(1.0), 
                       lambda event: self.handle_event(Event.START_CMD), 
                       oneshot=True)

    # 服务启动状态机的回调函数
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

    # 领取二维码识别到的任务类型的回调函数
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

    # 服务重置状态机的回调函数
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

    # 导航完成回调函数
    def navigation_done_callback(self, status, result):
        self.navigation_active = False
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("导航成功，启动0.5秒定时器...")
            rospy.Timer(rospy.Duration(0.5), 
                      lambda event: self.delayed_nav_success(), 
                      oneshot=True)
        else:
            self.handle_event(Event.NAV_DONE_FAILURE)

    # 延迟导航成功处理
    def delayed_nav_success(self):
        rospy.loginfo("0.5秒定时结束，执行状态转换...")
        self.handle_event(Event.NAV_DONE_SUCCESS)

    #*********************** 后半段特有功能函数 ***********************#
    
    # 货物检测功能
    def check_for_target_goods(self, timeout=3.0):
        """
        在指定的超时时间内监听/darknet_ros/bounding_boxes话题，
        寻找符合当前任务的货物，采用"先到先得"策略。
        """
        rospy.loginfo("开始在当前位置检测货物，任务类型: %s，监听时间: %.1f秒", 
                    self.task_type, timeout)
        
        # 获取当前任务对应的目标货物列表
        target_goods = self.goods_categories.get(self.task_type, [])
        if not target_goods:
            rospy.logwarn("当前任务'%s'没有对应的目标货物列表", self.task_type)
            return None
        
        rospy.loginfo("正在寻找以下货物：%s，置信度阈值：%.2f", 
                    ', '.join(target_goods), self.yolo_confidence_threshold)
        
        # 使用Python原生的threading.Event进行同步
        detection_event = threading.Event()
        
        # 使用一个列表（或任何可变对象）来在线程间传递结果
        detection_result = {'found_item': None}
        
        # 用于日志节流的时间戳管理器
        log_throttle_manager = {'last_log_time': 0.0}
        
        # 回调函数，处理收到的边界框信息
        def bounding_boxes_callback(msg):
            # 如果事件已经设置，说明已经找到目标，直接返回避免重复处理
            if detection_event.is_set():
                return
            
            # 新增：日志节流阀
            current_time = rospy.get_time()
            if (current_time - log_throttle_manager['last_log_time']) < 3.0:
                return # 距离上次打印不足3秒，直接忽略此帧
            log_throttle_manager['last_log_time'] = current_time
            
            # 计算最小和最大允许面积（以像素为单位）
            min_area_pixels = self.IMAGE_WIDTH * self.IMAGE_HEIGHT * self.MIN_AREA_RATIO
            max_area_pixels = self.IMAGE_WIDTH * self.IMAGE_HEIGHT * self.MAX_AREA_RATIO
            
            for box in msg.bounding_boxes:
                rospy.loginfo("--- 开始分析检测框: %s (x1:%d, y1:%d, x2:%d, y2:%d) ---", 
                            box.Class, box.xmin, box.ymin, box.xmax, box.ymax)

                # --- 第一关：条件化宽度过滤器 ---
                rospy.loginfo("  [第一关 - 位置检查] 区域定义: 中心区 >%d 且 <%d 像素.", 
                            self.CENTER_ZONE_MARGIN, self.IMAGE_WIDTH - self.CENTER_ZONE_MARGIN)
                
                is_in_edge_zone = False
                # 检查左边缘区
                if box.xmin < self.CENTER_ZONE_MARGIN:
                    is_in_edge_zone = True
                    rospy.loginfo("    -> 物体位于左侧边缘区, 因为 xmin(%d) < %d.", 
                                box.xmin, self.CENTER_ZONE_MARGIN)
                # 检查右边缘区
                if box.xmax > (self.IMAGE_WIDTH - self.CENTER_ZONE_MARGIN):
                    is_in_edge_zone = True
                    rospy.loginfo("    -> 物体位于右侧边缘区, 因为 xmax(%d) > %d.", 
                                box.xmax, self.IMAGE_WIDTH - self.CENTER_ZONE_MARGIN)

                if is_in_edge_zone:
                    box_width = box.xmax - box.xmin
                    rospy.loginfo("    -> 检查边缘区宽度要求: %d >= %d?", 
                                box_width, self.EDGE_ZONE_MIN_WIDTH)
                    if box_width < self.EDGE_ZONE_MIN_WIDTH:
                        rospy.loginfo("  [拒绝] 宽度不满足边缘区要求.")
                        continue
                else:
                    rospy.loginfo("  [第一关 - 位置检查] 物体位于中心区, 免除宽度检查.")

                # --- 第二关：面积过滤器 ---
                box_area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
                min_area_pixels = self.IMAGE_WIDTH * self.IMAGE_HEIGHT * self.MIN_AREA_RATIO
                max_area_pixels = self.IMAGE_WIDTH * self.IMAGE_HEIGHT * self.MAX_AREA_RATIO
                rospy.loginfo("  [第二关 - 面积检查] 检查面积: %d <= %d <= %d?", 
                            int(min_area_pixels), box_area, int(max_area_pixels))
                if not (min_area_pixels <= box_area <= max_area_pixels):
                    rospy.loginfo("  [拒绝] 面积不在有效范围内.")
                    continue

                # --- 第三关：置信度过滤器 ---
                rospy.loginfo("  [第三关 - 置信度检查] 检查置信度: %.2f >= %.2f?", 
                            box.probability, self.yolo_confidence_threshold)
                if box.probability < self.yolo_confidence_threshold:
                    rospy.loginfo("  [拒绝] 置信度过低.")
                    continue

                # --- 第四关：类别过滤器 ---
                rospy.loginfo("  [第四关 - 类别检查] 检查类别: '%s' 是否在 %s 中?", 
                            box.Class, target_goods)
                if box.Class not in target_goods:
                    rospy.loginfo("  [拒绝] 类别不在目标列表中.")
                    continue
                
                # --- 最终接受 ---
                rospy.loginfo("  [成功] 所有检查通过, 已锁定目标!")
                detection_result['found_item'] = box.Class
                detection_event.set()
                return

        # 订阅YOLO检测结果话题
        subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes', 
                                    BoundingBoxes, 
                                    bounding_boxes_callback)
        
        # 等待事件被设置（即回调函数找到目标）或超时
        detection_event.wait(timeout=timeout)
        
        # 任务结束，必须取消订阅，释放资源
        subscriber.unregister()
        
        # 返回找到的结果
        return detection_result['found_item']

    # 加载巡检目标点
    def load_patrol_goals(self):
        """加载YAML文件中的目标点"""
        goal_poses = []
        
        try:
            # 获取上平面和下平面的所有goal*.yaml文件
            yaml_files_up = glob.glob(os.path.join(self.goals_up_dir, 'goal*.yaml'))
            yaml_files_down = glob.glob(os.path.join(self.goals_down_dir, 'goal*.yaml'))
            
            # 合并文件列表
            yaml_files = yaml_files_up + yaml_files_down
            
            # 如果没有找到文件，返回空列表
            if not yaml_files:
                rospy.logerr("在目录下未找到任何goal*.yaml文件")
                return goal_poses
            
            # 按文件名数字排序
            yaml_files.sort(key=lambda f: int(''.join(filter(str.isdigit, os.path.basename(f)))))
            
            rospy.loginfo("找到{}个目标点文件".format(len(yaml_files)))
            
            for file_path in yaml_files:
                try:
                    # 读取YAML文件
                    with open(file_path, 'r') as yaml_file:
                        goal_data = yaml.safe_load(yaml_file)
                    
                    # 从YAML数据创建PoseStamped对象
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = goal_data['header']['frame_id']
                    # header.stamp在发送前设置，这里可以先设为0
                    pose_stamped.header.stamp = rospy.Time(0)
                    
                    pose_stamped.pose.position.x = goal_data['pose']['position']['x']
                    pose_stamped.pose.position.y = goal_data['pose']['position']['y']
                    pose_stamped.pose.position.z = goal_data['pose']['position']['z']
                    
                    pose_stamped.pose.orientation.x = goal_data['pose']['orientation']['x']
                    pose_stamped.pose.orientation.y = goal_data['pose']['orientation']['y']
                    pose_stamped.pose.orientation.z = goal_data['pose']['orientation']['z']
                    pose_stamped.pose.orientation.w = goal_data['pose']['orientation']['w']
                    
                    # 添加到目标点列表
                    goal_poses.append(pose_stamped)
                    rospy.loginfo("已加载目标点: {}".format(os.path.basename(file_path)))
                
                except Exception as e:
                    rospy.logerr("处理文件{}时出错: {}".format(file_path, str(e)))
        
        except Exception as e:
            rospy.logerr("加载YAML目标点时出错: {}".format(str(e)))
        
        return goal_poses
    
    # 执行巡检序列
    def execute_patrol_sequence(self):
        """执行多点巡检导航逻辑，寻找目标货物"""
        rospy.loginfo("开始执行多点巡检导航，寻找 %s 类别的货物...", self.task_type)
        
        # 获取当前任务需要检测的货物列表
        target_goods = self.goods_categories.get(self.task_type, [])
        if not target_goods:
            rospy.logwarn("当前任务'%s'没有对应的目标货物，巡检将无法找到目标", self.task_type)
        else:
            rospy.loginfo("本次巡检目标货物: %s", ", ".join(target_goods))
        
        # 加载巡检点
        waypoints = self.load_patrol_goals()
        
        # 检查是否成功加载了目标点
        if not waypoints:
            rospy.logerr("没有找到有效的目标点，巡检终止。")
            self.handle_event(Event.PATROL_SEQUENCE_COMPLETED)
            return
        
        rospy.loginfo("成功加载了{}个巡检点".format(len(waypoints)))
        
        # 遍历所有目标点，发送并等待每个目标
        for i, pose_stamped in enumerate(waypoints):
            # 创建MoveBaseGoal实例
            goal = MoveBaseGoal()
            goal.target_pose = pose_stamped
            # 在发送前更新时间戳
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # 打印当前要发送的目标点信息
            rospy.loginfo("正在发送第{}个巡检点 (总共{}个)".format(i + 1, len(waypoints)))
            rospy.loginfo("目标位置: x={:.2f}, y={:.2f}".format(
                pose_stamped.pose.position.x, pose_stamped.pose.position.y))
            
            # 发送目标点
            self.move_base_client.send_goal(goal)
            
            # 阻塞式等待导航结果，设置20秒超时
            rospy.loginfo("等待导航结果，最多20秒...")
            succeeded = self.move_base_client.wait_for_result(rospy.Duration(20.0))
            
            # 获取导航状态
            status = self.move_base_client.get_state()
            
            # 根据状态打印结果
            status_str = {
                0: "PENDING",
                1: "ACTIVE",
                2: "PREEMPTED",
                3: "SUCCEEDED",
                4: "ABORTED",
                5: "REJECTED",
                6: "PREEMPTING",
                7: "RECALLING",
                8: "RECALLED",
                9: "LOST"
            }.get(status, "UNKNOWN({})".format(status))
            
            if succeeded and status == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("巡检点{}导航成功!".format(i + 1))
                
                # 在当前位置检测目标货物
                found_item = self.check_for_target_goods(timeout=3.0)
                
                # 如果找到目标货物，记录名称并结束巡检
                if found_item is not None:
                    self.found_good_name = found_item
                    rospy.loginfo("在巡检点{}成功找到目标货物: {}!".format(i + 1, found_item))
                    self.handle_event(Event.GOODS_FOUND)
                    return
                else:
                    rospy.loginfo("巡检点{}未检测到目标货物，继续前往下一个点。".format(i + 1))
            else:
                rospy.logwarn("巡检点{}导航失败: {}，跳过此点。".format(i + 1, status_str))
        
        # 所有巡检点都检查完毕，未找到目标货物
        rospy.logwarn("所有巡检点检查完毕，未找到目标货物。将进入错误状态。")
        self.handle_event(Event.PATROL_SEQUENCE_COMPLETED)

    # 启动上平面板子检测
    def start_board_detection(self):
        rospy.loginfo("开始启动外部脚本进行板子检测...")
        
        # 检查脚本是否存在且可执行
        if not os.path.exists(self.board_detect_script_path):
            rospy.logerr("检测脚本不存在: %s", self.board_detect_script_path)
            self.handle_event(Event.SEARCH_DONE_FAILURE)
            return
        
        try:
            # 启动外部脚本
            self.board_detect_process = subprocess.Popen(["python", self.board_detect_script_path])
            rospy.loginfo("检测脚本已启动，进程ID: %s", self.board_detect_process.pid)
            
            # 启动监控定时器
            self.search_monitor_timer = rospy.Timer(rospy.Duration(0.5),
                                                  self.monitor_search_process_callback)
            
            # 启动超时定时器
            self.search_timeout_timer = rospy.Timer(rospy.Duration(self.search_timeout_duration),
                                                  self.search_timeout_callback,
                                                  oneshot=True)
        except Exception as e:
            rospy.logerr("启动检测脚本失败: %s", str(e))
            self.handle_event(Event.SEARCH_DONE_FAILURE)
    
    # 启动下平面板子检测
    def start_down_board_detection(self):
        rospy.loginfo("开始启动外部脚本进行下平面板子检测...")
        
        # 检查脚本是否存在且可执行
        if not os.path.exists(self.board_detect_down_script_path):
            rospy.logerr("检测脚本不存在: %s", self.board_detect_down_script_path)
            self.handle_event(Event.SEARCH_DONE_FAILURE)
            return
        
        try:
            # 启动外部脚本
            self.board_detect_process = subprocess.Popen(["python", self.board_detect_down_script_path])
            rospy.loginfo("检测脚本已启动，进程ID: %s", self.board_detect_process.pid)
            
            # 启动监控定时器
            self.search_monitor_timer = rospy.Timer(rospy.Duration(0.5),
                                                  self.monitor_search_process_callback)
            
            # 启动超时定时器
            self.search_timeout_timer = rospy.Timer(rospy.Duration(self.search_timeout_duration),
                                                  self.search_timeout_callback,
                                                  oneshot=True)
        except Exception as e:
            rospy.logerr("启动检测脚本失败: %s", str(e))
            self.handle_event(Event.SEARCH_DONE_FAILURE)

    # 监控检测进程的回调函数
    def monitor_search_process_callback(self, event):
        if self.board_detect_process is not None:
            return_code = self.board_detect_process.poll()
            if return_code is not None:
                rospy.loginfo("检测脚本已结束，返回代码: %s", return_code)
                self.stop_search_timers()
                
                # 关键修复：在这里将进程句柄清空
                self.board_detect_process = None
                
                self.handle_event(Event.SEARCH_DONE_SUCCESS)    

    # 检测超时回调函数
    def search_timeout_callback(self, event):
        rospy.logerr("检测脚本执行超时")
        self.stop_search_timers()
        if self.board_detect_process is not None:
            self.board_detect_process.terminate()
            self.board_detect_process = None
        self.handle_event(Event.SEARCH_DONE_FAILURE)

    # 停止所有检测相关定时器
    def stop_search_timers(self):
        if self.search_monitor_timer is not None:
            self.search_monitor_timer.shutdown()
            self.search_monitor_timer = None
        
        if self.search_timeout_timer is not None:
            self.search_timeout_timer.shutdown()
            self.search_timeout_timer = None

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
        if self.current_state == RobotState.SPEAK_TASK_TYPE or self.current_state == RobotState.SPEAK_GOODS:
            self.handle_event(Event.SPEAK_TIMEOUT)
            
    #*********************** 节点关闭相关功能 ***********************#
    
    # 检查QR节点关闭状态
    def _check_qr_node_shutdown_status(self, event):
        """检查QR节点是否已经关闭"""
        # 获取当前所有节点列表
        node_list = rosnode.get_node_names()
        
        # 检查QR节点是否还在运行
        if '/qr_detect' not in node_list:
            rospy.loginfo("QR节点已成功关闭")
            
            # 停止所有定时器
            if self.shutdown_check_timer:
                self.shutdown_check_timer.shutdown()
                self.shutdown_check_timer = None
                
            if self.shutdown_timeout_timer:
                self.shutdown_timeout_timer.shutdown()
                self.shutdown_timeout_timer = None
                
            # 触发节点关闭完成事件
            self.handle_event(Event.QR_NODE_SHUTDOWN_COMPLETE)
        else:
            rospy.loginfo("QR节点仍在运行，继续等待...")
    
    # 处理QR节点关闭超时
    def _handle_qr_node_shutdown_timeout(self, event):
        """处理QR节点关闭超时事件"""
        rospy.logerr("QR节点关闭超时")
        
        # 停止检查定时器
        if self.shutdown_check_timer:
            self.shutdown_check_timer.shutdown()
            self.shutdown_check_timer = None
            
        # 触发节点关闭超时事件
        self.handle_event(Event.QR_NODE_SHUTDOWN_TIMEOUT)
    
    # 检查YOLO节点关闭状态
    def _check_yolo_node_shutdown_status(self, event):
        """检查YOLO节点是否已经关闭"""
        # 获取当前所有节点列表
        node_list = rosnode.get_node_names()
        
        # 检查YOLO节点是否还在运行
        if '/darknet_ros' not in node_list:
            rospy.loginfo("YOLO节点已成功关闭")
            
            # 停止所有定时器
            if self.yolo_shutdown_check_timer:
                self.yolo_shutdown_check_timer.shutdown()
                self.yolo_shutdown_check_timer = None
                
            if self.yolo_shutdown_timeout_timer:
                self.yolo_shutdown_timeout_timer.shutdown()
                self.yolo_shutdown_timeout_timer = None
                
            # 触发节点关闭完成事件
            self.handle_event(Event.YOLO_SHUTDOWN_COMPLETE)
        else:
            rospy.loginfo("YOLO节点仍在运行，继续等待...")
    
    # 处理YOLO节点关闭超时
    def _handle_yolo_node_shutdown_timeout(self, event):
        """处理YOLO节点关闭超时事件"""
        rospy.logerr("YOLO节点关闭超时")
        
        # 停止检查定时器
        if self.yolo_shutdown_check_timer:
            self.yolo_shutdown_check_timer.shutdown()
            self.yolo_shutdown_check_timer = None
            
        # 触发节点关闭超时事件
        self.handle_event(Event.YOLO_SHUTDOWN_TIMEOUT)

    #*********************** 后半段特有定时器回调 ***********************#
    
    def _simulation_timer_callback(self, event):
        self.handle_event(Event.SIMULATION_DONE)
        
    def _speak_room_timer_callback(self, event):
        self.handle_event(Event.SPEAK_DONE)
        
    def _traffic_timer_callback(self, event):
        self.handle_event(Event.SPEAK_DONE)
    
    def _light_detection_callback(self, msg):
        """红绿灯检测回调函数，处理YOLO检测结果"""
        for box in msg.bounding_boxes:
            if box.Class == 'green':
                self.stop_light_detection_activities()
                rospy.Timer(rospy.Duration(0.1), 
                           lambda event: self.handle_event(Event.LIGHT_DETECTED_GREEN), 
                           oneshot=True)
                return
            elif box.Class == 'red':
                self.stop_light_detection_activities()
                rospy.Timer(rospy.Duration(0.1), 
                           lambda event: self.handle_event(Event.LIGHT_DETECTED_RED), 
                           oneshot=True)
                return
    
    def _light_detection_timeout_callback(self, event):
        """红绿灯检测超时回调函数"""
        rospy.logerr("红绿灯检测超时")
        self.stop_light_detection_activities()
        self.handle_event(Event.LIGHT_DETECT_TIMEOUT)
    
    def start_light_detection(self):
        """开始红绿灯检测"""
        rospy.loginfo("开始检测红绿灯...")
        self.light_detection_timer = rospy.Timer(
            rospy.Duration(self.light_detection_timeout),
            self._light_detection_timeout_callback,
            oneshot=True
        )
        self.light_detection_subscriber = rospy.Subscriber(
            '/darknet_ros/bounding_boxes',
            BoundingBoxes,
            self._light_detection_callback
        )
    
    def stop_light_detection_activities(self):
        """停止所有红绿灯检测相关活动"""
        if self.light_detection_timer is not None:
            self.light_detection_timer.shutdown()
            self.light_detection_timer = None
        
        if self.light_detection_subscriber is not None:
            self.light_detection_subscriber.unregister()
            self.light_detection_subscriber = None

    #*********************** 仿真、节点管理与巡线 (新增) ***********************#

    def _call_simulation_service(self):
        """
        连接到电脑端的rosbridge并调用仿真服务，阻塞等待结果。
        """
        rospy.loginfo("准备连接电脑端 rosbridge at %s:9090", self.PC_IP)
        client = roslibpy.Ros(host=self.PC_IP, port=9090)
        
        try:
            client.run()
            connect_timeout = 5.0
            start_time = time.time()
            while not client.is_connected:
                if time.time() - start_time > connect_timeout:
                    rospy.logerr("连接电脑端 rosbridge 超时。")
                    client.terminate()
                    return False
                time.sleep(0.1)

            rospy.loginfo("连接电脑端成功！准备调用 /start_state_machine 服务。")
            
            service_name = '/start_state_machine'
            find_item_service = roslibpy.Service(client, service_name, 'ucar2pc/FindItem')
            
            chosen_item = self.task_type # 使用从二维码获取的任务类型
            log_display_names = {'fruits': '水果', 'vegetables': '蔬菜', 'desserts': '甜品'}
            rospy.loginfo("请求仿真任务，寻找目标: %s (发送: %s)", log_display_names.get(chosen_item, '未知'), chosen_item)
            
            request = roslibpy.ServiceRequest({'item_to_find': chosen_item})
            
            rospy.loginfo("正在发送请求并等待仿真任务完成...")
            response = find_item_service.call(request)
            
            self.simulation_room_location = response.get('room_location', 'unknown_room')
            self.simulation_found_item = response.get('found_item_name', 'not_found')
            
            rospy.loginfo("收到仿真端响应: 房间=%s, 物品=%s", self.simulation_room_location, self.simulation_found_item)
            
            return True

        except Exception as e:
            rospy.logerr("调用仿真服务时发生错误: %s", e)
            return False
        finally:
            if client.is_connected:
                client.terminate()
                rospy.loginfo("与电脑端的连接已关闭。")

    def _check_nodes_shutdown_status(self, event):
        """检查/darknet_ros, /move_base, /amcl以及摄像头节点是否已关闭"""
        try:
            node_names = rosnode.get_node_names()
            
            if self.camera_node_name not in node_names and \
               '/darknet_ros' not in node_names and \
               '/move_base' not in node_names and \
               '/amcl' not in node_names:
                rospy.loginfo("确认所有目标节点已成功关闭")
                
                if self.nodes_shutdown_check_timer is not None:
                    self.nodes_shutdown_check_timer.shutdown()
                    self.nodes_shutdown_check_timer = None
                
                if self.nodes_shutdown_timeout_timer is not None:
                    self.nodes_shutdown_timeout_timer.shutdown()
                    self.nodes_shutdown_timeout_timer = None
                
                self.handle_event(Event.NODES_SHUTDOWN_COMPLETE)
            else:
                rospy.loginfo("等待节点关闭，仍在运行的节点: camera=%s, darknet=%s, move_base=%s, amcl=%s",
                              self.camera_node_name in node_names,
                              '/darknet_ros' in node_names,
                              '/move_base' in node_names,
                              '/amcl' in node_names)
        except Exception as e:
            rospy.logerr("检查节点状态时出错: %s", str(e))
            if self.nodes_shutdown_check_timer is not None:
                self.nodes_shutdown_check_timer.shutdown()
                self.nodes_shutdown_check_timer = None
            
            if self.nodes_shutdown_timeout_timer is not None:
                self.nodes_shutdown_timeout_timer.shutdown()
                self.nodes_shutdown_timeout_timer = None

    def _handle_nodes_shutdown_timeout(self, event):
        """处理节点关闭超时情况"""
        rospy.logerr("关闭一个或多个最终节点超时")
        
        if self.nodes_shutdown_check_timer is not None:
            self.nodes_shutdown_check_timer.shutdown()
            self.nodes_shutdown_check_timer = None
        
        self.handle_event(Event.NODES_SHUTDOWN_TIMEOUT)

    def _line_following_status_callback(self, msg):
        """处理巡线状态话题的回调"""
        if msg.data == 'done':
            rospy.loginfo("接收到巡线完成信号。")
            
            if self.line_following_timeout_timer:
                self.line_following_timeout_timer.shutdown()
                self.line_following_timeout_timer = None
            if self.line_following_status_sub:
                self.line_following_status_sub.unregister()
                self.line_following_status_sub = None
                
            self.handle_event(Event.LINE_FOLLOWING_DONE)

    def _line_following_timeout_callback(self, event):
        """处理巡线超时"""
        rospy.logerr("巡线任务执行超时。")
        
        if self.line_following_status_sub:
            self.line_following_status_sub.unregister()
            self.line_following_status_sub = None
            
        self.handle_event(Event.NAV_DONE_FAILURE) # 复用一个通用失败事件

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

    #*********************** 资源管理 ***********************#
    # 停止所有活动
    def stop_all_activities(self):
        # 停止检测相关活动
        self.stop_search_timers()
        if self.board_detect_process is not None:
            self.board_detect_process.terminate()
            self.board_detect_process = None
        
        # 停止导航相关活动
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
        
        # 停止红绿灯检测相关活动
        self.stop_light_detection_activities()
        
        # 停止QR节点关闭相关定时器
        if self.shutdown_check_timer:
            self.shutdown_check_timer.shutdown()
            self.shutdown_check_timer = None
        if self.shutdown_timeout_timer:
            self.shutdown_timeout_timer.shutdown()
            self.shutdown_timeout_timer = None
            
        # 停止最终节点关闭相关定时器
        if self.nodes_shutdown_check_timer:
            self.nodes_shutdown_check_timer.shutdown()
            self.nodes_shutdown_check_timer = None
        if self.nodes_shutdown_timeout_timer:
            self.nodes_shutdown_timeout_timer.shutdown()
            self.nodes_shutdown_timeout_timer = None
            
        # 停止巡线相关活动
        if self.line_following_timeout_timer:
            self.line_following_timeout_timer.shutdown()
            self.line_following_timeout_timer = None
        if self.line_following_status_sub:
            self.line_following_status_sub.unregister()
            self.line_following_status_sub = None
        
        # 确保相机进程被终止
        if self.camera_process is not None:
            self.camera_process.terminate()
            self.camera_process = None

        self.is_awake = False
        self.task_processed = False
        self.current_voice_cmd = None
        self.voice_service_called = False
        rospy.loginfo("已停止所有活动")

#*********************** 主函数 ***********************#
if __name__ == '__main__':
    try:
        robot_sm = UnifiedStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass