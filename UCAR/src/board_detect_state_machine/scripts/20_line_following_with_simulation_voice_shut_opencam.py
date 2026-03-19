#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
命令行启动：
rosservice call /go_board_detect "{}"
"""


import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import subprocess
import os
import yaml  # 新增导入
import glob  # 新增导入
import threading
import roslibpy  # 新增导入
import time      # 新增导入
from darknet_ros_msgs.msg import BoundingBoxes  # 新增导入YOLO检测结果消息类型
from xf_mic_asr_offline.srv import VoiceCmd, VoiceCmdRequest
import rosnode  # 添加rosnode模块导入

# 状态常量定义
class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_UP_POINT = 1
    SEARCH_UP_BOARD = 2
    NAVIGATE_TO_DOWN_POINT = 3
    SEARCH_DOWN_BOARD = 4
    PICK_UP_GOODS = 5
    SPEAK_GOODS = 6  # 新增状态
    NAV_TO_SIMULATION = 7  # 新增状态
    DO_SIMULATION_TASKS = 8  # 新增状态
    SPEAK_ROOM = 9  # 修改：原SPEAK_SIMULATION_GOODS
    NAV_TO_TRAFFIC = 10  # 新增状态
    NAV_TO_LANE1_OBSERVE_POINT = 11
    DETECTING_LANE1_LIGHT = 12
    SPEAK_LANE1_CLEAR = 13
    NAV_TO_LANE1_WAITING_POINT = 14
    NAV_TO_LANE2_OBSERVE_POINT = 15
    DETECTING_LANE2_LIGHT = 16
    SPEAK_LANE2_CLEAR = 17
    NAV_TO_LANE2_WAITING_POINT = 18
    SHUTDOWN_FINAL_NODES = 19
    RELAUNCH_CAMERA_FOR_LINE_FOLLOWING = 20 # 新增：为巡线重启摄像头状态
    LINE_FOLLOWING = 21
    ERROR = 99

# 事件常量定义
class Event(object):
    START_CMD = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2
    SEARCH_DONE_SUCCESS = 3
    SEARCH_DONE_FAILURE = 4
    GOODS_FOUND = 5                         # 新增事件：成功找到目标货物
    PATROL_SEQUENCE_COMPLETED = 6           # 新增事件：巡检完成但未找到目标
    SPEAK_DONE = 7                          # 新增：语音播放完成事件
    SPEAK_TIMEOUT = 8                       # 新增：语音播放超时事件
    SIMULATION_DONE = 9                     # 新增：仿真任务完成事件
    LIGHT_DETECTED_GREEN = 10               # 新增：检测到绿灯事件
    LIGHT_DETECTED_RED = 11                 # 新增：检测到红灯事件
    LIGHT_DETECT_TIMEOUT = 12               # 新增：红绿灯检测超时事件
    NODES_SHUTDOWN_COMPLETE = 13             # 新增：YOLO节点成功关闭事件
    NODES_SHUTDOWN_TIMEOUT = 14              # 新增：YOLO节点关闭超时事件

# 主类框架
class RobotStateMachine(object):
    """
    机器人状态机类，用于控制机器人在不同状态间的转换和执行相应的动作。
    
    如何调整机器人行为:
    1. 修改self.current_task可以改变机器人当前的任务类型（'fruits', 'desserts', 'vegetables'）
    2. 修改self.yolo_confidence_threshold可以调整YOLO检测的最低置信度阈值
    3. self.goods_categories字典定义了每种任务下的目标货物名称列表
    4. 修改self.CENTER_ZONE_MARGIN定义中心安全区的范围
    5. 修改self.EDGE_ZONE_MIN_WIDTH调整对边缘物体的宽度要求
    6. 修改self.MIN_AREA_RATIO和self.MAX_AREA_RATIO调整有效物体的大小范围
    
    红绿灯处理流程（状态11-19）:
    - 机器人首先导航到第一车道观察点，检测红绿灯
    - 如果检测到绿灯，播报通行信息并导航到第一车道等待点，然后进入最终等待状态
    - 如果检测到红灯，导航到第二车道观察点继续检测
    - 在第二车道，如果检测到绿灯，播报通行信息并导航到第二车道等待点，然后进入最终等待状态
    - 如果第二车道也是红灯或检测超时，则进入错误状态
    """
    
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
        self.board_detect_script_path = "/home/ucar/lby_ws/src/board_detect/scripts/board_detect_up.py"
        self.board_detect_down_script_path = "/home/ucar/lby_ws/src/board_detect/scripts/board_detect_down.py"  # 新增下平面检测脚本路径
        self.search_timeout_duration = 25.0
        self.board_detect_process = None
        self.search_monitor_timer = None
        self.search_timeout_timer = None
        self.camera_process = None
        
        # 新增：为巡线任务重启摄像头相关变量
        # 请根据 `rosnode list` 确认摄像头的确切节点名称
        self.camera_node_name = "/usb_cam"
        # 用于巡线的高帧率摄像头launch命令
        self.camera_line_following_launch_command = ['roslaunch', 'lby_usb_cam', 'usb_cam_noimg.launch']
        
        # 新增目标点目录路径
        self.goals_up_dir = "/home/ucar/lby_ws/src/board_detect/goals_up"
        self.goals_down_dir = "/home/ucar/lby_ws/src/board_detect/goals_down"
        
        # 新增货物分类、任务管理和置信度控制
        self.goods_categories = {
            'fruits': ['watermelon', 'banana', 'apple'],
            'desserts': ['milk', 'cake', 'cola'],
            'vegetables': ['pepper', 'potato', 'tomato']
        }
        self.current_task = 'fruits'  # 默认任务
        self.yolo_confidence_threshold = 0.5 # YOLO识别的置信度阈值
        self.found_good_name = None   # 用于存储找到的货物名称
        
        # 图像和过滤器参数
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        self.CENTER_ZONE_MARGIN = 30  # 中心安全区的边界 (从图像边缘向内)
        self.EDGE_ZONE_MIN_WIDTH = 200 # 边缘区域物体的最小宽度要求
        self.MIN_AREA_RATIO = 0.05     # 最小面积占图像百分比
        self.MAX_AREA_RATIO = 0.80     # 最大面积占图像百分比
        
        # 新增语音播报相关变量
        self.speak_timer = None
        self.speak_timeout = 10.0  # 语音播放超时时间(秒)
        self.current_voice_cmd = None
        self.voice_service_called = False
        
        # 新增红绿灯检测相关变量
        self.light_detection_timeout = 3.0  # 红绿灯检测超时时间(秒)
        self.light_detection_timer = None
        self.light_detection_subscriber = None
        
        # 新增YOLO节点关闭控制相关变量
        self.nodes_shutdown_check_timer = None
        self.nodes_shutdown_timeout_timer = None
        self.nodes_shutdown_timeout_duration = 5.0  # 关闭节点的总超时时间(秒)

        # 新增：仿真任务相关变量
        self.PC_IP = '192.168.68.206'  # 请确保这是您电脑的正确IP地址
        self.simulation_found_item = None
        self.simulation_room_location = None

    # 新增：调用电脑端仿真服务
    def _call_simulation_service(self):
        """
        连接到电脑端的rosbridge并调用仿真服务，阻塞等待结果。
        """
        rospy.loginfo("准备连接电脑端 rosbridge at %s:9090", self.PC_IP)
        client = roslibpy.Ros(host=self.PC_IP, port=9090)
        
        try:
            # 启动客户端连接线程
            client.run()
            
            # 使用带超时的循环等待连接建立
            connect_timeout = 5.0  # 5秒连接超时
            start_time = time.time()
            while not client.is_connected:
                if time.time() - start_time > connect_timeout:
                    rospy.logerr("连接电脑端 rosbridge 超时。")
                    client.terminate()
                    return False
                time.sleep(0.1)

            rospy.loginfo("连接电脑端成功！准备调用 /start_state_machine 服务。")
            
            # 定义服务
            service_name = '/start_state_machine'
            find_item_service = roslibpy.Service(client, service_name, 'ucar2pc/FindItem')
            
            # 准备请求
            chosen_item = self.current_task
            log_display_names = {'fruits': '水果', 'vegetables': '蔬菜', 'desserts': '甜品'}
            rospy.loginfo("请求仿真任务，寻找目标: %s (发送: %s)", log_display_names.get(chosen_item, '未知'), chosen_item)
            
            request = roslibpy.ServiceRequest({'item_to_find': chosen_item})
            
            # 调用服务并等待结果
            rospy.loginfo("正在发送请求并等待仿真任务完成...")
            response = find_item_service.call(request)
            
            # 【关键步骤：记录结果】
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


    # 新增：检查目标货物方法
   # 修正后的 check_for_target_goods 方法
    def check_for_target_goods(self, timeout=3.0):
        """
        在指定的超时时间内监听/darknet_ros/bounding_boxes话题，
        寻找符合当前任务的货物，采用"先到先得"策略。
        
        Args:
            timeout (float): 监听的最长时间（秒）
        
        Returns:
            str or None: 找到的货物名称，如果未找到则返回None
        """
        rospy.loginfo("开始在当前位置检测货物，任务类型: %s，监听时间: %.1f秒", 
                    self.current_task, timeout)
        
        # 获取当前任务对应的目标货物列表
        target_goods = self.goods_categories.get(self.current_task, [])
        if not target_goods:
            rospy.logwarn("当前任务'%s'没有对应的目标货物列表", self.current_task)
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

    # 新增：加载巡检目标点
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
    
    # 重构：执行巡检序列
    def execute_patrol_sequence(self):
        """执行多点巡检导航逻辑，寻找目标货物"""
        rospy.loginfo("开始执行多点巡检导航，寻找 %s 类别的货物...", self.current_task)
        
        # 获取当前任务需要检测的货物列表
        target_goods = self.goods_categories.get(self.current_task, [])
        if not target_goods:
            rospy.logwarn("当前任务'%s'没有对应的目标货物，巡检将无法找到目标", self.current_task)
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

    # ROS接口初始化
    def init_ros_comm(self):
        self.state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)
        self.start_service = rospy.Service('/go_board_detect', Trigger, self.start_callback)
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待导航服务器...")
        self.move_base_client.wait_for_server()
        
        # 新增：等待语音播报服务
        rospy.loginfo("等待语音播报服务 /robot/voice_cmd ...")
        try:
            rospy.wait_for_service('/robot/voice_cmd', timeout=5.0)
            rospy.loginfo("语音播报服务连接成功")
        except rospy.ROSException:
            rospy.logwarn("语音播报服务在启动时不可用")

    # 初始化位置
    def init_locations(self):
        self.locations = {
            'up_point': self.create_pose(1.25, 3.0, 0.7071, 0.7071),
            'down_point': self.create_pose(1.25, 4.10, -0.7071, 0.7071),
            'simulation_area': self.create_pose(1.25, 3.75, 0.7071, 0.7071), # 更新：使用指定的仿真区坐标
            'lane1_observe_point': self.create_pose(3.25, 4.25, 0.7071, 0.7071),
            'lane2_observe_point': self.create_pose(4.25, 4.25, 0.7071, 0.7071),
            'lane1_waiting_point': self.create_pose(2.75, 3.75, -0.7071, 0.7071),
            'lane2_waiting_point': self.create_pose(4.75, 3.75, -0.7071, 0.7071)
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
                
                # >>> 关键修复：在这里将进程句柄清空 <<<
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
    
    #*********************** 语音播报相关功能 ***********************#
    
    def call_voice_service(self):
        if self.voice_service_called:
            return

        try:
            rospy.wait_for_service('/robot/voice_cmd', timeout=2.0)
        except rospy.ROSException:
            rospy.logerr("语音服务 '/robot/voice_cmd' 不可用")
            self.handle_event(Event.SPEAK_TIMEOUT)
            return

        try:
            voice_client = rospy.ServiceProxy('/robot/voice_cmd', VoiceCmd)
            request = VoiceCmdRequest(voice_cmd=self.current_voice_cmd)
            
            self.voice_service_called = True
            self.start_speak_timer()

            rospy.loginfo("开始调用语音服务，请求播报: %s", self.current_voice_cmd)
            response = voice_client(request)
            
            self.stop_speak_timer()
            
            if response.success:
                rospy.loginfo("语音服务成功返回: %s", response.message)
                # 使用Timer确保状态转换在当前回调之外执行
                rospy.Timer(rospy.Duration(0.1), lambda e: self.handle_event(Event.SPEAK_DONE), oneshot=True)
            else:
                rospy.logwarn("语音服务返回失败: %s", response.message)
                self.handle_event(Event.SPEAK_TIMEOUT)
                
        except rospy.ServiceException as e:
            rospy.logerr("调用语音服务失败: %s", str(e))
            self.stop_speak_timer()
            self.handle_event(Event.SPEAK_TIMEOUT)

    def start_speak_timer(self):
        if self.speak_timer:
            self.speak_timer.shutdown()
        self.speak_timer = rospy.Timer(rospy.Duration(self.speak_timeout), 
                                     self.handle_speak_timeout, 
                                     oneshot=True)

    def stop_speak_timer(self):
        if self.speak_timer:
            self.speak_timer.shutdown()
            self.speak_timer = None

    def handle_speak_timeout(self, event=None):
        rospy.logwarn("语音播放超时")
        if self.current_state == RobotState.SPEAK_GOODS:
            self.handle_event(Event.SPEAK_TIMEOUT)
    
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
    
    # 新增：检查最终节点关闭状态
    def _check_nodes_shutdown_status(self, event):
        """检查/darknet_ros, /move_base, /amcl以及摄像头节点是否已关闭"""
        try:
            # 获取当前所有节点名称
            node_names = rosnode.get_node_names()
            
            # 检查所有目标节点是否已不存在
            if self.camera_node_name not in node_names and \
               '/darknet_ros' not in node_names and \
               '/move_base' not in node_names and \
               '/amcl' not in node_names:
                rospy.loginfo("确认所有目标节点已成功关闭")
                
                # 停止相关定时器
                if self.nodes_shutdown_check_timer is not None:
                    self.nodes_shutdown_check_timer.shutdown()
                    self.nodes_shutdown_check_timer = None
                
                if self.nodes_shutdown_timeout_timer is not None:
                    self.nodes_shutdown_timeout_timer.shutdown()
                    self.nodes_shutdown_timeout_timer = None
                
                # 触发节点关闭完成事件
                self.handle_event(Event.NODES_SHUTDOWN_COMPLETE)
            else:
                rospy.loginfo("等待节点关闭，仍在运行的节点: camera=%s, darknet=%s, move_base=%s, amcl=%s",
                              self.camera_node_name in node_names,
                              '/darknet_ros' in node_names,
                              '/move_base' in node_names,
                              '/amcl' in node_names)
                # 不做任何操作，等待下一次检查
        except Exception as e:
            rospy.logerr("检查节点状态时出错: %s", str(e))
            # 停止定时器，避免持续出错
            if self.nodes_shutdown_check_timer is not None:
                self.nodes_shutdown_check_timer.shutdown()
                self.nodes_shutdown_check_timer = None
            
            if self.nodes_shutdown_timeout_timer is not None:
                self.nodes_shutdown_timeout_timer.shutdown()
                self.nodes_shutdown_timeout_timer = None

    # 新增：处理最终节点关闭超时
    def _handle_nodes_shutdown_timeout(self, event):
        """处理节点关闭超时情况"""
        rospy.logerr("关闭一个或多个最终节点超时")
        
        # 停止检查定时器
        if self.nodes_shutdown_check_timer is not None:
            self.nodes_shutdown_check_timer.shutdown()
            self.nodes_shutdown_check_timer = None
        
        # 触发节点关闭超时事件
        self.handle_event(Event.NODES_SHUTDOWN_TIMEOUT)
    
    # 停止所有检测相关定时器
    def stop_search_timers(self):
        if self.search_monitor_timer is not None:
            self.search_monitor_timer.shutdown()
            self.search_monitor_timer = None
        
        if self.search_timeout_timer is not None:
            self.search_timeout_timer.shutdown()
            self.search_timeout_timer = None

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
        
        # 停止语音相关活动
        self.stop_speak_timer()
        self.voice_service_called = False
        self.current_voice_cmd = None
        
        # 停止红绿灯检测相关活动
        self.stop_light_detection_activities()
        
        # 停止YOLO节点关闭相关定时器
        if self.nodes_shutdown_check_timer is not None:
            self.nodes_shutdown_check_timer.shutdown()
            self.nodes_shutdown_check_timer = None
            
        if self.nodes_shutdown_timeout_timer is not None:
            self.nodes_shutdown_timeout_timer.shutdown()
            self.nodes_shutdown_timeout_timer = None
        
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
            self.start_board_detection()
        elif self.current_state == RobotState.NAVIGATE_TO_DOWN_POINT:
            rospy.loginfo("开始导航至下平面位置...")
            self.send_nav_goal('down_point')
            self.navigation_active = True
        elif self.current_state == RobotState.SEARCH_DOWN_BOARD:
            self.start_down_board_detection()
        elif self.current_state == RobotState.PICK_UP_GOODS:
            rospy.loginfo("开始执行多点巡检...")
            self.execute_patrol_sequence()
        elif self.current_state == RobotState.SPEAK_GOODS:
            if self.found_good_name:
                self.current_voice_cmd = "get_" + self.found_good_name
                rospy.loginfo("状态[SPEAK_GOODS]: 准备播报找到的货物: %s", self.current_voice_cmd)
                self.voice_service_called = False
                self.call_voice_service()
            else:
                rospy.logerr("进入SPEAK_GOODS状态，但未指定货物名称")
                self.handle_event(Event.SPEAK_TIMEOUT)
        elif self.current_state == RobotState.NAV_TO_SIMULATION:
            rospy.loginfo("开始导航至最终目标点：仿真区...")
            self.send_nav_goal('simulation_area')
            self.navigation_active = True
        elif self.current_state == RobotState.DO_SIMULATION_TASKS:
            rospy.loginfo("开始执行仿真任务，调用电脑端服务...")
            # 调用仿真服务并根据结果触发事件
            success = self._call_simulation_service()
            if success:
                self.handle_event(Event.SIMULATION_DONE)
            else:
                rospy.logerr("仿真任务执行失败。")
                self.handle_event(Event.NAV_DONE_FAILURE) # 复用导航失败事件
        elif self.current_state == RobotState.SPEAK_ROOM:
            rospy.loginfo("状态[SPEAK_ROOM]: 准备播报仿真任务找到的房间...")
            if self.simulation_room_location and self.simulation_room_location != 'unknown_room':
                # 如果成功找到了房间，就播报房间名
                self.current_voice_cmd = self.simulation_room_location
                self.voice_service_called = False  # 重置标记
                self.call_voice_service()
            else:
                # 如果没有找到房间，则直接跳过播报
                rospy.logwarn("未记录有效的仿真房间位置，跳过播报。")
                # 使用Timer确保状态转换在当前回调之外执行
                rospy.Timer(rospy.Duration(0.1), lambda e: self.handle_event(Event.SPEAK_DONE), oneshot=True)
        elif self.current_state == RobotState.NAV_TO_TRAFFIC:
            rospy.loginfo("进入红绿灯区域状态，准备导航至第一车道观察点...")
            rospy.Timer(rospy.Duration(0.5), self._traffic_timer_callback, oneshot=True)
            
        elif self.current_state == RobotState.NAV_TO_LANE1_OBSERVE_POINT:
            rospy.loginfo("开始导航至第一车道观察点...")
            self.send_nav_goal('lane1_observe_point')
            self.navigation_active = True

        elif self.current_state == RobotState.DETECTING_LANE1_LIGHT:
            rospy.loginfo("开始检测第一车道红绿灯...")
            self.start_light_detection()
        elif self.current_state == RobotState.SPEAK_LANE1_CLEAR:
            rospy.loginfo("第一车道绿灯，准备播报通行信息...")
            self.current_voice_cmd = "way_1"
            self.voice_service_called = False
            self.call_voice_service()
        elif self.current_state == RobotState.NAV_TO_LANE1_WAITING_POINT:
            rospy.loginfo("开始导航至第一车道等待点...")
            self.send_nav_goal('lane1_waiting_point')
            self.navigation_active = True
        elif self.current_state == RobotState.NAV_TO_LANE2_OBSERVE_POINT:
            rospy.loginfo("开始导航至第二车道观察点...")
            self.send_nav_goal('lane2_observe_point')
            self.navigation_active = True
        elif self.current_state == RobotState.DETECTING_LANE2_LIGHT:
            rospy.loginfo("开始检测第二车道红绿灯...")
            self.start_light_detection()
        elif self.current_state == RobotState.SPEAK_LANE2_CLEAR:
            rospy.loginfo("第二车道绿灯，准备播报通行信息...")
            self.current_voice_cmd = "way_2"
            self.voice_service_called = False
            self.call_voice_service()
        elif self.current_state == RobotState.NAV_TO_LANE2_WAITING_POINT:
            rospy.loginfo("开始导航至第二车道等待点...")
            self.send_nav_goal('lane2_waiting_point')
            self.navigation_active = True
        elif self.current_state == RobotState.SHUTDOWN_FINAL_NODES:
            rospy.loginfo("正在尝试关闭最终节点: %s, /darknet_ros, /move_base, /amcl...", self.camera_node_name)
            # 首先关闭摄像头
            subprocess.call(['rosnode', 'kill', self.camera_node_name])
            # 然后关闭其他导航和检测相关的节点
            subprocess.call(['rosnode', 'kill', '/darknet_ros'])
            subprocess.call(['rosnode', 'kill', '/move_base'])
            subprocess.call(['rosnode', 'kill', '/amcl'])
            
            # 启动周期性检查定时器
            self.nodes_shutdown_check_timer = rospy.Timer(
                rospy.Duration(0.25),  # 每0.25秒检查一次
                self._check_nodes_shutdown_status
            )
            
            # 启动超时定时器
            self.nodes_shutdown_timeout_timer = rospy.Timer(
                rospy.Duration(self.nodes_shutdown_timeout_duration),
                self._handle_nodes_shutdown_timeout,
                oneshot=True
            )
        elif self.current_state == RobotState.RELAUNCH_CAMERA_FOR_LINE_FOLLOWING:
            rospy.loginfo("所有节点已关闭。等待2秒后为巡线任务重启摄像头...")
            rospy.sleep(2.0)
            try:
                rospy.loginfo("正在执行命令: %s", ' '.join(self.camera_line_following_launch_command))
                self.camera_process = subprocess.Popen(self.camera_line_following_launch_command)
                rospy.loginfo("摄像头启动命令已发送，进程ID: %s", self.camera_process.pid)
                # 启动后稍作等待，确保节点有时间初始化
                rospy.sleep(3.0)
                self.transition(RobotState.LINE_FOLLOWING)
            except Exception as e:
                rospy.logerr("启动摄像头launch文件失败: %s", str(e))
                self.transition(RobotState.ERROR)
        elif self.current_state == RobotState.LINE_FOLLOWING:
            rospy.loginfo("进入最终巡线状态，任务完成。")
            pass
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
        elif self.current_state == RobotState.DO_SIMULATION_TASKS:
            if event == Event.SIMULATION_DONE:
                self.transition(RobotState.SPEAK_ROOM)
            elif event == Event.NAV_DONE_FAILURE: # 复用导航失败事件
                self.transition(RobotState.ERROR)
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
                self.transition(RobotState.NAV_TO_LANE1_WAITING_POINT)
            elif event == Event.SPEAK_TIMEOUT:
                self.transition(RobotState.ERROR)
        elif self.current_state == RobotState.NAV_TO_LANE1_WAITING_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.SHUTDOWN_FINAL_NODES)
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
                self.transition(RobotState.NAV_TO_LANE2_WAITING_POINT)
            elif event == Event.SPEAK_TIMEOUT:
                self.transition(RobotState.ERROR)
        elif self.current_state == RobotState.NAV_TO_LANE2_WAITING_POINT:
            if event == Event.NAV_DONE_SUCCESS:
                self.transition(RobotState.SHUTDOWN_FINAL_NODES)
            elif event == Event.NAV_DONE_FAILURE:
                self.transition(RobotState.ERROR)
        elif self.current_state == RobotState.SHUTDOWN_FINAL_NODES:
            if event == Event.NODES_SHUTDOWN_COMPLETE:
                self.transition(RobotState.RELAUNCH_CAMERA_FOR_LINE_FOLLOWING)
            elif event == Event.NODES_SHUTDOWN_TIMEOUT:
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