#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 命令行启动：rosservice call /go_board_detect "{}"

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

# 状态常量定义
class RobotState(object):
    IDLE = 0
    NAVIGATE_TO_UP_POINT = 1
    SEARCH_UP_BOARD = 2
    NAVIGATE_TO_DOWN_POINT = 3
    SEARCH_DOWN_BOARD = 4
    PICK_UP_GOODS = 5
    SPEAK_GOODS = 6  # 新增状态
    ERROR = 99

# 事件常量定义
class Event(object):
    START_CMD = 0
    NAV_DONE_SUCCESS = 1
    NAV_DONE_FAILURE = 2
    SEARCH_DONE_SUCCESS = 3
    SEARCH_DONE_FAILURE = 4
    PICK_UP_DOWN = 5  # 新增事件

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
        self.board_detect_script_path = "/home/ucar/lby_ws/src/board_detect/scripts/board_detect_up.py"
        self.board_detect_down_script_path = "/home/ucar/lby_ws/src/board_detect/scripts/board_detect_down.py"  # 新增下平面检测脚本路径
        self.search_timeout_duration = 25.0
        self.board_detect_process = None
        self.search_monitor_timer = None
        self.search_timeout_timer = None
        
        # 新增目标点目录路径
        self.goals_up_dir = "/home/ucar/lby_ws/src/board_detect/goals_up"
        self.goals_down_dir = "/home/ucar/lby_ws/src/board_detect/goals_down"

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
    
    # 新增：执行巡检序列
    def execute_patrol_sequence(self):
        """执行多点巡检导航逻辑"""
        rospy.loginfo("开始执行多点巡检导航...")
        
        # 加载巡检点
        waypoints = self.load_patrol_goals()
        
        # 检查是否成功加载了目标点
        if not waypoints:
            rospy.logerr("没有找到有效的目标点，巡检终止。")
            self.handle_event(Event.PICK_UP_DOWN)
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
                # --- 开始模拟识别与决策 ---
                rospy.loginfo("正在模拟识别过程...")
                rospy.sleep(1.0)  # 暂停1秒
                rospy.loginfo("正在模拟决策过程...")
                rospy.sleep(0.5)  # 暂停0.5秒
                rospy.loginfo("--- 巡检点{} 处理完毕，准备前往下一个点。---".format(i + 1))
            else:
                rospy.logwarn("巡检点{}导航失败: {}，跳过此点。".format(i + 1, status_str))
        
        rospy.loginfo("所有巡检点已遍历完毕。")
        # 触发巡检完成事件
        self.handle_event(Event.PICK_UP_DOWN)

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
            'up_point': self.create_pose(1.25, 3.50, 0.7071, 0.7071),
            'down_point': self.create_pose(1.25, 4.00, -0.7071, 0.7071)
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
            rospy.loginfo("已进入[语音播报货物]状态，等待后续功能实现。")
            # 当前状态机流程在此结束，等待开发后续逻辑或手动重启。
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
            if event == Event.PICK_UP_DOWN:
                self.transition(RobotState.SPEAK_GOODS)

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
