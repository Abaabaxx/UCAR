## 项目简介

"ZZU-AIR" 智能汽车竞赛——讯飞创意组


## 结构说明

`src` 目录下包含以下基础功能包：

- **ydlidar**: 激光雷达基础驱动包
- **xf_mic_asr_offline**: 麦克风基础驱动包
- **ucar_nav**: 导航基础驱动包
- **ucar_controller**: 底盘控制基础驱动包
- **lby_usb_cam**: 摄像头基础驱动包

具体启动的 `launch` 文件请查看 `/state_machine` 中比赛的 `launch` 文件。

## 功能包说明

1. **qr_code**
   - 功能：二维码识别节点，订阅摄像头数据，发布解析得到的文本信息。

2. **board_detect**
   - 功能：找板脚本对应的功能包，输出物品板正前方正对物品板的目标姿态，并保存到 `yaml` 文件夹中以便后续读取。

3. **board_detect_state_machine**
   - 功能：开发找板时新开的状态机功能包，主要用于后半平面的状态机开发，完成后合并到主状态机。

4. **darknet_ros**
   - 功能：`darknet_ros` 是 `yolov3` 对应的功能包，输入摄像头话题数据，返回检测过的图像。不过因为 `darknet_ros` 是一个 git 仓库，我没有对其追踪，源码从 `darknet_ros` 的官方仓库 clone 就行。

5. **follow_line**
   - 功能：巡线功能包，启动后接受服务调用，开始巡线，包含国赛和省赛的巡线脚本。

6. **state_machine**
   - 功能：最最最最最核心的功能包!!! 比赛的核心大脑节点，主状态机功能包，控制整个比赛流程。

## qr_code

- **功能**: 二维码识别节点，用于识别图像中的二维码并根据内容进行任务分类。
- **订阅**: `/usb_cam/image_raw` (摄像头图像)
- **发布**: 
  - `/QR/result` (二维码原始文本)
  - `/QR/task_type` (解析后的任务类型，如: `fruits`, `vegetables`)
- **启动**: `roslaunch qr_code qr_code.launch`

## board_detect

- **功能**: 自动识别场内物品板，并生成正对板子的导航目标点 (YAML文件)。
- **输入**: `/scan` (激光雷达数据)
- **输出**: `/goals_up` 或 `/goals_down` 目录下的 `goal{N}.yaml` 导航目标文件。
- **核心脚本**: `board_detect_up.py`, `board_detect_down.py`
- **说明**: 脚本通过激光点云聚类、长度及ROI筛选识别目标，融合多帧数据以确保精度。运行后自动生成文件并退出。`duiban.py`是在确定找到对应品类的货物以后用于精确靠近板子的对板脚本。


## board_detect_state_machine
该功能包为开发找板时新开的状态机功能包，主要用于后半平面的状态机开发，完成后合并到主状态机。不实现具体的功能


## darknet_ros
`darknet_ros` 是 `yolov3` 对应的功能包，输入摄像头话题数据，返回检测过的图像。不过因为 `darknet_ros` 是一个 git 仓库，我没有对其追踪，源码从 `darknet_ros` 的官方仓库 clone 就行。


## follow_line

- **功能**: 复杂赛道任务包，通过一个精密的**有限状态机 (FSM)** 完成巡线、入环岛、避障、冲刺停车等一系列动作。
- **技术栈**: 深度融合**视觉** (逆透视变换、沿墙算法)、**激光雷达** (检测赛道边界板、障碍物) 和**里程计** (精确定位)。
- **版本**: 内含 `guosai` (国赛) 和 `shengsai` (省赛) 两种模式的脚本。
- **控制方式**: 通过 `rosservice call /follow_line/run "data: true"` 服务启动，并通过 `/line_following_status` 话题反馈任务完成状态。
- **核心脚本**: `guosai/guosai_left.py`, `shengsai/shengsai_left.py` 等。


## state_machine

- **功能**: **项目核心大脑**。一个包含31个主状态的事件驱动**有限状态机 (FSM)**，负责调度所有功能包，控制从语音唤醒、二维码任务识别、自主标定、多点巡检、分布式仿真、红绿灯决策到最后巡线冲刺的完整比赛流程。
- **技术亮点**:
    - **动态资源管理**: 在任务不同阶段自动启动和关闭节点 (`qr_detect`, `darknet_ros`, `move_base` 等) 以优化性能。
    - **智能重试机制**: 包含多层巡检重试逻辑，并根据重试次数**动态调整YOLO检测的置信度阈值**。
    - **健壮性设计**: 包含导航失败后的**自主恢复机制**（全局重定位+原地旋转）和全面的超时处理。
    - **分布式任务**: 通过 `rosbridge` 与外部PC协作完成仿真任务。
- **核心脚本**: `all_guosai.py` (国赛), `all_shengsai.py` (省赛)。
- **启动**: `roslaunch state_machine all_guosai.launch` 会启动所有依赖及状态机本身。
