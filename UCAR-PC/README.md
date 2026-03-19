# 项目简介
"ZZU-AIR" 智能汽车竞赛——讯飞创意组——PC端代码

## 结构说明
`src` 目录下包含以下基础功能包：

- `gazebo_map`: 仿真端建图以及rviz可视化相关功能包
- `gazebo_nav`: 仿真端导航功能包
- `gazebo_pkg`: 仿真基础环境功能包,拉起gazebo,提供基础的仿真环境
- `usb_cam`: 摄像头功能包

> 具体启动的 launch 文件请查看 `/state_machine` 中比赛的仿真端 `simulation_all.launch` 文件。

## 功能包说明

### ucar_rviz
功能: 因为小车端没法可视化,不想用有线hdmi图传或者无线hdmi中转或者X-server图传，我们采用的是主从机配置，电脑端为从机，接受ucar端的话题数据来可视化，所以该功能宝存放到为rviz的配置文件以及指定配置文件启动的laucnh文件

### darknet_ros
功能：`darknet_ros` 是 yolov3 对应的功能包，输入摄像头话题数据，返回检测过的图像。不过因为 `darknet_ros` 是一个 git 仓库，我没有对其追踪，源码从 `darknet_ros` 的官方仓库 clone 就行。

### ucar2pc
功能：实现ucar与pc之间的rosbrige通信，用于传递任务类型以及返回对应物品所在房间。实际上正统的方案应该是主从机，但是因为官方给的配置文件中没有命名空间，所以ucar与pc端有话题服务名冲突的问题，比如均为`move_base`,都为`darknet_ros`,所以崛起了原始ros的标准答案主从机,选择转向了rosbrige的通信方案。该功能包中，`srv`下为自定义的服务通信。其中的`start_simulation.py`为通信demo。

### state_machine
功能：仿真端比赛核心状态机功能包，负责仿真任务的完成。核心的逻辑实现在该功能包的节点py脚本中实现。

## 分支说明
本项目主要包含以下分支：

- **main**: 主分支。整理后的最终版本，是项目的默认分支。
- **final**: 比赛分支。比赛现场使用的最终代码版本。
- **tidy**: 代码整理分支。基于 final 分支创建，对比赛代码进行了重构和清理，可读性更高。
