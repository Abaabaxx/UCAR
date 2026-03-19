# 详情见/home/ucar/lby_ws/src/ucar_controller/README.md

# 使用话题发布速度指令
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

  # 使用话题发布停止指令
  rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

  # 使用服务命令停止
  rosservice call /stop_move


