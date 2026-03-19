#!/bin/bash

# 为每一个 roslaunch 命令打开一个新的终端窗口

# -- 选项后面的内容是新终端要执行的命令
# bash -c "..." 的作用是执行一个命令字符串
# "..." 中的 roslaunch ...; exec bash 的作用是：先执行roslaunch，当roslaunch结束后(比如你按Ctrl+C)，
# 终端不会马上关闭，而是会给你一个可交互的bash shell，方便你查看错误信息。

echo "在终端中启动仿真部分的launch文件..."

roslaunch /home/lby/ucar_ws/src/state_machine/launch/simulation_all.launch
