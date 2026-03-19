#!/bin/bash


echo "正在用rosbag录制视频..."
rosbag record /usb_cam/image_raw -O xunxian.bag
