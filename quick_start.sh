#!/bin/bash

# 快速启动脚本 - 自动设置环境并启动 Gazebo

echo "=== 设置 ROS 环境 ==="
cd /home/ff/intent-mpc
source devel/setup.bash

echo ""
echo "=== 检查包是否可用 ==="
if rospack find uav_simulator > /dev/null 2>&1; then
    echo "✅ uav_simulator 包找到"
else
    echo "❌ uav_simulator 包未找到"
    echo "请先运行: cd /home/ff/intent-mpc && catkin_make"
    exit 1
fi

echo ""
echo "=== 启动 Gazebo ==="
roslaunch uav_simulator start.launch
