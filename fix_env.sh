#!/bin/bash

echo "========================================"
echo "修复 Intent-MPC 环境变量"
echo "========================================"
echo ""

# 1. 清理所有 ROS 环境变量
echo "1. 清理旧的环境变量..."
unset ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES
unset CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH

# 2. 重新加载 ROS 基础环境
echo "2. 加载 ROS Noetic..."
source /opt/ros/noetic/setup.bash

# 3. 加载 intent-mpc 工作空间
echo "3. 加载 intent-mpc 工作空间..."
cd /home/ff/intent-mpc
source devel/setup.bash

# 4. 验证环境变量
echo ""
echo "========================================"
echo "✅ 环境变量已修复！"
echo "========================================"
echo ""
echo "当前 ROS_PACKAGE_PATH:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | head -5
echo ""
echo "现在可以运行："
echo "  roslaunch uav_simulator start.launch"
echo "========================================"


