#!/bin/bash

echo "=========================================="
echo "Intent-MPC 完全清理重启脚本"
echo "=========================================="
echo ""

# 1. 杀死所有相关进程
echo "步骤 1/6: 停止所有 ROS 和 Gazebo 进程..."
killall -9 gzserver gzclient rosmaster roslaunch roscore 2>/dev/null
sleep 2

# 2. 清理临时文件
echo "步骤 2/6: 清理临时文件..."
rm -rf /tmp/.gazebo* /tmp/.ros* 2>/dev/null
rm -rf ~/.ros/log/* 2>/dev/null

# 3. 清理环境变量
echo "步骤 3/6: 清理环境变量..."
unset ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES
unset CMAKE_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH

# 4. 重新设置 ROS 环境
echo "步骤 4/6: 加载 ROS Noetic..."
source /opt/ros/noetic/setup.bash

# 5. 加载 intent-mpc 工作空间
echo "步骤 5/6: 加载 intent-mpc 工作空间..."
cd /home/ff/intent-mpc
source devel/setup.bash

# 6. 验证环境
echo "步骤 6/6: 验证环境变量..."
echo ""
echo "ROS_PACKAGE_PATH 的前3个路径:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | head -3
echo ""

# 检查是否正确
if echo "$ROS_PACKAGE_PATH" | grep -q "intent-mpc"; then
    echo "✅ 环境变量正确！"
else
    echo "❌ 错误：环境变量仍然不正确！"
    echo "完整的 ROS_PACKAGE_PATH:"
    echo "$ROS_PACKAGE_PATH"
    exit 1
fi

echo ""
echo "=========================================="
echo "✅ 环境准备完成！"
echo ""
echo "现在启动 Gazebo 仿真..."
echo "=========================================="
echo ""

# 启动仿真
roslaunch uav_simulator start.launch


