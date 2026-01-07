#!/bin/bash

echo "============================================"
echo "测试原始 GitHub 版本"
echo "============================================"
echo ""

# 清理环境
echo "步骤 1/5: 停止所有进程..."
killall -9 gzserver gzclient rosmaster roslaunch roscore 2>/dev/null
sleep 2

# 清理临时文件
echo "步骤 2/5: 清理临时文件..."
rm -rf /tmp/.gazebo* /tmp/.ros* 2>/dev/null
rm -rf ~/.ros/log/* 2>/dev/null

# 清理并重置环境变量
echo "步骤 3/5: 重置环境变量..."
unset ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES
unset CMAKE_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH

source /opt/ros/noetic/setup.bash
cd /home/ff/intent-mpc
source devel/setup.bash

# 验证环境
echo "步骤 4/5: 验证环境..."
echo "ROS_PACKAGE_PATH 前3项:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | head -3

if echo "$ROS_PACKAGE_PATH" | grep -q "intent-mpc"; then
    echo "✅ 环境变量正确"
else
    echo "❌ 环境变量错误！"
    echo "完整路径: $ROS_PACKAGE_PATH"
    exit 1
fi

# 验证包能被找到
echo ""
echo "验证关键包..."
if rospack find uav_simulator > /dev/null 2>&1; then
    echo "✅ uav_simulator 包找到了"
else
    echo "❌ uav_simulator 包未找到！"
    exit 1
fi

echo ""
echo "步骤 5/5: 启动原始版本 Gazebo..."
echo "============================================"
echo ""

# 启动
roslaunch uav_simulator start.launch

