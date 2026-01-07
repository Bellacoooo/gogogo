#!/bin/bash

echo "=========================================="
echo "深度清理并重新构建"
echo "=========================================="
echo ""

# 1. 停止所有进程
echo "步骤 1/7: 停止所有进程..."
killall -9 gzserver gzclient rosmaster roslaunch roscore gazebo 2>/dev/null
sleep 3

# 2. 清理所有临时文件和缓存
echo "步骤 2/7: 清理临时文件和缓存..."
rm -rf /tmp/.gazebo* 2>/dev/null
rm -rf /tmp/.ros* 2>/dev/null
rm -rf ~/.gazebo/log/* 2>/dev/null
rm -rf ~/.ros/log/* 2>/dev/null
rm -f ~/.ros/rospack_cache_* 2>/dev/null

# 3. 清理 Gazebo 模型缓存
echo "步骤 3/7: 清理 Gazebo 模型缓存..."
# 不删除整个 models 目录，只清理可能损坏的缓存
find ~/.gazebo/models -name "*.tar.gz.*" -delete 2>/dev/null
find ~/.gazebo -name "*.log" -delete 2>/dev/null

# 4. 清理并重新构建工作空间
echo "步骤 4/7: 重新构建工作空间..."
cd /home/ff/intent-mpc

# 清理旧的构建文件
catkin_make clean
rm -rf build/Intent-MPC/flight_data_recorder 2>/dev/null
rm -rf devel/lib/flight_data_recorder 2>/dev/null
rm -rf devel/share/flight_data_recorder 2>/dev/null

# 重新构建
echo "开始构建..."
catkin_make 2>&1 | tail -20

# 5. 重新加载环境
echo "步骤 5/7: 加载环境..."
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 6. 验证环境
echo "步骤 6/7: 验证环境..."
echo ""
echo "检查关键包:"
for pkg in uav_simulator autonomous_flight onboard_detector; do
    if rospack find $pkg > /dev/null 2>&1; then
        echo "  ✅ $pkg"
    else
        echo "  ❌ $pkg 找不到！"
        exit 1
    fi
done

echo ""
echo "ROS_PACKAGE_PATH 前3项:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | head -3

echo ""
echo "GAZEBO_MODEL_PATH:"
echo "$GAZEBO_MODEL_PATH" | tr ':' '\n'

# 7. 启动测试
echo ""
echo "步骤 7/7: 启动 Gazebo..."
echo "=========================================="
echo ""

roslaunch uav_simulator start.launch

