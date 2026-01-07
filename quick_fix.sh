#!/bin/bash

echo "======================================"
echo "快速修复：清理 Gazebo 缓存"
echo "======================================"
echo ""

# 停止所有进程
echo "1. 停止所有进程..."
killall -9 gzserver gzclient rosmaster roslaunch roscore 2>/dev/null
sleep 2

# 清理 Gazebo 状态文件（关键！）
echo "2. 清理 Gazebo 状态文件..."
rm -rf ~/.gazebo/server-* 2>/dev/null
rm -rf ~/.gazebo/client-* 2>/dev/null
rm -f ~/.gazebo/gui.ini 2>/dev/null
echo "   ✅ 已删除 Gazebo 状态文件"

# 清理临时文件
echo "3. 清理临时文件..."
rm -rf /tmp/.gazebo* 2>/dev/null
rm -rf /tmp/.ros* 2>/dev/null
rm -f ~/.ros/rospack_cache_* 2>/dev/null
echo "   ✅ 已清理临时文件"

# 加载环境
echo "4. 加载环境..."
cd /home/ff/intent-mpc
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 验证
echo "5. 验证环境..."
if rospack find uav_simulator > /dev/null 2>&1; then
    echo "   ✅ uav_simulator 包找到"
else
    echo "   ❌ uav_simulator 包未找到！"
    exit 1
fi

echo ""
echo "======================================"
echo "✅ 清理完成！现在启动 Gazebo..."
echo "======================================"
echo ""

# 启动
roslaunch uav_simulator start.launch
