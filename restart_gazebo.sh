#!/bin/bash

echo "=========================================="
echo "重启 Gazebo 和 ROS"
echo "=========================================="
echo ""

echo "1. 杀死所有相关进程..."
killall -9 gzserver gzclient rosmaster roslaunch 2>/dev/null
sleep 2

echo "2. 清理临时文件..."
rm -rf /tmp/.gazebo* 2>/dev/null
rm -rf /tmp/.ros* 2>/dev/null
echo "   清理完成"

echo ""
echo "3. 等待3秒..."
sleep 3

echo ""
echo "=========================================="
echo "✅ 清理完成！"
echo ""
echo "现在可以重新运行："
echo "  roslaunch uav_simulator start.launch"
echo "=========================================="



