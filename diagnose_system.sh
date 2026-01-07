#!/bin/bash

echo "======================================"
echo "系统诊断脚本"
echo "======================================"
echo ""

echo "1. 检查ROS是否运行..."
if pgrep -x "roscore" > /dev/null || pgrep -x "rosmaster" > /dev/null; then
    echo "   ✓ ROS正在运行"
else
    echo "   ✗ ROS未运行"
fi
echo ""

echo "2. 检查关键节点..."
echo "   正在运行的节点:"
rosnode list 2>/dev/null | grep -E "(mpc_navigation|data_recorder|fake_detector|gazebo)" || echo "   无相关节点运行"
echo ""

echo "3. 检查关键话题..."
echo "   /CERLAB/quadcopter/odom:"
rostopic info /CERLAB/quadcopter/odom 2>/dev/null | head -3 || echo "   话题不存在"
echo ""

echo "4. 检查障碍物服务..."
rosservice list 2>/dev/null | grep -E "fake_detector|dynamic_obstacles" || echo "   服务不可用"
echo ""

echo "5. 检查参数配置..."
echo "   use_predefined_goal: $(rosparam get /autonomous_flight/use_predefined_goal 2>/dev/null || echo '未设置')"
echo "   use_fake_detector: $(rosparam get /autonomous_flight/use_fake_detector 2>/dev/null || echo '未设置')"
echo "   takeoff_height: $(rosparam get /autonomous_flight/takeoff_height 2>/dev/null || echo '未设置')"
echo ""

echo "6. 检查数据文件..."
DATA_DIR="/home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder"
if [ -d "$DATA_DIR" ]; then
    FILE_COUNT=$(ls -1 $DATA_DIR/flight_data_*.csv 2>/dev/null | wc -l)
    echo "   找到 $FILE_COUNT 个CSV文件"
    if [ $FILE_COUNT -gt 0 ]; then
        LATEST=$(ls -t $DATA_DIR/flight_data_*.csv 2>/dev/null | head -1)
        echo "   最新文件: $(basename $LATEST)"
        echo "   文件大小: $(du -h $LATEST 2>/dev/null | cut -f1)"
        echo "   前5行数据:"
        head -5 "$LATEST" 2>/dev/null | sed 's/^/      /'
    fi
else
    echo "   ✗ 数据目录不存在"
fi
echo ""

echo "======================================"
echo "诊断完成"
echo "======================================"



