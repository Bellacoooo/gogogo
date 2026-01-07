#!/bin/bash

LOG_FILE="/tmp/gazebo_start_$(date +%Y%m%d_%H%M%S).log"

echo "=========================================="
echo "无人机 Spawn 问题诊断"
echo "=========================================="
echo ""
echo "日志将保存到: $LOG_FILE"
echo ""

cd /home/ff/intent-mpc
source devel/setup.bash

# 清理
killall -9 gzserver gzclient rosmaster 2>/dev/null
sleep 2
rm -rf /tmp/.gazebo* /tmp/.ros*

echo "步骤 1/4: 启动 Gazebo (后台，30秒)..."
timeout 30 roslaunch uav_simulator start.launch > $LOG_FILE 2>&1 &
LAUNCH_PID=$!

echo "  PID: $LAUNCH_PID"
echo "  等待 15 秒让 Gazebo 启动..."
sleep 15

echo ""
echo "步骤 2/4: 检查进程状态..."
if ps -p $LAUNCH_PID > /dev/null; then
    echo "  ✅ roslaunch 还在运行"
else
    echo "  ❌ roslaunch 已退出"
fi

echo ""
ps aux | grep -E "gzserver|gzclient|rosmaster" | grep -v grep | head -5

echo ""
echo "步骤 3/4: 检查模型状态..."
rostopic echo /gazebo/model_states -n 1 2>&1 | head -50

echo ""
echo "步骤 4/4: 分析日志..."
echo "==========================================  "
echo ""

echo "🔍 Spawn 相关日志:"
grep -i "spawn" $LOG_FILE | head -20

echo ""
echo "🔍 Error/Fail 日志:"
grep -iE "error|fail" $LOG_FILE | grep -v "Queue limit" | head -20

echo ""
echo "🔍 Quadcopter 相关日志:"
grep -i "quadcopter" $LOG_FILE | head -20

echo ""
echo "🔍 Plugin 相关日志:"
grep -i "plugin" $LOG_FILE | head -10

echo ""
echo "=========================================="
echo "完整日志文件: $LOG_FILE"
echo "查看完整日志: cat $LOG_FILE"
echo ""
echo "停止 Gazebo: killall -9 gzserver gzclient rosmaster"
echo "=========================================="

