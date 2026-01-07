#!/bin/bash

echo "========================================="
echo "检查障碍物检测服务状态"
echo "========================================="

if ! rostopic list &>/dev/null; then
    echo "❌ ROS master 未运行"
    exit 1
fi

echo ""
echo "1. 检查节点..."
echo "fake_detector 节点:"
if rosnode list | grep -q "fake_detector"; then
    echo "  ✓ 正在运行"
    rosnode info /fake_detector_node 2>/dev/null | grep -E "Services:|Publications:" | head -5
else
    echo "  ❌ 未运行"
fi

echo ""
echo "2. 检查服务..."
SERVICE_NAME="/fake_detector/getDynamicObstacles"
if rosservice list | grep -q "$SERVICE_NAME"; then
    echo "  ✓ 服务存在: $SERVICE_NAME"
    
    echo ""
    echo "3. 测试服务调用..."
    echo "  调用位置: (0, 0, 1.0), 范围: 50m"
    rosservice call $SERVICE_NAME "current_position:
  x: 0.0
  y: 0.0
  z: 1.0
range: 50.0" 2>&1 | head -30
    
    echo ""
    echo "4. 检查 Gazebo 中的障碍物..."
    timeout 2 rostopic echo /gazebo/model_states -n 1 2>&1 | grep -A 1 "name:" | grep "dynamic" | head -5
    
else
    echo "  ❌ 服务不存在: $SERVICE_NAME"
    echo ""
    echo "  可用的服务:"
    rosservice list | grep -i "fake_detector\|obstacle" | head -10
fi

echo ""
echo "========================================="
echo "检查完成"
echo "========================================="

