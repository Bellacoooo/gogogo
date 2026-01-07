#!/bin/bash

echo "=========================================="
echo "测试无人机生成过程"
echo "=========================================="
echo ""

cd /home/ff/intent-mpc
source devel/setup.bash

# 检查 URDF 文件
echo "步骤 1/5: 检查 URDF 文件..."
URDF_FILE="/home/ff/intent-mpc/src/Intent-MPC/uav_simulator/urdf/quadcopter.urdf"
if [ -f "$URDF_FILE" ]; then
    echo "✅ URDF 文件存在: $URDF_FILE"
    echo "   大小: $(ls -lh $URDF_FILE | awk '{print $5}')"
else
    echo "❌ URDF 文件不存在！"
    exit 1
fi

# 检查 mesh 文件
echo ""
echo "步骤 2/5: 检查 mesh 文件..."
MESH_DIR="/home/ff/intent-mpc/src/Intent-MPC/uav_simulator/urdf/quadcopter/meshes"
if [ -d "$MESH_DIR" ]; then
    echo "✅ Mesh 目录存在"
    ls -lh $MESH_DIR/*.dae 2>/dev/null | awk '{print "   " $9 " - " $5}'
else
    echo "❌ Mesh 目录不存在！"
fi

# 验证 URDF 语法
echo ""
echo "步骤 3/5: 验证 URDF 语法..."
check_urdf $URDF_FILE 2>&1 | head -10

# 测试 robot_description 参数加载
echo ""
echo "步骤 4/5: 测试参数加载..."
ROBOT_DESC=$(cat $URDF_FILE)
if [ ! -z "$ROBOT_DESC" ]; then
    echo "✅ URDF 内容可以读取"
    echo "   长度: ${#ROBOT_DESC} 字符"
else
    echo "❌ 无法读取 URDF 内容"
fi

# 启动空的 Gazebo 并尝试手动 spawn
echo ""
echo "步骤 5/5: 启动 Gazebo 并手动 spawn..."
echo "=========================================="
echo ""
echo "第一个终端: 启动空的 Gazebo"
echo "  roslaunch gazebo_ros empty_world.launch"
echo ""
echo "第二个终端: 加载并 spawn 无人机"
echo "  cd /home/ff/intent-mpc"
echo "  source devel/setup.bash"
echo "  rosparam load $URDF_FILE robot_description"
echo "  rosrun gazebo_ros spawn_model -urdf -param robot_description -model quadcopter -x 0 -y 0 -z 0.5"
echo ""
echo "或者，一次性测试:"
echo "  roslaunch uav_simulator start.launch --screen 2>&1 | grep -i 'spawn\|error\|fail'"
echo ""

