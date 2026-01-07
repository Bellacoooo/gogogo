# Gazebo 无人机消失问题修复

## 🐛 问题描述

无人机spawn后立即掉到地下 -80000+ 米，在 Gazebo GUI 中看不见。

## 原因分析

从 `/gazebo/model_states` 看到：
```
position:
  x: 0.0
  y: 0.0
  z: -80038.9570658968  ← 掉到地底
```

可能原因：
1. 无人机插件初始化问题
2. 物理引擎碰撞问题
3. spawn 时的朝向/初始状态问题

## ✅ 解决方案

### 方法1：重启 Gazebo（快速）

```bash
# 杀死所有 Gazebo 进程
killall -9 gzserver gzclient rosmaster roslaunch

# 清理临时文件
rm -rf /tmp/.gazebo* ~/.gazebo/log/*

# 重新启动
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

### 方法2：检查 URDF 和插件

问题可能在 `quadcopter.urdf` 或无人机控制插件。

### 方法3：调整 spawn 参数

当前 spawn 参数（start.launch 第110行）：
```xml
<node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" 
      args="-urdf -param robot_description -model quadcopter 
            -x 0.0 -y 0.0 -z 0.1 -Y -3.14" respawn="false" />
```

尝试修改：
```xml
<node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" 
      args="-urdf -param robot_description -model quadcopter 
            -x 0.0 -y 0.0 -z 1.0 -Y 0" respawn="false" />
```

变化：
- z: 0.1 → 1.0（提高初始高度）
- Y: -3.14 → 0（去掉180度旋转）

## 🔍 检查当前状态

```bash
# 检查无人机位置
rostopic echo /gazebo/model_states -n 1 | grep -A 10 quadcopter

# 检查无人机 odom
rostopic echo /CERLAB/quadcopter/odom -n 1
```

## ⚠️ fake_detector 问题

同时发现：fake_detector服务不存在

这是因为 `start.launch` 只启动了仿真环境，fake_detector 在 `intent_mpc_demo.launch` 中启动。

**这是正常的！** 需要先启动仿真，再启动导航。

## 📋 正确的启动步骤

### 步骤1：清理并重启 Gazebo

```bash
# 终端1
killall -9 gzserver gzclient rosmaster
rm -rf /tmp/.gazebo*

cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

### 步骤2：等待 Gazebo 加载（约10秒）

在 Gazebo GUI 中应该看到：
- ✅ 3个红色障碍物
- ✅ 1个无人机（应该在地面上）

### 步骤3：启动导航

```bash
# 终端2
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

## 🎯 预期结果

启动后：
1. ✅ 无人机在 (0, 0, 0.1) 可见
2. ✅ 无人机自动起飞到 z=1.0
3. ✅ fake_detector 服务可用
4. ✅ 数据开始记录

## 🐛 如果还是掉下去

如果重启后无人机还是掉下去，说明是无人机模型或插件问题。

检查：

```bash
# 查看无人机控制插件日志
rostopic echo /CERLAB/quadcopter/odom -n 5

# 查看 Gazebo 日志
cat ~/.gazebo/server-*.log | grep -i error | tail -20
```

可能需要：
1. 检查 quadcopter.urdf
2. 检查 quadcopterPlugin.cpp
3. 检查物理参数



