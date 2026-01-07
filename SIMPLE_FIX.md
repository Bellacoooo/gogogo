# 🎯 简单直接的修复方案

根据您的描述，这是一个**间歇性问题**（有时有无人机，有时没有），最可能的原因：

## 🔧 快速修复步骤

### 方案1：清理 Gazebo 的隐藏状态文件（最可能有效）

```bash
# 1. 停止所有进程
killall -9 gzserver gzclient rosmaster roslaunch

# 2. 删除 Gazebo 的状态文件（关键！）
rm -rf ~/.gazebo/server-*/
rm -rf ~/.gazebo/client-*/
rm -f ~/.gazebo/gui.ini

# 3. 清理临时文件
rm -rf /tmp/.gazebo*
rm -rf /tmp/.ros*

# 4. 启动
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

---

### 方案2：使用空世界测试（排除障碍物干扰）

```bash
# 启动空世界
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch gazebo_ros empty_world.launch
```

**在另一个终端**：
```bash
cd /home/ff/intent-mpc
source devel/setup.bash

# 手动 spawn 无人机
rosrun gazebo_ros spawn_model \
  -file src/Intent-MPC/uav_simulator/urdf/quadcopter.urdf \
  -urdf -model test_quad -x 0 -y 0 -z 0.5
```

如果能看到无人机，说明问题在 world 文件或障碍物上。

---

### 方案3：使用其他 world 文件测试

编辑 `src/Intent-MPC/uav_simulator/launch/start.launch`，尝试其他已经工作过的 world：

```xml
<!-- 改成简单的测试场景 -->
<arg name="world_name" value="$(find uav_simulator)/worlds/test/empty.world" />
```

---

### 方案4：检查是不是 GUI 渲染问题

无人机的物理模型可能存在，但 GUI 渲染失败：

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch

# 在另一个终端检查
rostopic echo /CERLAB/quadcopter/pose -n 5
```

如果能看到位置数据，说明无人机存在，只是渲染有问题。

---

## 📊 根据您之前的描述

> "昨天改了什么又出现了，今天改了什么又消失了"

这种间歇性问题通常是：

1. **Gazebo 缓存的世界状态**
   - 解决：删除 `~/.gazebo/server-*/` 和 `~/.gazebo/client-*/`

2. **不同终端使用不同环境**
   - 解决：每次都在新终端中 `source devel/setup.bash`

3. **某个launch文件被修改后没保存/恢复**
   - 解决：`git status` 检查修改

4. **ROS 缓存的参数**
   - 解决：删除 `~/.ros/rospack_cache_*`

---

## 🎯 我的建议

**直接运行方案1**，这最可能解决间歇性问题。

如果方案1不行，请告诉我：
1. Gazebo GUI 能打开吗？
2. 能看到障碍物吗？
3. 运行 `rostopic echo /CERLAB/quadcopter/pose -n 1` 有输出吗？

这样我能更准确判断问题。

