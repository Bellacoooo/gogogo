# 无人机消失问题调试步骤

## 🔍 已执行的修改

### 1. 移除了 start.launch 中的 data_recorder
- 原因：data_recorder 应该和导航节点一起启动
- 已移到 intent_mpc_demo.launch

### 2. 临时切换到已知正常的 world 文件
- 切换为 test_keep_some.world（已知正常工作）
- 用于测试是否是 test_head_on.world 的问题

## 🧪 测试步骤

### 测试1：使用 test_keep_some.world

```bash
# 1. 清理
killall -9 gzserver gzclient rosmaster
rm -rf /tmp/.gazebo* /tmp/.ros*

# 2. 启动（会使用 test_keep_some.world）
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

**预期结果**：
- ✅ 无人机应该正常出现在 z=0.1
- ✅ 不会掉到地下

**检查方法**：
```bash
# 新终端
rostopic echo /gazebo/model_states -n 1 | grep -A 5 quadcopter
```

应该看到 `z: 0.1` 左右（不是 -80000）

### 测试2：如果 test_keep_some 正常

说明问题在 test_head_on.world 文件。

检查：
1. world 文件格式
2. 障碍物配置是否有冲突
3. physics 设置

### 测试3：如果 test_keep_some 也异常

说明问题不在 world 文件，可能是：
1. URDF 或插件问题
2. Gazebo 环境问题
3. ROS 包版本问题

## 🔍 详细检查列表

### 如果无人机还是掉下去

#### 1. 检查 Gazebo 版本
```bash
gazebo --version
```

#### 2. 检查插件路径
```bash
echo $GAZEBO_PLUGIN_PATH
ls -la /home/ff/intent-mpc/src/Intent-MPC/uav_simulator/plugins/
```

#### 3. 检查 spawn 日志
```bash
cat ~/.ros/log/latest/spawn_gazebo_model-*.log
```

#### 4. 检查 Gazebo 错误
```bash
cat ~/.gazebo/server-*.log | grep -i error | tail -20
```

#### 5. 测试最简单的 world
```bash
# 使用空世界
roslaunch uav_simulator start.launch world_name:=empty.world
```

## 📊 诊断信息收集

如果问题持续，收集以下信息：

```bash
# 1. ROS 环境
echo $ROS_PACKAGE_PATH
echo $GAZEBO_PLUGIN_PATH
echo $GAZEBO_MODEL_PATH

# 2. Gazebo 状态
rostopic echo /gazebo/model_states -n 1

# 3. 无人机话题
rostopic list | grep quadcopter

# 4. 插件文件
ls -la /home/ff/intent-mpc/devel/lib/libuav_simulator.so
ldd /home/ff/intent-mpc/devel/lib/libuav_simulator.so | grep "not found"

# 5. 最近的日志
ls -lht ~/.ros/log/latest/*.log | head -10
```

## 🐛 可能的原因

### 原因1：data_recorder 冲突
- ✅ 已修复：移除了 start.launch 中的 data_recorder

### 原因2：test_head_on.world 有问题
- 🧪 测试中：临时切换为 test_keep_some.world

### 原因3：障碍物插件冲突
- 可能性：障碍物运动插件影响了无人机spawn
- 检查：使用空世界测试

### 原因4：初始位置冲突
- 可能性：无人机初始位置(-3.14 yaw)与障碍物冲突
- 当前：障碍物在 x=-16，无人机在 x=0，应该不冲突

### 原因5：Gazebo 内部状态问题
- 可能性：Gazebo 数据库损坏
- 解决：删除 ~/.gazebo 重建

```bash
rm -rf ~/.gazebo
gazebo --verbose  # 会重新初始化
```

## 📝 下一步

1. ⏳ 测试 test_keep_some.world 是否正常
2. 根据结果决定：
   - 如果正常 → 修复 test_head_on.world
   - 如果异常 → 深入检查系统配置

## 🆘 如果都不行

最后的手段：

1. 重新编译 uav_simulator
```bash
cd /home/ff/intent-mpc
catkin_make clean
catkin_make --pkg uav_simulator
```

2. 检查是否有其他人遇到类似问题
```bash
# 搜索日志中的关键错误
grep -r "80000\|fell\|collision" ~/.ros/log/latest/
```

3. 使用备份的 world 文件（如果有）


