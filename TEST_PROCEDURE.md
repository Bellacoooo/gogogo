# 测试步骤

## 已执行操作

✅ 恢复了以下文件到原始状态：
1. `ref_trajectory.txt` → `0.0 -0.0 0.0 1.0`
2. `flight_base.yaml` → `use_predefined_goal: false`
3. `mapping_param.yaml` → 恢复静态地图配置

## 🧪 测试1：恢复后测试

```bash
# 1. 清理
killall -9 gzserver gzclient rosmaster
rm -rf /tmp/.gazebo* /tmp/.ros*

# 2. 启动
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

### 检查无人机

**新终端**：
```bash
rostopic echo /gazebo/model_states -n 1 | grep -A 5 quadcopter
```

### 预期结果

如果恢复后无人机**出现了**：
- ✅ 说明是某个配置文件的修改导致的
- 需要逐个修改测试找出原因

如果还是**没有**：
- ❌ 说明不是配置文件的问题
- 可能是系统级问题

## 🧪 测试2：如果恢复后正常

### 逐个修改测试

#### 步骤1：只改 use_predefined_goal

```bash
# 编辑 flight_base.yaml
use_predefined_goal: false → true
```

重启测试，看无人机是否还在。

#### 步骤2：只改 ref_trajectory.txt

```bash
# 编辑 ref_trajectory.txt
0.0 -0.0 0.0 1.0 → 0.0 -16.0 0.0 1.0
```

重启测试，看无人机是否还在。

#### 步骤3：改 mapping_param.yaml

注释掉静态地图，重启测试。

## 🔍 可能的原因分析

### 如果是 mapping_param.yaml 的问题

可能原因：
1. 静态地图路径问题导致启动失败
2. 地图加载影响了物理引擎

解决：使用正确的地图路径或禁用静态地图

### 如果是 ref_trajectory.txt 的问题

这**理论上不可能**，因为：
- ref_trajectory.txt 只在 intent_mpc_demo.launch 中读取
- start.launch 不会读取这个文件
- 无人机 spawn 与目标点无关

除非：
- 文件编码问题？
- 修改时意外损坏了其他文件？

### 如果是 flight_base.yaml 的问题

也**不太可能**，因为：
- flight_base.yaml 在 intent_mpc_demo.launch 加载
- start.launch 不依赖这个文件

## 🐛 其他可能

### 1. 缓存问题

```bash
cd /home/ff/intent-mpc
rm -rf build/ devel/
catkin_make
source devel/setup.bash
```

### 2. 环境变量问题

```bash
env | grep ROS
env | grep GAZEBO
```

检查是否有异常的环境变量。

### 3. 最近的改动

```bash
cd /home/ff/intent-mpc
git log --oneline -10
git diff HEAD~1
```

查看最近的所有改动。

## 📋 报告结果

请测试后告诉我：

1. **恢复后是否有无人机？**
   - 有 → 继续逐个测试
   - 没有 → 不是配置文件问题

2. **如果有，是哪个修改导致消失？**
   - ref_trajectory.txt
   - flight_base.yaml
   - mapping_param.yaml
   - 多个文件组合

3. **无人机的位置**
   ```bash
   rostopic echo /gazebo/model_states -n 1 | grep -A 10 quadcopter
   ```

## 🎯 最终目标

找出是什么配置导致无人机消失，然后修复它。


