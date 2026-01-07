# 环境变量问题修复指南

## 🎯 问题确认

无人机物理上存在但看不见的原因：**ROS环境变量指向了错误的工作空间**

### 当前问题

```bash
ROS_PACKAGE_PATH=/home/ff/sipp/src:...  ← 错误！指向sipp项目
```

应该是：
```bash
ROS_PACKAGE_PATH=/home/ff/intent-mpc/src:...  ← 正确！
```

### 症状

- ✅ 无人机物理上存在（rostopic 能看到）
- ❌ Gazebo GUI 中看不见无人机
- ❌ 模型文件找不到（因为路径错误）

---

## 🔧 解决方案

### 方法1：每次启动前 source（临时）

**在启动仿真的终端**：
```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

**在启动导航的终端**：
```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

### 方法2：使用修复脚本

```bash
cd /home/ff/intent-mpc
source fix_env.sh
roslaunch uav_simulator start.launch
```

### 方法3：修复 .bashrc（永久）

#### 检查 .bashrc

```bash
cat ~/.bashrc | grep -E "source.*setup.bash|ROS"
```

#### 找出问题行

可能看到：
```bash
source /home/ff/sipp/devel/setup.bash  ← 删除或注释这行！
```

#### 修改 .bashrc

1. 打开文件：
```bash
gedit ~/.bashrc
```

2. 找到并**注释掉或删除**：
```bash
# source /home/ff/sipp/devel/setup.bash  ← 注释掉
```

3. 确保只有这一行（如果没有就添加）：
```bash
source /opt/ros/noetic/setup.bash
```

4. **不要**在 .bashrc 中 source 具体项目的 setup.bash！
   - 原因：会影响所有终端
   - 应该在需要时手动 source

5. 保存并重新加载：
```bash
source ~/.bashrc
```

---

## 🧪 验证修复

### 1. 新开一个终端

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
```

### 2. 检查环境变量

```bash
echo $ROS_PACKAGE_PATH | tr ':' '\n' | head -5
```

**应该看到**：
```
/home/ff/intent-mpc/src  ← 第一行应该是这个！
/opt/ros/noetic/share
...
```

### 3. 测试启动

```bash
killall -9 gzserver gzclient rosmaster
roslaunch uav_simulator start.launch
```

**在 Gazebo GUI 中应该看到无人机！**

---

## 📋 完整启动流程

### 终端1：启动仿真

```bash
# 1. 进入目录
cd /home/ff/intent-mpc

# 2. 加载环境（重要！）
source devel/setup.bash

# 3. 验证环境
echo $ROS_PACKAGE_PATH | head -c 50
# 应该显示：/home/ff/intent-mpc/src:...

# 4. 启动
roslaunch uav_simulator start.launch
```

### 终端2：启动导航

```bash
# 1. 进入目录
cd /home/ff/intent-mpc

# 2. 加载环境（每个终端都要！）
source devel/setup.bash

# 3. 启动
roslaunch autonomous_flight intent_mpc_demo.launch
```

---

## 🔍 常见错误

### 错误1：忘记 source

```bash
roslaunch uav_simulator start.launch
# 错误：找不到包
```

**解决**：
```bash
source devel/setup.bash
roslaunch uav_simulator start.launch
```

### 错误2：在错误的目录

```bash
cd /home/ff/sipp  ← 错误的目录！
source devel/setup.bash
```

**解决**：
```bash
cd /home/ff/intent-mpc  ← 正确！
source devel/setup.bash
```

### 错误3：.bashrc 冲突

多个项目的 setup.bash 冲突。

**解决**：
- 从 .bashrc 中移除所有项目的 setup.bash
- 只保留 `source /opt/ros/noetic/setup.bash`
- 需要时手动 source 项目的 setup.bash

---

## 🎯 为什么会发生

### 原因

昨天配置 IDE 时，可能：
1. 修改了 .bashrc
2. 添加了其他项目的环境变量
3. 导致 ROS_PACKAGE_PATH 指向错误

### 影响

- Gazebo 找不到正确的模型路径
- 无人机 mesh 文件加载失败
- 物理引擎中存在，但没有视觉模型

---

## 💡 最佳实践

### 1. .bashrc 只放基础设置

```bash
# ~/.bashrc
source /opt/ros/noetic/setup.bash
# 不要 source 具体项目！
```

### 2. 每个项目单独 source

```bash
# 工作目录1
cd ~/project1
source devel/setup.bash
roslaunch ...

# 工作目录2（新终端）
cd ~/project2
source devel/setup.bash
roslaunch ...
```

### 3. 使用别名简化

在 .bashrc 中添加：
```bash
alias intent_mpc='cd /home/ff/intent-mpc && source devel/setup.bash'
```

使用：
```bash
intent_mpc
roslaunch uav_simulator start.launch
```

---

## 🆘 如果还是不行

1. 完全重置环境：
```bash
# 关闭所有终端
# 打开新终端
unset ROS_PACKAGE_PATH
source /opt/ros/noetic/setup.bash
cd /home/ff/intent-mpc
source devel/setup.bash
```

2. 重新编译：
```bash
cd /home/ff/intent-mpc
catkin_make clean
catkin_make
source devel/setup.bash
```

3. 检查 .bashrc：
```bash
grep -n "sipp\|ROS\|setup.bash" ~/.bashrc
```

找出所有相关的行并清理。


