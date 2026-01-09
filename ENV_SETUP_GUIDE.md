# 无人机加载问题 - 环境变量未设置

## 问题原因

运行 `roslaunch uav_simulator start.launch` 时报错：
```
[rospack] Error: package 'uav_simulator' not found
```

**根本原因**：当前终端没有 source 工作空间的环境变量，ROS 找不到包。

## 解决方案

### 方案 1：每次启动前手动 source（推荐用于测试）

```bash
# 终端 1 - 启动 Gazebo
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch

# 终端 2 - 启动 MPC
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

### 方案 2：使用快速启动脚本

```bash
# 终端 1
cd /home/ff/intent-mpc
./quick_start.sh

# 终端 2
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

### 方案 3：添加到 .bashrc（永久生效，但需小心）

⚠️ **注意**：只有当你确定只使用这个工作空间时才这样做

```bash
echo "source /home/ff/intent-mpc/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**不推荐原因**：
- 如果同时使用多个 ROS 工作空间会冲突
- 每次打开新终端都会自动 source，可能干扰其他项目

## 为什么会出现这个问题？

### ROS 工作空间的工作原理

1. **编译后生成环境文件**
   ```
   /home/ff/intent-mpc/devel/setup.bash
   ```

2. **source 后设置环境变量**
   - `ROS_PACKAGE_PATH`: 告诉 ROS 去哪里找包
   - `PYTHONPATH`: Python 模块路径
   - `CMAKE_PREFIX_PATH`: CMake 查找路径
   - 等等...

3. **如果不 source**
   - ROS 只能找到系统默认路径 `/opt/ros/noetic/share`
   - 找不到你的工作空间中的包
   - `roslaunch` 会报错 "package not found"

### 检查当前环境

```bash
# 检查 uav_simulator 是否可用
rospack find uav_simulator

# 查看 ROS_PACKAGE_PATH
echo $ROS_PACKAGE_PATH
# 应该包含: /home/ff/intent-mpc/src
```

## 启动步骤总结

### ✅ 正确的启动流程

```bash
# 第一步：进入工作空间
cd /home/ff/intent-mpc

# 第二步：source 环境
source devel/setup.bash

# 第三步：验证包可用
rospack find uav_simulator
# 应该输出: /home/ff/intent-mpc/src/Intent-MPC/uav_simulator

# 第四步：启动系统
roslaunch uav_simulator start.launch
```

### ❌ 错误的启动方式

```bash
# 直接运行（错误！）
roslaunch uav_simulator start.launch
# ❌ 报错: package 'uav_simulator' not found
```

## 常见问题

### Q: 为什么之前能用，现在不能用了？
A: 可能原因：
- 打开了新终端，没有 source
- 之前的终端关闭了
- 环境变量被其他操作覆盖了

### Q: 每次都要 source 太麻烦？
A: 两个选择：
1. 把 `source /home/ff/intent-mpc/devel/setup.bash` 加到 `~/.bashrc`（永久）
2. 使用 tmux/screen 保持终端会话（不用每次重新打开）

### Q: 我已经 source 了但还是报错？
A: 检查：
```bash
# 1. 确认在正确的目录
pwd
# 应该输出: /home/ff/intent-mpc

# 2. 确认 devel 目录存在
ls -la devel/setup.bash

# 3. 重新编译
catkin_make

# 4. 重新 source
source devel/setup.bash
```

## 多工作空间管理

如果你有多个 ROS 工作空间（如 intent-mpc, sipp 等）：

```bash
# 为每个工作空间创建 alias
echo 'alias ws_mpc="cd /home/ff/intent-mpc && source devel/setup.bash"' >> ~/.bashrc
echo 'alias ws_sipp="cd /home/ff/sipp && source devel/setup.bash"' >> ~/.bashrc

# 使用时
ws_mpc       # 切换到 intent-mpc 工作空间
ws_sipp      # 切换到 sipp 工作空间
```
