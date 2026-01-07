# 恢复到 GitHub 原始版本

## ✅ 已完成的恢复操作

### 1. 恢复的文件
```bash
✅ src/Intent-MPC/uav_simulator/launch/start.launch
✅ src/Intent-MPC/autonomous_flight/launch/intent_mpc_demo.launch
```

这两个文件已经恢复到 GitHub `risk-map-branch` 分支的最新版本。

### 2. 保留的新文件（未删除）
```
- src/Intent-MPC/flight_data_recorder/          # 数据记录包
- src/Intent-MPC/uav_simulator/worlds/test/test_head_on.world  # 新世界文件
- 各种 .md 文档和脚本
```

这些是新增的文件，不影响原系统运行。

---

## 🎯 现在测试原始版本

### 方案A：使用测试脚本（推荐）

**在新终端中运行**：

```bash
cd /home/ff/intent-mpc
./test_original.sh
```

这个脚本会：
1. 停止所有进程
2. 清理临时文件
3. 重置并正确加载环境变量
4. 验证包能被找到
5. 启动原始的 Gazebo 仿真

### 方案B：手动启动

#### 打开第一个新终端：

```bash
# 停止所有进程
killall -9 gzserver gzclient rosmaster roslaunch roscore
sleep 2

# 清理
rm -rf /tmp/.gazebo* /tmp/.ros*

# 加载环境
cd /home/ff/intent-mpc
source devel/setup.bash

# 验证
rospack find uav_simulator

# 启动
roslaunch uav_simulator start.launch
```

等待 Gazebo 完全启动，应该看到：
- ✅ Gazebo GUI 打开
- ✅ 无人机出现在地面上（不是地底）
- ✅ 障碍物正常

#### 打开第二个新终端（如果需要控制无人机）：

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

---

## 🔍 如何判断成功？

### Gazebo 启动后检查：

```bash
# 新开一个终端
cd /home/ff/intent-mpc
source devel/setup.bash

# 检查模型状态
rostopic echo /gazebo/model_states -n 1 | grep -A 5 quadcopter
```

**成功的标志**：
```
name: quadcopter
position:
  x: 0.0
  y: 0.0
  z: 0.1          <-- 接近地面，不是 -2727！
```

**失败的标志**：
```
  z: -2727.xxx    <-- 还在地底
```

---

## 📊 可能的结果

### 结果1：原始版本正常 ✅

**说明**：我们的修改导致了问题（虽然修改很小）

**可能原因**：
- `flight_data_recorder` 包可能与某些东西冲突
- 或者是构建过程中的问题

**解决方案**：
```bash
# 重新构建
cd /home/ff/intent-mpc
catkin_make clean
catkin_make
source devel/setup.bash

# 然后再试我们修改的版本
```

### 结果2：原始版本也不正常 ❌

**说明**：不是代码的问题，是环境的问题

**需要检查**：
1. **环境变量仍然错误**
   - 确认是在**新终端**中运行
   - 确认 `echo $ROS_PACKAGE_PATH` 第一项是 `/home/ff/intent-mpc/src`

2. **系统层面的问题**
   - Gazebo 版本：`gazebo --version`（应该是 11.x）
   - OpenGL：`glxinfo | grep "OpenGL version"`
   - ROS 安装：`dpkg -l | grep ros-noetic-gazebo`

3. **工作空间损坏**
   ```bash
   cd /home/ff/intent-mpc
   catkin_make clean
   rm -rf build/ devel/
   catkin_make
   source devel/setup.bash
   ```

---

## 🆘 如果原始版本也不行

### 选项1：重新构建工作空间

```bash
cd /home/ff/intent-mpc
catkin_make clean
rm -rf build/ devel/
catkin_make
source devel/setup.bash
roslaunch uav_simulator start.launch
```

### 选项2：检查子模块

```bash
cd /home/ff/intent-mpc
git submodule status
git submodule update --init --recursive
```

### 选项3：从 GitHub 重新克隆（最彻底）

```bash
# 备份您的工作（如果有重要修改）
cp -r /home/ff/intent-mpc /home/ff/intent-mpc-backup

# 重新克隆
cd /home/ff
rm -rf intent-mpc
git clone <您的仓库URL> intent-mpc
cd intent-mpc
git checkout risk-map-branch
catkin_make
source devel/setup.bash
roslaunch uav_simulator start.launch
```

---

## 🎯 我们学到的教训

### 1. 环境变量至关重要
`.bashrc` 的修改必须在**新终端**中才生效。

### 2. 可能的根本问题
- 不是我们添加的 `fake_detector_node`（这只是一个简单的服务节点）
- 不是 `flight_data_recorder`（还没运行就出问题了）
- **很可能是环境变量或构建状态的问题**

### 3. 无人机掉到 z=-2727 的原因
这通常意味着：
- URDF 模型加载失败
- Mesh 文件找不到
- Gazebo 无法正确渲染模型

所有这些都指向**环境变量或路径问题**，而不是代码逻辑问题。

---

## 📝 下一步建议

1. **首先**：在新终端中运行 `./test_original.sh`
2. **如果成功**：说明是我们的修改有问题（虽然不太可能）
3. **如果失败**：说明是环境问题，需要重新构建或重新克隆

请告诉我测试结果！

