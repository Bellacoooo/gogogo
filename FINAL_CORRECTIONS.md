# 最终修正

## ❌ 之前的错误理解

我之前错误地修改了：
1. ❌ 将障碍物高度从 z=0 改为 z=1.0
2. ❌ ref_trajectory.txt 格式理解错误

## ✅ 正确的配置

### 1. 障碍物配置

**障碍物应该保持 z=0**（贴地运动）

原因：
- 障碍物有高度（圆柱1.3m，盒子1.75m和1.32m）
- 中心在z=0时，障碍物向上延伸
- 无人机在z=1.0飞行，会与障碍物碰撞

**已修复**：所有障碍物恢复为 z=0

| 障碍物 | 起点 | 终点 | 高度范围 |
|--------|------|------|----------|
| O1 (圆柱) | (-16, 0, 0) | (0, 0, 0) | z=0 到 z≈1.3 |
| O2 (盒子) | (-16, -3, 0) | (0, 2, 0) | z=0 到 z≈1.75 |
| O3 (盒子) | (-16, 3, 0) | (0, -2, 0) | z=0 到 z≈1.32 |

### 2. 目标点格式

**ref_trajectory.txt 格式：`dt x y z`**

- `dt`: 时间差（通常设为0.0表示立即目标）
- `x, y, z`: 目标位置坐标

**错误的配置**（我之前的）:
```
-16.0 0.0 1.0 0.0
```
系统读取为：dt=-16.0, x=0.0, y=1.0, z=0.0 → 目标点(0, 1, 0) ❌

**正确的配置**（已修复）:
```
0.0 -16.0 0.0 1.0
```
系统读取为：dt=0.0, x=-16.0, y=0.0, z=1.0 → 目标点(-16, 0, 1.0) ✅

## 🎯 正确的实验设置

### 场景配置

- **无人机起点**: (0, 0, 1.0)
- **无人机目标**: (-16, 0, 1.0)
- **飞行方向**: 向西（负x方向）
- **飞行高度**: z=1.0

### 障碍物配置

- **O1**: (-16, 0, 0) → (0, 0, 0)，速度1.0 m/s，高度~1.3m
- **O2**: (-16, -3, 0) → (0, 2, 0)，速度0.9 m/s，高度~1.75m
- **O3**: (-16, 3, 0) → (0, -2, 0)，速度1.2 m/s，高度~1.32m

### 为什么会碰撞

```
无人机飞行路径:  (0, 0, 1.0) ←→ (-16, 0, 1.0)
                          ↓
O1障碍物范围:    (-16~0, 0, 0~1.3)
                          ↑
                     会发生碰撞！
```

无人机在z=1.0，障碍物从z=0延伸到z=1.3（或1.75），所以会发生碰撞。

## 📝 修改的文件

### 1. `test_head_on.world` ✅ 已修复

恢复所有障碍物为 z=0：
```xml
<pose>-16 0 0 0 0 0</pose>  <!-- O1 -->
<pose>-16 -3 0 0 0 0</pose>  <!-- O2 -->
<pose>-16 3 0 0 0 0</pose>  <!-- O3 -->

<waypoint>-16 0 0</waypoint>  <!-- O1路径 -->
<waypoint>0 0 0</waypoint>

<waypoint>-16 -3 0</waypoint>  <!-- O2路径 -->
<waypoint>0 2 0</waypoint>

<waypoint>-16 3 0</waypoint>  <!-- O3路径 -->
<waypoint>0 -2 0</waypoint>
```

### 2. `ref_trajectory.txt` ✅ 已修复

```
0.0 -16.0 0.0 1.0
```

格式：`dt x y z`
- dt = 0.0（立即目标）
- 目标点 = (-16.0, 0.0, 1.0)

## 🚀 现在可以运行

### 终端1：启动仿真
```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

### 终端2：启动MPC导航（等待10秒）
```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

## ✅ 预期行为

1. 无人机从 (0, 0, 0.1) 起飞
2. 上升到 z=1.0
3. 向负x方向飞向 (-16, 0, 1.0)
4. 遇到从 (-16, 0, 0) 迎面飞来的O1障碍物（高度0~1.3m）
5. MPC规划避障路径
6. 到达目标点 (-16, 0, 1.0)

## 📊 数据记录

CSV文件应该显示：
```csv
time,uav_x,uav_y,uav_z,dist_min,path_length,infeasible_flag,collision_flag
0.00,0.000,0.000,1.000,16.00,0.00,0,0
0.10,-0.100,0.000,1.000,15.90,0.10,0,0
...
```

注意：
- ✅ `uav_x` 向负方向移动（0 → -16）
- ✅ `dist_min` 不是999（因为fake_detector已启动）
- ✅ `uav_z` 保持在1.0左右

## ⚠️ 关于无人机消失的问题

如果无人机消失，检查：

1. **Gazebo是否正确加载**
```bash
# 查看Gazebo中的模型
rostopic echo /gazebo/model_states
```

2. **spawn是否成功**
```bash
# start.launch 第110行
-x 0.0 -y 0.0 -z 0.1 -Y -3.14
```

3. **重启Gazebo**
```bash
killall gzserver gzclient
roslaunch uav_simulator start.launch
```

## 🐛 常见问题

### Q: 无人机不见了
**A**: 
- 检查Gazebo是否崩溃
- 重启Gazebo
- 查看spawn日志

### Q: 无人机飞到错误的位置
**A**: 
- ✅ 已修复：ref_trajectory.txt格式正确了
- 目标点现在是 (-16, 0, 1.0)

### Q: dist_min还是999
**A**: 
- ✅ 已修复：fake_detector_node已在launch文件中启动
- 障碍物高度也正确了（z=0）

## 📚 参考

代码解析（mpcNavigation.cpp 第240行）：
```cpp
double dt, x, y, z;
if (!(iss >> dt >> x >> y >> z)) break;

pose.pose.position.x = x;
pose.pose.position.y = y;
pose.pose.position.z = z;
```

格式必须是：`dt x y z`

## ✨ 总结

所有问题已修复：
1. ✅ 障碍物高度恢复为 z=0
2. ✅ ref_trajectory.txt 格式修正为 `0.0 -16.0 0.0 1.0`
3. ✅ fake_detector_node 在launch文件中启动

现在系统应该能正常工作了！



