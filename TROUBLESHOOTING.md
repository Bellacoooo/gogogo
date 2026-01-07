# 故障排查指南

## 已修复的问题

### 问题1：dist_min 全部显示 999

**原因**: 
1. `fake_detector_node` 没有在launch文件中启动
2. 障碍物高度设置不正确（z=0而不是z=1.0）

**解决方案**:
1. ✅ 已在 `intent_mpc_demo.launch` 中添加 `fake_detector_node`
2. ✅ 已将所有障碍物高度调整为 z=1.0（与无人机飞行高度一致）

### 问题2：障碍物高度不匹配

**原因**: 
- 障碍物中心在 z=0，但无人机在 z=1.0
- 圆柱体高度1.3米，中心z=0意味着范围是 z=-0.65 到 z=0.65
- 无人机在 z=1.0 完全在障碍物上方，无法碰撞

**解决方案**:
✅ 所有障碍物位置和路径点已更新为 z=1.0

**修改内容**:
- O1: (-16, 0, **1.0**) → (0, 0, **1.0**)
- O2: (-16, -3, **1.0**) → (0, 2, **1.0**)
- O3: (-16, 3, **1.0**) → (0, -2, **1.0**)

### 问题3：无人机往y方向飞然后崩溃

**可能原因**:
1. MPC规划器遇到无解情况
2. A*全局规划器找到了不合理的路径
3. 地图未正确更新障碍物信息

**解决方案**:
- ✅ 确保障碍物高度正确（z=1.0）
- ✅ fake_detector正确启动
- 检查MPC和A*参数配置

## 使用诊断脚本

运行诊断脚本检查系统状态：

```bash
cd /home/ff/intent-mpc
./diagnose_system.sh
```

诊断脚本会检查：
1. ROS是否运行
2. 关键节点是否启动
3. 话题是否可用
4. 服务是否存在
5. 参数配置
6. 数据文件

## 正确的启动顺序

### 终端1：启动仿真和数据记录

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

**等待**：确保Gazebo完全加载（约10秒）

### 终端2：启动MPC导航

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

### 终端3（可选）：监控

```bash
# 查看数据记录节点输出
rostopic echo /CERLAB/quadcopter/odom

# 检查障碍物服务
rosservice call /fake_detector/get_dynamic_obstacles \
  "current_position: {x: 0.0, y: 0.0, z: 1.0}
   range: 50.0"
```

## 检查数据记录

### 实时监控

```bash
# 查看最新CSV文件
cd /home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/
tail -f $(ls -t flight_data_*.csv | head -1)
```

### 验证障碍物检测

如果 `dist_min` 仍然是 999：

1. **检查fake_detector节点**:
```bash
rosnode list | grep fake_detector
# 应该输出: /fake_detector_node
```

2. **测试障碍物服务**:
```bash
rosservice call /fake_detector/get_dynamic_obstacles \
  "current_position: {x: -8.0, y: 0.0, z: 1.0}
   range: 50.0"
```

应该返回障碍物信息。

3. **查看fake_detector日志**:
```bash
rosnode info /fake_detector_node
```

## 常见错误信息

### 错误1: "Obstacle service does not exist"

**解决**: fake_detector_node没有启动
```bash
roslaunch onboard_detector run_fake_detector.launch
```

### 错误2: "Failed to call obstacle service"

**解决**: 服务存在但调用失败
- 检查参数配置
- 重启fake_detector_node

### 错误3: "Found 0 obstacles"

**解决**: 障碍物不在检测范围内
- 检查 `detection_range` 参数（默认50m）
- 检查无人机和障碍物位置
- 检查障碍物world文件是否正确加载

## 参数配置检查

### 检查关键参数

```bash
# 使用fake detector
rosparam get /autonomous_flight/use_fake_detector
# 应该是: true

# 使用预定义目标
rosparam get /autonomous_flight/use_predefined_goal
# 应该是: true

# 起飞高度
rosparam get /autonomous_flight/takeoff_height
# 应该是: 1.0

# 目标点
cat /home/ff/intent-mpc/src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/ref_trajectory.txt
# 应该是: -16.0 0.0 1.0 0.0
```

## 预期的正常输出

### 数据记录节点

```
[DataRecorder]: Data recorder initialized. Waiting for odometry...
[DataRecorder]: First odometry received. Starting recording...
[DataRecorder]: CSV file created: /home/.../flight_data_20260107_123456.csv
[DataRecorder]: Found 3 obstacles
```

### CSV文件内容

```csv
time,uav_x,uav_y,uav_z,dist_min,path_length,infeasible_flag,collision_flag
0.00,0.000,0.000,1.000,16.00,0.00,0,0
0.10,-0.100,0.000,1.000,15.90,0.10,0,0
0.20,-0.200,0.000,1.000,15.80,0.20,0,0
```

**注意**: 
- `uav_x` 应该向**负方向**移动（从0到-16）
- `dist_min` 应该逐渐减小然后增大（经过障碍物）
- 不应该全是 999

## 调试MPC崩溃问题

如果无人机飞行不稳定或崩溃：

### 1. 降低速度

编辑 `flight_base.yaml`:
```yaml
desired_velocity: 1.0  # 从1.5改为1.0
desired_acceleration: 1.0  # 从1.5改为1.0
```

### 2. 调整MPC参数

编辑 `planner_param.yaml`:
```yaml
mpc_planner/static_safety_dist: 1.0  # 增加安全距离
mpc_planner/dynamic_safety_dist: 0.8  # 增加动态安全距离
```

### 3. 检查A*规划器

```yaml
astar/w3_dynamic: 100.0  # 增加动态避障权重（当前50.0）
```

### 4. 查看MPC日志

```bash
rosnode info /mpc_navigation_node
rostopic echo /autonomous_flight/mpc_traj
```

## 场景配置

当前配置的场景：`test_head_on.world`

### 障碍物配置

| 障碍物 | 起点 | 终点 | 速度 | 高度 |
|--------|------|------|------|------|
| O1 | (-16, 0, 1.0) | (0, 0, 1.0) | 1.0 m/s | 1.0m |
| O2 | (-16, -3, 1.0) | (0, 2, 1.0) | 0.9 m/s | 1.0m |
| O3 | (-16, 3, 1.0) | (0, -2, 1.0) | 1.2 m/s | 1.0m |

### 无人机任务

- **起点**: (0, 0, 1.0)
- **目标**: (-16, 0, 1.0)
- **方向**: 向西（负x方向）
- **距离**: 16米

### 预期行为

1. 无人机从(0,0,1.0)起飞
2. 向负x方向飞向(-16, 0, 1.0)
3. 与从(-16,0,1.0)飞向(0,0,1.0)的O1迎面相遇
4. MPC应该规划避障路径（偏离y方向）
5. 最终到达目标点(-16, 0, 1.0)

## 联系信息

如果问题仍然存在，请：
1. 运行诊断脚本并保存输出
2. 查看 `/home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/` 中的CSV文件
3. 检查rosout日志



