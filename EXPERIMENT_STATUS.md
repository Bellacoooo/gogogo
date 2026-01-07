# 实验状态分析

## RViz 退出原因

### ✅ 不是崩溃
- 日志显示: **"process has finished cleanly"** (正常退出)
- 没有任何错误、异常或崩溃信息
- 可能是手动关闭或 launch 文件中 `required="true"` 导致的连锁关闭

## CSV 数据分析

### 文件信息
- 文件: `flight_data_20260107_215815.csv`
- 大小: 12KB
- 时长: ~31秒

### 数据情况

| 指标 | 状态 | 数值 |
|------|------|------|
| `time` | ✅ 正常 | 0.00 → 31.74s |
| `uav_x, uav_y, uav_z` | ❌ **异常** | **一直是 (0, 0, 0)** |
| `dist_min` | ✅ 正常 | 0.88 → 0.93m |
| `path_length` | ❌ 静止 | 一直是 27.31m (没变化) |
| `infeasible_flag` | ⚠️ 待验证 | 一直是 0 |
| `collision_flag` | ✅ 正常 | 0 (距离 > 0.4m) |

## 🚨 关键问题：无人机位置一直是 (0, 0, 0)

### 可能原因

#### 1. 无人机没有起飞
- **最可能**：无人机还在原点，没有收到起飞指令
- MPC 还没开始执行轨迹
- 或者等待用户手动设置目标点

#### 2. 里程计 topic 问题
- `data_recorder` 订阅的是 `/CERLAB/quadcopter/odom`
- 但实际的 topic 名称可能不同
- 或者 Gazebo 没有正确发布里程计数据

#### 3. 无人机在 Gazebo 中但位置异常
- 无人机可能卡在地面或某个位置
- 物理引擎可能有问题

## 调试步骤

### 1. 检查无人机是否在 Gazebo 中
```bash
# 重新启动 Gazebo
roslaunch uav_simulator start.launch

# 检查模型列表
gz model -l
```

### 2. 检查里程计 topic
```bash
# 启动系统后，检查 odom topic
rostopic list | grep odom

# 查看 odom 数据
rostopic echo /CERLAB/quadcopter/odom | head -30
```

### 3. 检查无人机位置
```bash
# 查看模型状态
gz model -m quadcopter -p
```

### 4. 手动设置目标点
在 RViz 中使用 "2D Nav Goal" 工具设置目标点，观察：
- 无人机是否开始移动
- CSV 文件中 uav_x, uav_y, uav_z 是否更新

## 推测

从 CSV 数据看：
- `dist_min` 从 0.88 变化到 0.93，说明**障碍物在移动**
- `path_length` 固定在 27.31m，说明这可能是预设路径的长度
- 无人机位置 (0,0,0) 不变，说明**无人机没有移动**

**结论**：
- ✅ 障碍物检测工作正常（`dist_min` 在更新）
- ✅ CSV 记录工作正常
- ❌ 无人机可能还在等待起飞或目标点设置
- ⚠️ `infeasible_flag` 一直是 0 可能因为 MPC 还没开始真正执行

## 建议

1. **重新运行，确保设置目标点**
   - 在 RViz 中用 "2D Nav Goal" 设置目标
   - 或者确保 `use_predefined_goal: true`

2. **检查起飞高度**
   - 确认无人机起飞到正确高度（如 1.0m）

3. **监控实时状态**
   ```bash
   rostopic echo /CERLAB/quadcopter/odom
   rostopic echo /mpcNavigation/infeasible
   ```
