# MPC 实验指标说明

## CSV 文件格式

数据记录器每 **0.1 秒**（10Hz）记录一次数据，保存到 CSV 文件：
```
time,uav_x,uav_y,uav_z,dist_min,path_length,infeasible_flag,collision_flag
```

## 指标详细说明

### 1. **time** (秒)
- **含义**: 从开始记录起经过的时间
- **计算方式**: `ros::Time::now().toSec() - startTime_`
- **单位**: 秒
- **精度**: 2位小数

### 2. **uav_x, uav_y, uav_z** (米)
- **含义**: 无人机当前位置坐标
- **数据来源**: `/CERLAB/quadcopter/odom` 话题
- **计算方式**: 直接从 odometry 消息中读取
  ```cpp
  currentPos_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
  );
  ```
- **单位**: 米
- **精度**: 3位小数

### 3. **dist_min** (米)
- **含义**: 无人机到所有障碍物的**最小距离**
- **计算方式**:
  1. 调用 `/fake_detector/getDynamicObstacles` 服务获取范围内的障碍物
  2. 对每个障碍物计算距离：
     ```cpp
     // 计算到障碍物中心的距离
     Eigen::Vector3d diff = currentPos_ - obstaclePos;
     double distToCenter = diff.norm();
     
     // 减去障碍物半径，得到到表面的距离
     double obstacleRadius = obstacleSize.norm() / 2.0;
     double distToSurface = max(0.0, distToCenter - obstacleRadius);
     ```
  3. 取所有障碍物中的最小值
- **特殊值**: 
  - `999.0` = 没有障碍物在检测范围内（`detection_range = 50.0m`）
  - 或服务调用失败
- **单位**: 米
- **精度**: 2位小数

### 4. **path_length** (米)
- **含义**: 无人机从开始到当前时刻的**累计飞行路径长度**
- **计算方式**: 
  ```cpp
  // 每次收到 odometry 时更新
  Eigen::Vector3d displacement = currentPos_ - previousPos_;
  pathLength_ += displacement.norm();  // 累加位移的欧氏距离
  previousPos_ = currentPos_;
  ```
- **特点**: 
  - 累计值，只增不减
  - 从第一次收到 odometry 开始计算
- **单位**: 米
- **精度**: 2位小数

### 5. **infeasible_flag** (0/1)
- **含义**: MPC 优化求解是否失败（无解）
- **当前状态**: ⚠️ **未实现**
- **代码位置**: 
  ```cpp
  ros::Subscriber mpcInfeasibleSub_;  // 已声明但未订阅
  bool mpcInfeasible_ = false;        // 始终为 false
  ```
- **预期实现**:
  - 需要订阅 MPC 节点的无解标志话题
  - 当 MPC 优化失败时设为 1，否则为 0
- **当前值**: 始终为 `0`

### 6. **collision_flag** (0/1)
- **含义**: 是否发生碰撞
- **计算方式**:
  ```cpp
  collisionDetected_ = (minDist < collisionThreshold_);
  // collisionThreshold_ = 0.4 米（默认值）
  ```
- **判断标准**: 
  - `dist_min < 0.4m` → `collision_flag = 1`
  - `dist_min >= 0.4m` → `collision_flag = 0`
- **单位**: 标志位（0 或 1）

## 实验指标计算

根据 CSV 数据，可以计算以下实验指标：

### 1. **碰撞率 (CR - Collision Rate)**
```
CR = (碰撞次数) / (总采样次数) × 100%
```
- **碰撞次数**: `collision_flag = 1` 的次数
- **总采样次数**: CSV 文件的总行数（减去表头）

### 2. **最小距离 (Dmin - Minimum Distance)**
```
Dmin = min(dist_min)  # 整个飞行过程中的最小值
```
- **统计**: 可以计算平均值、最小值、最大值、标准差

### 3. **路径长度 (L - Path Length)**
```
L = path_length  # 最后一行的 path_length 值
```
- **含义**: 整个飞行过程的累计路径长度

### 4. **无解率 (IR - Infeasible Rate)**
```
IR = (无解次数) / (总采样次数) × 100%
```
- **当前状态**: ⚠️ 需要实现 `infeasible_flag` 的订阅

## 配置参数

配置文件: `src/Intent-MPC/flight_data_recorder/cfg/data_recorder.yaml`

```yaml
data_recorder:
  collision_threshold: 0.4   # 碰撞阈值（米）
  record_rate: 10.0           # 记录频率（Hz），即每 0.1 秒一次
  detection_range: 50.0      # 障碍物检测范围（米）
```

## 数据文件位置

CSV 文件保存在:
```
src/Intent-MPC/flight_data_recorder/flight_data_YYYYMMDD_HHMMSS.csv
```

例如: `flight_data_20260107_202459.csv`

## 数据示例

```csv
time,uav_x,uav_y,uav_z,dist_min,path_length,infeasible_flag,collision_flag
0.00,0.000,0.000,1.000,999.00,0.00,0,0
0.10,0.100,0.000,1.000,8.50,0.10,0,0
0.20,0.200,0.000,1.000,8.40,0.20,0,0
...
10.00,10.000,0.000,1.000,0.35,10.00,0,1  # 发生碰撞
```

## 注意事项

### 1. dist_min = 999 的问题
如果 `dist_min` 一直是 999，可能原因：
- 服务 `/fake_detector/getDynamicObstacles` 未启动或调用失败
- 障碍物超出检测范围（50m）
- `fake_detector_node` 未正确获取障碍物信息

### 2. infeasible_flag 未实现
当前 `infeasible_flag` 始终为 0，需要：
1. 找到 MPC 节点发布无解标志的话题
2. 在 `data_recorder_node.cpp` 中添加订阅
3. 在回调函数中更新 `mpcInfeasible_` 标志

### 3. 路径长度计算
- 基于 odometry 的位移计算，可能有累积误差
- 如果 odometry 更新频率低于记录频率，路径长度可能不准确

## 数据分析脚本示例

可以使用 Python 分析 CSV 数据：

```python
import pandas as pd
import numpy as np

# 读取 CSV
df = pd.read_csv('flight_data_20260107_202459.csv')

# 计算指标
collision_rate = (df['collision_flag'] == 1).sum() / len(df) * 100
min_distance = df['dist_min'].min()
path_length = df['path_length'].iloc[-1]  # 最后一行
infeasible_rate = (df['infeasible_flag'] == 1).sum() / len(df) * 100

print(f"碰撞率: {collision_rate:.2f}%")
print(f"最小距离: {min_distance:.2f} m")
print(f"路径长度: {path_length:.2f} m")
print(f"无解率: {infeasible_rate:.2f}%")
```

## 改进建议

1. **实现 infeasible_flag**:
   - 查找 MPC 节点的无解标志话题
   - 添加订阅并更新标志

2. **优化 dist_min 计算**:
   - 使用更精确的障碍物-无人机距离计算（考虑障碍物形状）
   - 添加调试日志确保服务调用成功

3. **添加更多指标**:
   - 速度、加速度
   - 控制输入
   - 能量消耗

