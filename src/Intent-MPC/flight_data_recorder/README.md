# Flight Data Recorder

用于记录无人机飞行实验数据的ROS节点。

## 功能

自动记录以下数据（每0.1秒采样一次）：

- **time**: 相对于开始的时间（秒）
- **uav_x, uav_y, uav_z**: 无人机的3D位置（米）
- **dist_min**: 到最近障碍物的最小距离（米）
- **path_length**: 累计飞行路径长度（米）
- **infeasible_flag**: MPC求解失败标志（0=正常，1=无解）
- **collision_flag**: 碰撞标志（0=无碰撞，1=碰撞）

## 使用方法

### 1. 编译

```bash
cd /home/ff/intent-mpc
catkin_make
source devel/setup.bash
```

### 2. 启动仿真和数据记录

**方式1：使用主launch文件（推荐）**
```bash
roslaunch uav_simulator start.launch
```

**方式2：单独启动数据记录节点**
```bash
# 先启动仿真
roslaunch uav_simulator start.launch

# 在另一个终端启动数据记录
roslaunch flight_data_recorder data_recorder.launch
```

### 3. 开始飞行

无人机会自动飞向目标点 (16, 0, 1.0)。

### 4. 查看数据

数据会自动保存为CSV文件，路径为：
```
/home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/flight_data_YYYYMMDD_HHMMSS.csv
```

## CSV文件格式示例

```csv
time,uav_x,uav_y,uav_z,dist_min,path_length,infeasible_flag,collision_flag
0.00,0.000,0.000,1.000,8.40,0.00,0,0
0.10,0.100,0.000,1.000,8.30,0.10,0,0
0.20,0.200,0.000,1.000,8.20,0.20,0,0
...
19.90,15.800,0.000,1.000,0.45,15.80,0,1
```

## 参数配置

在 `cfg/data_recorder.yaml` 中配置：

- `collision_threshold`: 碰撞判定阈值（米），默认0.4m
- `record_rate`: 记录频率（Hz），默认10Hz（0.1秒间隔）
- `detection_range`: 障碍物检测范围（米），默认50m

## 计算指标

### 1. 碰撞率 (CR)
```
CR = N_collision / N_total × 100%
```
其中 N_collision 是 collision_flag=1 的帧数

### 2. 最小距离 (Dmin)
```
Dmin = min(dist_min) 在整个飞行过程中
```

### 3. 路径长度 (L)
```
L = path_length 的最终值
```

### 4. 无解率 (IR)
```
IR = N_infeasible / N_total × 100%
```
其中 N_infeasible 是 infeasible_flag=1 的帧数

## Python分析脚本示例

```python
import pandas as pd
import numpy as np

# 读取CSV
df = pd.read_csv('flight_data_20260107_123456.csv')

# 计算指标
collision_rate = (df['collision_flag'].sum() / len(df)) * 100
min_distance = df['dist_min'].min()
path_length = df['path_length'].iloc[-1]
infeasible_rate = (df['infeasible_flag'].sum() / len(df)) * 100

print(f"碰撞率 (CR): {collision_rate:.2f}%")
print(f"最小距离 (Dmin): {min_distance:.2f} m")
print(f"路径长度 (L): {path_length:.2f} m")
print(f"无解率 (IR): {infeasible_rate:.2f}%")
```



