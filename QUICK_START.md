# 快速开始指南

## 准备工作

确保项目已编译：

```bash
cd /home/ff/intent-mpc
catkin_make
source devel/setup.bash
```

## 运行实验

### 终端1：启动仿真

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

等待 Gazebo 完全启动（约10秒）。

### 终端2：启动MPC导航

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

### 观察

无人机将：
1. 自动起飞到 1.0m 高度
2. 飞向目标点 (16, 0, 1.0)
3. 避开迎面而来的障碍物（O1、O2、O3）

数据自动记录到：
```
/home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/flight_data_YYYYMMDD_HHMMSS.csv
```

## 查看结果

### 方法1：查看CSV文件

```bash
cd /home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/
cat flight_data_*.csv | head -20
```

### 方法2：分析数据

```bash
cd /home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/
python3 scripts/analyze_flight_data.py flight_data_*.csv
```

## 当前配置

### 障碍物场景

文件：`test_head_on.world`

- **O1**: (-16, 0, 1.0) → (0, 0, 1.0)，速度 1.0 m/s
- **O2**: (-16, -3, 1.0) → (0, 2, 1.0)，速度 0.9 m/s
- **O3**: (-16, 3, 1.0) → (0, -2, 1.0)，速度 1.2 m/s

所有障碍物循环运动，高度z=1.0。

### 无人机任务

- **起点**: (0, 0, 1.0)
- **目标**: (-16, 0, 1.0)
- **方向**: 向西（负x方向）

### 数据记录设置

- **采样频率**: 10 Hz（每0.1秒）
- **碰撞阈值**: 0.4 m
- **记录内容**: time, uav_x, uav_y, uav_z, dist_min, path_length, infeasible_flag, collision_flag

## CSV格式示例

```csv
time,uav_x,uav_y,uav_z,dist_min,path_length,infeasible_flag,collision_flag
0.00,0.000,0.000,1.000,16.00,0.00,0,0
0.10,0.100,0.000,1.000,15.90,0.10,0,0
0.20,0.200,0.000,1.000,15.80,0.20,0,0
...
```

## 评估指标

运行分析脚本后将得到：

1. **碰撞率 (CR)**: 碰撞帧数 / 总帧数 × 100%
2. **最小距离 (Dmin)**: 飞行全程的最小障碍物距离
3. **路径长度 (L)**: 实际飞行路径长度
4. **无解率 (IR)**: MPC求解失败次数 / 总帧数 × 100%

## 故障排除

### Q: 数据记录节点没有启动

检查：
```bash
rosnode list | grep data_recorder
```

手动启动：
```bash
roslaunch flight_data_recorder data_recorder.launch
```

### Q: dist_min 全是999

**原因**: fake_detector_node没有启动

检查：
```bash
rosnode list | grep fake_detector
```

**解决**: 已在 `intent_mpc_demo.launch` 中自动启动，如果还是有问题，重新编译：
```bash
cd /home/ff/intent-mpc
catkin_make
source devel/setup.bash
```

### Q: 无人机不动

检查配置：
```bash
rosparam get /autonomous_flight/use_predefined_goal  # 应该是 true
rosparam get /autonomous_flight/predefined_goal_directory
```

### Q: 找不到CSV文件

检查：
```bash
ls -lh /home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/flight_data_*.csv
```

## 更多信息

详细说明请参阅：`EXPERIMENT_GUIDE.md`

