# 无人机避障实验指南

本指南说明如何运行无人机避障实验并收集数据，用于评估MPC导航性能。

## 实验场景

当前配置的场景：`test_head_on.world` - 三个障碍物迎面运动场景

### 障碍物设置

- **O1**: 从 (-16, 0, 0) → (0, 0, 0)，速度 1.0 m/s，循环运动
- **O2**: 从 (-16, -3, 0) → (0, 2, 0)，速度 0.9 m/s，循环运动
- **O3**: 从 (-16, 3, 0) → (0, -2, 0)，速度 1.2 m/s，循环运动

### 无人机任务

- **起点**: (0, 0, 1.0)
- **目标点**: (16, 0, 1.0)
- **任务**: 从起点飞向目标点，避开迎面而来的障碍物

## 快速开始

### 1. 编译项目

```bash
cd /home/ff/intent-mpc
catkin_make
source devel/setup.bash
```

### 2. 启动仿真（包含数据记录）

```bash
roslaunch uav_simulator start.launch
```

这将自动启动：
- Gazebo仿真环境
- 障碍物（test_head_on.world）
- 数据记录节点

### 3. 在新终端启动MPC导航

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

无人机将自动：
1. 起飞到高度 1.0m
2. 开始飞向目标点 (16, 0, 1.0)
3. 使用MPC规划避障轨迹

### 4. 监控飞行（可选）

在新终端查看实时数据：

```bash
# 查看无人机位置
rostopic echo /CERLAB/quadcopter/odom

# 查看MPC轨迹
rostopic echo /autonomous_flight/mpc_traj
```

### 5. 实验结束

当无人机到达目标点或发生碰撞后：

1. 停止所有节点：`Ctrl+C`
2. 数据已自动保存到CSV文件

## 数据分析

### CSV文件位置

```bash
cd /home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/
ls -lh flight_data_*.csv
```

### 分析单次飞行数据

```bash
cd /home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/
python3 scripts/analyze_flight_data.py flight_data_20260107_123456.csv
```

输出示例：
```
==========================================================
Analyzing: flight_data_20260107_123456.csv
==========================================================

飞行数据统计:
  总帧数: 200
  飞行时间: 20.00 秒

关键指标:
  1. 碰撞率 (CR): 0.00% (0/200 帧)
  2. 最小距离 (Dmin): 0.450 m
     平均距离: 2.340 ± 1.230 m
  3. 路径长度 (L): 16.234 m
  4. 无解率 (IR): 0.00% (0/200 帧)

额外信息:
  起点: (0.00, 0.00, 1.00)
  终点: (15.98, 0.02, 1.00)
  平均速度: 0.812 m/s
==========================================================
```

### 分析多次实验数据

运行多次实验后，分析所有数据：

```bash
python3 scripts/analyze_flight_data.py 'flight_data_*.csv'
```

输出聚合统计：
```
==========================================================
聚合统计 (所有实验)
==========================================================

总实验次数: 10

关键指标均值 ± 标准差:
  碰撞率 (CR): 5.00 ± 3.50%
  最小距离 (Dmin): 0.423 ± 0.089 m
  路径长度 (L): 16.456 ± 0.234 m
  无解率 (IR): 0.50 ± 0.30%
  平均速度: 0.823 ± 0.012 m/s

成功率 (无碰撞): 90.00% (9/10)
==========================================================
```

## 评估指标说明

### 1. 碰撞率 (Collision Rate, CR)

- **定义**: 发生碰撞的帧数占总帧数的比例
- **计算**: `CR = N_collision / N_total × 100%`
- **判定**: 当 UAV 与障碍物距离 < 0.4m 时判定为碰撞
- **期望值**: 越低越好，0% 为理想

### 2. 最小距离 (Minimum Distance, Dmin)

- **定义**: 飞行全程中 UAV 与障碍物的最小距离
- **计算**: `Dmin = min(dist_min)` 在整个飞行过程中
- **安全阈值**: > 0.4m
- **期望值**: 越大越好，表示避障更安全

### 3. 路径长度 (Path Length, L)

- **定义**: UAV 实际飞行的路径总长度
- **计算**: `L = Σ ||p_i - p_{i-1}||`
- **理论最短**: 16.0m（直线距离）
- **期望值**: 接近理论值，表示路径更高效

### 4. 无解率 (Infeasible Rate, IR)

- **定义**: MPC 优化求解失败的比例
- **计算**: `IR = N_infeasible / N_total × 100%`
- **期望值**: 越低越好，0% 为理想

## 修改实验配置

### 改变目标点

编辑文件：`src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/ref_trajectory.txt`

```
# 格式：x y z yaw
16.0 0.0 1.0 0.0
```

### 改变障碍物场景

编辑文件：`src/Intent-MPC/uav_simulator/launch/start.launch`

```xml
<!-- 修改这一行 -->
<arg name="world_name" value="$(find uav_simulator)/worlds/test/test_head_on.world" />
```

可选场景：
- `test_keep_some.world` - 保持某些障碍物
- `test_turn.world` - 急转弯场景
- `test_stop.world` - 急停场景
- `test_A_star.world` - A*前瞻性场景

### 修改数据记录参数

编辑文件：`src/Intent-MPC/flight_data_recorder/cfg/data_recorder.yaml`

```yaml
data_recorder:
  collision_threshold: 0.4  # 碰撞判定阈值（米）
  record_rate: 10.0         # 采样频率（Hz）
  detection_range: 50.0     # 障碍物检测范围（米）
```

## 批量实验脚本

创建一个脚本 `run_experiments.sh` 来自动运行多次实验：

```bash
#!/bin/bash

# 运行10次实验
for i in {1..10}
do
    echo "======================================"
    echo "Starting experiment $i/10"
    echo "======================================"
    
    # 启动仿真
    roslaunch uav_simulator start.launch &
    GAZEBO_PID=$!
    sleep 10  # 等待Gazebo启动
    
    # 启动MPC导航
    timeout 30s roslaunch autonomous_flight intent_mpc_demo.launch
    
    # 停止仿真
    kill $GAZEBO_PID
    sleep 5
    
    echo "Experiment $i completed"
done

echo "======================================"
echo "All experiments completed!"
echo "Analyzing results..."
python3 /home/ff/intent-mpc/src/Intent-MPC/flight_data_recorder/scripts/analyze_flight_data.py 'flight_data_*.csv'
```

## 可视化（可选）

启动RViz查看实时飞行：

```bash
roslaunch remote_control mpc_navigation_rviz.launch
```

## 故障排除

### 问题1: 数据记录节点未启动

**症状**: 没有生成CSV文件

**解决**:
```bash
# 检查节点是否运行
rosnode list | grep data_recorder

# 手动启动
roslaunch flight_data_recorder data_recorder.launch
```

### 问题2: 无人机不飞

**症状**: 无人机保持在地面

**解决**:
```bash
# 检查MPC导航节点
rosnode list | grep mpc_navigation

# 检查目标点是否设置
rosparam get /autonomous_flight/use_predefined_goal
```

### 问题3: Gazebo启动失败

**解决**:
```bash
# 清理Gazebo
killall gzserver gzclient
rm -rf /tmp/gazebo*

# 重新启动
roslaunch uav_simulator start.launch
```

## 实验结果示例

成功的实验结果应该显示：

- ✅ 碰撞率 (CR): 0%
- ✅ 最小距离 (Dmin): > 0.4m
- ✅ 路径长度 (L): ~16-18m
- ✅ 无解率 (IR): < 1%

## 联系与反馈

如有问题，请检查日志文件或联系开发者。



