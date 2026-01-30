# RiskMap25D - 2.5D融合风险地图

## 📋 概述

`RiskMap25D` 是一个2.5D融合风险地图模块，用于运动规划中的风险评估。它结合了：

1. **静态几何风险**：基于3D ESDF（欧氏符号距离场）
2. **动态障碍物风险**：基于多意图概率加权的预测轨迹

## 🎯 核心特性

### 1. 2.5D结构

- **高度层离散化**：Z轴分为多个离散层（如0m, 0.5m, 1.0m, 1.5m, 2.0m）
- **每层独立2D栅格**：每层维护一个2D风险栅格
- **高效查询**：查询时选择最近的层或在相邻层之间插值

### 2. 静态风险

基于ESDF的二次惩罚：

```
R_s(q) = max(0, d_s - d(q))²
```

其中：
- `d(q)` 是从ESDF查询的到最近障碍物的距离
- `d_s` 是安全距离阈值（如0.5m）

### 3. 动态风险

多意图概率加权 + 时间衰减 + 高阶椭圆核：

```
R_d(q) = Σ_i Σ_I P_i(I) * Σ_k ω_k * exp(-(m_k)^β)
```

其中：
- `i` 遍历所有动态障碍物
- `I` 遍历4种意图：FORWARD, LEFT, RIGHT, STOP
- `P_i(I)` 是意图后验概率（和为1）
- `k` 遍历预测步（如30步，3秒）
- `ω_k` 是时间权重（指数衰减，近期权重高）
- `m_k` 是Mahalanobis距离平方：`m = (dx/σx)² + (dy/σy)²`
- `β` 是椭圆核指数（默认1.0）

### 4. 融合

```
R(q) = γ_s * R_s(q) + γ_d * R_d(q)
```

## 📦 文件结构

```
global_planner/
├── include/global_planner/
│   └── risk_map_25d.h          # 头文件
├── src/
│   ├── risk_map_25d.cpp        # 实现
│   └── risk_map_25d_visualizer.cpp  # 可视化节点
├── test/
│   ├── test_risk_map_25d.cpp   # 单元测试
│   └── test_risk_map_25d.test  # 测试launch文件
└── CMakeLists.txt
```

## ⚙️ 配置参数

### 栅格参数

```yaml
risk_map_25d/resolution: 0.1        # 栅格分辨率 (m)
risk_map_25d/grid_width: 200        # 栅格宽度（格子数）
risk_map_25d/grid_height: 200       # 栅格高度（格子数）
```

### 高度层参数

```yaml
risk_map_25d/layer_min: 0.0         # 最低层高度 (m)
risk_map_25d/layer_max: 5.0         # 最高层高度 (m)
risk_map_25d/layer_thickness: 0.5   # 层厚度 (m)
```

**结果**：生成11层，高度为0.0, 0.5, 1.0, ..., 5.0m

### 静态风险参数

```yaml
risk_map_25d/d_s: 0.5               # 安全距离 (m)
risk_map_25d/gamma_s: 1.0           # 静态风险权重
```

### 动态风险参数

```yaml
risk_map_25d/gamma_d: 1.0           # 动态风险权重
risk_map_25d/beta: 1.0              # 椭圆核指数
risk_map_25d/num_pred_steps: 30     # 预测步数
risk_map_25d/lambda_time: 0.1       # 时间衰减系数
```

**时间权重**：自动计算为 `ω[k] = exp(-λ * k)` 然后归一化

### 数值稳定性参数

```yaml
risk_map_25d/sigma_min: 0.1         # 最小标准差 (m)
risk_map_25d/m_cut: 9.0             # Mahalanobis距离截断阈值（3-sigma）
risk_map_25d/risk_epsilon: 1e-4     # 最小风险阈值（光栅化截断）
```

## 🔧 使用方法

### 1. 创建RiskMap25D对象

```cpp
#include "global_planner/risk_map_25d.h"

ros::NodeHandle nh("~");
auto risk_map = std::make_shared<globalPlanner::RiskMap25D>(nh);

// 设置地图中心（通常是机器人位置）
risk_map->setMapCenter(Eigen::Vector3d(robot_x, robot_y, robot_z));
```

### 2. 更新静态风险

```cpp
// 假设已有ESDF地图
std::shared_ptr<mapManager::ESDFMap> esdf_map = ...;

risk_map->updateStatic(esdf_map);
```

### 3. 更新动态风险

```cpp
// 构造动态预测数据
globalPlanner::DynamicPredictions predictions;
predictions.timestamp = ros::Time::now();

// 添加障碍物
globalPlanner::DynamicObstacle obs;
obs.obstacle_id = 0;
obs.intent_probs = {0.6, 0.2, 0.15, 0.05};  // FORWARD, LEFT, RIGHT, STOP

// 为每个意图添加轨迹
for (int intent = 0; intent < 4; ++intent) {
    globalPlanner::DynamicIntentTrajectory traj;
    
    for (int k = 0; k < 30; ++k) {
        globalPlanner::DynamicPredictionStep step;
        step.mu = Eigen::Vector2d(x_pred[k], y_pred[k]);
        step.sigma_x = sigma_x[k];
        step.sigma_y = sigma_y[k];
        step.height_min = z_min;
        step.height_max = z_max;
        
        traj.steps.push_back(step);
    }
    
    obs.intent_trajs.push_back(traj);
}

predictions.obstacles.push_back(obs);

// 更新风险地图
risk_map->updateDynamic(predictions);
```

### 4. 查询风险

```cpp
// 查询某个3D点的风险值
Eigen::Vector3d query_point(x, y, z);
double risk = risk_map->query(query_point);

// risk >= 0.0，值越大风险越高
```

## 📊 可视化

### 运行可视化节点

```bash
rosrun global_planner risk_map_25d_visualizer _visualize_layer_idx:=2
```

参数：
- `visualize_layer_idx`: 可视化的层索引（默认2，即z=1.0m层）
- `publish_rate`: 发布频率（Hz，默认10）

### 查看可视化

```bash
rosrun rviz rviz
```

添加以下显示：
- `nav_msgs/OccupancyGrid` 话题: `/risk_map_25d/grid`
- `visualization_msgs/MarkerArray` 话题: `/risk_map_25d/markers`

**颜色说明**：
- 白色（0）：无风险
- 灰色到黑色：风险逐渐增大
- 黑色（100）：最高风险

## 🧪 单元测试

### 运行测试

```bash
cd /home/ff/intent-mpc
catkin_make run_tests_global_planner_rostest_test_test_risk_map_25d.test
```

### 测试覆盖

- ✅ 初始化检查
- ✅ 动态核函数（衰减特性）
- ✅ 多意图混合（概率加权）
- ✅ 高度层选择
- ✅ 时间衰减
- ✅ 导出层数据

## 🔬 技术细节

### 1. 椭圆边界框优化

为避免遍历整个栅格，使用椭圆边界框：

```
m = (dx/σx)² + (dy/σy)² <= m_cut
=> |dx| <= sqrt(m_cut) * σx
=> |dy| <= sqrt(m_cut) * σy
```

只在边界框内计算动态核。

### 2. 高度范围处理

障碍物有高度范围`[z_min, z_max]`，风险只影响高度在此范围内（或接近）的层：

```cpp
if (layer_z >= z_min - layer_thickness &&
    layer_z <= z_max + layer_thickness) {
    affected_layers.push_back(layer);
}
```

### 3. 双线性插值

查询风险时使用双线性插值，提高精度：

```
v(x,y) = v00*(1-fx)*(1-fy) + v10*fx*(1-fy) + v01*(1-fx)*fy + v11*fx*fy
```

## 📈 性能

### 典型参数下的性能（单线程）

- **栅格大小**：200x200x5层
- **动态障碍物**：3个
- **预测步数**：30步
- **意图数**：4个

**结果**：
- updateDynamic：~5-10ms
- query（单次）：~0.01ms（双线性插值）
- 内存占用：~2MB（栅格数据）

### 优化建议

1. **减少栅格分辨率**：0.2m而不是0.1m → 4倍速度提升
2. **减少预测步数**：20步而不是30步 → 1.5倍速度提升
3. **增大m_cut阈值**：但会降低精度
4. **多线程**：并行处理不同障碍物或不同层

## 🔧 与现有系统集成

### 集成到A*规划器

```cpp
// 在a_star_occ.cpp中
#include "global_planner/risk_map_25d.h"

// 创建RiskMap25D（替代RiskMap2D）
risk_map_25d_ = std::make_shared<globalPlanner::RiskMap25D>(nh);

// 在A*扩展节点时查询风险
Eigen::Vector3d neighbor_world = indexToPos(nx, ny, nz);
double risk = risk_map_25d_->query(neighbor_world);
double risk_cost = w_risk_ * risk;  // 直接使用风险值作为代价
```

**注意**：不需要`liftRiskGated`和`riskToCostLog`，因为：
- RiskMap25D已经处理了3D（多层）
- 风险值可以直接作为代价（已融合静态+动态）

## 📚 参考文档

1. **理论基础**：
   - Intent-MPC论文中的动态障碍物预测模型
   - ESDF地图原理

2. **相关文件**：
   - `TRAJECTORY_SAMPLING_ENV_INTERACTION.md` - 轨迹采样与环境交互
   - `ROBUST_INTENT_INERTIA_V3.md` - 自适应意图惯性机制
   - `ASTAR_RISK_FIXES.md` - A*与风险地图集成

## 🐛 常见问题

### Q1: 风险地图全是0

**可能原因**：
1. 没有调用`updateDynamic`
2. 动态预测数据为空
3. 查询点高度不在任何层附近

**解决**：
- 检查`dynamic_updated_`标志
- 打印predictions.obstacles.size()
- 检查查询点的z坐标

### Q2: 编译错误：找不到ESDFMap

**原因**：缺少map_manager依赖

**解决**：
```xml
<!-- package.xml -->
<depend>map_manager</depend>
```

### Q3: 性能太慢

**优化**：
- 增大resolution（0.1 -> 0.2）
- 减少num_pred_steps（30 -> 20）
- 增大risk_epsilon（1e-4 -> 1e-3）
- 增大m_cut（9.0 -> 6.0，但会截断更多）

## 📝 TODO

- [ ] 支持高度层之间的线性插值
- [ ] 添加静态风险的预计算缓存
- [ ] 支持异步更新（多线程）
- [ ] 添加ROS话题订阅（自动从dynamic_predictor获取数据）

---

**版本**：1.0  
**分支**：`feature/risk-map-25d`  
**日期**：2026-01-15
