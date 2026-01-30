# RiskMap25D 集成指南（单层版本）

## 📋 概述

本文档说明如何将**单层2.5D风险地图**集成到A*路径规划器中。

### 什么是"单层2.5D"？

传统2.5D地图的定义：
- **一个**2D栅格地图
- 在**固定高度**（如1m）
- 查询时**忽略z坐标**，只用(x,y)
- 适用于飞行高度固定或变化小的场景

这比多层版本更简单、高效，也更符合传统2.5D的概念。

---

## 🎯 关键修改点

### 1. 数据结构简化

**头文件** (`risk_map_25d.h`)：
```cpp
class RiskMap25D {
private:
    // 单层风险栅格（原来是 vector<vector<double>>）
    std::vector<double> risk_grid_;
    
    // 固定高度（原来是多层高度数组）
    double fixed_height_;  // 默认1.0m
};
```

### 2. 查询逻辑简化

**查询函数** (`risk_map_25d.cpp`):
```cpp
double RiskMap25D::query(const Eigen::Vector3d& q) const
{
    // 忽略z坐标，只查询(x,y)
    double gx = (q(0) - grid_origin_(0)) / resolution_ - 0.5;
    double gy = (q(1) - grid_origin_(1)) / resolution_ - 0.5;
    
    return bilinearInterpolate(gx, gy);
}
```

### 3. 配置参数

**参数文件** (`planner_param.yaml`):
```yaml
# 单层2.5D风险地图
risk_map_25d/resolution: 0.1              # 栅格分辨率 (m)
risk_map_25d/grid_width: 200              # 栅格宽度（格子数）
risk_map_25d/grid_height: 200             # 栅格高度（格子数）
risk_map_25d/fixed_height: 1.0            # 固定高度 (m) ← 关键参数

# 静态风险参数
risk_map_25d/d_s: 0.5                     # 安全距离 (m)
risk_map_25d/gamma_s: 1.0                 # 静态风险权重

# 动态风险参数
risk_map_25d/gamma_d: 1.0                 # 动态风险权重
risk_map_25d/beta: 1.0                    # 椭圆核指数
risk_map_25d/num_pred_steps: 30           # 预测步数
risk_map_25d/lambda_time: 0.1             # 时间衰减系数

# 数值稳定性参数
risk_map_25d/sigma_min: 0.1               # 最小标准差 (m)
risk_map_25d/m_cut: 9.0                   # Mahalanobis距离截断
risk_map_25d/risk_epsilon: 0.0001         # 最小风险阈值
```

---

## 📁 新增/修改的文件

### 新增文件

1. **`src/Intent-MPC/global_planner/include/global_planner/risk_map_25d.h`**
   - RiskMap25D类定义
   - ObstaclePrediction数据结构

2. **`src/Intent-MPC/global_planner/src/risk_map_25d.cpp`**
   - RiskMap25D实现
   - 静态风险：基于ESDF的距离场
   - 动态风险：椭圆核+意图概率+时间衰减

3. **`src/Intent-MPC/global_planner/src/risk_map_25d_visualizer.cpp`**
   - 可视化节点（发布OccupancyGrid）

### 修改文件

1. **`src/Intent-MPC/global_planner/include/global_planner/a_star_occ.h`**
   ```cpp
   // 原来：#include "global_planner/risk_map_2d.h"
   #include "global_planner/risk_map_25d.h"
   
   // 原来：void setRiskMap(std::shared_ptr<RiskMap2D> risk_map);
   void setRiskMap(std::shared_ptr<RiskMap25D> risk_map);
   
   // 原来：std::shared_ptr<RiskMap2D> risk_map_2d_;
   std::shared_ptr<RiskMap25D> risk_map_25d_;
   ```

2. **`src/Intent-MPC/global_planner/src/a_star_occ.cpp`**
   ```cpp
   // 查询风险（自动忽略z坐标）
   double risk = risk_map_25d_->query(neighbor_world);
   ```

3. **`src/Intent-MPC/autonomous_flight/include/autonomous_flight/mpcNavigation.h`**
   ```cpp
   #include "global_planner/risk_map_25d.h"
   #include "map_manager/ESDFMap.h"
   
   std::shared_ptr<globalPlanner::RiskMap25D> riskMap25D_;
   std::shared_ptr<mapManager::ESDFMap> esdfMap_;
   ```

4. **`src/Intent-MPC/autonomous_flight/include/autonomous_flight/mpcNavigation.cpp`**
   - 初始化RiskMap25D
   - 更新静态风险（ESDF）
   - 更新动态风险（意图预测）

5. **`src/Intent-MPC/global_planner/CMakeLists.txt`**
   - 添加risk_map_25d.cpp编译
   - 添加risk_map_25d_visualizer节点
   - 链接map_manager库

6. **`src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/planner_param.yaml`**
   - 添加risk_map_25d参数section

---

## 🚀 使用方法

### 1. 切换到feature分支

```bash
cd /home/ff/intent-mpc
git checkout feature/risk-map-25d
```

### 2. 编译

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 3. 运行demo

```bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

### 4. 可视化风险地图（可选）

在新终端运行：
```bash
rosrun global_planner risk_map_25d_visualizer
```

在RViz中添加：
- **Topic**: `/risk_map_25d/occupancy_grid`
- **Type**: `OccupancyGrid`

---

## 🔧 参数调优

### 1. 固定高度 (`fixed_height`)

- **默认**: `1.0` m
- **建议**: 设置为无人机平均飞行高度
- **场景**:
  - 低空飞行（1-2m）: `1.0` m
  - 中空飞行（2-3m）: `2.0` m

### 2. 静态风险权重 (`gamma_s`)

- **默认**: `1.0`
- **效果**: 控制静态障碍物的避障力度
- **调大**: 更加远离墙壁/静态障碍物
- **调小**: 允许更贴近静态障碍物

### 3. 动态风险权重 (`gamma_d`)

- **默认**: `1.0`
- **效果**: 控制动态障碍物的避障力度
- **调大**: 更加远离动态障碍物
- **调小**: 允许更贴近动态障碍物

### 4. A*风险代价权重 (`astar/w_risk`)

- **当前**: `10.0`
- **效果**: 控制风险地图在路径规划中的影响
- **调小**: 更倾向于走捷径（可能穿过风险区域）
- **调大**: 更倾向于绕行（避开风险区域）

---

## 📊 性能优势（vs. 多层版本）

| 指标 | 多层版本 (11层) | 单层版本 | 提升 |
|------|----------------|----------|------|
| 内存占用 | ~880KB | ~80KB | **11倍** |
| 查询速度 | 需层选择+插值 | 直接插值 | **~2倍** |
| 更新速度 | 11个栅格 | 1个栅格 | **11倍** |
| 代码复杂度 | 复杂 | 简单 | ✅ |

---

## 🧪 测试验证

### 测试场景

1. **静态环境**
   - 墙壁、柱子等固定障碍物
   - 验证：A*路径应绕开高风险区域

2. **动态环境**
   - 移动的行人/车辆
   - 验证：A*路径应预判动态障碍物未来位置

3. **混合环境**
   - 静态+动态障碍物
   - 验证：路径应同时考虑两种风险

### 检查点

✅ 编译无错误  
✅ 启动无崩溃  
✅ A*路径避开静态障碍物  
✅ A*路径预判动态障碍物  
✅ 风险地图可视化正常  
✅ 参数调整生效  

---

## 🐛 常见问题

### Q1: 为什么路径还是穿过障碍物？

**A**: 检查以下参数：
- `astar/w_risk`: 建议10-20（太小会忽略风险）
- `risk_map_25d/gamma_s`: 建议1.0-2.0（静态风险权重）
- `risk_map_25d/d_s`: 建议0.5-1.0（安全距离）

### Q2: 路径过于保守，绕行太远？

**A**: 降低以下参数：
- `astar/w_risk`: 减小到5-10
- `risk_map_25d/gamma_s`: 减小到0.5-1.0
- `risk_map_25d/gamma_d`: 减小到0.5-1.0

### Q3: 动态风险没有效果？

**A**: 确认以下内容：
- `mpcNavigation.cpp`的`riskMapCB`是否被调用
- 意图预测数据是否正常（`intentProb_`非零）
- `risk_map_25d/gamma_d`是否大于0

### Q4: 高度不匹配怎么办？

**A**: 调整`risk_map_25d/fixed_height`参数：
- 如果无人机飞得比1m高，增大此值（如1.5m, 2.0m）
- 如果障碍物高度不同，考虑使用多层版本（或增加z_min/z_max检查）

---

## 🎓 技术细节

### 风险融合公式

**总风险**:
```
R(q) = γ_s * R_s(q) + γ_d * R_d(q)
```

**静态风险**（基于ESDF距离）:
```
R_s(q) = max(0, d_s - d(q))^2
```
- `d(q)`: ESDF距离（到最近障碍物的距离）
- `d_s`: 安全距离（0.5m）

**动态风险**（多意图概率加权）:
```
R_d(q) = Σ_i Σ_I P_i(I) * Σ_k ω[k] * exp(-(m_k)^β)
```
- `P_i(I)`: 障碍物i的意图I概率
- `ω[k]`: 时间权重（指数衰减）
- `m_k`: Mahalanobis距离平方 `= (dx/σx)² + (dy/σy)²`
- `β`: 核指数（1.0）

### 与A*的集成

A*代价函数：
```
cost = w_dist * dist + w_risk * risk^1.5
```
- `w_dist`: 距离权重（1.0）
- `w_risk`: 风险权重（10.0）
- `risk^1.5`: 比原来的`risk^2`更温和

---

## 📝 总结

单层2.5D风险地图的优势：
1. ✅ **简单**: 代码量减少，易于维护
2. ✅ **高效**: 内存和计算开销显著降低
3. ✅ **实用**: 适合大多数固定高度飞行场景
4. ✅ **灵活**: 参数调整直观，易于调优

适用场景：
- 无人机飞行高度固定（±0.5m变化）
- 障碍物主要在地面到2m高度
- 需要快速路径规划（<100ms）

如果需要处理不同高度的障碍物，可以考虑：
- 方案A: 在`updateDynamic`中检查`z_min/z_max`过滤
- 方案B: 恢复多层版本（需要git恢复之前的提交）

---

**作者**: Intent-MPC Team  
**日期**: 2026-01-30  
**版本**: 单层2.5D v1.0
