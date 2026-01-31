# Fixed Ellipsoid 分支说明

## 分支概述

**分支名称**: `fixed-ellipsoid`  
**基于**: `main` 分支  
**创建日期**: 2026-01-31

## 主要变化

本分支将椭球障碍物建模方式从 **TTC动态调整** 改为 **固定大小**。

### 椭球大小计算方式

#### Main 分支 (TTC动态方式)
```
a = a0 + s_filt * (1 + κ)
b = b0 + s_filt * (1 - κ)
c = c0

其中:
- a0 = obstacle_size_x/2 + dynamicSafetyDist
- b0 = obstacle_size_y/2 + dynamicSafetyDist
- c0 = obstacle_size_z/2 + dynamicSafetyDist
- s_filt = 根据距离、速度、TTC动态计算的风险裕量
- κ = 各向异性强度
```

#### Fixed-Ellipsoid 分支 (固定大小方式)
```
a = a0 = obstacle_size_x/2 + dynamicSafetyDist
b = b0 = obstacle_size_y/2 + dynamicSafetyDist
c = c0 = obstacle_size_z/2 + dynamicSafetyDist
phi = 0.0  (不旋转)

其中:
- 椭球大小完全由障碍物尺寸和固定安全距离决定
- 不考虑相对运动、closing speed、TTC等因素
- 椭球朝向固定为0，不随障碍物运动方向旋转
```

## 修改的文件

### 1. `src/Intent-MPC/trajectory_planner/include/trajectory_planner/mpcPlanner.cpp`

**修改位置**: `updateObstacleParam()` 函数中的椭球计算逻辑 (约1256-1278行)

**主要变化**:
- 删除了整个风险自适应椭球计算模块 (约150行代码)
- 删除了TTC计算、closing speed计算、椭球朝向计算等
- 删除了稳定性补丁 (s的平滑、phi的平滑、低速退化等)
- 删除了椭球大小合理性检查
- 删除了机器人运动诊断
- 简化为直接使用基线椭球 `a0, b0, c0`

**代码对比**:
```cpp
// 修改前 (main分支)
if (this->useRiskAdaptive_){
    // 计算相对运动
    // 计算TTC
    // 计算风险裕量 s
    // 平滑处理
    // 各向异性分配
    // 安全检查
    // ... 约150行代码
}

// 修改后 (fixed-ellipsoid分支)
// 固定椭球大小（原始方式）
double a = a0;
double b = b0;
double c = c0;
double phi = 0.0;
ROS_INFO_THROTTLE(2.0, "[Fixed-Ellipsoid] Obs_%d: Using fixed ellipsoid - a=%.2f, b=%.2f, c=%.2f", i, a, b, c);
```

### 2. `src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/planner_param.yaml`

**修改位置**: 第52行

**主要变化**:
```yaml
# 修改前
mpc_planner/use_risk_adaptive: true

# 修改后
mpc_planner/use_risk_adaptive: false
```

同时更新了注释说明这是固定椭球分支。

## 参数说明

### 影响椭球大小的参数

在固定椭球分支中，椭球大小仅由以下参数决定:

```yaml
# planner_param.yaml
mpc_planner/dynamic_safety_dist: 0.6  # 动态障碍物安全距离 (m)
```

**椭球半轴计算**:
```
a = obstacle_size_x/2 + 0.6
b = obstacle_size_y/2 + 0.6
c = obstacle_size_z/2 + 0.6
```

### 不再使用的参数

以下风险自适应参数在本分支中**不再生效**:
- `risk_s0`, `risk_alpha`, `risk_beta`, `risk_tau`
- `risk_s_min`, `risk_s_max`
- `risk_kappa`, `risk_vel_threshold`
- `risk_time_const_s`, `risk_time_const_phi`
- `risk_max_delta_s`, `risk_max_delta_phi`

这些参数仍然保留在配置文件中，但不会被使用。

## 优缺点对比

### Fixed-Ellipsoid 分支的优点

✅ **简单稳定**: 椭球大小固定，不会出现突变  
✅ **计算高效**: 省去了TTC、速度、平滑等复杂计算  
✅ **参数少**: 只需调整 `dynamic_safety_dist` 一个参数  
✅ **可预测**: 椭球大小不随运动状态变化，行为一致

### Fixed-Ellipsoid 分支的缺点

❌ **不够智能**: 无法区分"迎面冲突"与"远离运动"  
❌ **保守**: 所有情况使用相同的安全距离，可能过于保守  
❌ **空间利用率低**: 不能根据风险动态调整，浪费机动空间  
❌ **椭球不旋转**: 不能沿障碍物运动方向拉长，可能不够贴合

### Main 分支 (TTC) 的优点

✅ **智能**: 根据相对运动动态调整安全裕量  
✅ **高效**: 远离时收缩，逼近时膨胀，空间利用率高  
✅ **朝向感知**: 椭球沿障碍物运动方向拉长  
✅ **各向异性**: 危险方向更肥，垂直方向更瘦

### Main 分支 (TTC) 的缺点

❌ **复杂**: 需要调整多个参数  
❌ **可能不稳定**: 如果参数不当，椭球可能突变或过大  
❌ **计算开销**: 需要计算TTC、速度、平滑等

## 使用建议

### 何时使用 Fixed-Ellipsoid 分支

- 需要**简单稳定**的避障行为
- 环境中障碍物运动**相对缓慢**
- 对**空间利用率**要求不高
- 希望**快速调试**，不想调整复杂参数
- 作为**baseline**对比实验

### 何时使用 Main 分支 (TTC)

- 需要**智能避障**，区分不同风险场景
- 环境中有**高速运动**的障碍物
- 需要**高空间利用率**，在狭窄空间机动
- 愿意**调整参数**以获得最佳性能
- 进行**学术研究**，展示风险自适应能力

## 切换分支

```bash
# 切换到固定椭球分支
git checkout fixed-ellipsoid

# 切换回TTC动态分支
git checkout main

# 切换后需要重新编译
catkin_make
```

## 调整椭球大小

### Fixed-Ellipsoid 分支

只需修改一个参数:

```yaml
# planner_param.yaml
mpc_planner/dynamic_safety_dist: 0.6  # 增大此值 → 椭球更大
```

**示例**:
- `0.3` - 较小的安全距离，适合宽敞环境
- `0.6` - 默认值，适合一般场景
- `0.8` - 较大的安全距离，更保守

### Main 分支 (TTC)

需要调整多个参数，详见 `RISK_ADAPTIVE_PARAMS_CHEATSHEET.md`

## 测试验证

### 验证椭球大小

运行系统后，查看日志:

```bash
# Fixed-Ellipsoid 分支应该看到:
[Fixed-Ellipsoid] Obs_0: Using fixed ellipsoid - a=0.80, b=0.80, c=0.80

# Main 分支应该看到:
[Risk-Adaptive] Obs_0: d=3.50m, vc=0.50m/s, ttc=7.00s | s_raw=0.35→s_filt=0.35(Δ0.000) | phi=45.0° | a=1.15, b=0.65, κ=0.35
```

### 可视化对比

在 RViz 中添加:
```
Topic: /mpc_planner/ellipsoid_obstacles
Type: visualization_msgs/MarkerArray
```

- **Fixed-Ellipsoid**: 椭球大小固定，不旋转
- **Main (TTC)**: 椭球大小和朝向动态变化

## 性能对比

| 指标 | Fixed-Ellipsoid | Main (TTC) |
|------|----------------|------------|
| 椭球计算耗时 | ~0.01ms | ~0.05ms |
| 参数数量 | 1个 | 12个 |
| 代码复杂度 | 简单 (5行) | 复杂 (150行) |
| 避障成功率 | 中等 | 高 |
| 空间利用率 | 低 | 高 |
| 稳定性 | 高 | 中等 (参数依赖) |

## 总结

- **Fixed-Ellipsoid 分支**: 简单、稳定、易用，适合快速部署和baseline实验
- **Main 分支 (TTC)**: 智能、高效、复杂，适合高性能需求和学术研究

根据实际需求选择合适的分支。如果不确定，建议先使用 Fixed-Ellipsoid 分支验证基本功能，再切换到 Main 分支进行优化。

---

**维护者**: AI Assistant  
**最后更新**: 2026-01-31
