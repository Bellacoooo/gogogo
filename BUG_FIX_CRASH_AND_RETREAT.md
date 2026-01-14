# 🚨 崩溃和倒退问题修复 v1.4

## 修复日期
2026-01-13

## 问题描述

用户报告：
1. **无人机倒退飞行**：在规划时找不到可行路径会往后退
2. **崩溃**：第二次运行时无人机倒退后崩溃
3. **可视化无效果**：椭球可视化看起来创新没有生效

## 根本原因分析

### 1. 椭球过大导致 MPC 无可行解

**原因**：
- 原始参数设置过于激进（`s_max=1.2m`, `alpha=0.4`, `beta=0.6`, `kappa=0.35`）
- 当障碍物接近时，风险自适应计算出的椭球半径可能超过机器人到障碍物的距离
- MPC 约束求解器发现所有前进路径都会穿过椭球（违反约束）
- 作为 fallback，MPC 选择倒退以满足约束

**倒退的逻辑链**：
```
障碍物接近 → s 增大 → 椭球半径 a/b 增大 → 椭球覆盖前方所有空间 
→ MPC 无法找到前进路径 → 选择倒退 → 持续倒退导致失控 → 崩溃
```

### 2. 缺少数值安全保护

**问题**：
- 没有检查距离过近的情况（d < 0.1m）
- 没有限制 TTC 范围，可能导致 `exp(-ttc/tau)` 溢出或下溢
- 没有检查 NaN/Inf
- 没有限制椭球的最大尺寸

### 3. 缺少实时诊断

**问题**：
- 无法在运行时发现椭球过大
- 无法检测倒退行为
- 难以调试问题根源

---

## 修复方案

### 修复 A：降低参数激进度

**文件**：`planner_param.yaml`

**修改**：

| 参数 | 原值 | 新值 | 说明 |
|------|------|------|------|
| `risk_s0` | 0.15 | **0.10** | 降低基线膨胀 |
| `risk_alpha` | 0.4 | **0.25** | 降低速度影响 |
| `risk_beta` | 0.6 | **0.4** | 降低 TTC 影响 |
| `risk_tau` | 1.5 | **2.0** | 增大衰减时间（更缓和） |
| `risk_s_max` | 1.2 | **0.8** | 降低最大膨胀 ⚠️ **关键** |
| `risk_kappa` | 0.35 | **0.20** | 降低各向异性强度 |
| `risk_time_const_s` | 0.5 | **0.8** | 增大时间常数（更平滑） |
| `risk_time_const_phi` | 0.5 | **0.8** | 增大时间常数（更平滑） |
| `risk_max_delta_s` | 0.3 | **0.2** | 降低最大变化率 |
| `risk_max_delta_phi` | 30° | **20°** | 降低最大变化率 |

**效果**：
- 椭球最大半径从 `a_max ≈ 0.5 + 1.2×(1+0.35) = 2.12m` 降低到 `a_max ≈ 0.5 + 0.8×(1+0.2) = 1.46m`
- 降低约 **31%**，显著减少椭球过大的风险

---

### 修复 B：添加数值安全保护

**文件**：`mpcPlanner.cpp`

#### B.1 距离过近保护
```cpp
// 安全检查：距离太近时使用保守策略
if (d < 0.1){
    ROS_WARN_THROTTLE(1.0, "[Risk-Adaptive] Obs_%d: distance too close (%.3fm), using conservative ellipsoid", i, d);
    // 使用最大膨胀作为保守策略
    double s_conservative = this->riskSMax_;
    a = a0 + s_conservative * (1.0 + this->riskKappa_);
    b = b0 + s_conservative * (1.0 - this->riskKappa_);
    c = c0;
    phi = 0.0;
    continue;  // 跳过正常计算
}
```

**原理**：当距离 < 10cm 时，相对运动计算不再可靠，直接使用固定的保守椭球。

#### B.2 TTC 范围限制
```cpp
// TTC 限制：避免过小的值导致 exp 爆炸
double ttc = d / (vc + eps);
ttc = std::max(ttc, 0.1);   // TTC 最小 0.1s
ttc = std::min(ttc, 100.0);  // TTC 最大 100s（避免 exp 下溢）
```

**原理**：
- `ttc < 0.1s`：说明碰撞即将发生，`exp(-ttc/tau)` 会接近 1
- `ttc > 100s`：说明碰撞遥远，`exp(-ttc/tau)` 会接近 0（下溢）
- 限制范围避免数值问题

#### B.3 Exp 项检查
```cpp
double exp_term = std::exp(-ttc / this->riskTau_);
// 检查 exp 是否有效
if (std::isnan(exp_term) || std::isinf(exp_term)){
    exp_term = 0.0;
    ROS_WARN_THROTTLE(1.0, "[Risk-Adaptive] Obs_%d: exp term invalid, set to 0", i);
}
```

#### B.4 椭球大小合理性检查
```cpp
const double MAX_ELLIPSOID_AXIS = 3.0;  // 椭球半轴不超过 3m

if (a > MAX_ELLIPSOID_AXIS || b > MAX_ELLIPSOID_AXIS || c > MAX_ELLIPSOID_AXIS){
    ROS_WARN_THROTTLE(1.0, "[Risk-Adaptive] Obs_%d: Ellipsoid too large! a=%.2f, b=%.2f, c=%.2f. Clamping to %.2fm", 
                      i, a, b, c, MAX_ELLIPSOID_AXIS);
    a = std::min(a, MAX_ELLIPSOID_AXIS);
    b = std::min(b, MAX_ELLIPSOID_AXIS);
    c = std::min(c, MAX_ELLIPSOID_AXIS);
}
```

**原理**：3m 是一个合理的上限，超过这个值椭球会过于保守，MPC 难以找到可行解。

#### B.5 NaN/Inf 检查
```cpp
if (std::isnan(a) || std::isinf(a) || std::isnan(b) || std::isinf(b) || 
    std::isnan(c) || std::isinf(c) || std::isnan(phi) || std::isinf(phi)){
    ROS_ERROR_THROTTLE(1.0, "[Risk-Adaptive] Obs_%d: NaN/Inf detected! Using baseline ellipsoid", i);
    a = a0;
    b = b0;
    c = c0;
    phi = 0.0;
}
```

#### B.6 椭球退化检查
```cpp
if (a < a0 * 0.5 || b < b0 * 0.5){
    ROS_WARN_THROTTLE(1.0, "[Risk-Adaptive] Obs_%d: Ellipsoid too small, using baseline", i);
    a = a0;
    b = b0;
}
```

**原理**：椭球不应该比基线小太多，否则说明计算有问题。

---

### 修复 C：添加实时诊断

#### C.1 椭球过大预警
```cpp
// 如果椭球比距离还大，MPC 可能找不到前进路径而倒退
double max_radius = std::max({a, b, c});
Eigen::Vector2d pi(pi3(0), pi3(1));
double dist_xy = (pr - pi).norm();

if (max_radius > dist_xy * 0.7){
    ROS_WARN_THROTTLE(0.5, "[Risk-Adaptive] ⚠️ Obs_%d: Ellipsoid TOO LARGE! "
                           "max_radius=%.2fm, dist=%.2fm (%.0f%%) - MPC may RETREAT!", 
                           i, max_radius, dist_xy, (max_radius/dist_xy)*100.0);
}
```

**触发条件**：椭球半径 > 70% 距离
**作用**：提前预警可能导致倒退的情况

#### C.2 倒退行为检测
```cpp
if (this->useRiskAdaptive_ && vr.x() < -0.1){
    ROS_WARN_THROTTLE(0.5, "[Risk-Adaptive] ⚠️ Robot RETREATING! vr=(%.2f, %.2f) m/s", vr.x(), vr.y());
}
```

**触发条件**：x 方向速度 < -0.1 m/s
**作用**：实时检测倒退行为

#### C.3 紧急禁用开关说明
在 `planner_param.yaml` 中添加注释：
```yaml
# 🚨 紧急禁用：如果遇到倒退、崩溃或椭球过大问题，将下面改为 false
mpc_planner/use_risk_adaptive: true
```

---

## 预期效果

### 1. 椭球大小更合理
- 最大膨胀从 1.2m 降低到 0.8m
- 各向异性强度从 0.35 降低到 0.20
- **结果**：椭球不会过度膨胀，MPC 有更大概率找到前进路径

### 2. 数值稳定性提升
- 所有计算都有边界检查和 fallback
- **结果**：不会因为 NaN/Inf 导致崩溃

### 3. 诊断信息更丰富
- 实时显示椭球大小与距离的比例
- 实时检测倒退行为
- **结果**：更容易发现和调试问题

### 4. 相对速度膨胀效果得到保留
- 虽然降低了参数，但核心逻辑（相对速度方向膨胀）没有改变
- **结果**：在 head-on 场景下，仍然会在相对速度方向有更大的膨胀（只是幅度更温和）

---

## 测试建议

### 测试步骤
```bash
cd ~/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight simulation.launch world:=test_head_on
```

### 观察重点

#### 1. 日志关键词
- ✅ 正常：`[Risk-Adaptive] Obs_0: d=X.XXm, vc=X.XXm/s, ttc=X.XXs | ... | a=X.XX, b=X.XX`
- ⚠️ 预警：`Ellipsoid TOO LARGE! max_radius=X.XXm, dist=X.XXm (XX%)`
- 🚨 危险：`Robot RETREATING! vr=(X.XX, X.XX) m/s`

#### 2. RViz 可视化
- 椭球大小应该**随距离和相对速度动态变化**
- 椭球不应该过大（半径不应超过距离的 70%）
- Head-on 时椭球应该在障碍物运动方向变长（但幅度温和）

#### 3. 无人机行为
- ✅ 应该：提前避让，绕过障碍物
- ❌ 不应该：倒退、卡住、崩溃

---

## 如果仍然出现问题

### 紧急禁用风险自适应
编辑 `planner_param.yaml`：
```yaml
mpc_planner/use_risk_adaptive: false  # 改为 false
```
然后重新测试，确认是否是风险自适应导致的问题。

### 进一步降低参数
如果椭球仍然过大：
```yaml
mpc_planner/risk_s_max: 0.5    # 从 0.8 降低到 0.5
mpc_planner/risk_kappa: 0.10   # 从 0.20 降低到 0.10（更接近圆形）
```

### 增大安全距离阈值
修改 `mpcPlanner.cpp` 中的预警阈值：
```cpp
if (max_radius > dist_xy * 0.5){  // 从 0.7 改为 0.5，更早预警
```

---

## 文件变更清单

1. **mpcPlanner.cpp**
   - 添加距离过近保护
   - 添加 TTC 范围限制
   - 添加 exp 项检查
   - 添加椭球大小合理性检查
   - 添加 NaN/Inf 检查
   - 添加椭球退化检查
   - 添加椭球过大预警
   - 添加倒退行为检测

2. **planner_param.yaml**
   - 降低 `risk_s0`：0.15 → 0.10
   - 降低 `risk_alpha`：0.4 → 0.25
   - 降低 `risk_beta`：0.6 → 0.4
   - 增大 `risk_tau`：1.5 → 2.0
   - 降低 `risk_s_max`：1.2 → 0.8 ⚠️
   - 降低 `risk_kappa`：0.35 → 0.20
   - 增大 `risk_time_const_s`：0.5 → 0.8
   - 增大 `risk_time_const_phi`：0.5 → 0.8
   - 降低 `risk_max_delta_s`：0.3 → 0.2
   - 降低 `risk_max_delta_phi`：30° → 20°
   - 添加紧急禁用开关说明

---

## 技术总结

### 核心问题
风险自适应椭球的"自适应"本质是**动态约束收紧**，如果收紧过度会导致 MPC 优化问题不可行。

### 解决方案哲学
1. **参数保守化**：宁可椭球小一点，也不要大到让 MPC 无解
2. **数值鲁棒性**：所有可能出错的地方都加边界检查
3. **可观测性**：实时输出诊断信息，方便调试

### 风险自适应 vs 固定椭球的权衡
- **固定椭球**：简单、稳定，但过于保守（远距离也保持大椭球）或过于激进（近距离仍用小椭球）
- **风险自适应**：理论上最优，但参数敏感、需要大量调试
- **当前策略**：使用风险自适应，但参数保守化 + 完善的保护机制

---

## 后续优化方向

1. **自适应参数调节**：根据 MPC 求解状态（是否可行）动态调整 `s_max`
2. **距离分段策略**：近距离用固定椭球（保守），远距离用风险自适应
3. **学习最优参数**：通过仿真实验或强化学习找到最优参数组合

---

**修复版本**：v1.4  
**编译状态**：✅ 通过  
**测试状态**：⏳ 待用户测试

