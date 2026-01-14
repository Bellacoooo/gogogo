# 风险自适应椭球 - 稳定性补丁详解

## ✅ 已实现的补丁

### 补丁 A: s 的平滑和限幅

**问题**：风险裕量 s 的突变会导致椭球尺寸突变，进而导致 QP 不可行或轨迹抖动。

**解决方案**（5 步）：

```cpp
// 1. 计算原始值
double s_raw = s0 + alpha*vc + beta*exp(-ttc/tau);

// 2. 硬限幅
s_raw = clamp(s_raw, s_min, s_max);

// 3. 低通滤波（指数平滑）
double s_filt = (1-lambda_s)*prevSFilt_[i] + lambda_s*s_raw;
s_filt = clamp(s_filt, s_min, s_max);

// 4. 变化率限幅
double ds = clamp(s_filt - prevSFilt_[i], -max_delta_s, max_delta_s);
s_filt = prevSFilt_[i] + ds;
s_filt = clamp(s_filt, s_min, s_max);

// 5. 保存状态
prevSFilt_[i] = s_filt;
```

**参数**：
- `risk_time_const_s`: 0.3-1.0s（越大越平滑）
- `risk_max_delta_s`: 0.2-0.5m/step（最大变化率）
- `lambda_s = dt / (T_s + dt)` 自动计算

---

### 补丁 B: phi 的平滑（考虑角度 wrap-around）

**问题**：角度 φ 的跳变会导致椭球朝向突变，进而导致约束方向突变。

**解决方案**（考虑 π/-π 边界）：

```cpp
// 角度归一化函数
auto wrapToPi = [](double a){
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
};

// 1. 计算角度差并 wrap
double dphi = wrapToPi(phi_raw - prevPhiFilt_[i]);

// 2. 低通滤波
double phi_filt = wrapToPi(prevPhiFilt_[i] + lambda_phi * dphi);

// 3. 变化率限幅
double dphi_limited = wrapToPi(phi_filt - prevPhiFilt_[i]);
dphi_limited = clamp(dphi_limited, -max_delta_phi, max_delta_phi);
phi_filt = wrapToPi(prevPhiFilt_[i] + dphi_limited);

// 4. 保存状态
prevPhiFilt_[i] = phi_filt;
```

**参数**：
- `risk_time_const_phi`: 0.3-1.0s
- `risk_max_delta_phi`: 20-45 deg/step

**为什么不能直接线性平均**：
- `phi = 179°` 和 `phi = -179°` 实际只差 2°，但数值差 358°
- 直接平均会得到 0°（错误！）
- 正确做法：识别出差值是 2°，平滑到 180° 或 -180°

---

### 补丁 C: 低速退化（退化成圆形）

**问题**：低速时速度方向噪声大，φ 会乱跳；如果 a≠b，椭球朝向乱跳会导致约束方向乱跳。

**解决方案**：

```cpp
double kappa_eff = this->riskKappa_;
double phi_use = phi_filt;

if (vi_norm < riskVelThreshold_){
    // 低速：退化成各向同性（圆形）
    kappa_eff = 0.0;
    phi_use = prevYaw_[i][j];  // 保持不变
}

// 各向异性分配
double a = a0 + s_filt * (1.0 + kappa_eff);
double b = b0 + s_filt * (1.0 - kappa_eff);

// 强制圆形
if (kappa_eff == 0.0){
    b = a;
}
```

**参数**：
- `risk_vel_threshold`: 0.15-0.3 m/s

**效果**：
- 高速（v > v_th）：椭球各向异性，a > b
- 低速（v < v_th）：椭球退化成圆形，a = b，φ 无影响

---

## 🔍 系统可能存在的其他问题

### 问题 1: 数值稳定性（exp 溢出）

**场景**：当 `ttc` 很小（即将碰撞）时，`exp(-ttc/tau)` 可能接近 `exp(0) = 1`，不会溢出；但如果 `ttc` 很大，`exp(-ttc/tau)` 会趋近 0，也没问题。

**结论**：exp() 在这个应用中数值稳定 ✅

但如果担心极端情况，可以添加：

```cpp
double exp_term = (ttc < 10.0 * tau) ? std::exp(-ttc / tau) : 0.0;
double s_raw = s0 + alpha*vc + beta*exp_term;
```

---

### 问题 2: 障碍物突然出现/消失

**场景**：
- 障碍物数量从 N 变成 M
- `prevSFilt_` 和 `prevPhiFilt_` 的索引可能不对应

**当前实现的保护**：

```cpp
if (prevSFilt_.size() != numDynamicOb){
    prevSFilt_.resize(numDynamicOb, this->riskS0_);  // 初始化为基线值
}
if (prevPhiFilt_.size() != numDynamicOb){
    prevPhiFilt_.resize(numDynamicOb, 0.0);
}
```

**问题**：如果障碍物 ID 顺序变化（例如障碍物 A 消失，B 和 C 的索引前移），历史状态会错配。

**推荐改进**（如果有障碍物 ID）：

```cpp
// 使用 std::map<int, double> 而不是 vector
std::map<int, double> prevSFiltMap_;
std::map<int, double> prevPhiFiltMap_;

// 访问时：
double s_prev = (prevSFiltMap_.count(obstacle_id) > 0) 
                ? prevSFiltMap_[obstacle_id] 
                : this->riskS0_;
```

**当前状态**：如果你的系统障碍物数量和顺序稳定，当前实现已足够 ✅

---

### 问题 3: 预测 Horizon 内的一致性

**场景**：对同一障碍物 i，不同预测步 j 的椭球参数可能不一致。

**当前实现**：
- 只对障碍物 i（时间维度）做平滑
- 对 horizon 步 j（空间维度）没有平滑

**分析**：
- 当前实现是**按障碍物**平滑，每个 MPC 重规划周期更新一次
- Horizon 内的 j 步是"预测未来"，应该基于当前平滑后的 s 和 phi 计算

**结论**：当前实现合理 ✅

原因：
- `s_filt` 和 `phi_filt` 是基于**当前时刻**的障碍物状态计算的
- 然后对 horizon 内所有步 j 使用相同的 `s_filt` 和 `phi_filt`
- 这是合理的，因为我们假设障碍物的"风险特性"在短时间内（MPC horizon）不变

---

### 问题 4: 极端场景保护

#### 场景 4.1: 距离 d 很小

```cpp
double d = r.norm();
const double eps = 1e-6;
Eigen::Vector2d r_hat = r / (d + eps);  // ✅ 已有保护
```

✅ 已实现

#### 场景 4.2: vc 很大（高速碰撞）

```cpp
double vc = std::max(0.0, -r_hat.dot(v_rel));  // ✅ vc >= 0
double s_raw = s0 + alpha*vc + beta*exp(-ttc/tau);
s_raw = clamp(s_raw, s_min, s_max);  // ✅ 已限幅
```

✅ 已实现

#### 场景 4.3: ttc 接近 0（即将碰撞）

```cpp
double ttc = d / (vc + eps);  // ✅ 避免除零

// 如果 ttc 很小，exp(-ttc/tau) ≈ 1，s ≈ s0 + alpha*vc + beta
// 然后被 s_max 限制
```

✅ 已实现

---

## 📊 补丁效果对比

| 场景 | 无补丁 | 有补丁 | 改进 |
|------|--------|--------|------|
| **迎面冲突** | 椭球突变，轨迹抖动 | 椭球平滑增大，轨迹稳定 | ✅ 稳定性大幅提升 |
| **低速场景** | φ 乱跳，椭球旋转 | 退化成圆形，φ 不变 | ✅ 消除抖动 |
| **快速接近** | QP 可能 infeasible | 变化率限制，始终可行 | ✅ 鲁棒性提升 |
| **障碍突然出现** | 初值不合理 | 初始化为基线值 | ✅ 平滑启动 |

---

## 🎛️ 稳定性参数调优指南

### 基本原则

**越平滑 = 越稳定，但反应越慢**

- `risk_time_const_s` ↑ → s 变化更慢，更稳定
- `risk_max_delta_s` ↓ → s 变化率受限，更稳定
- `risk_vel_threshold` ↑ → 更大范围内退化成圆形，更稳定

### 推荐配置

#### 保守模式（优先稳定）

```yaml
mpc_planner/risk_time_const_s: 1.0      # 慢
mpc_planner/risk_time_const_phi: 1.0    # 慢
mpc_planner/risk_max_delta_s: 0.2       # 小
mpc_planner/risk_max_delta_phi: 20.0    # 小
mpc_planner/risk_vel_threshold: 0.3     # 大范围退化
```

**适用**：室内、拥挤环境、低速

#### 激进模式（优先灵敏）

```yaml
mpc_planner/risk_time_const_s: 0.3      # 快
mpc_planner/risk_time_const_phi: 0.3    # 快
mpc_planner/risk_max_delta_s: 0.5       # 大
mpc_planner/risk_max_delta_phi: 45.0    # 大
mpc_planner/risk_vel_threshold: 0.1     # 小范围退化
```

**适用**：户外、开阔环境、高速

#### 均衡模式（默认）

```yaml
mpc_planner/risk_time_const_s: 0.5
mpc_planner/risk_time_const_phi: 0.5
mpc_planner/risk_max_delta_s: 0.3
mpc_planner/risk_max_delta_phi: 30.0
mpc_planner/risk_vel_threshold: 0.15
```

---

## 🐛 调试建议

### 1. 添加调试输出

在 `updateObstacleParam()` 中添加：

```cpp
ROS_INFO_THROTTLE(0.5, "[Stability] Obs_%d: s_raw=%.2f, s_filt=%.2f, ds=%.2f, phi_raw=%.1f°, phi_filt=%.1f°, v=%.2f, kappa_eff=%.2f", 
                  i, s_raw, s_filt, s_filt-prevSFilt_[i], 
                  phi_raw*180/M_PI, phi_filt*180/M_PI, 
                  vi_norm, kappa_eff);
```

### 2. 绘制时间序列

记录 `s_filt`, `phi_filt`, `a`, `b` 到 CSV，绘制时间序列：

```python
import matplotlib.pyplot as plt
plt.plot(time, s_filt, label='s_filt')
plt.plot(time, s_raw, label='s_raw', alpha=0.5)
plt.legend()
plt.show()
```

观察：
- `s_filt` 应该比 `s_raw` 更平滑
- 不应该有突变（跳变）

### 3. 检查 QP 可行性

如果仍然出现 infeasible：

1. **检查 s_max 是否过大**：`s_max` 太大会让椭球太大，可能与其他约束冲突
2. **检查 max_delta_s 是否过大**：单步变化太大仍然可能导致 infeasible
3. **检查初始化**：第一帧的 `prevSFilt_[i]` 应该是合理值（如 `s0`）

---

## 📝 代码实现检查清单

- [x] 添加历史状态变量（`prevSFilt_`, `prevPhiFilt_`）
- [x] 初始化历史状态（resize 到正确大小）
- [x] 实现 `wrapToPi` 函数
- [x] 计算 lambda 系数（基于 dt 和时间常数）
- [x] s 的硬限幅 → 低通滤波 → 变化率限幅
- [x] phi 的 unwrap → 低通滤波 → 变化率限幅
- [x] 低速退化逻辑（`kappa_eff = 0`, `b = a`）
- [x] 更新配置文件参数
- [x] 编译通过

---

## 🚀 效果预期

### 无补丁

```
Time    s       phi     a       b       QP Status
0.0s    0.5     0°      1.0     0.7     OK
0.1s    1.2     45°     1.8     0.9     OK
0.2s    0.8     90°     1.4     0.6     INFEASIBLE  ← 突变
0.3s    1.5     -90°    2.0     1.0     OK
0.4s    0.6     0°      1.1     0.7     OK (但轨迹左右摆)
```

### 有补丁

```
Time    s       phi     a       b       QP Status
0.0s    0.5     0°      1.0     0.7     OK
0.1s    0.7     15°     1.2     0.8     OK  ← 平滑过渡
0.2s    0.8     35°     1.3     0.8     OK  ← 平滑过渡
0.3s    1.0     55°     1.5     0.9     OK  ← 平滑过渡
0.4s    1.1     70°     1.6     0.9     OK  ← 平滑过渡
```

---

## 🎓 理论背景

### 为什么 MPC 对约束突变敏感？

MPC 的障碍约束通常线性化为：

```
∇f(x_k) · (x - x_k) ≥ f(x_k)
```

其中 `f(x) = (x·cos(phi) + y·sin(phi))²/a² + ... - 1`

如果 `phi`, `a`, `b` 突变：
1. **梯度方向 ∇f 突变**：约束边界旋转
2. **约束边界位置突变**：a, b 变化改变椭球大小
3. **上一帧的可行解可能被"剪掉"**：导致 QP infeasible

### 低通滤波的作用

一阶低通滤波器：

```
y[k] = (1-λ)y[k-1] + λx[k]
```

转换到连续时间的传递函数：

```
H(s) = 1 / (Ts + 1)
```

**截止频率**：`f_c = 1/(2πT)`

- `T = 0.5s` → `f_c ≈ 0.3 Hz`（滤掉 > 0.3Hz 的高频抖动）
- `T = 1.0s` → `f_c ≈ 0.16 Hz`（更平滑）

---

## 📚 参考文献

1. **MPC 约束线性化稳定性**
   - "Robust Model Predictive Control for Obstacle Avoidance"
   - 关键词：Sequential Convex Programming (SCP), Trust Region

2. **角度平滑**
   - "Quaternion and Rotation Sequence Averaging"
   - 关键词：SO(3) averaging, angle wrap-around

3. **低通滤波**
   - "Exponentially Weighted Moving Average (EWMA)"
   - 关键词：α-β filter, Kalman filter

---

**版本**: v1.1（稳定性补丁）  
**日期**: 2026-01-13  
**状态**: ✅ 已实现并编译通过

