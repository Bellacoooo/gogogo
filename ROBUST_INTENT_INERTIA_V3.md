# 鲁棒自适应意图惯性方案 V3

## 🎯 核心问题与解决思路

### 问题诊断

**您的观察**：
1. ✅ Dt确实是历史遗留，实际存储的是Mt
2. ✅ 加加速度(Jerk)容易抖动，对噪声敏感
3. ✅ 直接用加速度会把匀速转弯误判为异常

**关键洞察**（来自AI建议）：
- 需要检测的是**"意图切换的瞬间"**，不是"持续的运动状态"
- 匀速转弯是"稳定意图"，不应该持续压低s
- 角加速度α检测的是"转弯强度变化"，不是"正在转弯"

---

## ✅ V3改进方案：三指标鲁棒化

### 改进1：Jerk → 窗口化加速度变化量 Δa

**问题**：
```cpp
jerk = (a_t - a_{t-1}) / dt_;  // 逐帧差分，噪声敏感
```

**工程实践**：
- 加速度本身就是速度差分（一阶导数）
- Jerk是加速度差分（二阶导数）→ **噪声平方级放大**
- 即使滤波，也难以完全消除抖动

**V3方案A：窗口化Δa（推荐）** ⭐

```cpp
// 不除以dt，直接用加速度变化的模长
Eigen::Vector3d deltaAcc = currAcc - prevAcc;
double f_delta_a = std::min(deltaAcc.norm() / a_change_max, 1.0);
```

**参数设置**：
- `a_change_max = 2.0 m/s²`（典型的加速度变化量）
- 对于行人：急停时Δa ≈ 1-2 m/s²
- 对于汽车：急刹时Δa ≈ 3-5 m/s²

**优点**：
- ✅ 不再二阶差分，噪声敏感度降低
- ✅ 物理意义清晰：驱动力变化 ∝ Δa
- ✅ 数值稳定，不易饱和

**V3方案B：EMA平滑后再算Jerk（备选）**

```cpp
// 对加速度做指数移动平均
static Eigen::Vector3d a_smoothed_prev = prevAcc;
Eigen::Vector3d a_smoothed = 0.7 * currAcc + 0.3 * a_smoothed_prev;
a_smoothed_prev = a_smoothed;

// 用平滑后的加速度计算Jerk
Eigen::Vector3d jerk = (a_smoothed - prevAcc_smoothed) / dt_;
double f_jerk = std::min(jerk.norm() / j_max_, 1.0);
```

**对比**：
| 方案 | 计算量 | 噪声抑制 | 响应速度 | 推荐 |
|------|--------|---------|---------|------|
| 原始Jerk | 低 | 差 | 快 | ❌ |
| 窗口Δa | 低 | **好** | **快** | **✅** |
| EMA+Jerk | 中 | 好 | 慢 | ⚠️ |

---

### 改进2：角加速度计算改用速度方向

**当前问题**：
```cpp
// V2使用平滑速度
Eigen::Vector3d smoothedVel = 0.7 * currVel + 0.3 * prevVel;
double currAngle = atan2(smoothedVel(1), smoothedVel(0));
```

**隐藏问题**：
- 速度向量本身有噪声（来自位置差分）
- 即使平滑，在低速时角度仍然会抖动
- 平滑系数(0.7/0.3)需要手动调参

**V3方案：显式计算角速度和角加速度** ⭐

```cpp
// 方法1：速度向量夹角（更稳定）
double angle_change = atan2(currVel(1), currVel(0)) - atan2(prevVel(1), prevVel(0));
// 角度归一化到 [-π, π]
while (angle_change > M_PI) angle_change -= 2.0 * M_PI;
while (angle_change <= -M_PI) angle_change += 2.0 * M_PI;

double omega_curr = angle_change / dt_;  // 角速度

// 角加速度（需要前一帧的omega）
static double omega_prev = 0.0;
double alpha = (omega_curr - omega_prev) / dt_;
omega_prev = omega_curr;

double f_angular = std::min(std::abs(alpha) / alpha_max, 1.0);
```

**关键改进**：
- ✅ 直接用速度向量夹角，不依赖速度模长（低速时更稳）
- ✅ 角度归一化处理，避免±180°跳变
- ✅ 匀速转弯时：ω恒定 → α≈0 → 不会误判

**物理意义验证**：
| 运动状态 | ω | α | f_angular | s应该 |
|---------|---|---|-----------|------|
| 直线 | ≈0 | ≈0 | <0.05 | 高(4-5) |
| 匀速转弯 | 大且恒定 | ≈0 | <0.1 | **高**(4-5) ✅ |
| 开始转弯 | 0→大 | **大** | 0.5-0.8 | 低(1-2) |
| 急转弯变直线 | 大→0 | **大** | 0.5-0.8 | 低(1-2) |

---

### 改进3：预测误差保持不变（已经很鲁棒）

```cpp
// 恒速模型预测
Eigen::Vector3d predictedPos = prevPos + prevVel * dt_;
double error = (currPos - predictedPos).norm();
double f_error = std::min(error / e_max, 1.0);
```

**优点**：
- ✅ 低敏感度（e_max=0.3-0.5m，容忍度高）
- ✅ 物理直观（偏离惯性模型=有控制输入）
- ✅ 作为兜底指标，与Jerk/角加速度互补

---

### 改进4：Mt的EMA平滑（强烈推荐）⭐

**当前问题**：
```cpp
double Mt = w1 * f_jerk + w2 * f_angular + w3 * f_error;  // 瞬时值
```

**抖动来源**：
- 即使单个指标稳定，加权融合仍可能逐帧波动
- Mt的小幅波动经过Sigmoid会被放大

**V3方案：对Mt做指数移动平均**

```cpp
// 计算瞬时Mt
double Mt_instant = w1 * f_delta_a + w2 * f_angular + w3 * f_error;
Mt_instant = std::max(0.0, std::min(Mt_instant, 1.0));

// EMA平滑（强烈推荐）
static double Mt_smoothed_prev = 0.0;
double lambda = 0.8;  // 平滑系数：0.7-0.9
double Mt_smoothed = lambda * Mt_smoothed_prev + (1 - lambda) * Mt_instant;
Mt_smoothed_prev = Mt_smoothed;

// 用平滑后的Mt计算s
double s_adaptive = computeAdaptiveS(Mt_smoothed);
```

**参数调节**：
| lambda | 平滑强度 | 响应速度 | 适用场景 |
|--------|---------|---------|---------|
| 0.7 | 中等 | 快 | 一般场景 |
| **0.8** | **较强** | **中** | **推荐** ✅ |
| 0.9 | 很强 | 慢 | 高噪声环境 |

**效果**：
- ✅ 消除Mt的高频抖动
- ✅ 保留意图切换的低频信号
- ✅ s变化更平滑，MDP更稳定

---

## 🔧 V3完整实现方案

### 代码修改清单

#### 1. 改进Jerk计算（Δa方案）

**文件**：`dynamicPredictor.cpp` 第548-556行

```cpp
double predictor::computeJerkFeature(const Eigen::Vector3d& currAcc, 
                                      const Eigen::Vector3d& prevAcc) {
    // 🔧 V3改进：窗口化加速度变化量（鲁棒版Jerk）
    Eigen::Vector3d deltaAcc = currAcc - prevAcc;
    double deltaAccNorm = deltaAcc.norm();
    
    // 归一化：使用a_change_max（典型的加速度变化量级）
    double a_change_max = 2.0;  // m/s²，行人典型值
    double f_delta_a = std::min(deltaAccNorm / a_change_max, 1.0);
    
    return f_delta_a;
}
```

#### 2. 改进角加速度计算

**文件**：`dynamicPredictor.cpp` 第789-807行

```cpp
// 在genTransitionMatrix函数中
// 3.2 角加速度特征 f_angular(t)
double f_angular = 0.0;
if (hasValidPrevFrame) {
    // 🔧 V3改进：用速度向量夹角计算角速度和角加速度
    
    // 计算当前帧和前一帧的速度方向变化
    double angle_curr = atan2(currVel(1), currVel(0));
    double angle_prev = atan2(prevVel(1), prevVel(0));
    double angle_change = angle_curr - angle_prev;
    
    // 角度归一化到 [-π, π]
    while (angle_change > M_PI) angle_change -= 2.0 * M_PI;
    while (angle_change <= -M_PI) angle_change += 2.0 * M_PI;
    
    // 当前角速度
    double omega_curr = angle_change / dt_;
    
    // 角加速度（需要前一帧的角速度）
    // 从前前帧到前一帧的角速度
    double angle_prevprev = atan2(prevVel(1) - (prevPos(1) - prevPrevPos(1)), 
                                   prevVel(0) - (prevPos(0) - prevPrevPos(0)));
    double angle_change_prev = angle_prev - angle_prevprev;
    while (angle_change_prev > M_PI) angle_change_prev -= 2.0 * M_PI;
    while (angle_change_prev <= -M_PI) angle_change_prev += 2.0 * M_PI;
    double omega_prev = angle_change_prev / dt_;
    
    // 计算角加速度
    double alpha = (omega_curr - omega_prev) / dt_;
    f_angular = std::min(std::abs(alpha) / alpha_max_, 1.0);
}
```

#### 3. 添加Mt的EMA平滑

**文件**：`dynamicPredictor.cpp` 第815-822行

```cpp
// 4. 多维特征融合：综合运动变化指标 M_t
double Mt_instant = w1_ * f_delta_a + w2_ * f_angular + w3_ * f_error;
Mt_instant = std::max(0.0, std::min(Mt_instant, 1.0));

// 🔧 V3新增：EMA平滑（强烈推荐）
if (obsIdx >= 0 && obsIdx < static_cast<int>(currentMtSmoothed_.size())) {
    double lambda = 0.8;  // 平滑系数
    double Mt_smoothed = lambda * currentMtSmoothed_[obsIdx] + (1 - lambda) * Mt_instant;
    currentMtSmoothed_[obsIdx] = Mt_smoothed;
    Mt = Mt_smoothed;  // 使用平滑后的Mt
} else {
    Mt = Mt_instant;  // 第一帧或索引越界，使用瞬时值
}

// 5. Sigmoid映射：计算自适应意图惯性参数 s_adaptive
double s_adaptive = computeAdaptiveS(Mt);
```

#### 4. 添加平滑缓冲区

**文件**：`dynamicPredictor.h`

```cpp
// 在类的private成员中添加
std::vector<double> currentMtSmoothed_;  // Mt的EMA平滑值
```

**初始化**（在`intentProb`函数中）：

```cpp
currentMtSmoothed_.resize(numOb, 0.0);
```

---

## 📊 预期效果对比

### 直线运动场景

| 方案 | f_delta_a | f_angular | Mt | s | P_forward |
|------|-----------|-----------|----|----|-----------|
| **原始** | 0.4-0.6 | 0.3-0.6 | 0.5-1.0 | 1.2-3.0 | 0.4-0.7 |
| **V2** | 0.2-0.4 | 0.2-0.4 | 0.3-0.6 | 2.5-3.5 | 0.6-0.8 |
| **V3** | **0.05-0.15** | **0.05-0.10** | **0.05-0.15** | **4.5-4.9** | **>0.90** |

### 匀速转弯场景

| 方案 | f_delta_a | f_angular | Mt | s | 行为 |
|------|-----------|-----------|----|----|------|
| **原始** | 0.3-0.5 | **0.5-0.8** ⚠️ | 0.6-0.9 | 1.5-2.5 | **误判为异常** |
| **V3** | 0.1-0.2 | **<0.1** ✅ | 0.1-0.2 | **4.0-4.8** | **正确识别为稳定** |

### 转弯开始/结束场景

| 方案 | f_delta_a | f_angular | Mt | s | 行为 |
|------|-----------|-----------|----|----|------|
| **V3** | 0.2-0.4 | **0.6-0.9** | 0.4-0.6 | 2.0-3.0 | **正确检测到切换** |

---

## 🛠️ 参数调优指南

### 1. a_change_max（Δa归一化上限）

```yaml
# predictor_param.yaml
a_change_max: 2.0  # m/s²，行人/无人机
# a_change_max: 5.0  # m/s²，汽车/高速场景
```

### 2. lambda（Mt平滑系数）

```yaml
mt_lambda: 0.8  # 推荐值：0.7-0.9
```

| 场景 | lambda | 效果 |
|------|--------|------|
| 高噪声环境 | 0.9 | 更平滑，响应稍慢 |
| **一般场景** | **0.8** | **平衡** ✅ |
| 低延迟需求 | 0.7 | 响应快，略抖动 |

### 3. 权重配置（建议微调）

```yaml
w1: 0.4  # Δa权重（保持）
w2: 0.3  # 角加速度权重（保持或略降至0.25）
w3: 0.3  # 预测误差权重（保持）
```

---

## ✅ V3方案总结

### 核心改进

1. **Jerk → Δa**：消除二阶差分噪声
2. **速度方向角加速度**：匀速转弯时α≈0
3. **Mt的EMA平滑**：消除高频抖动
4. **保持V2的速度平滑**：作为第一层防护

### 鲁棒性提升

| 指标 | 原始 | V2 | V3 |
|------|------|----|----|
| 噪声抑制 | ⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 匀速转弯识别 | ❌ | ⚠️ | ✅ |
| 响应速度 | 快 | 中 | 中 |
| 实现复杂度 | 低 | 低 | **中**（最优平衡）|

### 工程优势

- ✅ 不需要复杂滤波器（Kalman/粒子滤波）
- ✅ 参数少，易调节
- ✅ 计算量小，实时性好
- ✅ 物理意义清晰，易解释

---

**推荐实施路径**：
1. 先应用"Δa + EMA平滑"（改动最小，效果明显）
2. 测试效果，如果匀速转弯仍误判，再改角加速度计算
3. 最后微调参数（a_change_max, lambda）

---

**修复日期**：2026-01-15  
**版本**：V3 - 鲁棒工程方案  
**状态**：✅ 设计完成，待实现
