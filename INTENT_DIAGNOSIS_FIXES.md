# 意图识别与自适应参数诊断修复报告

## 📊 数据分析

### 您的数据特征
- 障碍物0：从(-3.31, -1.076)移动到(-3.07, -1.096)
- **运动状态**：明显直线运动（Δx≈0.24m，Δy≈0.02m）
- **s_adaptive值**：只有1.2-3.0（远低于s_max=5.0）
- **意图分布**：P_forward不是压倒性优势，P_left/right/stop有较高概率

## 🔍 核心问题诊断

### 问题1：s_adaptive异常偏低 ⚠️

**现象**：
- s_max设置为5.0
- 但数据显示s只有1.2-3.0
- 说明Mt（运动变化指标）异常偏高

**原因分析**：

根据sigmoid映射公式：
```
s = 1.2 + 3.8 * sigmoid(-10*(Mt - 0.5))
```

| Mt值 | s_adaptive值 | 含义 |
|------|-------------|------|
| 0.0  | 4.97 | 完全稳定运动 |
| 0.3  | 4.54 | 较稳定 |
| 0.5  | 3.10 | 中等变化 |
| 0.7  | 1.66 | 较大变化 |
| 1.0  | 1.20 | 剧烈变化 |

**您的s≈1.2-3.0，说明系统认为Mt≈0.5-1.0（中等到剧烈变化）！**

但障碍物明明在直线运动，这不合理！

### 问题2：Mt计算异常 - 三大指标诊断

Mt = 0.4 * f_jerk + 0.3 * f_angular + 0.3 * f_error

#### 2.1 角加速度f_angular异常高（主要嫌疑） 🚨

**代码第681行的BUG**：
```cpp
double currAngle = atan2(currVel(1), currVel(0));  // ❌ 使用速度方向
```

**问题**：
- 速度估计有噪声（来自位置差分或滤波）
- 即使直线运动，速度方向会抖动：±5°的噪声
- 导致角速度剧烈波动：ω = Δθ/0.1 = 0.087 rad/s（5°）
- 角加速度更剧烈：α = Δω/0.1 可能达到1-2 rad/s²
- f_angular = min(|α|/π, 1.0) ≈ 0.3-0.6 **（异常高！）**

**验证方法**：
直线运动时，理论上：
- 角度变化应该接近0
- f_angular应该< 0.1
- 但由于使用速度（有噪声），实际f_angular可能0.3-0.6

#### 2.2 预测误差f_error可能偏高

**代码第593行**：
```cpp
Eigen::Vector3d predictedPos = prevPos + prevVel * dt_;  // 恒速模型
double predictionError = (currPos - predictedPos).norm();
f_error = std::min(predictionError / 0.3, 1.0);
```

**问题**：
- 如果障碍物有轻微加速/减速（即使是直线）
- 恒速模型预测误差会累积
- e_max=0.3m可能太小，导致f_error容易饱和到1.0

**示例**：
- 障碍物以0.5m/s直线运动，有0.1m/s²加速度
- 预测误差：e = 0.5 * a * dt² = 0.5 * 0.1 * 0.01 = 0.0005m ✅ 很小
- 但如果累积或有定位噪声（±5cm），e可能达到0.05-0.1m
- f_error = 0.1/0.3 = 0.33 **（偏高）**

#### 2.3 加加速度f_jerk

理论上直线运动应该很小，但如果加速度估计有噪声，也会偏高。

### 问题3：意图分布不准确 📊

**根本原因**：s_adaptive太小（1.2-3.0）→ 意图惯性不足 → 容易受瞬时观测噪声影响

**转移矩阵机制**：
```cpp
scale(i) = s_adaptive;  // 对角元素放大因子
transMat = generateMatrix(theta, r, scale);
```

当s=1.2时（接近1.0）：
- 意图惯性几乎消失
- 每帧的瞬时观测（角度、速度）直接主导概率
- 即使微小的角度噪声（±2°）都会导致P_left/right明显增大

**停止概率公式的潜在BUG**：
```cpp
ps = 1 - tanh(params_ / scale(3) * r);  // ❌ scale在分母！
```

当scale(3)=s_adaptive很小时：
- ps = 1 - tanh(params_ / 1.2 * r)
- 分母变小，tanh内的值变大，ps反而会偏小
- 这与设计初衷可能相反

---

## ✅ 修复方案

### 修复1：角度计算改用位置差分（推荐） 🔧

**问题根源**：使用速度方向（有噪声）计算角度

**修复方案A：改用位置差分**

```cpp
// 修改前（第681行）
double currAngle = atan2(currVel(1), currVel(0));  // ❌

// 修改后
double currAngle = atan2(currPos(1) - prevPos(1), currPos(0) - prevPos(0));  // ✅
```

**优点**：
- 位置比速度平滑（速度是位置差分得到的）
- 角度更稳定，f_angular会显著降低
- 更符合MDP论文的原始设计

**预期效果**：
- f_angular从0.3-0.6降低到0.05-0.15
- Mt从0.5-1.0降低到0.1-0.3
- s_adaptive从1.2-3.0提升到4.0-4.8

### 修复2：调整归一化上限（备选） 📏

如果修复1效果不够，可以适当放宽归一化上限：

```yaml
# predictor_param.yaml

# 原值：
alpha_max: 3.14159  # π rad/s²
e_max: 0.3          # 0.3m

# 建议调整：
alpha_max: 6.28     # 2π rad/s²（放宽，降低f_angular）
e_max: 0.5          # 0.5m（放宽，降低f_error）
```

**效果**：
- alpha_max翻倍 → f_angular减半
- e_max增大 → f_error降低

### 修复3：调整特征融合权重（微调） ⚖️

降低角加速度权重，避免被噪声主导：

```yaml
# 原值：
w1: 0.4  # 加加速度
w2: 0.3  # 角加速度
w3: 0.3  # 预测误差

# 建议：
w1: 0.5  # 增加（Jerk最可靠）
w2: 0.2  # 降低（角加速度易受噪声影响）
w3: 0.3  # 保持
```

### 修复4：修复停止概率公式BUG（可选） 🐛

**当前公式（第867行）**：
```cpp
ps = ((1-tanh(this->params_/scale(3)*r)));  // scale在分母
```

**问题**：当s小时，tanh内的值变大，ps可能异常

**建议修复**：
```cpp
ps = ((1-tanh(this->params_ * scale(3) * r)));  // scale变为乘数
```

或者保持原公式但调整params_参数。

---

## 🧪 诊断建议

### 步骤1：启用调试日志

**修改代码第825行**，取消注释：
```cpp
ROS_INFO_THROTTLE(2.0, "Obs %d: f_jerk=%.3f, f_angular=%.3f, f_error=%.3f => Mt=%.3f => s=%.3f",
                  obsIdx, f_jerk, f_angular, f_error, Mt, s_adaptive);
```

**运行后观察**：
- 如果f_angular > 0.3，说明角度计算有问题 → 应用修复1
- 如果f_error > 0.5，说明预测模型不准 → 调整e_max
- 如果f_jerk > 0.4，说明加速度估计有噪声 → 检查滤波

### 步骤2：分析具体数值

**直线运动场景的理想值**：
| 指标 | 理想值 | 您的可能值 | 问题 |
|------|--------|-----------|------|
| f_jerk | <0.1 | 0.1-0.2 | 轻微偏高 |
| f_angular | <0.05 | 0.3-0.6 | **严重偏高** ⚠️ |
| f_error | <0.15 | 0.2-0.4 | 偏高 |
| Mt | <0.15 | 0.5-1.0 | **异常高** |
| s | >4.5 | 1.2-3.0 | **异常低** |

### 步骤3：验证修复效果

**应用修复1后，预期数据**：
```
[INFO] Obs 0: f_jerk=0.08, f_angular=0.05, f_error=0.12 => Mt=0.08 => s=4.92
[INFO] Intent: forward=0.92, left=0.03, right=0.03, stop=0.02
```

---

## 📝 关于其他变量的说明

### riskMax_, s_filt, Dt的含义

根据代码搜索，这些变量**在当前自适应意图惯性实现中未使用**：

1. **Dt** ≠ 时间间隔dt_
   - 代码第832行：`currentDt_[obsIdx] = Mt;`
   - **实际存储的是Mt**，不是时间！
   - 变量名误导，建议改为currentMt_

2. **s_filt**
   - 未在当前代码中找到
   - 可能是旧版本的"滤波后的s值"
   - 当前实现直接使用s_adaptive，没有额外滤波

3. **riskMax_**
   - 与风险地图（Risk Map）相关
   - 但自适应意图惯性机制**不依赖风险地图**
   - 只依赖障碍物自身的运动状态

### 关于clamp

"clamp"不是变量，而是**数值限制操作**（C++中的std::clamp或手动min/max）

代码第632行：
```cpp
return std::max(s_min_, std::min(s_adaptive, s_max_));  // 等价于clamp
```

作用：确保s_adaptive∈[1.2, 5.0]

---

## 🎯 推荐修复顺序

### 优先级1：修复角度计算（最关键） 🔥

```cpp
// src/Intent-MPC/dynamic_predictor/include/dynamic_predictor/dynamicPredictor.cpp
// 第681行
double currAngle = atan2(currPos(1) - prevPos(1), currPos(0) - prevPos(0));
```

**预期效果**：
- s从1.2-3.0提升到4.0-4.9
- P_forward从0.5-0.7提升到0.85-0.95

### 优先级2：启用调试日志

取消第825行注释，观察三个指标的具体数值

### 优先级3：如果效果不够，调整参数

```yaml
# predictor_param.yaml
alpha_max: 6.28  # 原π改为2π
e_max: 0.5       # 原0.3改为0.5
w2: 0.2          # 原0.3改为0.2
```

---

## 🔬 意图识别原理回顾

您提到的MDP实现：

### 瞬时意图概率计算（genTransitionVector）

```cpp
pf = scale(0) * (exp(-0.5*(theta/paramf)²) + paraml)  // 前进：高斯分布
pl = scale(1) * paraml * (1 + sin(theta))              // 左转：正弦
pr = scale(2) * paramr * (1 - sin(theta))              // 右转：正弦
ps = 1 - tanh(params / scale(3) * r)                   // 停止：双曲正切
```

**关键**：scale = [s, s, s, s]，当s很小时：
- 意图惯性消失
- 瞬时角度θ直接主导概率
- 即使θ只有±5°（噪声），也会导致pl/pr显著增大

### 转移矩阵生成

对每个意图i，生成一列转移概率：
```cpp
scale(i) = s_adaptive;  // 只放大对角元素
transMat[:, i] = genTransitionVector(theta, r, scale);
```

**矩阵形式**（s=5时）：
```
      from: F    L    R    S
to F:      5.0  0.8  0.8  0.1
   L:      0.1  5.0  0.1  0.3
   R:      0.1  0.1  5.0  0.3
   S:      0.05 0.1  0.1  5.0
```

对角元素被放大5倍 → 强惯性

**当s=1.2时**：
```
      from: F    L    R    S
to F:      1.2  0.8  0.8  0.1
   L:      0.1  1.2  0.1  0.3
   R:      0.1  0.1  1.2  0.3
   S:      0.05 0.1  0.1  1.2
```

对角元素几乎没有优势 → 弱惯性 → 容易跳变

---

## 📊 数据格式说明

根据您的数据，列的顺序应该是：

1. 时间戳
2. 障碍物ID
3. 障碍物X (m)
4. 障碍物Y (m)
5. ADE (m) - Average Displacement Error
6. FDE (m) - Final Displacement Error
7. **Mt** - 运动变化指标（不是Dt！）
8. **s_adaptive** - 自适应参数
9. 最佳意图 (0=forward, 1=left, 2=right, 3=stop)
10. 有效步数
11-14. P_forward, P_left, P_right, P_stop

---

## 🎯 总结

**核心问题**：使用速度方向计算角度 → 角加速度f_angular异常高 → Mt偏大 → s偏小 → 意图惯性不足

**核心修复**：改用位置差分计算角度

**预期效果**：
- s从1-3提升到4-5
- P_forward从0.5-0.7提升到0.85-0.95
- 意图识别准确率显著提升

---

**修复日期**：2026-01-15  
**诊断者**：AI Assistant
**状态**：✅ 诊断完成，待应用修复
