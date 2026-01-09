# 自适应意图惯性机制实现说明

## 修改概述

本次修改将原有的自适应意图惯性机制替换为基于物理-统计双重诊断的新方案,核心思想是通过三个互补的物理指标(加加速度、角加速度、预测误差)来量化障碍物运动状态的变化程度,并通过Sigmoid函数平滑映射到自适应权重参数。

---

## 一、核心指标设计

### 1.1 物理突变指标：加加速度 (Jerk)

**理论基础**：加加速度衡量"加速度的变化率",是运动突变的直接体现。根据牛顿第二定律,驱动力的突变直接体现为加加速度的急剧变化。

**数学定义**：
```
jerk_t = (a_t - a_{t-1}) / Δt
f_jerk(t) = min(||jerk_t|| / j_max, 1.0)
```

**物理意义**：
- 稳定意图(如匀速直行)对应加加速度接近0
- 意图切换(如从"直行"变为"右转")导致加加速度显著增大

**实现位置**：`computeJerkFeature()` 函数

---

### 1.2 物理突变指标：角加速度 (Angular Acceleration)

**理论基础**：角加速度反映了转向决策的急剧程度,对于地面移动障碍物或空中无人机,航向角变化率的突增通常对应从"直行"意图向"转向"意图的切换。

**数学定义**：
```
ω_t = (θ_t - θ_{t-1}) / Δt  (角速度)
α_t = (ω_t - ω_{t-1}) / Δt  (角加速度)
f_angular(t) = min(|α_t| / α_max, 1.0)
```

**物理意义**：
- 匀速转弯过程中角速度恒定,α_t ≈ 0,系统保持较高惯性
- 突然转向的起始阶段α_t显著增大,提示意图可能发生变更

**实现位置**：`computeAngularAccelFeature()` 函数

---

### 1.3 统计突变指标：预测-观测误差 (Prediction-Observation Error)

**理论基础**：基于牛顿第一定律的推论——若障碍物无外力作用(意图未变),其运动应符合惯性假设(Constant Velocity Model)。

**数学定义**：
```
p̂_t = p_{t-1} + v_{t-1} * Δt  (惯性预测位置)
e_t = ||p_t - p̂_t||  (预测-观测误差)
f_error(t) = min(e_t / e_max, 1.0)
```

**物理意义**：
- 当e_t较小时,表明障碍物运动符合惯性假设,意图稳定
- 当e_t显著增大时,说明存在显著的控制输入,预示潜在的意图改变

**实现位置**：`computePredictionErrorFeature()` 函数

---

## 二、多维特征融合

**数学定义**：
```
M_t = w1 * f_jerk(t) + w2 * f_angular(t) + w3 * f_error(t)
```

**权重配置**：
- w1 = 0.4 (加加速度权重，最高权重以保证对机动性的快速响应)
- w2 = 0.3 (角加速度权重)
- w3 = 0.3 (预测误差权重)

**融合机制优势**：
- 能够有效应对单一指标失效的场景
  - 直线急停时 f_angular ≈ 0
  - 匀速转弯时 f_jerk ≈ 0
- 在各类复杂机动下实现可靠的意图变化检测

---

## 三、自适应权重映射函数

### 3.1 设计原则

1. **值域约束**：s_adaptive ∈ [s_min, s_max]
   - s_max：原方法的高惯性值(默认5.0)
   - s_min：低惯性下限(默认1.2),保留微弱的时间连续性

2. **负相关性**：s_adaptive 随 M_t 单调递减
   - 运动平稳(M_t → 0)时,强化历史惯性
   - 运动突变(M_t → 1)时,弱化历史依赖

3. **平滑性**：映射函数连续可微,避免参数跳变引入预测抖动

### 3.2 Sigmoid映射函数

**数学定义**：
```
s_adaptive(M_t) = s_min + (s_max - s_min) * σ(-k(M_t - μ))
其中 σ(x) = 1 / (1 + e^(-x)) 为标准Logistic函数
```

**参数设置**：
- μ = 0.5：中心偏移量,使得在M_t = 0.5时s_adaptive取中间值
- k = 10：陡峭度因子,控制状态切换的灵敏度

**特性分析**：
- M_t → 0 (稳定运动)：s_adaptive → s_max,系统表现出强意图惯性,有效过滤观测噪声
- M_t → 1 (剧烈机动)：s_adaptive → s_min,系统迅速降低对历史意图的信任
- M_t ≈ 0.5 (过渡区域)：函数具有较大斜率,保证意图识别的敏捷性

**实现位置**：`computeAdaptiveS()` 函数

---

## 四、代码修改清单

### 4.1 头文件修改 (dynamicPredictor.h)

**新增成员变量**：
```cpp
// 历史数据容器
std::vector<Eigen::Vector3d> accHistory_;      // 存储历史加速度
std::vector<double> angularVelHistory_;        // 存储历史角速度
std::vector<Eigen::Vector3d> posHistory_;      // 存储历史位置
std::vector<Eigen::Vector3d> velHistory_;      // 存储历史速度

// 指标计算参数
double w1_, w2_, w3_;          // 多维特征融合权重
double j_max_;                 // 加加速度归一化上限 (m/s^3)
double alpha_max_;             // 角加速度归一化上限 (rad/s^2)
double e_max_;                 // 预测误差归一化上限 (m)

// Sigmoid映射参数
double s_min_;                 // 最小自适应权重
double s_max_;                 // 最大自适应权重
double sigmoid_k_;             // Sigmoid陡峭度因子
double sigmoid_mu_;            // Sigmoid中心偏移量
```

**新增成员函数**：
```cpp
double computeJerkFeature(const Eigen::Vector3d& currAcc, const Eigen::Vector3d& prevAcc);
double computeAngularAccelFeature(double currAngle, double prevAngle);
double computePredictionErrorFeature(const Eigen::Vector3d& currPos, 
                                      const Eigen::Vector3d& prevPos, 
                                      const Eigen::Vector3d& prevVel);
double computeAdaptiveS(double Mt);
```

**删除的成员**：
- `std::vector<Eigen::Vector3d> caPredHistory_`
- `double gamma1_, gamma2_, lambda1_, lambda2_, M_thresh_`
- `computeJerkNorm()`, `computeNIS()`, `computeIntentEntropy()` (旧版本函数)

---

### 4.2 实现文件修改 (dynamicPredictor.cpp)

#### 4.2.1 参数初始化 (initParam函数)

**删除的参数**：
- gamma1, gamma2 (NIS和加加速度权重)
- lambda1, lambda2 (熵敏感度和运动诊断敏感度)
- M_thresh (运动突变阈值)

**新增参数**：
- w1, w2, w3 (多维特征融合权重)
- j_max (加加速度上限，默认10.0 m/s³)
- alpha_max (角加速度上限，默认π rad/s²)
- e_max (预测误差上限，默认0.3 m)
- s_min (最小自适应权重，默认1.2)
- s_max (最大自适应权重，默认5.0)
- sigmoid_k (Sigmoid陡峭度因子，默认10.0)
- sigmoid_mu (Sigmoid中心偏移量，默认0.5)

**历史数据容器初始化**：
```cpp
this->accHistory_.reserve(this->historyWindow_ + 1);
this->angularVelHistory_.reserve(this->historyWindow_ + 1);
this->posHistory_.reserve(this->historyWindow_ + 1);
this->velHistory_.reserve(this->historyWindow_ + 1);
```

---

#### 4.2.2 核心指标计算函数实现

**位置**：第504-632行

**新增函数**：
1. `computeJerkFeature()` - 计算归一化加加速度特征
2. `computeAngularAccelFeature()` - 计算归一化角加速度特征
3. `computePredictionErrorFeature()` - 计算归一化预测误差特征
4. `computeAdaptiveS()` - Sigmoid映射函数

每个函数都包含详细的文档注释,说明物理意义、数学定义和特性分析。

---

#### 4.2.3 转移矩阵生成函数 (genTransitionMatrix)

**关键修改**（第692-747行）：

1. **指标计算流程**：
```cpp
// 3.1 计算加加速度特征
double f_jerk = 0.0;
if (!accHistory_.empty()) {
    Eigen::Vector3d prevAcc = accHistory_.back();
    f_jerk = computeJerkFeature(currAcc, prevAcc);
}

// 3.2 计算角加速度特征
double f_angular = computeAngularAccelFeature(currAngle, prevAngle);

// 3.3 计算预测误差特征
double f_error = 0.0;
if (!posHistory_.empty() && !velHistory_.empty()) {
    Eigen::Vector3d prevPos = posHistory_.back();
    Eigen::Vector3d prevVel = velHistory_.back();
    f_error = computePredictionErrorFeature(currPos, prevPos, prevVel);
}

// 4. 多维特征融合
double Mt = w1_ * f_jerk + w2_ * f_angular + w3_ * f_error;
Mt = std::max(0.0, std::min(Mt, 1.0));  // 安全截断

// 5. Sigmoid映射
double s_adaptive = computeAdaptiveS(Mt);
```

2. **转移矩阵生成（修复原有bug）**：
```cpp
// 7. 生成转移矩阵的每一列
double r = sqrt(pow(currVel(0), 2) + pow(currVel(1), 2));
for (int i = 0; i < this->numIntent_; ++i) {
    Eigen::VectorXd scale = Eigen::VectorXd::Ones(this->numIntent_);
    scale(i) = s_adaptive;  // 用自适应权重替代固定的pscale_
    Eigen::VectorXd probVec = this->genTransitionVector(theta, r, scale);
    transMat.block(0, i, this->numIntent_, 1) = probVec;  // 正确填充转移矩阵
}
```

**原bug说明**：第754行只设置了scale但没有调用`genTransitionVector`和填充`transMat`,导致转移矩阵未正确生成。

---

## 五、参数配置建议

### 5.1 默认参数配置

在 `.yaml` 配置文件中添加以下参数（如果不添加则使用默认值）：

```yaml
dynamic_predictor:
  # 多维特征融合权重
  w1: 0.4              # 加加速度权重
  w2: 0.3              # 角加速度权重
  w3: 0.3              # 预测误差权重
  
  # 归一化上限
  j_max: 10.0          # 加加速度上限 (m/s^3)，行人典型值
  alpha_max: 3.14159   # 角加速度上限 (rad/s^2)
  e_max: 0.3           # 预测误差上限 (m)
  
  # Sigmoid映射参数
  s_min: 1.2           # 最小自适应权重
  s_max: 5.0           # 最大自适应权重
  sigmoid_k: 10.0      # Sigmoid陡峭度因子
  sigmoid_mu: 0.5      # Sigmoid中心偏移量
  
  # 历史数据窗口大小
  history_window: 1    # 建议保持为1，避免过度平滑
```

### 5.2 参数调优建议

**针对不同障碍物类型**：

1. **行人**：
   - j_max: 10.0 m/s³ (人类舒适性约束)
   - alpha_max: π rad/s² (急转弯极限)
   - e_max: 0.3 m (步态波动范围)

2. **自行车/电动车**：
   - j_max: 20.0 m/s³
   - alpha_max: π/2 rad/s²
   - e_max: 0.5 m

3. **汽车**：
   - j_max: 30.0 m/s³
   - alpha_max: π/4 rad/s²
   - e_max: 0.8 m

**针对不同场景**：

1. **拥挤环境**（需要更敏捷的响应）：
   - sigmoid_k: 15.0 (增大陡峭度)
   - s_min: 1.1 (降低最小惯性)

2. **开放环境**（需要更稳定的预测）：
   - sigmoid_k: 8.0 (减小陡峭度)
   - s_max: 7.0 (增大最大惯性)

---

## 六、核心创新点总结

1. **物理-统计双重诊断**：
   - 首次结合物理动力学特征(Jerk、角加速度)与统计预测误差
   - 构建了多维度的意图变化检测体系

2. **平滑自适应机制**：
   - 通过连续的Sigmoid映射替代离散的规则切换
   - 解决了传统固定参数方法在稳定性与响应速度之间的固有矛盾

3. **可解释性强**：
   - 每个指标都有明确的物理意义
   - 参数设置与障碍物动力学特性直接关联

4. **鲁棒性高**：
   - 多维特征融合能够应对单一指标失效的场景
   - 平滑映射函数避免了参数跳变引入的预测抖动

---

## 七、验证与测试建议

### 7.1 功能验证

1. **编译测试**：
```bash
cd /home/ff/intent-mpc
catkin_make -DCATKIN_WHITELIST_PACKAGES="dynamic_predictor"
```

2. **运行时监控**：
   - 订阅 `/dynamic_predictor/current_s_value` 话题查看实时s值
   - 订阅 `/dynamic_predictor/adaptive_metrics` 话题查看Mt和s_adaptive

### 7.2 性能评估

建议在以下场景中测试：
1. 匀速直行（预期：s_adaptive ≈ s_max）
2. 急转弯起始阶段（预期：s_adaptive → s_min）
3. 匀速转弯（预期：s_adaptive在中间值附近）
4. 急停（预期：s_adaptive → s_min）

### 7.3 可视化分析

可以通过查看CSV日志文件（保存在桌面）分析：
- ADE/FDE随时间的变化
- s_adaptive与意图概率的关系
- Mt值的分布情况

---

## 八、与原方案的对比

| 维度 | 原方案 | 新方案 |
|------|--------|--------|
| **核心指标** | NIS、加加速度、意图熵 | 加加速度、角加速度、预测误差 |
| **指标数量** | 2个运动指标 + 1个统计指标 | 3个物理-统计互补指标 |
| **映射函数** | 双层衰减因子(熵约束×运动约束) | 单层Sigmoid平滑映射 |
| **可解释性** | 意图熵较抽象 | 全部为直观的物理量 |
| **实现复杂度** | 较高(需要迭代计算意图熵) | 较低(直接计算物理量) |
| **计算效率** | 中等(涉及协方差矩阵求逆) | 高(仅涉及简单算术运算) |
| **参数数量** | 5个(gamma1/2, lambda1/2, M_thresh) | 8个(w1/2/3, j/alpha/e_max, s_min/max, k, mu) |

---

## 九、注意事项

1. **时间对齐**：确保加速度、角度、位置、速度历史数据的时间戳对齐
2. **单位一致性**：所有物理量必须使用SI单位（米、秒、弧度）
3. **数值稳定性**：Sigmoid函数在极端输入下可能产生数值溢出，代码中已添加安全截断
4. **历史窗口大小**：建议保持 `history_window=1`，更大的窗口会引入延迟
5. **参数敏感性**：j_max、alpha_max、e_max对不同障碍物类型差异较大，需要根据实际情况调整

---

## 十、未来改进方向

1. **自适应归一化上限**：根据障碍物类型自动设置j_max等参数
2. **在线参数学习**：通过历史预测误差自动调整w1/w2/w3权重
3. **多模态扩展**：针对不同意图使用不同的s_adaptive值
4. **GPU加速**：对于大规模障碍物场景，考虑并行化指标计算

---

---

## 十一、重要修复：滑动窗口bug (v2.1)

### 问题发现

用户发现了一个关键问题：在意图预测的迭代过程中，每一帧都应该基于其**前一帧**的历史状态来计算 `s_adaptive`，但原实现使用了**全局历史容器**，导致所有帧共用同一份被不断覆盖的历史数据。

**原bug表现**：
- 只有最后一帧的 `s_adaptive` 是正确的
- 前面所有帧的 `s_adaptive` 都基于错误的历史状态
- 导致意图概率的迭代结果不准确

### 修复方案

1. **删除全局历史容器**：不再使用 `accHistory_`, `angularVelHistory_` 等全局变量
2. **扩展函数签名**：`genTransitionMatrix` 现在接收前一帧的所有必要数据作为参数
3. **修改 `intentProb` 循环**：在遍历历史帧时，为每一帧显式提取三个连续帧的数据（前前帧、前一帧、当前帧）

**详细修复说明**：请参考 `SLIDING_WINDOW_FIX.md`

### 修复效果

**修改前**：
```
第1帧: s_adaptive 基于空历史 → 错误
第2帧: s_adaptive 基于第1帧历史 → 错误
...
第k帧: s_adaptive 基于第k-1帧历史 → 正确
```

**修改后**：
```
第1帧: 无前一帧 → s_adaptive=s_max（默认高惯性）✓
第2帧: 使用第1帧作为前一帧 → s_adaptive 正确 ✓
第3帧: 使用第2帧作为前一帧 → s_adaptive 正确 ✓
...
第k帧: 使用第k-1帧作为前一帧 → s_adaptive 正确 ✓
```

---

**修改完成日期**: 2026-01-08  
**修改作者**: AI Assistant  
**版本**: v2.1 - 基于物理-统计双重诊断的自适应意图惯性机制（滑动窗口修复版）

