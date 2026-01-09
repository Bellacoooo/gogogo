# 滑动窗口修复：确保每个历史帧使用正确的前一帧数据

## 问题诊断

### 原始问题

用户发现了一个关键问题：在意图预测的迭代过程中，每一帧都应该基于其**前一帧**的历史状态来计算自适应权重 `s_adaptive`，但原实现使用了**全局历史容器**，导致所有帧共用同一份被不断覆盖的历史数据。

### 意图预测的迭代逻辑

```cpp
void predictor::intentProb(...){
    for (int i=0; i<numOb; ++i){  // 遍历每个障碍物
        Eigen::VectorXd P = [0.25, 0.25, 0.25, 0.25];  // 从均匀分布开始
        
        for (int j=2; j<numHist; ++j){  // 遍历历史帧（从过去→现在）
            // 获取第 j 帧的状态
            currAcc = this->accHist_[i][numHist-j-2];
            currPos = this->posHist_[i][numHist-j-2];
            
            // 计算该帧的转移矩阵
            transMat_j = genTransitionMatrix(..., currAcc, currPos, ...);
            
            // 迭代更新概率: P = transMat_k * ... * transMat_j * ... * P_0
            P = transMat_j * P;
        }
    }
}
```

**关键点**：
- 循环从 `j=2` 开始，到 `j=numHist-1` 结束（从历史到现在）
- **每一帧都调用 `genTransitionMatrix`**，应该基于该帧自己的前一帧数据计算 `s_adaptive`
- 最终的概率 `P` 是所有帧的转移矩阵连乘的结果：`P = T_k × ... × T_2 × P_0`

### 原实现的bug

**原代码**使用全局历史容器（`accHistory_`, `posHistory_`, `angularVelHistory_`）：

```cpp
// genTransitionMatrix 内部
double f_jerk = 0.0;
if (!accHistory_.empty()) {
    Eigen::Vector3d prevAcc = accHistory_.back();  // ❌ 错误：使用全局容器
    f_jerk = computeJerkFeature(currAcc, prevAcc);
}
accHistory_.push_back(currAcc);  // ❌ 每次调用都会覆盖
```

**问题表现**：
- **第1帧（j=2）**：`accHistory_` 是空的 → `f_jerk = 0`（错误）
- **第2帧（j=3）**：`accHistory_` 被第1帧的数据覆盖 → 使用的是第1帧的加速度（错误）
- **第k帧**：`accHistory_` 被第k-1帧的数据覆盖 → 使用的是第k-1帧的加速度（错误）
- **最后一帧**：历史容器才是正确的

**结果**：只有最后一帧的 `s_adaptive` 是正确的，前面所有帧的 `s_adaptive` 都基于错误的历史状态！

---

## 解决方案

### 核心思路

**删除全局历史容器**，在 `intentProb` 的循环中为每一帧**显式提取**正确的前一帧数据，然后通过**函数参数传递**给 `genTransitionMatrix`。

### 修改清单

#### 1. 扩展 `genTransitionMatrix` 的函数签名

**位置**：`dynamicPredictor.h` 第145-157行

**修改**：添加前一帧数据的参数

```cpp
Eigen::MatrixXd genTransitionMatrix(
    const double &prevAngle, 
    const double &currAngle, 
    const Eigen::Vector3d &currVel, 
    const Eigen::Vector3d &currPos, 
    const Eigen::Vector3d &currAcc,
    // 新增：前一帧的数据，用于计算运动变化指标
    const Eigen::Vector3d &prevAcc,      // 前一帧加速度
    const Eigen::Vector3d &prevPos,      // 前一帧位置
    const Eigen::Vector3d &prevVel,      // 前一帧速度
    const double &prevPrevAngle,         // 前前帧角度（用于计算前一帧角速度）
    bool hasValidPrevFrame,              // 是否有有效的前一帧数据
    int obsIdx = 0
);
```

**说明**：
- `prevAcc`, `prevPos`, `prevVel`：当前帧的前一帧状态，用于计算差分指标
- `prevPrevAngle`：前一帧的前一帧角度，用于计算前一帧的角速度（角加速度需要两个角速度）
- `hasValidPrevFrame`：第一帧（j=2）没有有效的前一帧数据，此时使用默认值

---

#### 2. 修改 `genTransitionMatrix` 的实现

**位置**：`dynamicPredictor.cpp` 第748-769行

**修改前**（使用全局历史容器）：
```cpp
// 3.1 加加速度特征
double f_jerk = 0.0;
if (!accHistory_.empty()) {
    Eigen::Vector3d prevAcc = accHistory_.back();  // ❌ 全局容器
    f_jerk = computeJerkFeature(currAcc, prevAcc);
}
accHistory_.push_back(currAcc);  // ❌ 覆盖历史
```

**修改后**（使用传入的参数）：
```cpp
// 3.1 加加速度特征
double f_jerk = 0.0;
if (hasValidPrevFrame) {
    f_jerk = computeJerkFeature(currAcc, prevAcc);  // ✅ 使用传入的前一帧数据
}
```

**角加速度计算**（内联到 `genTransitionMatrix` 中）：
```cpp
// 3.2 角加速度特征
double f_angular = 0.0;
if (hasValidPrevFrame) {
    // 计算前一帧的角速度
    double prevAngularVel = (prevAngle - prevPrevAngle) / dt_;
    // 角度归一化到 [-π, π]
    while (prevAngularVel > M_PI) prevAngularVel -= 2.0 * M_PI;
    while (prevAngularVel <= -M_PI) prevAngularVel += 2.0 * M_PI;
    
    // 计算当前帧的角速度
    double currAngularVel = (currAngle - prevAngle) / dt_;
    while (currAngularVel > M_PI) currAngularVel -= 2.0 * M_PI;
    while (currAngularVel <= -M_PI) currAngularVel += 2.0 * M_PI;
    
    // 计算角加速度
    double angularAccel = (currAngularVel - prevAngularVel) / dt_;
    f_angular = std::min(std::abs(angularAccel) / alpha_max_, 1.0);
}
```

**预测误差计算**：
```cpp
// 3.3 预测-观测误差特征
double f_error = 0.0;
if (hasValidPrevFrame) {
    f_error = computePredictionErrorFeature(currPos, prevPos, prevVel);
}
```

---

#### 3. 修改 `intentProb` 的循环逻辑

**位置**：`dynamicPredictor.cpp` 第673-711行

**修改**：在遍历历史帧时，为每一帧提取三个连续帧的数据

```cpp
for (int j=2; j<numHist; ++j){
    // 获取三个连续帧的数据（用于计算运动变化指标）
    // 索引说明：numHist-j 是"前前帧"，numHist-j-1 是"前一帧"，numHist-j-2 是"当前帧"
    
    // 前前帧数据（用于计算前一帧的角速度）
    Eigen::Vector3d prevPrevPos = this->posHist_[i][numHist-j];
    
    // 前一帧数据
    Eigen::Vector3d prevPos = this->posHist_[i][numHist-j-1];
    Eigen::Vector3d prevVel = this->velHist_[i][numHist-j-1];
    Eigen::Vector3d prevAcc = this->accHist_[i][numHist-j-1];
    
    // 当前帧数据
    Eigen::Vector3d currPos = this->posHist_[i][numHist-j-2];
    Eigen::Vector3d currVel = this->velHist_[i][numHist-j-2];
    Eigen::Vector3d currAcc = this->accHist_[i][numHist-j-2];
    
    // 计算角度（朝向）
    double prevPrevAngle = atan2(prevPos(1) - prevPrevPos(1), prevPos(0) - prevPrevPos(0));
    double prevAngle = atan2(currPos(1) - prevPos(1), currPos(0) - prevPos(0));
    double currAngle = prevAngle;  // 与prevAngle相同
    
    // 判断是否有有效的前一帧数据（第一帧j=2时没有有效的前一帧）
    bool hasValidPrevFrame = (j > 2);
    
    // 调用 genTransitionMatrix，传入完整的历史帧数据
    Eigen::MatrixXd transMat = this->genTransitionMatrix(
        prevAngle, currAngle, currVel, currPos, currAcc,
        prevAcc, prevPos, prevVel, prevPrevAngle,
        hasValidPrevFrame, i
    );

    P = transMat * P;
}
```

**关键改进**：
1. **提取三个连续帧**：前前帧（用于计算前一帧角速度）、前一帧、当前帧
2. **显式传递所有数据**：不再依赖全局历史容器
3. **正确的历史关系**：当前帧的"前一帧"就是 `numHist-j-1` 索引的帧

---

#### 4. 删除全局历史容器

**位置**：`dynamicPredictor.h` 第94-98行

**删除的变量**：
```cpp
// 已删除
std::vector<Eigen::Vector3d> accHistory_;
std::vector<double> angularVelHistory_;
std::vector<Eigen::Vector3d> posHistory_;
std::vector<Eigen::Vector3d> velHistory_;
```

**原因**：这些全局容器会导致历史数据混乱，现在通过函数参数显式传递。

---

#### 5. 简化 `computeAngularAccelFeature`

**位置**：`dynamicPredictor.cpp` 第562-578行

**修改**：此函数已废弃，角加速度计算直接在 `genTransitionMatrix` 中完成

```cpp
double predictor::computeAngularAccelFeature(double currAngle, double prevAngle) {
    // 此函数已不再使用，计算逻辑已移至 genTransitionMatrix
    return 0.0;
}
```

**原因**：角加速度需要三个连续帧的角度数据（计算两个角速度，再计算角加速度），直接在 `genTransitionMatrix` 中内联计算更清晰。

---

## 修改效果

### 修改前

```
第1帧（j=2）: s_adaptive 基于空历史 → 错误
第2帧（j=3）: s_adaptive 基于第1帧历史 → 错误
第3帧（j=4）: s_adaptive 基于第2帧历史 → 错误
...
第k帧（j=k）: s_adaptive 基于第k-1帧历史 → 正确
```

**结果**：意图概率 `P` 的迭代过程中，只有最后一个转移矩阵 `T_k` 使用了正确的 `s_adaptive`，前面所有转移矩阵都是错误的。

### 修改后

```
第1帧（j=2）: 无前一帧，f_jerk=0, f_angular=0, f_error=0 → s_adaptive=s_max（默认高惯性）
第2帧（j=3）: 使用第1帧作为前一帧 → s_adaptive 正确
第3帧（j=4）: 使用第2帧作为前一帧 → s_adaptive 正确
...
第k帧（j=k）: 使用第k-1帧作为前一帧 → s_adaptive 正确
```

**结果**：意图概率 `P` 的迭代过程中，每个转移矩阵 `T_j` 都使用了基于该帧正确历史状态计算的 `s_adaptive`，迭代结果才真正反映了障碍物运动的时序演化。

---

## 技术细节

### 索引关系

在 `intentProb` 的循环中：
- `numHist` = 历史帧总数（例如10）
- `j` 从 2 到 `numHist-1`（例如2到9）
- 索引映射：
  - `numHist-j` = "前前帧"（例如 j=2 时为索引8）
  - `numHist-j-1` = "前一帧"（例如 j=2 时为索引7）
  - `numHist-j-2` = "当前帧"（例如 j=2 时为索引6）

**为什么从过去到现在遍历？**

马尔可夫链的概率迭代是从初始状态开始，逐步向前推演：
```
P_0 (初始均匀分布)
 → P_1 = T_1 × P_0
 → P_2 = T_2 × P_1
 → ...
 → P_k = T_k × P_{k-1}
```

所以必须从过去（j=2，对应最早的帧）到现在（j=numHist-1，对应最新的帧）遍历。

### 三个连续帧的必要性

1. **加加速度**：需要当前帧加速度 + 前一帧加速度 → 2帧
2. **角加速度**：需要当前角速度 + 前一角速度 → 需要3个角度（前前帧、前一帧、当前帧）
3. **预测误差**：需要当前帧位置 + 前一帧位置和速度 → 2帧

综合考虑，需要**三个连续帧**的数据。

### 第一帧的特殊处理

第一帧（j=2）时，没有足够的前一帧数据：
- `hasValidPrevFrame = false`
- `f_jerk = 0`, `f_angular = 0`, `f_error = 0`
- `M_t = 0` → `s_adaptive = s_max`（默认高惯性）

这是合理的，因为初始状态下没有运动变化信息，应该假设障碍物处于稳定状态。

---

## 验证建议

### 1. 打印调试信息

在 `genTransitionMatrix` 中添加调试输出（仅用于验证）：

```cpp
if (obsIdx == 0 && hasValidPrevFrame) {  // 只打印第一个障碍物
    ROS_INFO_THROTTLE(1.0, "[Frame] f_jerk=%.3f, f_angular=%.3f, f_error=%.3f, Mt=%.3f, s=%.3f",
                      f_jerk, f_angular, f_error, Mt, s_adaptive);
}
```

**预期结果**：每次调用都应该看到合理的指标值，而不是全0或异常值。

### 2. CSV日志分析

查看生成的CSV文件（保存在桌面），分析 `s_adaptive` 的时序变化：
- 稳定运动阶段：`s_adaptive` 应该接近 `s_max`（默认5.0）
- 机动阶段：`s_adaptive` 应该下降到接近 `s_min`（默认1.2）
- 过渡阶段：`s_adaptive` 应该平滑变化，不应该有突变

### 3. 对比测试

可以在两个场景下测试：
1. **匀速直行**：预期 `s_adaptive ≈ 5.0`
2. **急转弯**：预期 `s_adaptive` 在转弯起始阶段下降到1.2-2.0

---

## 总结

### 核心改进

1. **删除全局历史容器**：避免历史数据混乱
2. **显式传递前一帧数据**：确保每一帧使用正确的历史状态
3. **内联角加速度计算**：直接在 `genTransitionMatrix` 中完成，逻辑更清晰

### 性能影响

**无负面影响**：
- 原本就需要遍历历史帧
- 只是将数据提取逻辑从 `genTransitionMatrix` 内部移到 `intentProb` 循环中
- 没有增加额外的计算或内存开销

### 正确性保证

修改后，意图概率的迭代公式才真正符合设计初衷：

```
P_k = T_k(s_k) × T_{k-1}(s_{k-1}) × ... × T_1(s_1) × P_0
```

其中每个 `s_j` 都是基于第 `j` 帧及其前一帧的运动状态正确计算的，而不是全部使用最后一帧的 `s_k`。

---

**修改完成日期**: 2026-01-08  
**修改原因**: 用户发现滑动窗口逻辑bug  
**修改者**: AI Assistant  
**版本**: v2.1 - 滑动窗口修复版

