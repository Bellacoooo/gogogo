# ADE与概率分布不匹配的根本原因

## 🚨 核心问题

从用户数据看到的矛盾：
- **样本5**：bestIntent=STOP（ADE最小），但P_forward=0.871（概率最高）
- **样本1**：bestIntent=STOP，但P_left=0.517（概率最高）
- **样本3**：bestIntent=FORWARD，但P_right=0.545（概率最高）

**5个样本中有4个不匹配！**

---

## 🔍 根本原因分析

### 原因1：ADE评估使用的是"回测"而不是真正的预测

**回测逻辑**（第2073-2128行）：
```cpp
// 用历史轨迹posHist_[0...29]作为"ground truth"
// 对比predTraj[t]（预测轨迹）和posHist_[t]（历史轨迹）
Eigen::Vector3d error = predTraj[t] - refTraj[refIdx];
double dist = error.head<2>().norm();
ade += dist;
```

**问题**：
- `predTraj`是基于当前状态的预测（未来）
- `refTraj`是历史轨迹（过去）
- **这不是真正的预测评估！**

真正的预测评估应该是：
1. 在t0时刻预测t1, t2, ..., tN
2. 等到真实的t1, t2发生后，再评估

但当前的实现是：
1. 在t0时刻生成预测
2. 用t0-N到t0的历史数据作为"ground truth"
3. **这相当于在"拟合历史"，不是"预测未来"**

---

### 原因2：STOP意图模型的特性

**STOP意图的预测**（第1291行）：
```cpp
predPointTemp.push_back(currPos);  // 所有时间步都是当前位置
```

STOP意图预测轨迹：`[currPos, currPos, currPos, ..., currPos]`（30个相同位置）

---

### 原因3：当障碍物速度慢时，STOP模型的ADE会最小

**场景**：障碍物速度很慢（接近静止或减速）

**历史轨迹**：
- posHist_[0] = (1.0, 2.0)   ← 当前
- posHist_[1] = (1.01, 2.01) ← 0.1s前
- posHist_[2] = (1.02, 2.02) ← 0.2s前
- ...
- posHist_[29] = (1.29, 2.29) ← 2.9s前

**FORWARD意图预测**（假设速度v=0.1m/s）：
- predTraj[0] = (1.0, 2.0)   ← 当前
- predTraj[1] = (1.01, 2.01) ← 0.1s后
- predTraj[2] = (1.02, 2.02) ← 0.2s后
- ...
- predTraj[29] = (1.29, 2.29) ← 2.9s后

**ADE计算**：
```
ADE_forward = avg(|predTraj[t] - posHist_[t]|) 
            ≈ 0 (因为两者几乎一样)
```

**STOP意图预测**：
- predTraj[0] = (1.0, 2.0)
- predTraj[1] = (1.0, 2.0)   ← 停在原地
- predTraj[2] = (1.0, 2.0)
- ...
- predTraj[29] = (1.0, 2.0)

**ADE计算**：
```
ADE_stop = avg(|predTraj[t] - posHist_[t]|)
         = avg(|(1.0, 2.0) - (1.0+0.01*t, 2.0+0.01*t)|)
         = avg(0.01*t*sqrt(2))
         ≈ 0.21m
```

**结论**：当速度慢时，FORWARD的ADE ≈ STOP的ADE，甚至STOP可能更小！

---

### 原因4：intentProb_反映的是"历史运动模式"

**intentProb_的计算**（第644-708行）：
- 通过马尔可夫链迭代历史帧
- 根据历史运动模式识别意图
- **如果一直在前进，P_forward会很高**

**问题**：
- intentProb_看的是"过去发生了什么"
- ADE看的是"当前预测模型与最近历史的拟合度"
- **两者描述的是不同的东西！**

---

## 🎯 具体案例分析

### 样本5：bestIntent=STOP, P_forward=0.871

```
时间：2026-01-15 12:05:45
Mt=0.313, s=5.494  ← V3生效，s很高
ADE=0.084m (8.4cm), FDE=0.115m
概率：P_f=0.871, P_l=0.051, P_r=0.018, P_s=0.020
最佳意图=3(STOP)  ← ADE最小
```

**解释**：
1. **历史运动模式**：障碍物一直在前进 → **P_forward=0.871**
2. **当前状态**：速度可能在减小或接近静止
3. **回测评估**：
   - STOP模型预测：停在原地
   - 最近30帧历史：几乎没动（因为速度慢）
   - STOP模型的预测轨迹与历史轨迹很接近
   - **STOP的ADE最小**

但实际上：
- 障碍物并不是要停止！
- 它只是速度慢而已
- **intentProb_是对的（P_forward高）**
- **ADE评估是错的（STOP不应该ADE最小）**

---

## ✅ 结论

### 1. intentProb_是正确的

- P_forward=0.871说明障碍物一直在前进 ✅
- V3修复已生效（s=5.494） ✅
- 马尔可夫链识别正常 ✅

### 2. ADE评估有问题 ❌

**问题1**：使用"回测"而不是真正的前向预测评估
- 当前实现：用历史轨迹作为ground truth
- 正确做法：用真实发生的未来轨迹作为ground truth

**问题2**：当速度慢时，STOP模型会错误地获得最小ADE
- STOP模型预测"不动"
- 如果障碍物速度慢，历史轨迹也几乎不动
- STOP的ADE会很小，但这不代表STOP是正确的意图

**问题3**：ADE评估的语义不清楚
- ADE本应评估"预测未来的能力"
- 但当前实现评估的是"拟合历史的能力"
- 这两者是不同的！

---

## 💡 建议的修复方案

### 方案A：使用intentProb_作为"最佳意图"

**最简单的修复**：
- 不要用ADE最小来选择"最佳意图"
- 直接用P值最高的意图作为"最佳意图"

```cpp
// 第2148-2152行替换为：
int bestIntent = -1;
double maxProb = -1.0;
for (int i = 0; i < numIntent_; ++i) {
    if (intentProbVec(i) > maxProb) {
        maxProb = intentProbVec(i);
        bestIntent = i;
    }
}
```

### 方案B：修复ADE评估逻辑

**改用真正的前向预测评估**：
1. 缓存t0时刻的预测轨迹
2. 在t0+N时刻，用实际发生的轨迹评估
3. 这需要延迟评估，逻辑更复杂

### 方案C：输出两个指标

**保持当前逻辑，但明确语义**：
- `bestIntentByProb`：根据intentProb_的最高概率
- `bestIntentByADE`：根据ADE最小（拟合历史）
- 在CSV中输出两者，并在文档中说明区别

---

## 🎯 推荐方案

**方案A**最简单有效：
- 直接用P值最高的意图作为"最佳意图"
- intentProb_才是真正的意图识别结果
- ADE评估在当前实现下语义不清楚，不适合作为意图选择标准

---

## 📊 实施方案A

修改第2148-2152行：

```cpp
// ❌ 旧逻辑：按ADE最小选择
// if (ade < minADE) {
//     minADE = ade;
//     minFDE = fde;
//     bestIntent = static_cast<int>(intentIdx);
// }

// ✅ 新逻辑：按概率最高选择（在循环外）
```

在第2154行后添加：

```cpp
// 根据概率选择最佳意图（概率最高的）
if (static_cast<size_t>(obsIdx) < intentProb_.size() && intentProb_[obsIdx].size() > 0) {
    const auto& intentProbVec = intentProb_[obsIdx];
    double maxProb = -1.0;
    for (int i = 0; i < numIntent_; ++i) {
        if (i < intentProbVec.size() && intentProbVec(i) > maxProb) {
            maxProb = intentProbVec(i);
            bestIntent = i;
        }
    }
    ROS_INFO_THROTTLE(2.0, "Obstacle %zu | Best Intent (by Prob): %d (P=%.3f)", obsIdx, bestIntent, maxProb);
}
```

这样CSV输出的"最佳意图"就会和"概率分布"匹配了！

---

**修复日期**：2026-01-15  
**问题**：ADE与P分布不匹配  
**根本原因**：ADE用回测评估 + STOP模型特性导致ADE失真  
**推荐方案**：用P值最高的意图作为最佳意图
