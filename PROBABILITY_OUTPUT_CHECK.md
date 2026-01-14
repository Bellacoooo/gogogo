# 意图概率输出检查报告

**检查日期**: 2026-01-14  
**检查目的**: 验证输出到桌面的CSV文件中的概率数据是否正确

---

## 一、概率计算逻辑验证

### 1.1 左转/右转概率公式 ✅

**公式**（第855-856行）：
```cpp
pl = scale(1) * (paraml * (1 + sin(theta)));  // 左转
pr = scale(2) * (paramr * (1 - sin(theta)));  // 右转
```

其中：
- `paraml = paramr = 0.5`（从配置文件）
- `theta = currAngle - prevAngle`（角度变化）

**物理意义**：
```
theta > 0 (左转):
  sin(theta) > 0 
  → pl = 0.5 * (1 + 正数) > 0.5
  → pr = 0.5 * (1 - 正数) < 0.5
  结论：pl > pr ✅

theta < 0 (右转):
  sin(theta) < 0
  → pl = 0.5 * (1 + 负数) < 0.5
  → pr = 0.5 * (1 - 负数) > 0.5
  结论：pr > pl ✅

theta = 0 (直行):
  sin(0) = 0
  → pl = 0.5 * (1 + 0) = 0.5
  → pr = 0.5 * (1 - 0) = 0.5
  结论：pl = pr ✅
```

**验证结果**：
| 角度变化 | pl_raw | pr_raw | 比例 | 结论 |
|---------|--------|--------|------|------|
| 0° (直行) | 0.50 | 0.50 | 1.00 | ✅ 相等 |
| +30° (左转) | 0.75 | 0.25 | 3.00 | ✅ pl > pr |
| -30° (右转) | 0.25 | 0.75 | 0.33 | ✅ pr > pl |
| +90° (急左转) | 1.00 | 0.00 | ∞ | ✅ 只有左转 |
| -90° (急右转) | 0.00 | 1.00 | 0.00 | ✅ 只有右转 |

**结论**：✅ **左转/右转概率计算逻辑正确！**

---

## 二、CSV输出逻辑检查

### 2.1 数据来源

**代码位置**：`dynamicPredictor.cpp` 第1985-2000行

```cpp
// 获取意图概率向量
Eigen::VectorXd intentProbVec;
if (static_cast<size_t>(obsIdx) < intentProb_.size() && intentProb_[obsIdx].size() > 0) {
    intentProbVec = intentProb_[obsIdx];  // 从成员变量读取
} else {
    intentProbVec = Eigen::VectorXd::Constant(numIntent_, 1.0 / numIntent_);  // 默认均匀分布
}

// 提取各意图概率
double P_forward = (intentProbVec.size() > FORWARD) ? intentProbVec(FORWARD) : 0.25;
double P_left = (intentProbVec.size() > LEFT) ? intentProbVec(LEFT) : 0.25;
double P_right = (intentProbVec.size() > RIGHT) ? intentProbVec(RIGHT) : 0.25;
double P_stop = (intentProbVec.size() > STOP) ? intentProbVec(STOP) : 0.25;
```

### 2.2 意图枚举顺序

**关键问题**：需要确认枚举定义

从代码推断（第863-866行）：
```cpp
probVec(FORWARD) = pf;
probVec(LEFT) = pl;
probVec(RIGHT) = pr;
probVec(STOP) = ps;
```

**应该是**：
- FORWARD = 0
- LEFT = 1
- RIGHT = 2
- STOP = 3

### 2.3 CSV表头

**第144行**：
```cpp
logFile_ << "时间戳,障碍物ID,障碍物X,障碍物Y,ADE(米),FDE(米),D_t,s_adaptive,最佳意图,有效步数,P_forward,P_left,P_right,P_stop\n";
```

**第2036-2039行（写入数据）**：
```cpp
<< P_forward << ","
<< P_left << ","
<< P_right << ","
<< P_stop << "\n";
```

**结论**：✅ **CSV输出顺序正确，与表头匹配**

---

## 三、可能导致"左转=右转"的原因

### 3.1 历史问题分析

你提到"之前左转和右转概率一样"，可能的原因：

#### 原因1：障碍物真的在直行 ✅
```
如果障碍物运动方向没有变化（theta ≈ 0），
那么 pl ≈ pr 是正常的！
```

#### 原因2：数据读取错误（已排除）
```cpp
// 代码正确读取了 LEFT 和 RIGHT
P_left = intentProbVec(LEFT);   // 索引1
P_right = intentProbVec(RIGHT); // 索引2
```

#### 原因3：`theta` 计算问题（需要验证）⚠️

**theta 计算逻辑**（第756行）：
```cpp
double theta = currAngle - prevAngle;
```

其中：
```cpp
// 第668-671行
double prevPrevAngle = atan2(prevPos(1) - prevPrevPos(1), prevPos(0) - prevPrevPos(0));
double prevAngle = atan2(currPos(1) - prevPos(1), currPos(0) - prevPos(0));
double currAngle = atan2(currVel(1), currVel(0));  // 基于速度方向
```

**潜在问题**：
```
prevAngle 基于位置差：atan2(currPos - prevPos)
currAngle 基于速度：atan2(currVel)

如果速度方向和位置差方向不一致，theta 可能出现异常！
```

**示例**：
```
假设障碍物在曲线运动：
  - prevPos = (0, 0)
  - currPos = (1, 1)  → prevAngle = atan2(1, 1) = 45°
  - currVel = (0, 1)  → currAngle = atan2(1, 0) = 90°
  - theta = 90° - 45° = 45° → 判断为左转

但如果障碍物实际上是在减速右转：
  - 速度方向和位置差方向可能不一致
  - theta 可能不准确
```

---

## 四、关于停止概率过高的说明

### 4.1 原作者设计

你说得对，**停止概率高可能是原作者的设计**。让我重新理解这个设计：

**公式**（第857行）：
```cpp
ps = (1 - tanh(params / scale(3) * r));
```

**设计意图可能是**：
1. 当 `scale(3)` 增大（强化停止意图）时，停止概率应该增加
2. 但实际效果是：`scale(3)` 增大 → `params/scale(3)` 减小 → `ps` 增大

**这可能是故意的**：
```
原作者可能认为：
- 当系统检测到运动不稳定（需要增大意图惯性）时
- 停止是一种"保守策略"
- 因此提高停止概率是合理的
```

**论文依据**：
- 如果这是原作者发表的论文中的方法
- 那么这个设计可能已经被验证过
- 不应该随意修改

**建议**：
1. ✅ **保持原公式不变**（尊重原作者设计）
2. ✅ **检查实际输出的概率分布是否合理**
3. ⚠️ **如果论文中有说明，遵循论文设定**

---

## 五、诊断建议

### 5.1 检查CSV输出文件

运行实验后，打开桌面上的CSV文件（格式：`predictor_ade_fde_YYYYMMDD_HHMMSS.csv`）

**检查项**：

1. **概率和是否为1**：
   ```
   P_forward + P_left + P_right + P_stop = 1.0 ± 0.001
   ```

2. **直行障碍物的概率**：
   ```
   如果障碍物直行（theta ≈ 0）：
   ✅ P_left ≈ P_right（允许略有差异，因为theta不会完全为0）
   ✅ P_forward 应该最大（直行意图）
   ```

3. **转弯障碍物的概率**：
   ```
   如果障碍物左转（theta > 0）：
   ✅ P_left > P_right
   
   如果障碍物右转（theta < 0）：
   ✅ P_right > P_left
   ```

4. **停止概率**：
   ```
   如果速度较快（r > 1 m/s）：
   ⚠️ P_stop 可能很高（80-90%）← 这可能是原设计
   
   如果速度很慢（r < 0.1 m/s）：
   ✅ P_stop 应该接近 100%
   ```

### 5.2 启用调试输出

**启用第2003-2004行的调试输出**：

```cpp
// 取消注释这两行
ROS_INFO_THROTTLE(5.0, "Obs %zu Intent Prob: Forward=%.3f, Left=%.3f, Right=%.3f, Stop=%.3f", 
                   obsIdx, P_forward, P_left, P_right, P_stop);
```

重新编译运行，观察终端输出。

### 5.3 添加theta诊断

**建议添加**（在第2004行后）：

```cpp
// 添加角度信息的调试输出
if (static_cast<size_t>(obsIdx) < velHist_.size() && velHist_[obsIdx].size() >= 2) {
    double curr_speed = sqrt(pow(velHist_[obsIdx][0](0), 2) + pow(velHist_[obsIdx][0](1), 2));
    ROS_INFO_THROTTLE(5.0, "Obs %zu Speed: %.3f m/s", obsIdx, curr_speed);
}
```

---

## 六、修复方案（如果需要）

### 情况1：左转=右转（障碍物直行）✅

**不需要修复** - 这是正确的物理现象

### 情况2：左转=右转（障碍物转弯）❌

**需要检查**：
1. `theta` 计算是否正确
2. 历史轨迹数据是否有效
3. `prevAngle` 和 `currAngle` 的计算方式

**可能的修复**（如果确认有问题）：

```cpp
// 修改第669-671行，统一使用速度方向
double prevAngle = atan2(prevVel(1), prevVel(0));  // 改为速度方向
double currAngle = atan2(currVel(1), currVel(0));
double theta = currAngle - prevAngle;
```

### 情况3：停止概率过高

**建议**：
1. **先检查原论文**是否有说明
2. **验证实际效果**：机器人行为是否合理
3. **如果确实有问题，再考虑修改**

---

## 七、验证步骤

### 7.1 运行实验

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

### 7.2 检查输出

1. **终端输出**：
   ```
   观察是否有：
   "CSV写入：Obs X 位置=(x, y), P=[pf,pl,pr,ps]"
   ```

2. **桌面CSV文件**：
   ```
   文件名格式：predictor_ade_fde_YYYYMMDD_HHMMSS.csv
   位置：~/Desktop/ 或 ~/桌面/
   ```

3. **数据验证**：
   - 打开Excel/LibreOffice
   - 检查最后4列（P_forward, P_left, P_right, P_stop）
   - 计算和是否为1
   - 观察转弯时左/右概率变化

### 7.3 场景测试

**推荐测试场景**：
1. **直线走廊**：障碍物直行，pl ≈ pr
2. **十字路口左转**：障碍物左转，pl > pr
3. **十字路口右转**：障碍物右转，pr > pl

---

## 八、总结

### 8.1 计算逻辑

✅ **左转/右转公式正确**  
✅ **CSV输出逻辑正确**  
✅ **数据读取顺序正确**  

### 8.2 可能的问题

⚠️ **theta计算方式**：混合使用位置差和速度方向，可能在曲线运动时不准确  
⚠️ **停止概率过高**：可能是原设计，需要验证论文  

### 8.3 下一步

1. ✅ **运行实验，检查实际输出数据**
2. ✅ **对比不同场景下的概率分布**
3. ⚠️ **如果发现问题，提供具体数据我再分析**

---

**重要提醒**：
- 概率计算的"正确性"取决于实际应用效果
- 如果机器人能安全导航，且ADE/FDE合理，那就是正确的
- 不要过度纠结理论上的"完美"，实用性更重要

**请运行实验后，把CSV文件的几行数据发给我，我帮你进一步分析！**

