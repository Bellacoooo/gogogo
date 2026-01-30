# 诊断总结报告

## 🚨 从您的数据发现的严重问题

### 问题1：s值严重异常 ❌

```
行1：Mt=0.14, s=4.62  ✅ 正常
行2：Mt=0.61, s=0.31  ❌ **s < s_min(1.2)！严重违反约束**
行3：Mt=0.62, s=2.08  ⚠️ 偏低但可能合理
```

**不合理之处**：
- s=0.31 < s_min=1.2，这在代码中应该被clamp住！
- 代码第636行明确写了：`return std::max(s_min_, std::min(s_adaptive, s_max_));`
- **这个值不可能出现，除非代码根本没运行！**

### 问题2：概率全是0或不归一 ❌

```
行1：P=[0, 0, 0, 0]         → 概率和=0 ❌
行2：P=[0.03, 0.93, 0, 0]   → 概率和=0.96 ⚠️
行3：P=[0, 0, 0, 1]         → 概率和=1.0 ✅
```

**不合理之处**：
- 行1所有概率都是0，说明intentProb_没有正确赋值
- 行2概率和≠1，说明归一化有问题
- 行3虽然和=1，但P_stop=1（停止概率100%），而ADE=0.048m（很小），说明预测很准，不应该是停止意图

### 问题3：ADE与意图分布矛盾 ⚠️

```
行1：ADE=0.014m（1.4cm） + 最佳意图=FORWARD + 但P=[0,0,0,0] ❌
行2：ADE=0.042m（4.2cm） + 最佳意图=STOP + P_left=0.93 ❌ 最佳意图不匹配！
行3：ADE=0.048m（4.8cm） + 最佳意图=STOP + P_stop=1.0 ✅
```

---

## 🔍 **根本原因分析**

### 可能性A：数据来自旧代码（未重启节点）

**最可能的原因**：
- 您编译后**没有重启ROS节点**
- 旧的节点进程仍在运行
- 所以数据还是旧代码生成的

**验证方法**：
```bash
# 1. 查看节点进程启动时间
ps aux | grep dynamic_predictor

# 2. 重启节点
rosnode kill /dynamic_predictor_node
rosrun dynamic_predictor dynamic_predictor_node

# 3. 重新采集数据
```

### 可能性B：代码逻辑有bug导致s<s_min

**但这不太可能**，因为代码有明确的clamp：
```cpp
return std::max(s_min_, std::min(s_adaptive, s_max_));
```

除非s_min_本身就被设置成了0.31？

**验证方法**：
```bash
rosparam get /dynamic_predictor/s_min
# 应该返回1.2
```

### 可能性C：CSV输出有bug

**可能问题**：
- CSV写入时搞混了不同障碍物的数据
- 或者搞混了不同意图的数据

**从代码第2193行看**：
```cpp
double s_adap = (static_cast<size_t>(obsIdx) < currentSAdaptive_.size()) 
                ? currentSAdaptive_[obsIdx] : 1.0;
```

如果`obsIdx`超出范围，会返回默认值1.0，但不会返回0.31。

---

## 🔧 **立即执行的诊断步骤**

### 步骤1：确认节点是否重启 ⭐

```bash
# 1. 查看节点进程
ps aux | grep dynamic_predictor

# 2. 查看进程启动时间（应该是19:55之后）
ls -l /proc/$(pgrep -f dynamic_predictor_node)/exe

# 3. 如果不是最新的，重启节点
rosnode kill /dynamic_predictor_node
roslaunch your_package your_launch.launch
```

### 步骤2：启用调试日志

在代码第842行已经启用了：
```bash
rostopic echo /rosout | grep "Obs.*f_jerk"
```

应该看到：
```
[INFO] Obs 0: f_jerk=0.08, f_angular=0.10, f_error=0.12 => Mt=0.11 => s_adaptive=4.75
```

**关键验证**：
- 如果看到这个日志，说明V3代码在运行
- s_adaptive应该在4-5范围
- 如果没有看到这个日志，说明旧代码还在运行

### 步骤3：检查参数

```bash
# 检查s_min和s_max
rosparam get /dynamic_predictor/s_min  # 应该1.2
rosparam get /dynamic_predictor/s_max  # 应该5.0

# 检查平滑系数
# （这个可能还没加到yaml，在代码里hardcode为0.8）
```

### 步骤4：检查intentProb_

添加临时调试：
```cpp
// 在第2172行后添加
ROS_WARN("DEBUG: obsIdx=%zu, intentProb_.size()=%zu, intentProbVec.size()=%ld, sum=%.3f",
         obsIdx, intentProb_.size(), intentProbVec.size(),
         intentProbVec(0)+intentProbVec(1)+intentProbVec(2)+intentProbVec(3));
```

---

## 💡 **我的诊断结论**

### 最可能的情况（90%概率）：

**您的数据来自旧代码，因为节点没有重启！**

证据：
1. ✅ s=0.31根本不可能出现在新代码中（有clamp）
2. ✅ 编译确实成功了（19:55，库已更新）
3. ✅ 但数据可能是19:55之前采集的，或节点是旧的

### 次可能的情况（10%概率）：

**代码有bug，或者参数设置错误**

但这需要更多证据。

---

## 🎯 **立即行动方案**

```bash
# 1. 杀掉所有相关节点
rosnode kill -a

# 2. 重新启动
roslaunch your_launch_file.launch

# 3. 确认日志
rostopic echo /rosout | grep -E "Obs.*f_jerk|CSV写入"

# 4. 重新采集数据（等待30秒让数据稳定）
sleep 30

# 5. 查看新的CSV文件
tail -20 ~/Desktop/intent_eval_*.csv
```

---

## 📊 **如果重启后还有问题**

请提供：

1. **ROS日志输出**：
```bash
rostopic echo /rosout | grep -A 2 "Obs 0: f_jerk"
```

2. **参数值**：
```bash
rosparam get /dynamic_predictor/s_min
rosparam get /dynamic_predictor/s_max
```

3. **新采集的数据**（重启后的）

---

**诊断日期**：2026-01-15  
**结论**：99%可能是节点未重启导致  
**行动**：立即重启节点并重新采集数据
