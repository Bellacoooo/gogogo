# V3鲁棒方案 - 快速测试指南

## ✅ 已实施的改进

### 1. Jerk → 窗口化Δa

**改动**：`dynamicPredictor.cpp` 第548-560行

```cpp
// 原来：jerk = (a_t - a_{t-1}) / dt_  // 二阶差分，噪声敏感
// 现在：deltaAcc = a_t - a_{t-1}     // 一阶差分，更稳定
```

**物理意义**：
- Δa ≈ jerk × dt（保留了驱动力变化的物理意义）
- 但噪声敏感度降低一个数量级

### 2. Mt的EMA平滑

**改动**：`dynamicPredictor.cpp` 第820-831行

```cpp
// 新增：对Mt进行指数移动平均(EMA)平滑
Mt = 0.8 * Mt_prev + 0.2 * Mt_instant
```

**效果**：
- 消除Mt的逐帧抖动
- 保留意图切换的低频信号
- s变化更平滑

### 3. 保留V2的速度平滑

**改动**：`dynamicPredictor.cpp` 第681-686行

```cpp
// V2：速度平滑，降低角度噪声
smoothedVel = 0.7 * currVel + 0.3 * prevVel
currAngle = atan2(smoothedVel(1), smoothedVel(0))
```

---

## 🧪 测试步骤

### 1. 重新编译

```bash
cd /home/ff/intent-mpc
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 2. 运行测试

启动您的预测节点

### 3. 观察日志输出

```bash
rostopic echo /rosout | grep "Obs"
```

**期望输出**（直线运动）：
```
[INFO] Obs 0: f_jerk=0.08, f_angular=0.10, f_error=0.12 => Mt=0.11 => s=4.75
[INFO] Obs 0: f_jerk=0.06, f_angular=0.08, f_error=0.10 => Mt=0.09 => s=4.88
[INFO] Obs 0: f_jerk=0.07, f_angular=0.09, f_error=0.11 => Mt=0.10 => s=4.82
```

**关键指标**（直线运动）：
- ✅ f_jerk（即f_delta_a）< 0.15
- ✅ f_angular < 0.15
- ✅ Mt < 0.2（平滑后）
- ✅ s > 4.5

### 4. 检查数据文件

**期望的直线运动数据**：

```
时间     ID  X       Y       Mt    s     P_f   P_l   P_r   P_s
19:30:13 0  -3.088  -1.088  0.12  4.7   0.89  0.05  0.04  0.02
19:30:14 0  -3.084  -1.089  0.10  4.8   0.91  0.04  0.03  0.02
19:30:15 0  -3.080  -1.090  0.11  4.7   0.90  0.05  0.03  0.02
```

**对比您之前的数据**：

| 指标 | 修复前 | V3期望 | 改善 |
|------|--------|--------|------|
| Mt | 1.0（异常！） | **0.1-0.2** | **↓80%** |
| s | 1.225 | **4.5-4.9** | **↑300%** |
| P_forward | 0.4 | **>0.85** | **↑112%** |

---

## 📊 诊断方法

### 问题1：Mt还是偏高（>0.3）

**可能原因**：
- f_delta_a还是偏高（加速度噪声大）
- f_angular还是偏高（角度噪声大）
- EMA平滑不够

**解决方案A：调整a_change_max**

如果f_delta_a经常>0.5，说明归一化上限太小：

```cpp
// 在computeJerkFeature函数中
double a_change_max = 3.0;  // 从2.0增大到3.0
```

**解决方案B：增强EMA平滑**

```cpp
// 在genTransitionMatrix函数中
double lambda = 0.9;  // 从0.8增大到0.9，更强平滑
```

**解决方案C：调整归一化上限**

```yaml
# predictor_param.yaml
alpha_max: 6.28  # 从π(3.14)改为2π(6.28)
e_max: 0.5       # 从0.3改为0.5
```

---

### 问题2：匀速转弯时s还是偏低（<4.0）

**症状**：
- 障碍物在做稳定转弯（ω恒定）
- 但f_angular仍然>0.3
- 导致Mt偏高，s偏低

**可能原因**：
- 速度平滑不够（角度还是抖动）
- 角加速度计算有问题

**解决方案A：增强速度平滑**

```cpp
// 在genTransitionMatrix函数中（第681-686行）
Eigen::Vector3d smoothedVel = 0.5 * currVel + 0.5 * prevVel;  // 增强到50%-50%
```

**解决方案B：实施V3完整的角加速度改进**

参考`ROBUST_INTENT_INERTIA_V3.md`的"改进2"，改用速度向量夹角计算。

---

### 问题3：响应太慢（意图切换检测延迟）

**症状**：
- 障碍物开始转弯后，s仍然保持高值
- 几帧后才降低

**可能原因**：
- EMA平滑太强（lambda太大）

**解决方案：降低lambda**

```cpp
double lambda = 0.7;  // 从0.8降低到0.7，响应更快
```

---

## ⚙️ 参数快速调节表

| 症状 | 参数 | 调整方向 | 效果 |
|------|------|---------|------|
| Mt > 0.3（直线） | a_change_max | 2.0 → 3.0 | 降低f_delta_a |
| Mt > 0.3（直线） | lambda | 0.8 → 0.9 | 增强平滑 |
| s < 4.0（转弯） | 速度平滑 | 0.7/0.3 → 0.5/0.5 | 降低角度噪声 |
| 响应慢 | lambda | 0.8 → 0.7 | 加快响应 |
| f_angular高（转弯） | 实施V3完整版 | - | 用角速度夹角 |

---

## 🎯 成功标准

### 直线运动场景

✅ **合格标准**：
- Mt < 0.3（平滑后）
- s > 4.0
- P_forward > 0.80

✅ **优秀标准**：
- Mt < 0.15
- s > 4.5
- P_forward > 0.90

### 匀速转弯场景

✅ **合格标准**：
- Mt < 0.4（转弯中）
- s > 3.5
- 主意图稳定（不频繁跳变）

✅ **优秀标准**：
- Mt < 0.25
- s > 4.0
- 主意图完全稳定

### 转弯开始/结束

✅ **合格标准**：
- Mt快速上升到0.5-0.7
- s快速下降到2.0-3.0
- 1-2帧内检测到意图切换

---

## 📝 调试技巧

### 1. 打印详细信息

取消注释第829行：

```cpp
ROS_INFO_THROTTLE(2.0, "Obs %d: f_jerk=%.3f, f_angular=%.3f, f_error=%.3f => Mt_inst=%.3f, Mt_smooth=%.3f => s=%.3f",
                  obsIdx, f_jerk, f_angular, f_error, Mt_instant, Mt, s_adaptive);
```

### 2. 绘制时间序列图

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('your_data.csv')
df_obs0 = df[df['ID'] == 0]

plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(df_obs0['time'], df_obs0['Mt'], label='Mt')
plt.axhline(y=0.2, color='r', linestyle='--', label='Target (<0.2)')
plt.ylabel('Mt')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(df_obs0['time'], df_obs0['s'], label='s')
plt.axhline(y=4.5, color='g', linestyle='--', label='Target (>4.5)')
plt.ylabel('s_adaptive')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(df_obs0['time'], df_obs0['P_forward'], label='P_forward')
plt.plot(df_obs0['time'], df_obs0['P_left'], label='P_left')
plt.plot(df_obs0['time'], df_obs0['P_right'], label='P_right')
plt.plot(df_obs0['time'], df_obs0['P_stop'], label='P_stop')
plt.ylabel('Intent Prob')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('v3_diagnosis.png', dpi=150)
plt.show()
```

---

## 🚀 如果V3效果显著

**建议进一步优化**（可选）：

1. **实施完整的角加速度改进**
   - 改用速度向量夹角
   - 匀速转弯识别更准确

2. **添加可配置参数**
   ```yaml
   # predictor_param.yaml
   a_change_max: 2.0   # Δa归一化上限
   mt_lambda: 0.8      # Mt平滑系数
   vel_smooth_alpha: 0.7  # 速度平滑系数
   ```

3. **添加自适应参数调节**
   - 根据障碍物类型（行人/汽车）自动调整
   - 根据场景（拥挤/开放）自动调整

---

## ✅ 总结

**V3核心改进**：
1. ✅ Jerk → Δa（降低噪声敏感度）
2. ✅ Mt的EMA平滑（消除高频抖动）
3. ✅ 保留V2速度平滑（双重保护）

**预期效果**：
- Mt从1.0降低到0.1-0.2 ✅
- s从1.225提升到4.5-4.9 ✅
- P_forward从0.4提升到>0.85 ✅

**下一步**：
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
# 运行测试，观察Mt, s, P值
```

---

**测试日期**：2026-01-15  
**版本**：V3 - Δa + EMA平滑  
**状态**：✅ 已实现，待测试
