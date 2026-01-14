# 关键 Bug 修复 v1.2 - 风险计算重复和调试增强

## 🐛 发现的严重问题

### 问题 1: 风险计算在 for j 循环内部（严重性能和逻辑错误）

**症状**：
- 第二次运行时可能崩溃
- 无人机后退（找不到可行路径）
- 性能浪费

**原因**：
风险计算（vc, ttc, s, phi）在 `for(int j=0; j<mpcWindow; j++)` 循环内部，导致：

1. **重复计算 30 次**（mpcWindow=30）
   - 每个 horizon 步 j 都重新计算风险
   - 但实际上应该只计算一次（基于当前时刻）

2. **历史状态被覆盖 30 次**
   ```cpp
   // 在 j=0 时：prevSFilt_[i] = s_filt_0
   // 在 j=1 时：prevSFilt_[i] = s_filt_1  // 覆盖了！
   // ...
   // 在 j=29 时：prevSFilt_[i] = s_filt_29 // 最终值
   ```
   - 下次调用时，`prevSFilt_[i]` 是 j=29 的值，而不是 j=0 的值
   - 导致平滑逻辑错误

3. **使用错误的障碍物位置**
   - 对于 j=10，使用的是未来 10 步的障碍物位置
   - 但计算的是相对于**当前**机器人位置的风险
   - 导致 d, vc, ttc 都不对

**后果**：
- ❌ 平滑逻辑失效（用错误的历史值）
- ❌ 风险计算不准确（用未来位置算当前风险）
- ❌ 性能浪费（计算 30 次本应计算 1 次的东西）
- ❌ 可能导致椭球过大或过小，MPC 找不到可行路径

---

## ✅ 修复方案

### 修复 1: 风险计算移到 j 循环外部

**新逻辑**：
1. **第一步**：对每个障碍物 i，基于 j=0（当前时刻）计算风险
   - 计算 vc, ttc, s_filt, phi_filt
   - 只计算一次，保存结果

2. **第二步**：填充 horizon 内的所有步 j
   - 使用相同的椭球参数（a, b, c, phi）
   - 只更新障碍物位置

**代码结构**：
```cpp
// 第一步：计算风险（每个障碍物一次）
std::vector<double> a_per_obstacle(numDynamicOb);
std::vector<double> b_per_obstacle(numDynamicOb);
std::vector<double> c_per_obstacle(numDynamicOb);
std::vector<double> phi_per_obstacle(numDynamicOb);

for(int i=0; i<numDynamicOb; i++){
    // 使用 j=0 的位置计算风险
    Eigen::Vector3d pi3 = dynamicObstaclesPos[i][0];
    
    // 计算 vc, ttc, s, phi
    // ...
    
    // 保存结果
    a_per_obstacle[i] = a;
    b_per_obstacle[i] = b;
    c_per_obstacle[i] = c;
    phi_per_obstacle[i] = phi;
}

// 第二步：填充 horizon（使用相同参数）
for(int j=0; j<mpcWindow; j++){
    for(int i=0; i<numDynamicOb; i++){
        // 获取步 j 的障碍物位置
        Eigen::Vector3d pi3 = dynamicObstaclesPos[i][j];
        
        // 使用预计算的椭球参数
        osize[j](i,0) = a_per_obstacle[i];
        osize[j](i,1) = b_per_obstacle[i];
        osize[j](i,2) = c_per_obstacle[i];
        yaw[j](i,0) = phi_per_obstacle[i];
    }
}
```

---

### 修复 2: 添加调试输出

**新增输出**：
```cpp
ROS_INFO_THROTTLE(1.0, "[Risk-Adaptive] Obs_%d: d=%.2fm, vc=%.2fm/s, ttc=%.2fs | "
                       "s_raw=%.2f→s_filt=%.2f(Δ%.3f) | phi=%.1f° | "
                       "a=%.2f, b=%.2f, κ=%.2f", 
                       i, d, vc, ttc,
                       s_raw, s_filt, (s_filt - s_prev),
                       phi*180/M_PI,
                       a, b, kappa_eff);
```

**输出含义**：
- `d`: 距离（米）
- `vc`: closing speed（米/秒）
- `ttc`: time to collision（秒）
- `s_raw→s_filt(Δ)`: 原始裕量 → 平滑后裕量（变化量）
- `phi`: 椭球朝向（度）
- `a, b`: 椭球半轴（米）
- `κ`: 各向异性强度

---

## 📊 修复效果预期

### 修复前（v1.1）

```
Time    计算次数    prevSFilt_[0]    实际用的值    问题
0.0s    30次        s_filt(j=29)     错误         用未来位置算风险
0.1s    30次        s_filt(j=29)     错误         历史值错误
崩溃     -           -                -            历史状态混乱
```

### 修复后（v1.2）

```
Time    计算次数    prevSFilt_[0]    实际用的值    问题
0.0s    1次         s_filt(j=0)      正确         ✅
0.1s    1次         s_filt(j=0)      正确         ✅
稳定运行 1次         s_filt(j=0)      正确         ✅
```

---

## 🔍 如何验证修复效果

### 1. 查看调试输出

运行系统后，观察终端输出：

```bash
roslaunch autonomous_flight simulation.launch world:=test_head_on
```

**期望看到**（每秒一次）：

```
[Risk-Adaptive] Obs_0: d=5.23m, vc=2.15m/s, ttc=2.43s | 
                       s_raw=1.05→s_filt=0.78(Δ0.152) | phi=180.0° | 
                       a=1.45, b=0.87, κ=0.35
```

**验证要点**：
- ✅ `vc > 0` 在 head-on 场景（迎面逼近）
- ✅ `ttc` 随着 `d` 减小而减小
- ✅ `s_filt` 比 `s_raw` 更平滑（Δ 应该较小）
- ✅ `phi ≈ 180°` 在 head-on 场景（障碍物朝向相反）
- ✅ `a > b`（朝向障碍运动方向更肥）

### 2. 验证椭球膨胀

**Head-On 场景（迎面冲突）**：

| 距离 d | Closing Speed vc | 期望 s_filt | 期望 a | 期望行为 |
|--------|------------------|-------------|--------|----------|
| 10m | 2.0 m/s | ~0.5m | ~1.2m | 椭球较小 |
| 5m | 2.0 m/s | ~0.8m | ~1.5m | 椭球膨胀 |
| 2m | 2.0 m/s | ~1.2m | ~1.9m | 椭球很大，提前绕行 ✅ |

**如果无人机后退**，检查：
1. `s_max` 是否过大（建议 < 1.5m）
2. `a` 是否过大（a = a0 + s*(1+κ)，a0 ≈ 0.5-0.7m）
3. MPC 是否真的 infeasible（查看 QP 求解状态）

### 3. 对比无人机行为

**修复前**：
- ❌ 椭球大小不稳定（每次不同）
- ❌ 无人机找不到路径，后退
- ❌ 第二次运行崩溃

**修复后**：
- ✅ 椭球大小稳定、平滑变化
- ✅ 无人机能提前绕行
- ✅ 多次运行稳定

---

## 🎯 关于"相对速度方向膨胀"的说明

### 你的理解是对的！

风险自适应的核心思想就是：
```
在相对速度方向（closing speed 大的方向）膨胀更多
→ 椭球长轴朝向障碍物运动方向
→ 无人机能提前识别危险方向并绕行
```

### 为什么可能没看到预期效果？

#### 原因 1: Bug 导致风险计算错误

修复前，使用未来位置计算风险：
```cpp
// j=10 时，用的是未来 10 步的障碍物位置
Eigen::Vector3d pi = dynamicObstaclesPos[i][10];  // 错误！

// 但机器人位置是当前的
Eigen::Vector2d pr(currPos_(0), currPos_(1));

// 算出来的 d, vc 都不对
double d = (pr - pi).norm();  // 这是未来的距离！
```

修复后，正确使用当前位置：
```cpp
// 只用 j=0（当前时刻）的障碍物位置
Eigen::Vector3d pi = dynamicObstaclesPos[i][0];  // 正确！
```

#### 原因 2: Closing Speed 可能计算错误

**检查公式**：
```
vc = max(0, -r̂·(v_r - v_i))
```

**验证**（Head-On 场景）：
- 机器人速度：`vr = (1, 0)` m/s
- 障碍物速度：`vi = (-1, 0)` m/s
- 相对位置：`r = pr - pi = (-3, 0)`（障碍在前方）
- 单位向量：`r_hat = (-1, 0)`
- 相对速度：`v_rel = (1, 0) - (-1, 0) = (2, 0)`
- 投影：`-r_hat·v_rel = -(-1, 0)·(2, 0) = 2.0`
- Closing speed：`vc = max(0, 2.0) = 2.0` ✅

**在日志中验证**：
- Head-On 场景，`vc` 应该 > 0（迎面）
- 如果 `vc = 0`，说明速度估计或公式有问题

#### 原因 3: 参数可能需要调整

如果 `s_max` 太小或 `alpha` 太小，膨胀不够明显：

```yaml
# 当前配置
mpc_planner/risk_s_max: 1.2      # 最大膨胀
mpc_planner/risk_alpha: 0.4      # closing speed 系数

# 如果需要更明显的膨胀，可以增大
mpc_planner/risk_s_max: 1.5      # ↑
mpc_planner/risk_alpha: 0.6      # ↑
```

---

## 📝 调试步骤

### 步骤 1: 清理并重新编译

```bash
cd ~/intent-mpc
catkin_make --pkg trajectory_planner
source devel/setup.bash
```

### 步骤 2: 运行 Head-On 场景

```bash
roslaunch autonomous_flight simulation.launch world:=test_head_on
```

### 步骤 3: 观察日志输出

**查找关键信息**：
```bash
# 实时查看风险自适应日志
rostopic echo /rosout | grep "Risk-Adaptive"
```

**或者在终端直接看**（每秒一次）

### 步骤 4: 分析数据

记录以下数据到表格：

| 时间 | d (m) | vc (m/s) | ttc (s) | s_raw | s_filt | a (m) | b (m) | 行为 |
|------|-------|----------|---------|-------|--------|-------|-------|------|
| 0s | 10.0 | 2.0 | 5.0 | 0.5 | 0.5 | 1.2 | 0.8 | 正常前进 |
| 1s | 8.0 | 2.0 | 4.0 | 0.7 | 0.6 | 1.3 | 0.8 | 正常前进 |
| 2s | 5.0 | 2.0 | 2.5 | 1.0 | 0.8 | 1.5 | 0.9 | 开始绕行 ✅ |
| 3s | 3.0 | 2.0 | 1.5 | 1.2 | 1.0 | 1.7 | 1.0 | 侧向绕行 ✅ |

### 步骤 5: 可视化椭球

在 RViz 中观察 `/mpc_planner/ellipsoid_obstacles`：
- ✅ 椭球应该沿障碍物运动方向变长
- ✅ 随着距离减小，椭球应该膨胀
- ✅ 椭球应该平滑变化（无突变）

---

## 🚨 如果仍然有问题

### 问题 A: 无人机仍然后退

**可能原因**：
1. `s_max` 过大，椭球太大，MPC 找不到路径
2. 静态障碍物约束过紧
3. 速度/加速度限制太小

**解决方案**：
```yaml
# 减小最大膨胀
mpc_planner/risk_s_max: 0.8      # 从 1.2 降到 0.8

# 或者降低 alpha/beta
mpc_planner/risk_alpha: 0.3      # 从 0.4 降到 0.3
```

### 问题 B: 仍然看不到膨胀效果

**检查清单**：
- [ ] `use_risk_adaptive: true`
- [ ] 日志中看到 `[Risk-Adaptive]` 输出
- [ ] `vc > 0` 在 head-on 场景
- [ ] `s_filt` 随 `vc` 和 `ttc` 变化
- [ ] `a > b`（各向异性）

**如果都正常但效果不明显**：
```yaml
# 增大膨胀参数
mpc_planner/risk_alpha: 0.6      # 提高 closing speed 影响
mpc_planner/risk_beta: 0.8       # 提高 TTC 影响
mpc_planner/risk_kappa: 0.5      # 提高各向异性
```

### 问题 C: 第二次运行仍崩溃

**检查**：
- 编译是否成功（无警告）
- 是否使用了最新编译的版本
- 查看崩溃日志（`dmesg` 或 `core dump`）

---

## ✅ 修复完成检查清单

- [x] 风险计算移到 j 循环外部
- [x] 只在 j=0 时计算风险
- [x] 历史状态正确保存和使用
- [x] 添加详细调试输出
- [x] 编译通过
- [ ] 实际测试验证（等待你运行）

---

**版本**: v1.2（关键 Bug 修复）  
**日期**: 2026-01-13  
**状态**: ✅ 已修复并编译通过  
**下一步**: 运行测试，查看调试输出，验证修复效果

