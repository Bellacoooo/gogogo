# 🔍 问题诊断指南

## 你报告的问题

1. ❌ **看不到椭球可视化**
2. ⚠️ **感觉椭球变得很大**
3. 💥 **稍微靠近障碍物运动方向就崩了**

---

## 我们修改了什么？（重要！）

### ✅ 我们**只改了这些**

1. **`mpcPlanner.h`** - 添加成员变量（风险自适应参数、历史值）
2. **`mpcPlanner.cpp`**:
   - `initParam()` - 加载参数
   - `updateObstacleParam()` - ⭐ **核心修改**：根据相对运动计算椭球大小和朝向
   - `publishEllipsoidObstacles()` - 添加了调试输出（可视化逻辑本身没改）
3. **`planner_param.yaml`** - 添加风险自适应参数

### ❌ 我们**没改这些**（MPC 核心）

- ✅ MPC 求解器 (`solveMPC()`) - **完全没动**
- ✅ 目标函数 (`computeObjective()`) - **完全没动**
- ✅ 动态模型 (`updateDynamics()`) - **完全没动**
- ✅ 椭球约束的数学形式 - **完全没动**
- ✅ 静态障碍物处理 - **完全没动**

**我们只做了一件事**：
> 在 `updateObstacleParam()` 中，根据你提供的公式计算风险，然后更新 `osize` 和 `yaw`。

详细说明 → [WHAT_WE_CHANGED.md](WHAT_WE_CHANGED.md)

---

## 诊断步骤

### 1. 运行测试

```bash
cd ~/intent-mpc
source devel/setup.bash

# 清理之前的日志
rm -f ~/.ros/log/latest/*.log

# 运行测试
roslaunch autonomous_flight simulation.launch world:=test_head_on
```

### 2. 观察日志（**非常重要！**）

#### 日志关键词和含义

##### A. 椭球参数计算（每秒一次）
```
[Risk-Adaptive] Obs_0: d=X.XXm, vc=X.XXm/s, ttc=X.XXs | 
                       s_raw=X.XX→s_filt=X.XX(ΔX.XXX) | phi=X.X° | 
                       a=X.XX, b=X.XX, κ=X.XX
```

**含义**：
- `d` - 距离（米）
- `vc` - closing speed（m/s），越大越危险
- `ttc` - time to collision（秒），越小越危险
- `s_raw` - 原始风险裕量
- `s_filt` - 滤波后的风险裕量
- `a, b` - 椭球半轴（米）⚠️ **如果超过 2m 就很大了！**
- `κ` - 各向异性强度

**正常范围**：
- `a, b` 应该在 0.5m ~ 1.5m 之间
- 如果 `a > 2m` 或 `b > 2m`，说明**椭球过大**！

##### B. 椭球过大预警（0.5秒一次）
```
⚠️ Obs_0: Ellipsoid TOO LARGE! max_radius=X.XXm, dist=X.XXm (XX%) - MPC may RETREAT!
```

**含义**：椭球半径超过距离的 70%，MPC 可能找不到前进路径

**如果看到这个**：说明参数太激进了！

##### C. 机器人倒退检测（0.5秒一次）
```
⚠️ Robot RETREATING! vr=(X.XX, X.XX) m/s
```

**含义**：机器人 x 方向速度为负，正在倒退

**如果看到这个**：说明 MPC 被逼无奈，只能倒退了

##### D. 数值安全保护触发
```
⚠️ Obs_0: distance too close (X.XXXm), using conservative ellipsoid
⚠️ Obs_0: Ellipsoid too large! a=X.XX, b=X.XX, c=X.XX. Clamping to X.XXm
❌ Obs_0: NaN/Inf detected! Using baseline ellipsoid
```

**含义**：计算出现极端情况，触发了保护机制

##### E. 椭球参数保存（每秒一次）
```
[updateObstacleParam] 💾 Saved X dynamic obstacles, horizon=X steps
   Obs_0 @ t=0: osize=(X.XX,X.XX,X.XX), yaw=X.X°
```

**含义**：显示最终写入 MPC 的椭球参数

##### F. 可视化发布状态（每秒一次）
```
[Vis] ✅ Publishing ellipsoids: X horizon steps, obstacles at t=0: X
[Vis] Obs_0 @ t=0: pos=(X.XX,X.XX,X.XX), size=(a=X.XX,b=X.XX,c=X.XX), yaw=X.X°
```

**含义**：显示可视化正在发布的椭球

**如果看到**：
```
[Vis] ❌ No ellipsoid data to visualize (lastOxyz/Osize/Yaw empty)
```
说明可视化数据为空，这不正常！

---

### 3. 可视化检查

#### RViz 设置
1. 确保订阅了 Topic：`/mpc_planner/ellipsoid_obstacles`
2. 类型：`MarkerArray`
3. 颜色：应该能看到半透明的椭球

#### 预期效果
- 椭球应该**围绕障碍物**
- 椭球应该**动态变化**（大小和朝向）
- Head-on 时，椭球在障碍物运动方向应该更长

#### 如果看不到可视化
检查日志中是否有：
- `[Vis] ✅ Publishing ellipsoids` - 如果有，说明在发布
- `[Vis] ❌ No ellipsoid data` - 如果有，说明数据为空

---

## 问题分析

### 问题 1：看不到可视化

**可能原因**：
1. RViz 没有订阅正确的 Topic
2. 椭球太大，超出了视野范围
3. 数据为空（日志会显示 `❌ No ellipsoid data`）

**解决方法**：
1. 检查日志，看是否有 `[Vis] ✅ Publishing ellipsoids`
2. 在 RViz 中调整视角，放大视野
3. 检查 `rostopic hz /mpc_planner/ellipsoid_obstacles` 是否有数据

---

### 问题 2：椭球变得很大

**诊断**：看日志中的 `a=X.XX, b=X.XX`

**如果 a 或 b > 2.0m**：参数太激进了！

**原因分析**：
```
椭球大小 = 基线尺寸 + 风险裕量 * (1 + 各向异性)

a = (obstacle_size_x/2 + dynamicSafetyDist) + s * (1 + κ)

例如：
基线 = 0.5m
s = 0.8m (当前最大值)
κ = 0.2 (当前值)

a_max = 0.5 + 0.8 * (1 + 0.2) = 0.5 + 0.96 = 1.46m

如果还觉得大，说明：
1. 基线尺寸本身就很大（obstacle_size + dynamicSafetyDist）
2. 或者 s 达到了最大值 0.8m
```

**解决方法**：
1. 降低 `risk_s_max`：从 0.8 降到 0.5
2. 降低 `risk_kappa`：从 0.2 降到 0.1
3. 或者**临时禁用**风险自适应：
   ```yaml
   mpc_planner/use_risk_adaptive: false
   ```

---

### 问题 3：靠近障碍物就崩

**诊断**：看日志中是否有

1. **椭球过大预警**：
   ```
   ⚠️ Ellipsoid TOO LARGE! max_radius=X.XXm, dist=X.XXm
   ```

2. **倒退检测**：
   ```
   ⚠️ Robot RETREATING!
   ```

**崩溃流程**：
```
1. 障碍物接近
2. s 增大（风险增大）
3. 椭球半径增大
4. 椭球半径 > 70% 距离 → ⚠️ 预警
5. MPC 找不到前进路径
6. MPC 选择倒退 → ⚠️ 倒退检测
7. 持续倒退 → 超出边界或碰撞 → 💥 崩溃
```

**解决方法**：
1. **立即**降低参数（见下方"紧急修复"）
2. 或者临时禁用风险自适应

---

## 紧急修复方案

### 方案 A：降低参数（推荐）

编辑 `planner_param.yaml`：

```yaml
# 从 v1.4 进一步降低
mpc_planner/risk_s_max: 0.5      # 从 0.8 降到 0.5
mpc_planner/risk_kappa: 0.10     # 从 0.20 降到 0.10
mpc_planner/risk_alpha: 0.15     # 从 0.25 降到 0.15
mpc_planner/risk_beta: 0.3       # 从 0.4 降到 0.3
```

**效果**：
- 椭球最大半径从 1.46m 降低到 1.05m
- 降低约 **28%**

---

### 方案 B：完全禁用（如果问题仍然存在）

编辑 `planner_param.yaml`：

```yaml
mpc_planner/use_risk_adaptive: false  # 改为 false
```

**效果**：
- 完全回退到原始的固定椭球
- 如果禁用后仍然崩溃，说明**不是我们的创新导致的**！

---

## 日志收集方法

运行测试后，收集日志：

```bash
# 收集最新的日志
cd ~/.ros/log/latest/
grep -E "\[Risk-Adaptive\]|\[Vis\]|WARN|ERROR" *.log > ~/intent-mpc/diagnosis_log.txt
cat ~/intent-mpc/diagnosis_log.txt
```

然后把 `diagnosis_log.txt` 的内容发给我。

---

## 下一步

### 测试流程

1. **运行测试**：
   ```bash
   roslaunch autonomous_flight simulation.launch world:=test_head_on
   ```

2. **观察终端输出**，重点看：
   - 椭球大小（`a=X.XX, b=X.XX`）
   - 是否有过大预警（`TOO LARGE`）
   - 是否有倒退警告（`RETREATING`）

3. **如果椭球过大**：
   - 立即 Ctrl+C 停止
   - 修改参数（方案 A）
   - 重新测试

4. **如果仍然崩溃**：
   - 禁用风险自适应（方案 B）
   - 重新测试
   - 如果禁用后不崩溃 → 说明是参数问题
   - 如果禁用后仍崩溃 → 说明不是我们的创新导致的

5. **收集日志**并发给我

---

## 重要提醒

### 我们没有修改原始 MPC 逻辑

- ✅ 求解器完全没动
- ✅ 目标函数完全没动
- ✅ 动态模型完全没动
- ⚠️ 只改了椭球参数的**计算方式**

### 如果禁用风险自适应

系统会完全回退到原始行为，就像我们什么都没改一样。

---

**当前状态**：✅ 编译通过，已添加详细调试输出  
**下一步**：请你运行测试并观察日志



