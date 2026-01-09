# MPC 崩溃问题修复总结

## 问题描述

**症状**：
- 无人机规划出路线后不飞
- 直接"闪现"回原点 (0,0,0)
- RViz 不崩溃，但无人机回到起点

**系统日志**：
```
[19105] general protection fault in libosqp.so
```

---

## 根本原因

### 1. MPC 求解器崩溃

**崩溃位置**: `libosqp.so`（OSQP 二次规划求解器）

**触发条件**:
- 静态地图加载：`output.pcd`
- 静态约束过硬：`static_constraint_slack_ratio = 0.01`（几乎是硬约束）
- 约束冲突 → 数值奇异 → 内存访问错误 → 程序崩溃

### 2. 崩溃后的连锁反应

```
OSQP 求解器崩溃
    ↓
mpc_navigation_node 进程退出
    ↓
控制指令停止发布
    ↓
Gazebo 检测到控制中断
    ↓
无人机重置回初始位置 (0, 0, 0.1)
    ↓
用户看到"闪现回原点"
```

---

## 修复方案

### ✅ 修复 1: 禁用静态地图

**文件**: `mapping_param.yaml`

**修改前**:
```yaml
prebuilt_map_directory: "/home/ff/intent-mpc/output.pcd"
```

**修改后**:
```yaml
prebuilt_map_directory: "No"  # ✅ 禁用静态地图
```

**效果**：
- 减少约束数量
- 降低数值复杂度
- 避免静态障碍物干扰

---

### ✅ 修复 2: 增大松弛比

**文件**: `planner_param.yaml`

**修改前**:
```yaml
mpc_planner/static_constraint_slack_ratio: 0.01   # 太小！
mpc_planner/dynamic_constraint_slack_ratio: 0.2
```

**修改后**:
```yaml
mpc_planner/static_constraint_slack_ratio: 0.1    # 增大 10 倍
mpc_planner/dynamic_constraint_slack_ratio: 0.3   # 增大 50%
```

**效果**：
- 约束更"软" → 数值更稳定
- 允许小幅度违反约束
- OSQP 求解器更鲁棒

---

## 松弛比的含义

### 什么是松弛比？

在 MPC 中，约束可以分为：
- **硬约束**: 必须严格满足，violation = 0
- **软约束**: 允许小幅度违反，有惩罚项

**松弛比 (slack ratio)** 控制约束的"软硬程度"：
- `0.0`: 完全硬约束（严格）
- `0.1`: 允许 10% 的松弛（软）
- `1.0`: 非常软的约束

### 为什么要增大？

| 松弛比 | 数值稳定性 | 避障能力 | 适用场景 |
|--------|-----------|---------|---------|
| 0.01 | ❌ 差 | ✅ 强 | 简单环境 |
| 0.1 | ✅ 好 | ✅ 足够 | 复杂环境 |
| 0.5 | ✅ 很好 | ⚠️ 弱 | 调试用 |

**本次修改**：
- 从 `0.01` → `0.1`：平衡稳定性和避障能力
- 适合复杂动态环境

---

## 验证方法

### 测试 1: 简单场景

```bash
# 1. 重新编译
cd /home/ff/intent-mpc
catkin_make

# 2. 启动 Gazebo
roslaunch uav_simulator start.launch world:=test_head_on

# 3. 启动导航
roslaunch autonomous_flight intent_mpc_demo.launch

# 4. 发送目标点（RViz 中点击 2D Nav Goal）
```

**预期结果**：
- ✅ 无人机起飞
- ✅ 避开动态障碍物
- ✅ 不会闪现回原点
- ✅ CSV 记录正常数据

---

### 测试 2: 查看日志

```bash
# 查看是否还有崩溃
dmesg | grep "mpc_navigation" | tail -5

# 查看 MPC 状态
tail -f ~/.ros/log/latest/rosout.log | grep "mpc"
```

**正常输出**：
- 没有 `segfault` 或 `general protection fault`
- 有 MPC 轨迹发布
- `infeasible_flag` 应该大部分为 0

---

### 测试 3: 检查 CSV 数据

```bash
# 查看最新的数据文件
ls -lt flight_data_*.csv | head -1

# 查看数据
tail -20 flight_data_*.csv | column -t -s,
```

**正常数据特征**：
- `dist_min` < 999（有障碍物距离）
- `collision_flag` = 0（无碰撞）
- `infeasible_flag` = 0（大部分时间可解）
- 位置持续变化（不是固定在 0,0,0）

---

## 修改文件清单

| 文件 | 修改内容 | 状态 |
|------|---------|------|
| `mapping_param.yaml` | 禁用静态地图 | ✅ 完成 |
| `planner_param.yaml` | 增大松弛比 | ✅ 完成 |

---

## 如果还崩溃怎么办？

### 进一步放松约束

```yaml
# planner_param.yaml
mpc_planner/static_constraint_slack_ratio: 0.3  # 继续增大
mpc_planner/dynamic_constraint_slack_ratio: 0.5
```

### 增大安全距离

```yaml
mpc_planner/static_safety_dist: 1.0  # 从 0.8 → 1.0
mpc_planner/dynamic_safety_dist: 0.8  # 从 0.6 → 0.8
```

### 使用更简单的场景

使用只有 1 个动态障碍物的测试场景：
```bash
roslaunch uav_simulator start.launch world:=test_head_on
```

---

## 技术细节

### OSQP 求解器

**全称**: Operator Splitting Quadratic Program

**作用**: 求解 MPC 的二次规划问题：
```
minimize: 1/2 x^T P x + q^T x
subject to: l <= Ax <= u
```

**崩溃原因**：
- 约束矩阵 A 病态（条件数很大）
- 上下界冲突：l > u
- 数值精度问题导致内存越界

### 约束松弛的数学表示

**硬约束**：
```
d(x, obstacle) >= d_safe
```

**软约束**（加入松弛变量 ε）：
```
d(x, obstacle) >= d_safe - ε
cost += weight * ε²
```

**松弛比** 控制 `weight`：
- 比例越大 → weight 越小 → 约束越软

---

## 总结

| 改动 | 原因 | 效果 |
|------|------|------|
| 禁用静态地图 | 减少约束数量 | 降低崩溃概率 |
| 增大松弛比 | 提高数值稳定性 | OSQP 更鲁棒 |

**预期结果**: 无人机不再闪现，稳定避障飞行！

---

**下一步**: 重新编译并测试！

```bash
cd /home/ff/intent-mpc && catkin_make
```
