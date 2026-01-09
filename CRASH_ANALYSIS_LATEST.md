# 最新崩溃分析报告

## 崩溃信息

```
[19105.904552] traps: mpc_navigation_[107424] general protection fault 
ip:7f80a316638c sp:7f805fdf39f8 error:0 in libosqp.so
```

**崩溃位置**: OSQP 求解器库（MPC 的 QP 求解器）
**崩溃类型**: General Protection Fault（内存访问错误）

---

## 历史崩溃记录

1. `[ 4968] segfault in libc-2.31.so`
2. `[ 6550] segfault in libc-2.31.so`
3. `[ 8466] segfault in libtrajectory_planner.so`
4. `[16553] segfault in libtrajectory_planner.so`
5. `[19105] **general protection fault in libosqp.so** ← 最新

---

## 崩溃原因分析

### MPC 求解器数值不稳定

**根本原因**：
- OSQP 求解器接收到了不合理的约束条件
- 导致求解过程中访问非法内存

**可能触发条件**：
1. **约束冲突**: 静态障碍物 + 动态障碍物约束过于严格
2. **数值奇异**: 障碍物距离过近，约束矩阵病态
3. **内存访问**: 障碍物数量/位置导致越界访问

---

## 闪现回原点的原因

当 MPC 崩溃时：
1. ROS 节点异常退出
2. 控制指令中断
3. Gazebo 物理引擎重置无人机位置
4. 无人机"闪现"回初始位置 (0, 0, 0.1)

---

## 用户反馈对应

用户说：
> "规划出来不飞，直接闪现回原点"

这正是 MPC 节点崩溃的表现：
- **有路线** ← A* 规划成功
- **不飞** ← MPC 执行时崩溃
- **闪现回原点** ← 控制中断，模拟器重置

---

## 解决方案：避开静态障碍物

### 方案 1: 禁用静态地图（推荐）

修改 `mapping_param.yaml`：

```yaml
# 当前静态地图配置
static_occupancy_map/use_prebuilt_map: false  # 已经是 false
static_occupancy_map/prebuilt_map_directory: ""

# 确保不加载静态地图
```

**当前状态**: 已经禁用✅

---

### 方案 2: 简化场景

只保留动态障碍物，方便调试：

1. 使用 `test_head_on.world` - 只有 1 个动态障碍物
2. 不加载静态地图
3. 简单的直线对头场景

**优点**：
- 减少约束数量
- 降低数值不稳定性
- 更容易定位问题

---

### 方案 3: 调整 MPC 参数

降低约束敏感度：

```yaml
# planner_param.yaml
mpc_planner/static_safety_dist: 1.0  # 增大安全距离
mpc_planner/static_constraint_slack_ratio: 0.1  # 增大松弛变量
mpc_planner/dynamic_safety_dist: 0.8  # 增大动态安全距离
mpc_planner/dynamic_constraint_slack_ratio: 0.3  # 增大松弛变量
```

**说明**：
- 更大的安全距离 → 更早避障
- 更大的松弛比 → 约束更"软"，数值更稳定

---

## 当前配置检查

### 静态地图状态

```yaml
# mapping_param.yaml (当前)
static_occupancy_map/use_prebuilt_map: false  ✅ 已禁用
```

### MPC 约束参数

```yaml
# planner_param.yaml (当前)
mpc_planner/static_safety_dist: 0.8
mpc_planner/dynamic_safety_dist: 0.6
mpc_planner/static_constraint_slack_ratio: 0.01  ⚠️ 很小
mpc_planner/dynamic_constraint_slack_ratio: 0.2
```

**问题点**：
- `static_constraint_slack_ratio: 0.01` 非常小
  → 静态障碍物约束几乎是硬约束
  → 数值容易出问题

---

## 建议的修改

### 立即修改：增大松弛比

```yaml
mpc_planner/static_constraint_slack_ratio: 0.1  # 从 0.01 → 0.1
mpc_planner/dynamic_constraint_slack_ratio: 0.3  # 从 0.2 → 0.3
```

**原因**：
- 更软的约束 → 求解器更稳定
- 减少数值奇异性
- 降低崩溃概率

---

## 快速测试方案

### 测试 1: 纯动态障碍物（无静态）

```bash
# 1. 确认静态地图已禁用
grep "use_prebuilt_map" mapping_param.yaml

# 2. 使用简单场景
roslaunch uav_simulator start.launch world:=test_head_on

# 3. 启动导航
roslaunch autonomous_flight intent_mpc_demo.launch
```

### 测试 2: 增大松弛比

修改 `planner_param.yaml`:
```yaml
mpc_planner/static_constraint_slack_ratio: 0.1
mpc_planner/dynamic_constraint_slack_ratio: 0.3
```

然后重新编译运行。

---

## 总结

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 闪现回原点 | MPC 崩溃 | 增大松弛比 |
| libosqp.so fault | 约束冲突 | 简化场景 |
| 不稳定 | 数值问题 | 调整参数 |

**优先级**：
1. ✅ 静态地图已禁用
2. 🔧 增大 `static_constraint_slack_ratio` 到 0.1
3. 🔧 增大 `dynamic_constraint_slack_ratio` 到 0.3
4. 🧪 测试简单场景
