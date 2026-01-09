# MPC 配置文件整理建议

## 当前情况

### 文件 1: `trajectory_planner/cfg/mpc_interactive/mpc_param.yaml`
- **用途**: 交互式测试工具 (`mpc_interactive.launch`)
- **使用场景**: 开发和单独测试 MPC 功能
- **参数值**: `dynamic_safety_dist: 0.8`

### 文件 2: `autonomous_flight/cfg/mpc_navigation/planner_param.yaml`
- **用途**: 完整导航系统 (`intent_mpc_demo.launch`)
- **使用场景**: 实际飞行和实验
- **参数值**: `mpc_planner/dynamic_safety_dist: 0.6`

## 重复的参数

```yaml
# 两个文件都有：
- horizon
- z_range_min/max
- static_safety_dist
- dynamic_safety_dist
- static_constraint_slack_ratio
- dynamic_constraint_slack_ratio
- cloud_res
- local_cloud_region_x/y
- ground_height
- ceiling_height
```

---

## 建议方案

### ✅ 方案 1: 保留两个文件，但添加注释（推荐）

**优点**:
- 不影响现有功能
- 开发测试和实际使用可以用不同参数
- 安全，不会破坏任何东西

**做法**:
在 `planner_param.yaml` 顶部添加注释：

```yaml
# ================================================================
# MPC 导航参数配置（用于完整自主导航系统）
# 
# 注意: 这是 intent_mpc_demo.launch 使用的配置文件
# 如果你在修改 MPC 参数，请修改这个文件！
# 
# 另一个配置文件 trajectory_planner/cfg/mpc_interactive/mpc_param.yaml
# 是给交互式测试工具用的，不影响实际飞行。
# ================================================================
```

在 `mpc_param.yaml` 顶部添加注释：

```yaml
# ================================================================
# MPC 交互式测试工具参数（仅用于 mpc_interactive.launch）
# 
# 注意: 这个文件仅用于开发测试，不用于实际导航！
# 实际飞行参数在 autonomous_flight/cfg/mpc_navigation/planner_param.yaml
# ================================================================
```

---

### ⚠️ 方案 2: 删除 mpc_interactive/mpc_param.yaml（不推荐）

**缺点**:
- 会破坏 `mpc_interactive.launch`（交互式测试工具）
- 可能影响开发和调试

**适用场景**:
- 你确定永远不会用交互式测试工具
- 只关注完整导航系统

**风险**: 如果将来想单独测试 MPC，需要重新创建配置文件

---

### 🔧 方案 3: 创建共享配置 + 特定配置（工程化，但复杂）

创建三个文件：

```
cfg/
├── mpc_common.yaml          # 共享的基础参数
├── mpc_interactive.yaml     # 交互式测试特定参数
└── mpc_navigation.yaml      # 导航系统特定参数
```

**优点**: 消除重复，参数集中管理
**缺点**: 需要修改 launch 文件，增加复杂度

---

## 我的建议

### 🎯 推荐：方案 1（添加注释）

**原因**:
1. **不破坏现有功能** - 两个工具都能正常工作
2. **清晰明确** - 未来维护时知道改哪个
3. **零风险** - 只是添加注释，不改任何逻辑
4. **保留灵活性** - 测试和实际飞行可以用不同参数

**实施步骤**:
1. 在两个文件顶部添加清晰注释
2. 说明各自用途和适用场景
3. 指明实际飞行要改哪个文件

---

## 参数差异对比

| 参数 | mpc_interactive | planner_param | 说明 |
|------|----------------|---------------|------|
| `horizon` | 30 | 30 | 相同 |
| `z_range_min` | 0.9 | 0.6 | 不同 |
| `z_range_max` | 1.2 | 2.9 | 不同 |
| `static_safety_dist` | 0.8 | 0.8 | 相同 |
| `dynamic_safety_dist` | **0.8** | **0.6** | ⚠️ 不同 |
| `static_constraint_slack_ratio` | 0.3 | 0.01 | 不同 |
| `dynamic_constraint_slack_ratio` | 0.5 | 0.2 | 不同 |
| `cloud_res` | 0.2 | 0.2 | 相同 |
| `local_cloud_region_x` | 5.0 | 6.0 | 不同 |
| `local_cloud_region_y` | 2.0 | 4.0 | 不同 |
| `ground_height` | 0.4 | 0.5 | 不同 |
| `ceiling_height` | 2.0 | 无 | - |

**关键发现**: 很多参数值**不同**，说明两个场景确实需要不同配置！

---

## 快速对照表

### 你现在想改 MPC 参数？

```
实际飞行实验 → 改 autonomous_flight/cfg/mpc_navigation/planner_param.yaml
单独测试 MPC → 改 trajectory_planner/cfg/mpc_interactive/mpc_param.yaml
```

### 哪个文件会影响你的实验？

```
intent_mpc_demo.launch → planner_param.yaml ✅
mpc_interactive.launch → mpc_param.yaml
```

---

## 总结

| 方案 | 优点 | 缺点 | 推荐度 |
|------|------|------|--------|
| 1. 添加注释 | 安全、清晰、零风险 | 仍有重复 | ⭐⭐⭐⭐⭐ |
| 2. 删除文件 | 消除重复 | 破坏测试工具 | ⭐ |
| 3. 重构共享 | 工程化 | 复杂度高 | ⭐⭐⭐ |

**建议**: 保留两个文件，在顶部添加清晰注释说明用途即可！
