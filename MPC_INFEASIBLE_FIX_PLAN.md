# MPC 问题修复方案

## 问题 1: MPC 崩溃 (libosqp.so general protection fault)

### 可能原因
1. **数值不稳定**：约束矩阵出现 NaN/Inf
2. **约束冲突**：静态/动态障碍物约束互相矛盾
3. **初始化问题**：第一次求解时状态不合理

### 调试方法
```bash
# 1. 查看 MPC 详细日志
roslaunch autonomous_flight intent_mpc_demo.launch 2>&1 | tee mpc_debug.log

# 2. 检查是否第一次 makePlan 就崩溃
grep -E "makePlan|successSolve|OSQP" mpc_debug.log

# 3. 查看崩溃时障碍物位置
grep "obstacle" mpc_debug.log | tail -20
```

### 临时解决方案
1. **简化场景**：先测试只有 1 个障碍物的情况
2. **降低速度**：减小 maxVel、maxAcc 参数
3. **增加安全距离**：增大 dynamicSafetyDist 参数

---

## 问题 2: infeasible_flag 一直是 0

### 根本原因
- MPC 原始代码 **不发布** 求解状态 topic
- `data_recorder` 的 `mpcInfeasibleSub_` 声明了但没有初始化
- 没有可以订阅的 topic

### 解决方案（遵守"不修改 MPC 原始代码"原则）

#### 方案 A：在 mpcNavigation 中发布状态（推荐）
在 `mpcNavigation.cpp` 的 `mpcCB()` 中，`newTrajReturn` 已经表示求解结果：

```cpp
// Line 466
bool newTrajReturn = this->mpc_->makePlan();

if (newTrajReturn) {
    // 求解成功
} else {
    // 求解失败 (infeasible) 👈 这里发布 topic
}
```

**修改位置**：
- `autonomous_flight/include/autonomous_flight/mpcNavigation.h`：添加 publisher
- `autonomous_flight/include/autonomous_flight/mpcNavigation.cpp`：发布状态

**不违反原则**：`mpcNavigation` 是封装层，不是 MPC 核心代码。

#### 方案 B：使用现有 ROS 日志解析
分析 ROS 日志中的 "MPC replan" 等关键词，但不准确。

---

## 建议优先级

1. **先解决崩溃问题**（最紧急）
   - 简化场景测试
   - 查看详细日志
   - 调整 MPC 参数
   
2. **再实现 infeasible_flag**
   - 在 `mpcNavigation` 中添加 publisher
   - 在 `data_recorder` 中订阅

---

## 用户确认

请问：
1. 是否允许在 `mpcNavigation.cpp` 中添加代码发布 MPC 求解状态？
2. 是否需要我先帮您简化场景（减少障碍物）来调试崩溃问题？
3. 是否需要详细日志来诊断 OSQP 崩溃原因？
