# MPC Infeasible Flag 问题分析

## 问题
`infeasible_flag` 一直是 0，即使 MPC 可能出现求解失败。

## 根本原因
1. **MPC 原始代码不发布 infeasible 状态**
   - `mpcPlanner::makePlan()` 返回布尔值 `successSolve`
   - 但这个值只在内部使用，没有通过 topic 发布出来
   
2. **data_recorder 无法获取状态**
   - `mpcInfeasibleSub_` 被声明但从未初始化
   - 没有对应的 topic 可以订阅

## 解决方案

### 方案 1：添加 MPC 状态发布 topic（推荐）
在 `mpcPlanner` 中添加 publisher，发布求解状态：
```cpp
// 在 mpcPlanner.h 中添加
ros::Publisher statusPub_;

// 在 makePlan() 中发布
std_msgs::Bool statusMsg;
statusMsg.data = !successSolve;  // true = infeasible
statusPub_.publish(statusMsg);
```

### 方案 2：通过 mpcNavigation 间接发布
在 `mpcNavigation.cpp` 中，`newTrajReturn` 已经表示了 MPC 求解结果：
```cpp
bool newTrajReturn = this->mpc_->makePlan();  // line 466

if (newTrajReturn) {
    // 求解成功
} else {
    // 求解失败 (infeasible)
}
```

可以在 `mpcNavigation` 中添加 publisher 发布这个状态。

## 当前状态
- `dist_min`: ✅ 正常
- `collision_flag`: ✅ 正常
- `infeasible_flag`: ❌ 一直为 0（无法获取）
- 崩溃问题: ❌ OSQP 求解器崩溃

## 注意
用户要求**不修改 MPC 原始代码**，所以：
- ✅ 可以修改 `mpcNavigation.cpp`（这是自定义的 navigation 层）
- ❌ 不能修改 `mpcPlanner.cpp`（这是原始 MPC 实现）

**建议方案**：在 `mpcNavigation` 中发布 MPC 状态。
