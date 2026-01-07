# 问题分析报告

## 问题描述

1. **MPC 崩溃**: 无人机飞行过程中 MPC 节点崩溃，导致无人机闪现回原点
2. **dist_min 一直是 999**: 数据记录器无法获取障碍物距离
3. **collision_flag 和 infeasible_flag 都是 0**: 无法正确检测碰撞

## 根本原因

### 1. 服务名称错误 ❌

**问题**: 数据记录器使用的服务名称不正确
- **代码中**: `/fake_detector/get_dynamic_obstacles`
- **实际服务**: `/fake_detector_node/getDynamicObstacles`

**影响**: 
- 服务调用失败
- `dist_min` 始终为 999（默认值）
- 无法检测碰撞（因为 `dist_min = 999 > 0.4`）

### 2. MPC 崩溃 ⚠️

**错误信息**:
```
[19105.904552] traps: mpc_navigation_[107424] general protection fault ip:7f80a316638c sp:7f805fdf39f8 error:0 in libosqp.so[7f80a3156000+12000]
```

**分析**:
- 崩溃发生在 `libosqp.so`（OSQP 优化求解器库）
- 类型：`general protection fault`（内存访问错误）
- 可能原因：
  1. OSQP 求解器内部错误
  2. 输入数据异常（NaN/Inf）
  3. 约束条件冲突导致求解器崩溃

**影响**:
- MPC 节点崩溃后，无人机失去控制
- 可能触发安全机制，导致无人机返回原点

## CSV 数据分析

### 飞行轨迹
- **起飞**: (0, 0, 0.965) → (0, 0, 1.000)
- **飞行**: 到达 (-2.649, 0.042, 1.025)
- **崩溃**: 之后闪现回 (0, 0, 0)
- **路径长度**: 停在 7.09 米（崩溃后不再更新）

### 数据问题
- **dist_min**: 全部为 999.00（服务调用失败）
- **collision_flag**: 全部为 0（因为 dist_min = 999 > 0.4）
- **infeasible_flag**: 全部为 0（未实现）

## 已修复

### ✅ 服务名称修复
- 将服务名称从 `/fake_detector/get_dynamic_obstacles` 
- 改为 `/fake_detector_node/getDynamicObstacles`

## 待解决问题

### 1. MPC 崩溃问题 ⚠️

**建议排查**:
1. 检查 MPC 输入数据是否包含 NaN/Inf
2. 检查约束条件是否合理
3. 添加异常处理，防止崩溃

**临时方案**:
- 使用 ROS 的节点重启机制
- 添加看门狗监控 MPC 节点

### 2. infeasible_flag 未实现

**需要**:
1. 查找 MPC 节点是否发布无解标志
2. 如果没有，需要在 MPC 代码中添加
3. 在数据记录器中订阅该标志

## 验证步骤

修复后，重新运行实验：

1. **检查服务**:
   ```bash
   rosservice list | grep getDynamicObstacles
   # 应该看到: /fake_detector_node/getDynamicObstacles
   ```

2. **测试服务调用**:
   ```bash
   rosservice call /fake_detector_node/getDynamicObstacles "current_position:
     x: 0.0
     y: 0.0
     z: 1.0
   range: 50.0"
   # 应该返回障碍物列表
   ```

3. **运行实验**:
   - 启动系统
   - 观察日志中是否有 `[DataRecorder-DEBUG]` 消息
   - 检查 CSV 文件中的 `dist_min` 是否不再是 999

4. **检查 MPC 稳定性**:
   - 监控 MPC 节点是否崩溃
   - 如果崩溃，查看崩溃时的日志和状态

## 预期结果

修复服务名称后：
- ✅ `dist_min` 应该显示实际距离（不再是 999）
- ✅ `collision_flag` 应该能正确检测碰撞
- ⚠️ MPC 崩溃问题需要进一步调查

## 下一步

1. **重新编译并测试**: 验证服务调用是否成功
2. **分析 MPC 崩溃**: 查看崩溃时的 MPC 状态和输入数据
3. **实现 infeasible_flag**: 添加 MPC 无解标志的订阅

