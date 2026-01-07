# dist_min=999 问题诊断指南

## 问题现象

- `dist_min` 一直是 999
- `collision_flag` 一直是 0（即使有碰撞）
- `infeasible_flag` 一直是 0
- 但实际飞行中有碰撞和卡住

## 可能原因

### 1. 服务不存在或未启动 ⚠️

**检查方法**:
```bash
# 运行诊断脚本
./check_service.sh
```

**如果服务不存在**:
- 检查 `fake_detector_node` 是否运行: `rosnode list | grep fake_detector`
- 检查 launch 文件是否包含: `<node pkg="onboard_detector" type="fake_detector_node" name="fake_detector_node" />`

### 2. 服务名称不匹配 ⚠️

**当前代码使用**: `/fake_detector_node/getDynamicObstacles`

**检查实际服务名称**:
```bash
rosservice list | grep -i "getDynamic\|obstacle"
```

如果看到不同的名称，需要修改代码中的服务名称。

### 3. fake_detector 没有获取到障碍物 ⚠️

**检查方法**:
```bash
# 检查 Gazebo 中的障碍物
rostopic echo /gazebo/model_states | grep "dynamic"

# 检查 fake_detector 是否发布可视化
rostopic echo /onboard_detector/GT_obstacle_bbox -n 1
```

**如果障碍物存在但 fake_detector 没获取到**:
- 检查 `fake_detector_param.yaml` 中的 `target_obstacle` 配置
- 确保包含 `"dynamic"`（匹配障碍物名称前缀）

### 4. 服务调用失败但无错误提示 ⚠️

**已添加详细日志**，重新运行后查看：
```bash
# 查看数据记录器日志
tail -f ~/.ros/log/latest/data_recorder_node*.log | grep -E "ERROR|STATUS|SUCCESS|FAILED"
```

应该看到：
- `[DataRecorder-ERROR]`: 服务不存在或调用失败
- `[DataRecorder-STATUS]`: 服务调用成功，显示找到的障碍物数量

## 诊断步骤

### 步骤 1: 检查服务状态

```bash
cd /home/ff/intent-mpc
./check_service.sh
```

**预期输出**:
```
✓ 服务存在: /fake_detector_node/getDynamicObstacles
✓ Service call SUCCESS! Found 3 obstacles
```

### 步骤 2: 查看数据记录器日志

启动系统后，查看日志：
```bash
# 实时查看
tail -f ~/.ros/log/latest/data_recorder_node*.log | grep -E "DataRecorder"

# 或查看完整日志
cat ~/.ros/log/latest/data_recorder_node*.log | grep -E "ERROR|STATUS|SUCCESS|FAILED" | tail -20
```

**应该看到**:
- `[DataRecorder]: ✓ Service /fake_detector_node/getDynamicObstacles is available!`
- `[DataRecorder-STATUS]: Service call SUCCESS! Found X obstacles`
- 或 `[DataRecorder-ERROR]: Service call FAILED!`

### 步骤 3: 手动测试服务

```bash
rosservice call /fake_detector_node/getDynamicObstacles "current_position:
  x: 0.0
  y: 0.0
  z: 1.0
range: 50.0"
```

**预期输出**: 应该返回障碍物列表（position, velocity, size）

### 步骤 4: 检查 fake_detector 配置

文件: `src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/fake_detector_param.yaml`

```yaml
target_obstacle: ["dynamic", "obstacle", "person"]  # 确保包含 "dynamic"
use_mocap: false
```

## 修复后的验证

重新编译并运行后：

1. **启动系统**
2. **查看日志**，应该看到：
   ```
   [DataRecorder]: ✓ Service /fake_detector_node/getDynamicObstacles is available!
   [DataRecorder-STATUS]: Service call SUCCESS! Found 3 obstacles at (0.00, 0.00, 1.00)
   ```
3. **检查 CSV 文件**，`dist_min` 应该不再是 999
4. **如果仍然失败**，日志会显示具体错误信息

## 常见问题

### Q: 服务存在但调用失败？

**可能原因**:
- 服务响应超时
- 服务内部错误
- 网络问题

**解决**: 查看 fake_detector 节点的日志

### Q: 服务调用成功但返回 0 个障碍物？

**可能原因**:
- 障碍物超出检测范围（50m）
- fake_detector 没有从 Gazebo 获取到障碍物
- 障碍物命名不匹配

**解决**: 
1. 检查 Gazebo 中是否有障碍物
2. 检查 fake_detector_param.yaml 配置
3. 查看 fake_detector 日志

### Q: 有碰撞但 collision_flag 还是 0？

**原因**: `dist_min = 999`，所以 `999 > 0.4`，不会触发碰撞标志

**解决**: 先修复 `dist_min = 999` 的问题，collision_flag 会自动修复

## 下一步

1. **运行诊断脚本**: `./check_service.sh`
2. **查看详细日志**: 重新运行系统，观察 `[DataRecorder-ERROR]` 和 `[DataRecorder-STATUS]` 消息
3. **根据日志信息定位问题**

## 已添加的调试功能

✅ 服务等待和检查
✅ 详细的成功/失败日志
✅ 每5秒打印一次状态（包括障碍物数量和位置）
✅ 服务调用统计（成功/失败次数）

现在重新运行系统，日志会告诉我们具体哪里出了问题！

