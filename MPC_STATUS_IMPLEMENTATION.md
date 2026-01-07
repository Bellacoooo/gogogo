# MPC Infeasible Flag 实现说明

## 已完成的修改

### 1. 在 mpcNavigation 中添加发布器

**文件**: `autonomous_flight/include/autonomous_flight/mpcNavigation.h`
- 添加头文件: `#include <std_msgs/Bool.h>`
- 添加 publisher: `ros::Publisher mpcStatusPub_;`

**文件**: `autonomous_flight/include/autonomous_flight/mpcNavigation.cpp`
- 在 `registerPub()` 中注册 topic:
  ```cpp
  this->mpcStatusPub_ = this->nh_.advertise<std_msgs::Bool>("mpcNavigation/infeasible", 10);
  ```
- 在 `mpcCB()` 中发布状态（在 makePlan 之后）:
  ```cpp
  bool newTrajReturn = this->mpc_->makePlan();
  
  // 发布 MPC 求解状态（仅发布，不改变逻辑）
  std_msgs::Bool mpcStatusMsg;
  mpcStatusMsg.data = !newTrajReturn;  // true = infeasible, false = success
  this->mpcStatusPub_.publish(mpcStatusMsg);
  ```

### 2. 在 data_recorder 中订阅状态

**文件**: `flight_data_recorder/src/data_recorder_node.cpp`
- 订阅 topic:
  ```cpp
  mpcInfeasibleSub_ = nh_.subscribe("/mpcNavigation/infeasible", 10, 
                                     &DataRecorder::mpcStatusCallback, this);
  ```
- 添加回调函数:
  ```cpp
  void mpcStatusCallback(const std_msgs::BoolConstPtr& msg) {
      mpcInfeasible_ = msg->data;  // true = infeasible, false = success
  }
  ```

## 工作原理

1. **MPC 求解**: `mpc_->makePlan()` 返回布尔值
   - `true` = 求解成功
   - `false` = 求解失败（infeasible）

2. **发布状态**: `mpcNavigation` 将结果发布到 `/mpcNavigation/infeasible` topic
   - `data = false` = 求解成功
   - `data = true` = 求解失败（infeasible）

3. **数据记录**: `data_recorder` 订阅该 topic，更新 `mpcInfeasible_` 标志
   - 写入 CSV 时: `infeasible_flag = (mpcInfeasible_ ? 1 : 0)`

## 验证方法

### 1. 检查 topic 是否发布
```bash
rostopic list | grep infeasible
# 应该显示: /mpcNavigation/infeasible

rostopic echo /mpcNavigation/infeasible
# 应该显示: data: false (成功) 或 data: true (失败)
```

### 2. 检查 CSV 文件
```bash
find ~/intent-mpc/src/Intent-MPC/flight_data_recorder -name "*.csv" -mtime -1 | xargs tail -20
```

infeasible_flag 列应该：
- 大部分时候是 `0` (求解成功)
- 遇到困难场景时变成 `1` (求解失败)

## 重要说明

✅ **遵守了"不修改 MPC 原始代码"的原则**
- 没有修改 `trajectory_planner/mpcPlanner.cpp` 中的任何逻辑
- 只在 `autonomous_flight/mpcNavigation` 封装层添加了发布功能
- 不影响 MPC 的求解过程和结果

✅ **只添加了发布代码，没有改变任何逻辑**
- 原有的 `if (newTrajReturn)` 逻辑完全不变
- 只是在获取结果后立即发布
- 不影响任何控制流程

## CSV 文件格式

```csv
time,uav_x,uav_y,uav_z,dist_min,path_length,infeasible_flag,collision_flag
0.00,0.000,0.000,1.000,8.45,0.00,0,0
0.10,0.100,0.000,1.000,8.35,0.10,0,0
...
10.50,5.230,0.150,1.000,2.10,10.45,1,0  👈 MPC 求解失败
10.60,5.230,0.150,1.000,2.05,10.45,1,0  👈 仍然失败
10.70,5.340,0.160,1.000,2.15,10.55,0,0  👈 恢复成功
```

## 下一步

重新运行实验，验证：
1. ✅ `dist_min` - 已验证正常
2. ✅ `collision_flag` - 已验证正常
3. ⏳ `infeasible_flag` - 需要测试验证
4. ❌ MPC 崩溃问题 - 仍需解决
