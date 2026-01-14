# 风险自适应椭球 - 快速开始指南

## 🚀 快速启用

### 1. 编译代码

```bash
cd ~/intent-mpc
catkin_make
source devel/setup.bash
```

### 2. 启用风险自适应椭球

编辑配置文件：
```bash
vim src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/planner_param.yaml
```

确保以下参数设置为：
```yaml
mpc_planner/use_risk_adaptive: true
```

### 3. 运行测试

```bash
# 启动仿真（例如 head-on 场景）
roslaunch autonomous_flight simulation.launch

# 在另一个终端观察椭球可视化
rviz
# 添加 MarkerArray，topic: /mpc_planner/ellipsoid_obstacles
```

---

## 📊 可视化验证

### RViz 设置

1. 添加 MarkerArray
   - Topic: `/mpc_planner/ellipsoid_obstacles`
   - 观察椭球形状、朝向、大小变化

2. 观察关键场景
   - **迎面冲突**：椭球应该沿障碍物运动方向变长、变大
   - **平行运动**：椭球应该垂直方向较窄
   - **静止或远离**：椭球应该接近圆形、尺寸较小

### 调试输出

在 `updateObstacleParam()` 中添加调试信息（可选）：

```cpp
// 在计算 s 之后添加：
ROS_INFO_THROTTLE(1.0, "[Risk-Adaptive] Obs_%d[%d]: d=%.2fm, vc=%.2fm/s, ttc=%.2fs → s=%.2fm, φ=%.1f°", 
                  i, j, d, vc, ttc, s, phi*180/M_PI);
```

---

## 🎛️ 快速调参指南

### 场景1：机器人太保守，绕远路

**症状**：机器人离障碍物很远就开始大幅绕行  
**原因**：膨胀量太大  
**调整**：

```yaml
mpc_planner/risk_s_max: 0.8    # 降低（从 1.2 → 0.8）
mpc_planner/risk_beta: 0.4     # 降低（从 0.6 → 0.4）
```

### 场景2：机器人太激进，险些碰撞

**症状**：机器人离障碍物很近才反应  
**原因**：膨胀量太小  
**调整**：

```yaml
mpc_planner/risk_s0: 0.25      # 提高基线（从 0.15 → 0.25）
mpc_planner/risk_alpha: 0.6    # 提高速度系数（从 0.4 → 0.6）
```

### 场景3：迎面冲突时反应太慢

**症状**：head-on 场景中机器人避让不及时  
**原因**：TTC 影响不够强  
**调整**：

```yaml
mpc_planner/risk_beta: 0.8     # 提高 TTC 系数（从 0.6 → 0.8）
mpc_planner/risk_tau: 1.2      # 缩短衰减时间（从 1.5 → 1.2）
```

### 场景4：椭球朝向抖动

**症状**：低速时椭球不断旋转  
**原因**：速度阈值太低  
**调整**：

```yaml
mpc_planner/risk_vel_threshold: 0.25   # 提高阈值（从 0.15 → 0.25）
```

### 场景5：想要更明显的各向异性（椭圆形）

**症状**：椭球看起来接近圆形，没有明显朝向  
**原因**：各向异性强度太弱  
**调整**：

```yaml
mpc_planner/risk_kappa: 0.5    # 提高（从 0.35 → 0.5）
```

⚠️ **注意**：`kappa` 不要超过 0.6，否则椭球会过扁，导致约束过紧。

---

## 🧪 推荐测试场景

### 1. Head-On（迎面冲突）

```bash
roslaunch autonomous_flight simulation.launch world:=test_head_on
```

**期望行为**：
- 椭球长轴沿障碍物运动方向
- 随着距离减小，椭球快速膨胀
- 机器人提前侧向绕行

**关键指标**：
- 最小距离 > 1.5m
- 绕行提前量（距离碰撞点的距离）
- 轨迹平滑度

### 2. Stop（紧急停车）

```bash
roslaunch autonomous_flight simulation.launch world:=test_stop
```

**期望行为**：
- 障碍物停止后，椭球保持朝向稳定（低速机制）
- closing speed 减小，膨胀量减小
- 机器人停在安全距离外

### 3. Parallel（平行运动）

（需自行创建场景）

**期望行为**：
- `vc ≈ 0`，膨胀量 ≈ `s0`
- 椭球垂直方向较窄（`b < a`）
- 机器人可以近距离跟随

---

## 📈 性能对比

### 对比实验设计

1. **固定椭球（baseline）**：
   ```yaml
   mpc_planner/use_risk_adaptive: false
   mpc_planner/dynamic_safety_dist: 0.6
   ```

2. **风险自适应椭球（新方法）**：
   ```yaml
   mpc_planner/use_risk_adaptive: true
   # 使用推荐参数
   ```

### 记录指标

- **安全性**：最小距离、碰撞次数
- **效率**：轨迹长度、平均速度、任务完成时间
- **舒适性**：加速度峰值、加速度变化率（jerk）
- **实时性**：MPC 求解时间

### 运行对比

```bash
# 运行多次（如 10 次），记录平均值和标准差
for i in {1..10}; do
    roslaunch autonomous_flight simulation.launch world:=test_head_on
    # 记录 rosbag 或 CSV 数据
done
```

---

## 🔧 高级调优

### 使用 rqt_reconfigure（推荐）

如果需要实时调参，可以添加 `dynamic_reconfigure` 支持（需额外开发）。

### 分场景调参

在 launch 文件中根据场景加载不同参数：

```xml
<launch>
  <!-- 保守模式（拥挤环境） -->
  <rosparam command="load" file="$(find autonomous_flight)/cfg/mpc_navigation/risk_params_conservative.yaml" />
  
  <!-- 激进模式（开阔环境） -->
  <!-- <rosparam command="load" file="$(find autonomous_flight)/cfg/mpc_navigation/risk_params_aggressive.yaml" /> -->
</launch>
```

### 日志记录与分析

```bash
# 记录椭球参数到 CSV
rostopic echo /mpc_planner/ellipsoid_obstacles | tee ellipsoid_log.txt

# 分析膨胀量分布
python3 analyze_ellipsoid.py ellipsoid_log.txt
```

---

## ❓ 常见问题排查

### Q: 编译错误 "prevYaw_ was not declared"

**A**: 检查 `mpcPlanner.h` 是否正确添加了：
```cpp
std::vector<std::vector<double>> prevYaw_;
```

### Q: 运行时 "Parameter not found"

**A**: 检查 `planner_param.yaml` 是否正确加载：
```bash
rosparam list | grep risk
```

应该看到：
```
/mpc_planner/risk_alpha
/mpc_planner/risk_beta
...
```

### Q: 椭球看起来没有变化

**A**: 
1. 检查 `use_risk_adaptive` 是否为 `true`
2. 检查障碍物是否有速度（动态障碍）
3. 添加调试输出，打印 `d, vc, ttc, s, phi`

### Q: 机器人行为异常

**A**: 
1. 先禁用风险自适应（`use_risk_adaptive: false`），确认原系统正常
2. 逐步启用新功能，观察哪个参数影响最大
3. 检查 `s_max` 是否过大（建议 < 1.5m）

---

## 📚 进一步阅读

- [完整实现文档](RISK_ADAPTIVE_ELLIPSOID_IMPLEMENTATION.md)
- [原始需求](TEST_STOP_WORLD_DESIGN.md)
- [崩溃分析](CRASH_ANALYSIS_LATEST.md)

---

## 🎯 成功标志

如果你看到以下现象，说明实现成功：

✅ **迎面冲突**：椭球明显变长变大，机器人提前绕行  
✅ **远离运动**：椭球保持较小尺寸，机器人不过度保守  
✅ **低速场景**：椭球朝向稳定，不抖动  
✅ **编译无误**：`catkin_make` 成功，无 linter 错误  
✅ **实时性**：MPC 求解时间无明显增加（< 0.1ms）

---

**祝调试顺利！** 🚁

如有问题，请查看详细文档或检查代码注释。

