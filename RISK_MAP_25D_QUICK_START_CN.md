# 2.5D风险地图快速开始（单层版本）

## 🎯 核心理解

### 什么是"单层2.5D"？

简单说：**一个2D地图悬浮在固定高度（比如1米）**

```
传统理解：
     地面
      ↓
   ━━━━━━━━━━━━  ← 这就是"2D地图"
      ↑
   固定在1m高度
   
查询时：
- 输入：3D点 (x, y, z)
- 实际：只用 (x, y)，忽略z
- 输出：风险值
```

**不是**11层那种复杂的（那个内存占用太大，也没必要）。

---

## ⚡ 快速使用

### 1. 切换分支并编译

```bash
cd /home/ff/intent-mpc
git checkout feature/risk-map-25d
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 2. 直接运行

```bash
roslaunch autonomous_flight intent_mpc_demo.launch
```

就这么简单！新的风险地图已经集成到A*里了。

### 3. 可视化（可选）

如果想看风险地图长什么样：

```bash
# 新开一个终端
rosrun global_planner risk_map_25d_visualizer

# 在RViz中添加
# Topic: /risk_map_25d/grid
# Type: OccupancyGrid
```

**注意**：现在可视化节点已经集成到主launch文件中，会自动启动，无需手动运行！

---

## 🔧 核心参数（只需要知道这几个）

打开 `src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/planner_param.yaml`

### 参数1: 固定高度
```yaml
risk_map_25d/fixed_height: 1.0  # 单位：米
```
**含义**：风险地图在哪个高度  
**建议值**：
- 低空飞行（1-2m）：设为 `1.0`
- 中空飞行（2-3m）：设为 `2.0`

### 参数2: A*风险权重
```yaml
astar/w_risk: 10.0
```
**含义**：风险地图对路径规划的影响力度  
**效果**：
- `5-10`：软约束，会尽量避开但不绝对
- `10-20`：中等约束，比较明显地避开
- `>20`：硬约束，几乎不会走风险区域

### 参数3: 静态风险权重
```yaml
risk_map_25d/gamma_s: 1.0
```
**含义**：静态障碍物（墙、柱子）的风险强度  
**效果**：
- 调大（1.5-2.0）：离墙更远
- 调小（0.5-1.0）：可以贴墙更近

### 参数4: 动态风险权重
```yaml
risk_map_25d/gamma_d: 1.0
```
**含义**：动态障碍物（行人、车辆）的风险强度  
**效果**：
- 调大（1.5-2.0）：离动态障碍物更远
- 调小（0.5-1.0）：可以靠得更近

---

## 🎨 效果对比

### 原来（RiskMap2D）
- ❌ 只有2D，没有高度信息
- ❌ 动态风险不够精确
- ❌ 代码复杂

### 现在（RiskMap25D单层）
- ✅ 单层2.5D，简单高效
- ✅ 融合静态风险（ESDF）+ 动态风险（意图预测）
- ✅ 内存占用小（~80KB）
- ✅ 查询速度快（直接插值）
- ✅ 代码简洁

---

## 🐛 遇到问题？

### 问题1: 路径还是会撞墙

**解决**：调大这两个参数
```yaml
astar/w_risk: 15.0            # 原来10.0
risk_map_25d/gamma_s: 1.5     # 原来1.0
```

### 问题2: 路径绕得太远了

**解决**：调小这两个参数
```yaml
astar/w_risk: 5.0             # 原来10.0
risk_map_25d/gamma_s: 0.5     # 原来1.0
```

### 问题3: 无人机飞得比1m高

**解决**：调整固定高度
```yaml
risk_map_25d/fixed_height: 1.5  # 或2.0，根据实际飞行高度
```

### 问题4: 动态障碍物预测不准

这个不是风险地图的问题，是意图识别的问题。去看：
- `src/Intent-MPC/dynamic_predictor/include/dynamic_predictor/dynamicPredictor.cpp`
- 检查`intentProb_`的值是否正常

---

## 📊 技术原理（简化版）

### 风险如何计算？

**总风险 = 静态风险 + 动态风险**

1. **静态风险**（来自ESDF距离场）
   ```
   R_static = max(0, 安全距离 - 实际距离)^2
   ```
   - 离墙越近，风险越大
   - 超过安全距离（0.5m），风险为0

2. **动态风险**（来自意图预测）
   ```
   R_dynamic = Σ(概率 × 时间权重 × 椭圆核)
   ```
   - 考虑4个意图（前进/左转/右转/停止）
   - 未来时间越远，权重越小
   - 椭圆核：离预测位置越近，风险越大

3. **融合**
   ```
   R_total = gamma_s × R_static + gamma_d × R_dynamic
   ```

### A*如何使用风险？

A*代价函数：
```
cost = 距离代价 + w_risk × 风险^1.5
```

- 风险越高，代价越大
- A*会倾向于选择低风险路径
- `w_risk`控制这个倾向的强度

---

## 🎓 文件结构

如果你想修改代码，关键文件在这里：

```
src/Intent-MPC/
├── global_planner/
│   ├── include/global_planner/
│   │   ├── risk_map_25d.h              ← 风险地图类定义
│   │   └── a_star_occ.h                ← A*规划器（已改用RiskMap25D）
│   └── src/
│       ├── risk_map_25d.cpp            ← 风险地图实现
│       ├── risk_map_25d_visualizer.cpp ← 可视化节点
│       └── a_star_occ.cpp              ← A*实现
└── autonomous_flight/
    ├── include/autonomous_flight/
    │   └── mpcNavigation.cpp           ← MPC导航（调用风险地图）
    └── cfg/mpc_navigation/
        └── planner_param.yaml          ← 参数配置
```

---

## ✅ 完成检查清单

- [x] 编译无错误
- [x] 启动无崩溃
- [x] A*路径会避开静态障碍物
- [x] A*路径会预判动态障碍物
- [ ] 参数调整到满意的效果
- [ ] 可视化验证风险地图正确

---

## 🚀 下一步

1. **测试效果**
   - 运行demo
   - 观察路径规划是否合理
   - 调整参数到满意

2. **收集数据**
   - 记录风险值（`ROS_INFO`）
   - 保存轨迹数据
   - 分析避障效果

3. **可能的改进**
   - 如果障碍物高度差异大，考虑增加z_min/z_max过滤
   - 如果需要更快的更新速度，考虑异步更新
   - 如果需要更精确的风险，考虑调整椭圆核参数

---

**最后提醒**：这个是单层版本，适合飞行高度相对固定的场景。如果你的无人机会在0-5m之间大幅变化，或者障碍物高度分布很广，可能需要考虑多层版本（但那个会复杂很多）。

**有问题随时问！** 🎉
