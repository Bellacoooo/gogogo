# 风险自适应椭球 - 文档导航

## 🚀 快速开始

**想立即使用？** → [快速开始指南](QUICK_START_RISK_ADAPTIVE.md)

**想了解完整实现？** → [完整实现文档](RISK_ADAPTIVE_ELLIPSOID_IMPLEMENTATION.md)

**想快速查参数？** → [参数速查表](RISK_ADAPTIVE_PARAMS_CHEATSHEET.md)

---

## 📚 文档清单

### 1. 核心文档

| 文档 | 描述 | 行数 | 适合人群 |
|------|------|------|----------|
| [**IMPLEMENTATION_SUMMARY_CN.md**](IMPLEMENTATION_SUMMARY_CN.md) | 中文总结，快速了解实现 | 500 | ⭐ 所有人 |
| [**STABILITY_PATCHES_SUMMARY_CN.md**](STABILITY_PATCHES_SUMMARY_CN.md) | ⭐ **稳定性补丁总结** | 600 | ⭐⭐ **必读** |
| [**QUICK_START_RISK_ADAPTIVE.md**](QUICK_START_RISK_ADAPTIVE.md) | 快速启用、调参、测试 | 300 | ⭐ 想快速使用的人 |
| [**RISK_ADAPTIVE_PARAMS_CHEATSHEET.md**](RISK_ADAPTIVE_PARAMS_CHEATSHEET.md) | 参数速查表、调参指南 | 400 | ⭐ 调参时查阅 |
| [**RISK_ADAPTIVE_ELLIPSOID_IMPLEMENTATION.md**](RISK_ADAPTIVE_ELLIPSOID_IMPLEMENTATION.md) | 完整理论和实现细节 | 600 | 深入理解原理 |
| [**STABILITY_PATCHES_DETAILED.md**](STABILITY_PATCHES_DETAILED.md) | 稳定性补丁详细文档 | 400 | 深入理解补丁 |
| [**CHANGELOG_RISK_ADAPTIVE.md**](CHANGELOG_RISK_ADAPTIVE.md) | 详细变更日志 | 500 | 代码审查、维护 |

### 2. Bug 修复文档

| 文档 | 描述 | 版本 | 状态 |
|------|------|------|------|
| [**BUG_FIX_CRASH_AND_RETREAT.md**](BUG_FIX_CRASH_AND_RETREAT.md) | 🚨 **崩溃和倒退问题修复** | v1.4 | ⭐⭐⭐ **最新** |
| [**CRITICAL_BUG_FIX_v1.2.md**](CRITICAL_BUG_FIX_v1.2.md) | 关键逻辑 Bug 修复（for j 循环） | v1.2 | 已修复 |
| [**VISUALIZATION_BUG_FIX_v1.3.md**](VISUALIZATION_BUG_FIX_v1.3.md) | 可视化 Bug 修复 | v1.3 | 已修复 |
| [**FINAL_BUG_FIXES_SUMMARY.md**](FINAL_BUG_FIXES_SUMMARY.md) | 所有 Bug 修复总结 | v1.3 | 已修复 |

### 3. 工具脚本

| 脚本 | 功能 | 使用方法 |
|------|------|----------|
| [**analyze_risk_adaptive.py**](analyze_risk_adaptive.py) | 可视化分析工具 | `python3 analyze_risk_adaptive.py` |

---

## 🎯 核心公式

```
s = clip(s0 + α·vc + β·exp(-ttc/τ), s_min, s_max)

其中：
  vc = max(0, -r̂·(v_r - v_i))  (closing speed)
  ttc = d / (vc + ε)            (time to collision)
  
椭球半轴：
  a = a0 + s(1 + κ)  (长轴，朝向障碍物运动方向)
  b = b0 + s(1 - κ)  (短轴，垂直方向)
  c = c0             (高度方向，不变)
```

---

## ⚙️ 参数配置

**配置文件**: `src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/planner_param.yaml`

**主开关**:
```yaml
mpc_planner/use_risk_adaptive: true  # 启用风险自适应
```

**关键参数**（v1.4 优化后默认值）:
```yaml
mpc_planner/risk_s0: 0.10          # 基线膨胀 (m) [优化↓]
mpc_planner/risk_alpha: 0.25       # closing speed 系数 [优化↓]
mpc_planner/risk_beta: 0.4         # TTC 指数系数 (m) [优化↓]
mpc_planner/risk_tau: 2.0          # TTC 衰减时间 (s) [优化↑]
mpc_planner/risk_kappa: 0.20       # 各向异性强度 [优化↓]
mpc_planner/risk_s_max: 0.8        # 最大膨胀 (m) [优化↓]
```

> ⚠️ v1.4 降低了参数激进度，防止椭球过大导致倒退和崩溃

详细说明 → [参数速查表](RISK_ADAPTIVE_PARAMS_CHEATSHEET.md)

---

## 🔧 修改的代码

### 核心文件

1. **`mpcPlanner.h`** - 添加成员变量
2. **`mpcPlanner.cpp`** - 实现风险自适应逻辑
   - `initParam()` - 参数初始化
   - `updateObstacleParam()` - 核心算法（★）
3. **`planner_param.yaml`** - 参数配置

### 关键函数

**`updateObstacleParam()`** 在每次 MPC 求解前调用，计算：
1. 障碍物速度（位置差分）
2. Closing speed 和 TTC
3. 风险裕量 s
4. 椭球朝向 φ
5. 各向异性分配到 a/b
6. 写回 `osize` 和 `yaw`

---

## 📊 效果演示

### 场景 1：迎面冲突（Head-On）

**预期效果**:
- 椭球沿障碍物运动方向变长
- 随距离减小，椭球快速膨胀
- 机器人提前侧向绕行

### 场景 2：远离或平行

**预期效果**:
- 椭球保持较小尺寸
- 接近圆形（各向异性较弱）
- 机器人可以较近距离通过

---

## 🧪 如何测试

### 1. 编译

```bash
cd ~/intent-mpc
catkin_make
source devel/setup.bash
```

**状态**: ✅ 已验证编译通过

### 2. 运行仿真

```bash
roslaunch autonomous_flight simulation.launch world:=test_head_on
```

### 3. 可视化

在 RViz 中添加：
- **MarkerArray**: `/mpc_planner/ellipsoid_obstacles`

观察椭球形状、朝向、大小变化。

### 4. 分析参数

```bash
python3 analyze_risk_adaptive.py
```

生成 5 张分析图表：
- 风险裕量随距离变化
- 风险裕量热力图
- 椭球形状对比
- 各向异性效果
- TTC 参数敏感性

---

## 🎛️ 快速调参

### 场景：机器人太保守（绕太远）

```yaml
mpc_planner/risk_s_max: 0.8    # ↓ 降低
mpc_planner/risk_beta: 0.4     # ↓ 降低
```

### 场景：机器人太激进（几乎碰撞）

```yaml
mpc_planner/risk_s0: 0.25      # ↑ 提高
mpc_planner/risk_alpha: 0.6    # ↑ 提高
```

### 场景：椭球朝向抖动

```yaml
mpc_planner/risk_vel_threshold: 0.25   # ↑ 提高
```

更多场景 → [快速开始指南](QUICK_START_RISK_ADAPTIVE.md#快速调参指南)

---

## ❓ 常见问题

### Q: 如何禁用风险自适应？

**A**: 设置 `use_risk_adaptive: false`，系统会回退到原始固定椭球。

### Q: 参数不知道怎么调？

**A**: 
1. 先用默认值测试
2. 运行 `analyze_risk_adaptive.py` 了解参数影响
3. 参考 [参数速查表](RISK_ADAPTIVE_PARAMS_CHEATSHEET.md)

### Q: 椭球看起来没有变化？

**A**: 检查：
1. `use_risk_adaptive` 是否为 `true`
2. 障碍物是否在运动（动态障碍）
3. 添加调试输出，打印 `vc, ttc, s, phi`

更多问题 → [完整实现文档 FAQ](RISK_ADAPTIVE_ELLIPSOID_IMPLEMENTATION.md#常见问题)

---

## 📞 获取帮助

**按优先级顺序**:

1. **快速问题** → [参数速查表](RISK_ADAPTIVE_PARAMS_CHEATSHEET.md)
2. **使用问题** → [快速开始指南](QUICK_START_RISK_ADAPTIVE.md)
3. **原理问题** → [完整实现文档](RISK_ADAPTIVE_ELLIPSOID_IMPLEMENTATION.md)
4. **代码问题** → 搜索代码中的 `Risk-Adaptive` 注释

---

## ✅ 实现状态

- [x] 理论公式实现
- [x] 代码编写
- [x] 编译验证
- [x] 参数配置
- [x] 文档编写（2000+ 行）
- [x] 分析工具
- [ ] 实际测试（等待你运行）
- [ ] 性能对比
- [ ] 参数调优

**当前状态**: ✅ 实现完成，可以使用

---

## 🌟 亮点

1. **理论严格**: 完全基于你提供的数学公式
2. **工程鲁棒**: 数值稳定、参数裁剪、低速稳定
3. **最小侵入**: 只改 `updateObstacleParam()`，不动 MPC 核心
4. **文档齐全**: 2000+ 行，涵盖所有细节
5. **即插即用**: 编译通过，开关控制

---

**开始使用** → [快速开始指南](QUICK_START_RISK_ADAPTIVE.md)

---

## 🆕 最新更新（v1.4）

**日期**: 2026-01-13

**修复内容**:
- 🚨 修复椭球过大导致的倒退和崩溃问题
- ✅ 添加完善的数值安全保护（距离、TTC、NaN/Inf 检查）
- ✅ 添加实时诊断（椭球过大预警、倒退检测）
- ✅ 优化参数默认值（降低 31% 最大膨胀）

**详细说明** → [崩溃和倒退问题修复文档](BUG_FIX_CRASH_AND_RETREAT.md)

---

**版本**: v1.4  
**日期**: 2026-01-13  
**状态**: ✅ 修复完成，待测试

