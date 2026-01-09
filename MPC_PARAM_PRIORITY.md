# MPC 参数优先级说明

## 问题：两个地方都有 dynamic_safety_dist

### 位置 1：planner_param.yaml（生效✅）
```yaml
# 文件: autonomous_flight/cfg/mpc_navigation/planner_param.yaml
mpc_planner/dynamic_safety_dist: 0.6  ✅ 这个生效！
```

### 位置 2：mpc_param.yaml（不生效❌）
```yaml
# 文件: trajectory_planner/cfg/mpc_interactive/mpc_param.yaml
dynamic_safety_dist: 0.8  ❌ 这个不生效！
```

---

## 为什么？

### 1. Launch 文件加载顺序

`intent_mpc_demo.launch` 中：

```xml
<rosparam file="$(find autonomous_flight)/cfg/mpc_navigation/planner_param.yaml" />
<!-- ✅ 加载了这个文件 -->

<!-- ❌ 没有加载 trajectory_planner/cfg/mpc_interactive/mpc_param.yaml -->
```

**关键**：`mpc_interactive/mpc_param.yaml` 根本没被加载！

### 2. 代码读取逻辑

`mpcPlanner.cpp` line 101:

```cpp
this->nh_.getParam(this->ns_ + "/dynamic_safety_dist", this->dynamicSafetyDist_)
                   ↑
            命名空间 "mpc_planner"
```

所以代码读取的是：`/mpc_planner/dynamic_safety_dist`

### 3. ROS 参数命名空间

```yaml
# planner_param.yaml 中的格式：
mpc_planner/dynamic_safety_dist: 0.6
     ↑             ↑
  命名空间      参数名

# 加载后在 ROS 参数服务器中：
/mpc_planner/dynamic_safety_dist = 0.6
```

---

## 结论

### ✅ 生效的参数

```yaml
文件: autonomous_flight/cfg/mpc_navigation/planner_param.yaml
参数: mpc_planner/dynamic_safety_dist: 0.6
```

**这是当前实际使用的值：0.6m**

### ❌ 不生效的参数

```yaml
文件: trajectory_planner/cfg/mpc_interactive/mpc_param.yaml
参数: dynamic_safety_dist: 0.8
```

**这个文件是给 mpc_interactive（交互式MPC）使用的，不是当前系统用的！**

---

## 如何修改参数？

### 方法 1：修改生效的文件（推荐）

```yaml
# 编辑: autonomous_flight/cfg/mpc_navigation/planner_param.yaml

mpc_planner/dynamic_safety_dist: 1.0  # 改为 1.0m
```

### 方法 2：验证参数是否生效

启动系统后，查看 ROS 参数：

```bash
rosparam get /mpc_planner/dynamic_safety_dist
# 应该输出: 0.6
```

### 方法 3：运行时动态修改（临时）

```bash
rosparam set /mpc_planner/dynamic_safety_dist 1.0
```

**注意**：这个修改重启后失效，永久修改要改 YAML 文件。

---

## 完整的 MPC 参数文件

### 当前生效的参数文件

```
autonomous_flight/cfg/mpc_navigation/planner_param.yaml
```

**包含的 MPC 参数**：
```yaml
mpc_planner/horizon: 20
mpc_planner/z_range_min: 0.9
mpc_planner/z_range_max: 2.9 
mpc_planner/static_safety_dist: 0.8
mpc_planner/dynamic_safety_dist: 0.6  ✅
mpc_planner/static_constraint_slack_ratio: 0.01
mpc_planner/dynamic_constraint_slack_ratio: 0.2
```

### 不生效的参数文件（忽略）

```
trajectory_planner/cfg/mpc_interactive/mpc_param.yaml
```

这个文件是给交互式测试用的，**不是 intent_mpc_demo.launch 使用的**！

---

## 总结

| 参数位置 | 值 | 是否生效 | 说明 |
|---------|-----|---------|------|
| `planner_param.yaml` | 0.6 | ✅ **生效** | launch 加载了这个文件 |
| `mpc_param.yaml` | 0.8 | ❌ 不生效 | launch 没加载这个文件 |

**要修改参数，改 `planner_param.yaml` 中的 `mpc_planner/dynamic_safety_dist`！**
