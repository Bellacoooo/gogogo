# A*风险处理重构说明

## 📋 问题背景

**旧方案的问题**：
- 把风险加到g(n)中：`g = g_cur + step + risk_cost`
- 导致路径长度失真（g值包含风险代价）
- 风险和距离混在一起，难以调参
- 与用户设计方案不符

## ✅ 新方案（按用户建议）

### 核心设计原则

```
风险地图的作用：
1. 硬约束（edgeFeasible）: risk > R_max → 禁入
2. 软引导（h_risk）: 启发式提前感知风险，早绕行

距离和风险分离：
- g(n): 只累计几何距离（纯路径长度）
- h(n): 距离启发 + 风险启发
```

---

## 📐 数学公式

### 1. g(n) - 累计代价

```
g_new = g_cur + w_d * distance(q, q_neighbor)
```

**说明**：
- 只计算几何距离
- `w_d`: 距离权重（默认1.0）
- `distance`: 欧氏距离（1, √2, √3 × resolution）

### 2. h(n) - 启发式

```
h(n) = h_dist(n) + eta * h_risk(n)

其中：
  h_dist(n) = w_d * ||q_goal - q||
  h_risk(n) = riskLookahead(q, q_goal)
```

**说明**：
- `h_dist`: 到目标的欧氏距离
- `h_risk`: 风险前瞻（从q朝goal方向前瞻S_h米）
- `eta`: 风险启发式权重（默认1.0，调参范围0.5~5）

### 3. edgeFeasible(q, q') - 边段可行性

```
对segment(q, q')上按sample_step采样：
  for each q_s:
    if !inMap(q_s): return false
    if isOccupied(q_s): return false
    if getRisk(q_s) > R_max: return false
  return true
```

**说明**：
- 检查整条边段，不只是端点
- `R_max`: 极高风险阈值（超过则禁入）
- `sample_step`: 采样步长（默认resolution/2）

### 4. riskLookahead(q, q_goal) - 风险前瞻

```
dir = normalize(q_goal - q)
h_risk = 0
for s = 0 to S_h step Δs:
  q_m = q + dir * s
  if !inMap(q_m) or isOccupied(q_m): break
  h_risk += getRisk(q_m) * Δs
return h_risk
```

**说明**：
- 从q朝goal方向前瞻S_h米
- 如果前方被障碍挡住，停止累加
- `S_h`: 前瞻距离（默认5m，调参范围2~10m）
- `Δs`: 前瞻采样间隔（默认resolution/2）

---

## 🎯 新增参数

| 参数 | 默认值 | 说明 | 调参范围 |
|------|--------|------|----------|
| `astar/w_d` | 1.0 | 距离权重（g中使用） | 0.5 ~ 2.0 |
| `astar/R_max` | 0.8 | 极高风险阈值（硬约束） | 0.5 ~ 0.95 |
| `astar/eta` | 1.0 | 风险启发式权重 | 0.5 ~ 5.0 |
| `astar/S_h` | 5.0 | 前瞻距离（米） | 2.0 ~ 10.0 |

**自动计算的参数**：
- `Delta_s = resolution / 2`: 前瞻采样间隔
- `sample_step = resolution / 2`: 边段采样步长

---

## 🔧 实现细节

### 1. 统一风险查询接口

```cpp
double AStarOccMap::getRisk(const Eigen::Vector3d& q) const
{
  if (use_risk_map_25d_ && risk_map_25d_) {
    return risk_map_25d_->query(q);  // 3D查询
  } else if (risk_map_) {
    return risk_map_->queryBilinear(q(0), q(1));  // 2D查询
  }
  return 0.0;
}
```

### 2. 边段可行性检查

```cpp
bool AStarOccMap::edgeFeasible(const Eigen::Vector3d& q_start, 
                                const Eigen::Vector3d& q_end) const
{
  double dist = (q_end - q_start).norm();
  int num_samples = ceil(dist / sample_step_);
  
  for (int i = 0; i <= num_samples; ++i) {
    double t = i / num_samples;
    Eigen::Vector3d q_s = q_start + t * (q_end - q_start);
    
    if (!map_->isInMap(q_s)) return false;
    if (map_->isInflatedOccupied(q_s)) return false;
    
    // 关键：风险阈值硬约束
    if (getRisk(q_s) > R_max_) return false;
  }
  
  return true;
}
```

### 3. 风险前瞻启发式

```cpp
double AStarOccMap::riskLookahead(const Eigen::Vector3d& q, 
                                   const Eigen::Vector3d& q_goal) const
{
  Eigen::Vector3d dir = (q_goal - q).normalized();
  double lookahead_dist = min(S_h_, (q_goal - q).norm());
  int num_samples = ceil(lookahead_dist / Delta_s_);
  
  double h_risk = 0.0;
  for (int i = 0; i <= num_samples; ++i) {
    double s = (i / num_samples) * lookahead_dist;
    Eigen::Vector3d q_m = q + dir * s;
    
    // 前方被障碍挡住，停止累加
    if (!map_->isInMap(q_m) || map_->isInflatedOccupied(q_m)) {
      break;
    }
    
    h_risk += getRisk(q_m) * Delta_s_;
  }
  
  return h_risk;
}
```

### 4. 启发式函数

```cpp
auto heuristic = [&](int x, int y, int z) -> double {
  Eigen::Vector3d q = indexToPos(x, y, z);
  
  // h_dist: 欧氏距离
  double dx = x - gx;
  double dy = y - gy;
  double dz = z - gz;
  double h_dist = sqrt(dx*dx + dy*dy + dz*dz) * grid_res_;
  
  // h_risk: 风险前瞻
  double h_risk = (eta_ > 0) ? riskLookahead(q, g) : 0.0;
  
  return w_d_ * h_dist + eta_ * h_risk;
};
```

### 5. 邻居扩展

```cpp
for (int k = 0; k < num_dirs; ++k) {
  int nx = cur.x + dx[k];
  int ny = cur.y + dy[k];
  int nz = cur.z + dz[k];
  
  Eigen::Vector3d neighbor_pos = indexToPos(nx, ny, nz);
  
  // 检查占据
  if (!isFree(nx, ny, nz)) continue;
  
  // 🔧 关键：检查边段可行性（包含风险阈值）
  Eigen::Vector3d cur_pos = indexToPos(cur.x, cur.y, cur.z);
  if (!edgeFeasible(cur_pos, neighbor_pos)) continue;
  
  // 🔧 关键：g只累计几何距离
  double step = sqrt(dx[k]*dx[k] + dy[k]*dy[k] + dz[k]*dz[k]) * grid_res_;
  double tentative_g = cur.g + w_d_ * step;  // 不加风险！
  
  // ... 剩余A*逻辑
}
```

---

## 📊 新旧方案对比

| 维度 | 旧方案 | 新方案 |
|------|--------|--------|
| **g(n)** | 距离 + 风险代价 | 只有距离 ✅ |
| **h(n)** | 只有距离 | 距离 + 风险前瞻 ✅ |
| **风险阈值** | ❌ 无 | R_max硬约束 ✅ |
| **边段检查** | 只检查端点 | 采样整条边段 ✅ |
| **路径长度** | 失真（包含风险） | 准确（纯距离） ✅ |
| **绕路行为** | 被动（节点展开时才感知） | 主动（提前前瞻） ✅ |

---

## 🎮 参数调优指南

### eta（风险启发式权重）- 最重要

```yaml
# 场景1：空旷环境，少量障碍物
astar/eta: 0.5  # 轻微绕路，偏好直线

# 场景2：中等密度障碍物
astar/eta: 1.0  # 默认，平衡绕路和路径长度

# 场景3：高密度障碍物
astar/eta: 2.0~5.0  # 积极绕路，避开高风险区
```

**效果**：
- `eta = 0`: 完全忽略风险（只靠R_max硬约束）
- `eta ↑`: 更早绕路，路径更保守
- `eta ↓`: 更晚绕路，路径更激进

### R_max（风险阈值）- 硬边界

```yaml
# 严格禁入
astar/R_max: 0.6  # 风险超过0.6立即禁入

# 默认
astar/R_max: 0.8  # 风险超过0.8禁入

# 宽松（不推荐）
astar/R_max: 0.95  # 只禁入极高风险区
```

**效果**：
- `R_max ↓`: 更多区域被禁入，路径绕路多
- `R_max ↑`: 更少区域被禁入，路径可能靠近障碍物

### S_h（前瞻距离）

```yaml
# 短视
astar/S_h: 2.0  # 只看前方2米

# 默认
astar/S_h: 5.0  # 看前方5米

# 远视
astar/S_h: 10.0  # 看前方10米（计算量大）
```

**效果**：
- `S_h ↑`: 更早感知远处风险，但计算量增加
- `S_h ↓`: 只看近处，计算快但可能陷入局部最优

---

## ⚠️ 注意事项

### 1. 不要把风险加到g中

```cpp
// ❌ 错误（旧方案）
double tentative_g = cur.g + step + risk_cost;

// ✅ 正确（新方案）
double tentative_g = cur.g + w_d_ * step;
```

**原因**：
- g应该反映纯路径长度
- 风险通过h和edgeFeasible处理

### 2. edgeFeasible采样必须做

```cpp
// ❌ 错误：只检查端点
if (!isFree(nx, ny, nz)) continue;

// ✅ 正确：检查整条边段
if (!edgeFeasible(cur_pos, neighbor_pos)) continue;
```

**原因**：
- 对角走格容易"穿障/穿高风险带"
- 端点free不代表中间free

### 3. riskLookahead只用于启发式

```cpp
// ✅ 正确：只在h中使用
double h_risk = riskLookahead(q, q_goal);
double h = w_d_ * h_dist + eta_ * h_risk;

// ❌ 错误：不要加到g中
// double g = cur.g + step + h_risk;  // NO!
```

**原因**：
- riskLookahead用于排序偏好
- 可行域由occupancy + R_max决定

---

## 🚀 效果

### 优势

1. **路径长度准确**：g不含风险，最终路径长度=实际几何距离
2. **风险感知提前**：通过h_risk提前感知风险结构，早绕行
3. **硬约束保护**：R_max确保不进入极高风险区
4. **参数解耦**：距离和风险分离，调参更直观
5. **不过度保守**：风险不累加到g，不会因长路径而过度惩罚

### 适用场景

- ✅ 静态+动态混合环境
- ✅ 风险地图表达障碍物周围风险场
- ✅ 需要提前绕路避开高风险区
- ✅ 需要准确的路径长度信息

---

## 📝 配置示例

```yaml
# 推荐配置（默认）
astar:
  w_d: 1.0         # 距离权重
  R_max: 0.8       # 风险阈值（0.8以上禁入）
  eta: 1.0         # 风险启发式权重
  S_h: 5.0         # 前瞻距离5米
  use_26_dir: true # 使用26邻域（允许对角）
  
# 激进配置（偏好直线，接受一定风险）
astar:
  w_d: 1.0
  R_max: 0.9       # 更宽松的阈值
  eta: 0.5         # 更低的风险权重
  S_h: 3.0         # 更短的前瞻
  
# 保守配置（积极绕路，避免风险）
astar:
  w_d: 1.0
  R_max: 0.6       # 更严格的阈值
  eta: 3.0         # 更高的风险权重
  S_h: 8.0         # 更长的前瞻
```

---

## 🔍 调试

### 1. 查看风险前瞻效果

```cpp
// 在A*中添加调试日志
if (expanded_nodes < 20) {
  double h_dist = sqrt(dx*dx + dy*dy + dz*dz) * grid_res_;
  double h_risk = riskLookahead(q, g);
  ROS_INFO("[A*-DEBUG] node(%d,%d,%d): h_dist=%.3f, h_risk=%.3f, h_total=%.3f",
           x, y, z, h_dist, h_risk, h_dist + eta_ * h_risk);
}
```

### 2. 查看边段禁入

```cpp
// 在edgeFeasible中添加日志
if (risk > R_max_) {
  ROS_WARN("[A*-EDGE] Edge blocked by risk: q_s=(%.3f,%.3f,%.3f), risk=%.3f > R_max=%.3f",
           q_s.x(), q_s.y(), q_s.z(), risk, R_max_);
  return false;
}
```

### 3. 比较新旧g值

```cpp
// 临时启用对比
double tentative_g_new = cur.g + w_d_ * step;
double tentative_g_old = cur.g + step + risk_cost;
ROS_INFO("[A*-COMPARE] g_new=%.3f, g_old=%.3f, diff=%.3f",
         tentative_g_new, tentative_g_old, tentative_g_old - tentative_g_new);
```

---

## ✅ 总结

**新方案核心思想**：
- **分离关注点**：距离→g，风险→h+edgeFeasible
- **提前感知**：风险前瞻让A*提前绕路
- **硬软结合**：R_max硬约束 + eta软引导

**不要做的事**：
- ❌ 不要把风险加到g中
- ❌ 不要只检查端点，要检查边段
- ❌ 不要把riskLookahead用于可行域判断

**调参优先级**：
1. `eta`（风险启发式权重）- 影响绕路积极性
2. `R_max`（风险阈值）- 影响禁入范围
3. `S_h`（前瞻距离）- 影响计算量和远见

