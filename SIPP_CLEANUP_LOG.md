# SIPP 清理记录

## 清理时间
2026-01-07

## 清理原因
用户已删除 sipp 相关包，但编译时仍有残留引用导致错误。

## 清理的文件

### 1. `src/Intent-MPC/global_planner/CMakeLists.txt`

**修改内容**:
```cmake
# 删除了 sipp_vendor 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap_ros
  map_manager
  # sipp_vendor  # 已删除
)

# 删除了 sipp_vendor 依赖声明
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_planner
  CATKIN_DEPENDS roscpp rospy std_msgs  # 移除 sipp_vendor
  #DEPENDS system_lib
)

# 删除了 sipp 链接和源文件
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})  # 移除 sipp
# library 增加 A* 和风险地图源
target_sources(${PROJECT_NAME} PRIVATE src/a_star_occ.cpp src/risk_map_2d.cpp)  # 移除 src/sipp_occ_map.cpp
```

### 2. `src/Intent-MPC/autonomous_flight/include/autonomous_flight/mpcNavigation.h`

**修改内容**:
```cpp
// 注释掉 SIPP 头文件
// #include <global_planner/sipp_occ_map.h>  // SIPP已删除

// 注释掉 SIPP 规划器成员变量
// std::shared_ptr<globalPlanner::SippOccMap> sippPlanner_;  // SIPP已删除

// 更新注释
std::string globalPlannerType_ = "rrt"; // rrt / astar (sipp已删除)
```

### 3. `src/Intent-MPC/autonomous_flight/include/autonomous_flight/mpcNavigation.cpp`

**修改内容**:
```cpp
// 删除 SIPP 初始化代码
// initialize global planner (rrt / astar)  // 移除了 sipp
if (this->globalPlannerType_ == "astar"){
    this->aStarPlanner_.reset(new globalPlanner::AStarOccMap (this->nh_));
    this->aStarPlanner_->setMap(this->map_);
}
// SIPP 已删除

// 删除 SIPP 规划调用代码
// SIPP 已删除
else if (this->rrtPlanner_){
    this->rrtPlanner_->updateStart(this->odom_.pose.pose);
    this->rrtPlanner_->updateGoal(this->goal_.pose);
    this->rrtPlanner_->makePlan(rrtPathMsgTemp);
}
```

## 未修改的文件

以下文件中可能仍有 SIPP 相关注释或配置，但不影响编译：

### `src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/flight_base.yaml`
```yaml
global_planner_type: "astar"  # "rrt" or "astar" or "sipp"
```
- 注释中仍提到 "sipp"，但不影响功能（只要不设置为 "sipp"）

## 被删除的 SIPP 源文件（应该已不存在）

以下文件应该已被用户删除，如果还在请手动删除：
- `src/Intent-MPC/global_planner/src/sipp_occ_map.cpp`
- `src/Intent-MPC/global_planner/include/global_planner/sipp_occ_map.h`
- `src/Intent-MPC/global_planner/include/global_planner/sipp_adapter.h`
- `src/Intent-MPC/sipp_vendor/` (整个包)

## 验证

### 编译验证
```bash
cd /home/ff/intent-mpc
catkin_make
```
**结果**: ✅ 编译成功，无错误

### 可用的全局规划器
现在只支持以下规划器：
- `"rrt"` - RRT 规划器（默认）
- `"astar"` - A* 规划器

**不再支持**:
- ~~`"sipp"`~~ - 已删除

## 配置建议

### flight_base.yaml
```yaml
# 使用 A* 规划器（推荐）
global_planner_type: "astar"

# 或使用 RRT 规划器
# global_planner_type: "rrt"
```

## 注意事项

1. **不要设置 `global_planner_type: "sipp"`**，否则会使用默认的 RRT 规划器

2. **SIPP 相关参数已无效**，如果配置文件中有 sipp 相关参数，会被忽略

3. **推荐使用 A* 规划器**进行动态障碍物避障实验

## 清理完整性检查

运行以下命令确认没有残留：
```bash
cd /home/ff/intent-mpc
grep -r "sipp" src/Intent-MPC/global_planner/CMakeLists.txt
grep -r "sippPlanner_" src/Intent-MPC/autonomous_flight/include/
```

如果有输出，说明还有残留需要清理。

## 编译状态

✅ 所有 SIPP 引用已清理
✅ 编译成功，无错误
✅ 系统可正常使用 RRT 和 A* 规划器

