#include <global_planner/sipp_occ_map.h>
#include <global_planner/sipp_adapter.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <limits>
#include <cmath>

namespace globalPlanner
{

SippOccMap::SippOccMap(const ros::NodeHandle &nh) : nh_(nh)
{
  nh_.param("sipp/local_map_size", local_map_size_, 5.0);
  nh_.param("sipp/max_time", max_time_, 100);
  
  ROS_INFO("[SIPP] SIPP planner initialized (local_map_size=%.2f m, max_time=%d).", 
           local_map_size_, max_time_);
}

void SippOccMap::setMap(const std::shared_ptr<mapManager::occMap> &map)
{
  map_ = map;
  if (map_)
  {
    map_res_ = map_->getRes();
    ROS_INFO("[SIPP] Map set, resolution: %.3f m", map_res_);
  }
}

void SippOccMap::updateStart(const geometry_msgs::Pose &start)
{
  start_ = start;
  // 更新当前位置（用于确定局部地图中心）
  current_pos_(0) = start.position.x;
  current_pos_(1) = start.position.y;
  current_pos_(2) = start.position.z;
}

void SippOccMap::updateGoal(const geometry_msgs::Pose &goal)
{
  goal_ = goal;
}

void SippOccMap::convertMapToGrid(const Eigen::Vector3d &center_pos, double local_size,
                                   Grid &grid, int &grid_width, int &grid_height,
                                   Eigen::Vector2d &grid_origin)
{
  // 计算局部地图的栅格大小
  grid_width = static_cast<int>(std::ceil(local_size / map_res_));
  grid_height = static_cast<int>(std::ceil(local_size / map_res_));
  
  // 确保是奇数，这样中心点正好在中间
  if (grid_width % 2 == 0) grid_width++;
  if (grid_height % 2 == 0) grid_height++;
  
  // 计算局部地图的世界坐标原点（左下角）
  grid_origin(0) = center_pos(0) - (grid_width * map_res_) / 2.0;
  grid_origin(1) = center_pos(1) - (grid_height * map_res_) / 2.0;
  
  // 创建 Grid 对象
  grid = Grid(grid_width, grid_height);
  
  // 遍历每个栅格单元，检查在无人机当前高度是否被占用
  double z_height = center_pos(2);
  
  for (int i = 0; i < grid_width; ++i)
  {
    for (int j = 0; j < grid_height; ++j)
    {
      // 计算该栅格单元的世界坐标
      double world_x = grid_origin(0) + (i + 0.5) * map_res_;
      double world_y = grid_origin(1) + (j + 0.5) * map_res_;
      
      Eigen::Vector3d world_pos(world_x, world_y, z_height);
      
      // 检查该位置是否被占用
      if (map_->isInflatedOccupied(world_pos))
      {
        grid.static_obstacles[i][j] = true;
      }
    }
  }
  
  ROS_DEBUG("[SIPP] Converted map to grid: %dx%d, center=(%.2f, %.2f, %.2f), origin=(%.2f, %.2f)",
            grid_width, grid_height, center_pos(0), center_pos(1), center_pos(2),
            grid_origin(0), grid_origin(1));
}

std::pair<int, int> SippOccMap::worldToGrid(const Eigen::Vector3d &world_pos,
                                             const Eigen::Vector2d &grid_origin,
                                             double resolution) const
{
  int grid_x = static_cast<int>(std::floor((world_pos(0) - grid_origin(0)) / resolution));
  int grid_y = static_cast<int>(std::floor((world_pos(1) - grid_origin(1)) / resolution));
  return std::make_pair(grid_x, grid_y);
}

Eigen::Vector3d SippOccMap::gridToWorld(const std::pair<int, int> &grid_pos,
                                         const Eigen::Vector2d &grid_origin,
                                         double resolution, double z_height) const
{
  double world_x = grid_origin(0) + (grid_pos.first + 0.5) * resolution;
  double world_y = grid_origin(1) + (grid_pos.second + 0.5) * resolution;
  return Eigen::Vector3d(world_x, world_y, z_height);
}

void SippOccMap::getDynamicObstacles(std::vector<DynamicObstacle> &obstacles) const
{
  // TODO: 从预测模块获取动态障碍物
  // 暂时返回空列表
  obstacles.clear();
  
  // 未来实现：从 dynamicPredictor 获取动态障碍物预测
  // 需要将预测的障碍物轨迹转换为 DynamicObstacle 格式
  // DynamicObstacle(x, y, start_time, end_time)
}

void SippOccMap::makePlan(nav_msgs::Path &path)
{
  path.poses.clear();
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  
  if (!map_)
  {
    ROS_ERROR("[SIPP] Map not set!");
    return;
  }
  
  // 获取起点和终点的世界坐标
  Eigen::Vector3d start_world(start_.position.x, start_.position.y, start_.position.z);
  Eigen::Vector3d goal_world(goal_.position.x, goal_.position.y, goal_.position.z);
  
  // 使用起点作为局部地图中心（无人机当前位置）
  Eigen::Vector3d center_pos = start_world;
  
  // 转换地图：3D occupancy map → 2D Grid
  Grid grid(1, 1);  // 临时初始化，会被 convertMapToGrid 覆盖
  int grid_width, grid_height;
  Eigen::Vector2d grid_origin;
  convertMapToGrid(center_pos, local_map_size_, grid, grid_width, grid_height, grid_origin);
  
  // 检查起点和终点是否在局部地图范围内
  std::pair<int, int> start_grid = worldToGrid(start_world, grid_origin, map_res_);
  std::pair<int, int> goal_grid = worldToGrid(goal_world, grid_origin, map_res_);
  
  // 检查起点和终点是否在有效范围内
  if (start_grid.first < 0 || start_grid.first >= grid_width ||
      start_grid.second < 0 || start_grid.second >= grid_height)
  {
    ROS_WARN("[SIPP] Start position out of local map range!");
    return;
  }
  
  if (goal_grid.first < 0 || goal_grid.first >= grid_width ||
      goal_grid.second < 0 || goal_grid.second >= grid_height)
  {
    ROS_WARN("[SIPP] Goal position out of local map range! Goal may be too far.");
    // 将目标点限制在局部地图范围内
    goal_grid.first = std::max(0, std::min(grid_width - 1, goal_grid.first));
    goal_grid.second = std::max(0, std::min(grid_height - 1, goal_grid.second));
  }
  
  // 检查起点和终点是否在障碍物上
  if (!grid.is_valid(start_grid.first, start_grid.second))
  {
    ROS_WARN("[SIPP] Start position is in obstacle!");
    return;
  }
  
  if (!grid.is_valid(goal_grid.first, goal_grid.second))
  {
    ROS_WARN("[SIPP] Goal position is in obstacle!");
    return;
  }
  
  // 获取动态障碍物
  std::vector<DynamicObstacle> obstacles;
  getDynamicObstacles(obstacles);
  
  // 调用 SIPP 算法前打印日志
  ROS_WARN("[SIPP] called");
  
  // 调用 SIPP 算法（从 sipp_vendor 库中链接）
  // 注意：sipp_adapter.h 中使用了 using namespace std，所以 sipp 函数返回的是 std::vector<std::pair<int, int>>
  // 但为了代码清晰，我们显式使用 std:: 前缀
  std::vector<std::pair<int, int>> grid_path = sipp(grid, start_grid, goal_grid, obstacles, max_time_);
  
  // 调用 SIPP 算法后打印日志
  ROS_WARN("[SIPP] cells.size() = %zu", grid_path.size());
  
  if (grid_path.empty())
  {
    ROS_WARN("[SIPP] No path found!");
    return;
  }
  
  // 将栅格坐标路径转换为世界坐标路径
  double z_height = center_pos(2);
  for (const auto &grid_point : grid_path)
  {
    Eigen::Vector3d world_point = gridToWorld(grid_point, grid_origin, map_res_, z_height);
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = world_point(0);
    pose.pose.position.y = world_point(1);
    pose.pose.position.z = world_point(2);
    pose.pose.orientation.w = 1.0;
    
    path.poses.push_back(pose);
  }
  
  ROS_INFO("[SIPP] Path found with %zu waypoints", path.poses.size());
}

} // namespace globalPlanner

