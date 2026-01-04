#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <map_manager/occupancyMap.h>
#include <Eigen/Dense>
#include <vector>
#include <utility>

// Forward declarations for SIPP types
struct Grid;
struct DynamicObstacle;

namespace globalPlanner
{

class SippOccMap
{
public:
  explicit SippOccMap(const ros::NodeHandle &nh);

  void setMap(const std::shared_ptr<mapManager::occMap> &map);
  void updateStart(const geometry_msgs::Pose &start);
  void updateGoal(const geometry_msgs::Pose &goal);
  
  void makePlan(nav_msgs::Path &path);

private:
  // 地图转换：3D occupancy map → 2D Grid（局部 5x5 米）
  void convertMapToGrid(const Eigen::Vector3d &center_pos, double local_size, 
                        Grid &grid, int &grid_width, int &grid_height,
                        Eigen::Vector2d &grid_origin);

  // 坐标转换：世界坐标 → 栅格坐标
  std::pair<int, int> worldToGrid(const Eigen::Vector3d &world_pos, 
                                   const Eigen::Vector2d &grid_origin, 
                                   double resolution) const;

  // 坐标转换：栅格坐标 → 世界坐标
  Eigen::Vector3d gridToWorld(const std::pair<int, int> &grid_pos,
                               const Eigen::Vector2d &grid_origin,
                               double resolution, double z_height) const;

  // 获取动态障碍物（从预测模块，暂时为空实现）
  void getDynamicObstacles(std::vector<DynamicObstacle> &obstacles) const;

  ros::NodeHandle nh_;
  std::shared_ptr<mapManager::occMap> map_;
  geometry_msgs::Pose start_, goal_;
  
  // 地图参数
  double map_res_{0.1};  // 地图分辨率
  double local_map_size_{5.0};  // 局部地图大小（米），5x5 米
  
  // 当前无人机位置（用于确定局部地图中心）
  Eigen::Vector3d current_pos_{Eigen::Vector3d::Zero()};
  
  // SIPP 参数
  int max_time_{100};  // 最大时间步数
};

} // namespace globalPlanner

