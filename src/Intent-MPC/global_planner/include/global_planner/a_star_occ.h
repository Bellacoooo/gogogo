#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <map_manager/occupancyMap.h>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace globalPlanner
{

class AStarOccMap
{
public:
  explicit AStarOccMap(const ros::NodeHandle &nh);

  void setMap(const std::shared_ptr<mapManager::occMap> &map);
  void updateStart(const geometry_msgs::Pose &start);
  void updateGoal(const geometry_msgs::Pose &goal);
  void makePlan(nav_msgs::Path &path);

private:
  struct Node
  {
    int x{0}, y{0};
    double g{0.0}, h{0.0};
    int parent{-1};
    inline double f() const { return g + h; }
  };

  bool posToIndex(const Eigen::Vector3d &pos, int &ix, int &iy) const;
  Eigen::Vector3d indexToPos(int ix, int iy, double z) const;
  bool isFree(int ix, int iy) const;

  ros::NodeHandle nh_;
  std::shared_ptr<mapManager::occMap> map_;
  geometry_msgs::Pose start_, goal_;
  double res_{0.1};
  bool use8dir_{true};
  double occThreshLog_{0.0};  // use inflated occupancy check instead
};

}  // namespace globalPlanner

