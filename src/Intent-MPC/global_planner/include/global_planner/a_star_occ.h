#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <map_manager/occupancyMap.h>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <functional>

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
    int x{0}, y{0}, z{0};
    double g{0.0}, h{0.0};
    int parent{-1};
    inline double f() const { return g + h; }
  };

  bool posToIndex(const Eigen::Vector3d &pos, int &ix, int &iy, int &iz) const;
  Eigen::Vector3d indexToPos(int ix, int iy, int iz) const;
  bool isFree(int ix, int iy, int iz) const;
  inline int idx1d(int x, int y, int z) const
  {
    // use a hash combining x,y,z; assumes map size < 1e5 on each axis
    return (z * 100000000) + (y * 100000) + x;
  }

  ros::NodeHandle nh_;
  std::shared_ptr<mapManager::occMap> map_;
  geometry_msgs::Pose start_, goal_;
  double res_{0.1};
  bool use26dir_{true};
};

}  // namespace globalPlanner

