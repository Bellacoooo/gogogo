#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <map_manager/occupancyMap.h>
#include <Eigen/Dense>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <vector>
#include <limits>

namespace globalPlanner
{

class AStarOccMap
{
public:
  explicit AStarOccMap(const ros::NodeHandle &nh);

  void setMap(const std::shared_ptr<mapManager::occMap> &map);
  void updateStart(const geometry_msgs::Pose &start);
  void updateGoal(const geometry_msgs::Pose &goal);
  
  // 保留接口以兼容现有代码，但不使用
  struct IntentPred
  {
    double probability{0.0};
    Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d cov_inv{Eigen::Matrix3d::Identity()};
  };
  struct DynObstaclePred
  {
    int id{0};
    std::vector<IntentPred> intents;
  };
  void setDynamicPredictions(const std::vector<DynObstaclePred> &preds);
  double calculateDynamicCost(const Eigen::Vector3d &pos) const;

  void makePlan(nav_msgs::Path &path);

private:
  struct Node
  {
    int x{0}, y{0}, z{0};
    double g{0.0}, h{0.0};
    long long parent{-1};  // 使用long long存储parent key
    inline double f() const { return g + h; }
  };

  bool posToIndex(const Eigen::Vector3d &pos, int &ix, int &iy, int &iz) const;
  Eigen::Vector3d indexToPos(int ix, int iy, int iz) const;
  bool isFree(int ix, int iy, int iz) const;
  
  // 使用 64 位索引避免在较大地图尺寸下发生整型溢出
  inline long long idx1d(int x, int y, int z) const
  {
    return (static_cast<long long>(z) * 100000000LL) +
           (static_cast<long long>(y) * 100000LL) +
           static_cast<long long>(x);
  }

  ros::NodeHandle nh_;
  ros::Publisher pathPub_;  // A*原始路径发布器
  std::shared_ptr<mapManager::occMap> map_;
  geometry_msgs::Pose start_, goal_;
  
  // 原始占据地图分辨率
  double res_{0.1};
  // A* 内部使用的"虚拟粗栅格"分辨率（单位：米）
  double grid_res_param_{0.0};
  double grid_res_{0.1};
  
  // 地图原点偏移（必须与occupancyMap一致）
  Eigen::Vector3d map_origin_{Eigen::Vector3d::Zero()};
  
  // 是否使用26邻域（否则使用6邻域）
  bool use26dir_{true};
  
  // 最大展开节点数
  std::size_t max_expanded_nodes_{300000};
};

}  // namespace globalPlanner
