#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <map_manager/occupancyMap.h>
#include <Eigen/Dense>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <functional>
#include <vector>

namespace globalPlanner
{

class AStarOccMap
{
public:
  explicit AStarOccMap(const ros::NodeHandle &nh);

  void setMap(const std::shared_ptr<mapManager::occMap> &map);
  void updateStart(const geometry_msgs::Pose &start);
  void updateGoal(const geometry_msgs::Pose &goal);
  // 提供动态障碍预测（已提前对协方差求逆）
  struct IntentPred
  {
    double probability{0.0};
    Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d cov_inv{Eigen::Matrix3d::Identity()};
  };
  struct DynObstaclePred
  {
    int id{0};
    std::vector<IntentPred> intents;  // 每个障碍物的多意图预测
  };
  void setDynamicPredictions(const std::vector<DynObstaclePred> &preds);

  // 估计到达时间并计算动态代价（按马氏距离加权）
  double calculateDynamicCost(const Eigen::Vector3d &pos) const;

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

  double estimateArrivalTime(const Eigen::Vector3d &pos) const;

  ros::NodeHandle nh_;
  std::shared_ptr<mapManager::occMap> map_;
  geometry_msgs::Pose start_, goal_;
  double res_{0.1};
  bool use26dir_{true};
  // 代价权重（可通过参数配置 astar/w1,w2,w3）
  double w1_dist_{1.0};
  double w2_static_{0.0};   // 当前未实现距离场，先保留接口
  double w3_dynamic_{0.0};

  // 动态风险相关
  std::vector<DynObstaclePred> dyn_preds_;
  double avg_velocity_{1.5};              // m/s，估计到达时间用
  Eigen::Vector3d start_pos_{0.0,0.0,0.0};
};

}  // namespace globalPlanner

