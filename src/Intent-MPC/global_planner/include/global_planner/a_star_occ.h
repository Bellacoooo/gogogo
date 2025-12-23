#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
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
  // 使用 64 位索引避免在较大地图尺寸下发生整型溢出
  inline long long idx1d(int x, int y, int z) const
  {
    // 组合哈希：假定单轴尺寸 < 1e5，在 int64 范围内安全
    return (static_cast<long long>(z) * 100000000LL) +
           (static_cast<long long>(y) * 100000LL) +
           static_cast<long long>(x);
  }

  double estimateArrivalTime(const Eigen::Vector3d &pos) const;

  ros::NodeHandle nh_;
  std::shared_ptr<mapManager::occMap> map_;
  geometry_msgs::Pose start_, goal_;
  // 原始占据地图分辨率（由 occMap 提供，用于参考，不直接参与 A* 网格）
  double res_{0.1};
  // A* 内部使用的“虚拟粗栅格”分辨率（单位：米），可独立于占据地图设置
  // 若 grid_res_param_ <= 0，则退化为使用 res_（占据地图分辨率）
  double grid_res_param_{0.0};
  double grid_res_{0.1};
  bool use26dir_{true};
  // 代价权重（可通过参数配置 astar/w1,w2,w3）
  double w1_dist_{1.0};
  double w2_static_{0.0};   // 当前未实现距离场，先保留接口
  double w3_dynamic_{0.0};

  // 局部重规划窗口尺寸（以当前起点为中心的 XY 范围，单位：米）
  double local_range_xy_{12.0};
  // 为防止在复杂环境中节点膨胀导致内存/时间问题，对单次搜索的最大展开节点数做上限
  std::size_t max_expanded_nodes_{200000};

  // 动态风险相关
  std::vector<DynObstaclePred> dyn_preds_;
  double avg_velocity_{1.5};              // m/s，估计到达时间用
  Eigen::Vector3d start_pos_{0.0,0.0,0.0};

  // 动态风险场（来自 dynamic_predictor/dynamic_risk_map）
  ros::Subscriber risk_map_sub_;
  nav_msgs::OccupancyGrid::ConstPtr latest_risk_map_;
  mutable std::mutex risk_map_mutex_;
  void riskMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  // 工具函数：查询某个世界坐标 (x,y) 在动态风险图上的风险值 [0,100]
public:
  double getDynamicRisk(double world_x, double world_y) const;
};

}  // namespace globalPlanner

