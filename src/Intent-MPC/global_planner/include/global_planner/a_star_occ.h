#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <map_manager/occupancyMap.h>
#include <global_planner/risk_map_2d.h>
#include <global_planner/risk_map_25d.h>  // 🔧 新增：支持2.5D风险地图
#include <Eigen/Dense>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include <mutex>

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

  /**
   * @brief 设置风险地图（用于风险感知路径规划）
   * @param risk_map 风险地图指针（可以为 nullptr，表示不使用风险地图）
   * @note 保留此接口以兼容现有代码，但推荐使用RiskMap25D
   */
  void setRiskMap(const std::shared_ptr<RiskMap2D>& risk_map);
  
  /**
   * @brief 设置2.5D风险地图（新版本，支持3D查询）
   * @param risk_map_25d 2.5D风险地图指针
   */
  void setRiskMap25D(const std::shared_ptr<RiskMap25D>& risk_map_25d);

  /**
   * @brief 设置动态障碍物方框（用于硬约束避障）
   * @param obstacles_pos 障碍物位置列表
   * @param obstacles_size 障碍物尺寸列表
   */
  void setDynamicObstacleBoxes(const std::vector<Eigen::Vector3d>& obstacles_pos,
                                const std::vector<Eigen::Vector3d>& obstacles_size);

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
  // 改进：使用更安全的哈希方法，避免溢出
  inline long long idx1d(int x, int y, int z) const
  {
    // 使用质数哈希，避免溢出
    // 限制坐标范围在合理范围内（±100000）
    const long long PRIME1 = 73856093LL;
    const long long PRIME2 = 19349663LL;
    const long long PRIME3 = 83492791LL;
    
    // 将坐标转换为无符号，然后使用质数哈希
    long long xl = static_cast<long long>(x);
    long long yl = static_cast<long long>(y);
    long long zl = static_cast<long long>(z);
    
    return (xl * PRIME1) ^ (yl * PRIME2) ^ (zl * PRIME3);
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
  
  // 风险地图相关
  std::shared_ptr<RiskMap2D> risk_map_;      // 风险地图指针（旧版2D）
  std::shared_ptr<RiskMap25D> risk_map_25d_; // 2.5D风险地图指针（新版）
  bool use_risk_map_25d_{false};             // 是否使用2.5D风险地图
  double w_risk_{0.0};                       // 风险代价权重
  double k_risk_{1.0};                       // 风险转换系数（仅RiskMap2D使用）
  double z_gate_{2.0};                       // 高度门限（米，仅RiskMap2D使用）
  
  // 动态障碍物方框硬约束
  struct DynamicObstacleBox {
    Eigen::Vector3d center;  // 障碍物中心位置
    Eigen::Vector3d size;    // 障碍物尺寸 (x_width, y_width, z_width)
  };
  mutable std::vector<DynamicObstacleBox> dynamic_obstacle_boxes_;  // 动态障碍物方框列表
  mutable std::mutex dynamic_obstacles_mutex_;  // 保护动态障碍物列表的互斥锁（mutable允许在const函数中使用）
  
  /**
   * @brief 检查点是否与动态障碍物方框碰撞
   * @param pos 要检查的世界坐标点
   * @return true if collides with any obstacle box
   */
  bool checkDynamicObstacleCollision(const Eigen::Vector3d& pos) const;
};

}  // namespace globalPlanner
