#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <vector>
#include <mutex>

namespace globalPlanner
{

/**
 * @brief 2D风险地图类，用于存储和查询风险值
 * 
 * 从 nav_msgs::OccupancyGrid 格式的风险地图中读取数据，
 * 提供双线性插值查询功能
 */
class RiskMap2D
{
public:
  RiskMap2D();
  ~RiskMap2D() = default;

  /**
   * @brief 从 OccupancyGrid 消息更新风险地图
   * @param msg 风险地图消息（值范围 0-100，表示归一化的风险概率）
   */
  void updateFromMsg(const nav_msgs::OccupancyGrid& msg);

  /**
   * @brief 查询世界坐标 (wx, wy) 处的风险值（使用双线性插值）
   * @param wx 世界坐标 x（米）
   * @param wy 世界坐标 y（米）
   * @return 风险值 [0.0, 1.0]，如果坐标在地图外返回 0.0
   */
  double queryBilinear(double wx, double wy) const;

  /**
   * @brief 检查风险地图是否有效
   * @return true 如果地图已初始化且有效
   */
  bool isValid() const;

  /**
   * @brief 获取地图原点（世界坐标）
   */
  Eigen::Vector2d getOrigin() const { return origin_; }

  /**
   * @brief 获取地图分辨率（米）
   */
  double getResolution() const { return resolution_; }

  /**
   * @brief 获取地图尺寸（栅格数）
   */
  Eigen::Vector2i getSize() const { return size_; }

private:
  mutable std::mutex mutex_;    // 保护并发访问的互斥锁
  bool valid_;
  double resolution_;           // 地图分辨率（米）
  Eigen::Vector2d origin_;      // 地图原点（世界坐标，左下角）
  Eigen::Vector2i size_;        // 地图尺寸（宽度，高度）
  std::vector<double> data_;    // 风险数据（归一化到 [0.0, 1.0]）

  /**
   * @brief 世界坐标转栅格坐标
   * @param wx 世界坐标 x
   * @param wy 世界坐标 y
   * @param ix 输出栅格 x 索引
   * @param iy 输出栅格 y 索引
   * @return true 如果坐标在地图范围内
   */
  bool worldToGrid(double wx, double wy, int& ix, int& iy) const;

  /**
   * @brief 获取栅格 (ix, iy) 处的风险值（不进行插值）
   * @param ix 栅格 x 索引
   * @param iy 栅格 y 索引
   * @return 风险值 [0.0, 1.0]，如果索引越界返回 0.0
   */
  double getValue(int ix, int iy) const;
  
  /**
   * @brief 获取栅格值（不加锁版本，用于内部调用）
   */
  double getValueUnlocked(int ix, int iy) const;
};

/**
 * @brief 根据 z_gate 门限提升风险值（从 2D 提升到 3D）
 * 
 * 逻辑：
 * - 如果 wz > z_gate：返回 0.0（高于门限，风险为0）
 * - 如果 wz < 0：返回 0.0（低于地面，风险为0）
 * - 如果 0 <= wz <= z_gate：返回 p2d（2D风险值）
 * 
 * @param p2d 2D风险值 [0.0, 1.0]
 * @param wz 世界坐标 z（高度，米）
 * @param z_gate 高度门限（米）
 * @return 3D风险值 [0.0, 1.0]
 */
inline double liftRiskGated(double p2d, double wz, double z_gate)
{
  if (wz > z_gate) return 0.0;  // 高于门限，风险为0
  if (wz < 0.0) return 0.0;      // 低于地面，风险为0
  return p2d;                    // 0 <= wz <= z_gate，返回2D风险值
}

/**
 * @brief 将风险值转换为代价（分段幂函数公式，软约束）
 * 
 * 🔧 改进：使用 1.5 次方而不是 2 次方，让低风险区域的代价更温和
 * 公式：cost = k_risk * risk^1.5
 * 
 * 特性：
 * - risk = 0 时，cost = 0
 * - risk = 0.5 时，cost = k_risk * 0.354（比原来的 0.25 稍高，更鼓励通过）
 * - risk = 1.0 时，cost = k_risk
 * - k_risk 控制转换的敏感度
 * 
 * 优点：
 * - 软约束：低风险区域代价较小，鼓励路径通过
 * - 避免完全回避：不会让 A* 完全避开低-中风险区域
 * - 梯度平滑：幂函数提供平滑的梯度，有利于路径优化
 * - 避免 NaN：不会出现 log(0) 的情况，数值稳定
 * 
 * @param risk 风险值 [0.0, 1.0]
 * @param k_risk 风险代价系数（默认 1.0）
 * @return 代价值（>= 0）
 */
inline double riskToCostLog(double risk, double k_risk = 1.0)
{
  if (risk <= 0.0) return 0.0;
  if (risk > 1.0) risk = 1.0;  // 限制在 [0, 1] 范围内
  // 使用 1.5 次方：比线性更重视高风险，比二次方更温和
  return k_risk * std::pow(risk, 1.5);
}

} // namespace globalPlanner
