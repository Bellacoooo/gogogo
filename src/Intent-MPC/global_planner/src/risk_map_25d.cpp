/**
 * @file risk_map_25d.cpp
 * @brief 2.5D融合风险地图实现（单层版本）
 */

#include "global_planner/risk_map_25d.h"
#include "map_manager/ESDFMap.h"
#include <cmath>
#include <algorithm>
#include <ros/console.h>

namespace globalPlanner {

RiskMap25D::RiskMap25D(const ros::NodeHandle& nh)
    : is_initialized_(false),
      static_updated_(false),
      dynamic_updated_(false)
{
    // 读取栅格参数
    nh.param("risk_map_25d/resolution", resolution_, 0.1);
    nh.param("risk_map_25d/grid_width", grid_width_, 200);
    nh.param("risk_map_25d/grid_height", grid_height_, 200);
    
    // 读取高度参数（单层）
    nh.param("risk_map_25d/fixed_height", fixed_height_, 1.0);  // 默认1m高度
    
    // 读取静态风险参数
    nh.param("risk_map_25d/d_s", d_s_, 0.5);           // 安全距离 0.5m
    nh.param("risk_map_25d/gamma_s", gamma_s_, 1.0);   // 静态权重
    
    // 读取动态风险参数
    nh.param("risk_map_25d/gamma_d", gamma_d_, 1.0);   // 动态权重
    nh.param("risk_map_25d/beta", beta_, 1.0);         // 椭圆核指数
    nh.param("risk_map_25d/num_pred_steps", num_pred_steps_, 30);
    nh.param("risk_map_25d/lambda_time", lambda_time_, 0.1);
    
    // 读取数值稳定性参数
    nh.param("risk_map_25d/sigma_min", sigma_min_, 0.1);      // 最小标准差 10cm
    nh.param("risk_map_25d/m_cut", m_cut_, 9.0);              // 3-sigma截断
    nh.param("risk_map_25d/risk_epsilon", risk_epsilon_, 1e-4);
    
    // 初始化
    initializeGrids();
    computeTimeWeights();
    
    // 设置默认地图中心
    map_center_ = Eigen::Vector3d::Zero();
    grid_origin_ = Eigen::Vector2d(-grid_width_ * resolution_ / 2.0,
                                     -grid_height_ * resolution_ / 2.0);
    
    is_initialized_ = true;
    
    ROS_INFO("[RiskMap25D] Initialized: %dx%d grid, res=%.2fm, fixed_height=%.1fm",
             grid_width_, grid_height_, resolution_, fixed_height_);
}

void RiskMap25D::initializeGrids()
{
    // 分配单层栅格
    size_t grid_size = static_cast<size_t>(grid_width_) * static_cast<size_t>(grid_height_);
    risk_grid_.resize(grid_size, 0.0);
}

void RiskMap25D::computeTimeWeights()
{
    // 指数衰减时间权重: ω[k] ∝ exp(-λ * k)
    time_weights_.resize(num_pred_steps_);
    double sum = 0.0;
    for (int k = 0; k < num_pred_steps_; ++k) {
        time_weights_[k] = std::exp(-lambda_time_ * k);
        sum += time_weights_[k];
    }
    
    // 归一化
    if (sum > 1e-8) {
        for (int k = 0; k < num_pred_steps_; ++k) {
            time_weights_[k] /= sum;
        }
    }
}

void RiskMap25D::setMapCenter(const Eigen::Vector3d& center)
{
    map_center_ = center;
    
    // 更新栅格原点（地图中心对齐）
    grid_origin_ = Eigen::Vector2d(center(0) - grid_width_ * resolution_ / 2.0,
                                     center(1) - grid_height_ * resolution_ / 2.0);
}

void RiskMap25D::updateStatic(const std::shared_ptr<mapManager::ESDFMap>& esdf_map)
{
    if (!esdf_map) {
        ROS_WARN_THROTTLE(5.0, "[RiskMap25D] updateStatic: nullptr ESDF map");
        return;
    }
    
    esdf_map_ = esdf_map;
    
    // 预计算静态风险到栅格（在固定高度）
    precomputeStaticRisk();
    
    static_updated_ = true;
}

void RiskMap25D::precomputeStaticRisk()
{
    if (!esdf_map_) return;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 清零
    std::fill(risk_grid_.begin(), risk_grid_.end(), 0.0);
    
    // 遍历栅格计算静态风险
    for (int iy = 0; iy < grid_height_; ++iy) {
        for (int ix = 0; ix < grid_width_; ++ix) {
            // 栅格→世界坐标（固定高度）
            Eigen::Vector3d world_pos;
            world_pos(0) = grid_origin_(0) + (ix + 0.5) * resolution_;
            world_pos(1) = grid_origin_(1) + (iy + 0.5) * resolution_;
            world_pos(2) = fixed_height_;
            
            // 计算静态风险
            double dist = esdf_map_->getDistance(world_pos);
            double r_s = 0.0;
            if (dist < d_s_) {
                double delta = d_s_ - dist;
                r_s = delta * delta;  // (d_s - d)^2
            }
            
            // 存储（乘以gamma_s）
            int idx = iy * grid_width_ + ix;
            risk_grid_[idx] = gamma_s_ * r_s;
        }
    }
}

void RiskMap25D::updateDynamic(const std::vector<ObstaclePrediction>& predictions)
{
    if (!is_initialized_) return;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 动态风险叠加到risk_grid_（不清零静态部分）
    // 先备份静态风险
    std::vector<double> static_backup = risk_grid_;
    
    // 遍历所有障碍物
    for (const auto& obs_pred : predictions) {
        // 检查障碍物高度范围是否覆盖fixed_height_
        if (obs_pred.z_max < fixed_height_ || obs_pred.z_min > fixed_height_) {
            continue;  // 障碍物不在这个高度层
        }
        
        // 遍历所有意图
        for (size_t intent_idx = 0; intent_idx < obs_pred.intent_trajs.size(); ++intent_idx) {
            const auto& intent_traj = obs_pred.intent_trajs[intent_idx];
            
            // 遍历预测时间步
            for (int k = 0; k < static_cast<int>(intent_traj.steps.size()) && k < num_pred_steps_; ++k) {
                const auto& pred = intent_traj.steps[k];
                
                // 椭圆参数
                double sigma_x = std::max(pred.sigma_x, sigma_min_);
                double sigma_y = std::max(pred.sigma_y, sigma_min_);
                Eigen::Vector2d mu = pred.mu;
                
                // 获取意图概率
                double intent_prob = (intent_idx < obs_pred.intent_probs.size()) ?
                                     obs_pred.intent_probs[intent_idx] : 0.0;
                
                // 计算影响范围（包围盒）
                double m_cut_sqrt = std::sqrt(m_cut_);
                double half_width_x = m_cut_sqrt * sigma_x;
                double half_width_y = m_cut_sqrt * sigma_y;
                
                // 栅格范围
                int ix_min = std::max(0, static_cast<int>(std::floor((mu(0) - half_width_x - grid_origin_(0)) / resolution_)));
                int ix_max = std::min(grid_width_ - 1, static_cast<int>(std::ceil((mu(0) + half_width_x - grid_origin_(0)) / resolution_)));
                int iy_min = std::max(0, static_cast<int>(std::floor((mu(1) - half_width_y - grid_origin_(1)) / resolution_)));
                int iy_max = std::min(grid_height_ - 1, static_cast<int>(std::ceil((mu(1) + half_width_y - grid_origin_(1)) / resolution_)));
                
                // 遍历栅格
                double inv_sigma_x_sq = 1.0 / (sigma_x * sigma_x);
                double inv_sigma_y_sq = 1.0 / (sigma_y * sigma_y);
                
                for (int iy = iy_min; iy <= iy_max; ++iy) {
                    for (int ix = ix_min; ix <= ix_max; ++ix) {
                        // 栅格中心世界坐标（2D）
                        Eigen::Vector2d q_2d;
                        q_2d(0) = grid_origin_(0) + (ix + 0.5) * resolution_;
                        q_2d(1) = grid_origin_(1) + (iy + 0.5) * resolution_;
                        
                        // 计算Mahalanobis距离平方
                        Eigen::Vector2d delta = q_2d - mu;
                        double m_sq = delta(0) * delta(0) * inv_sigma_x_sq +
                                      delta(1) * delta(1) * inv_sigma_y_sq;
                        
                        // 截断
                        if (m_sq > m_cut_) continue;
                        
                        // 计算椭圆核: exp(-m^beta)
                        double kernel = std::exp(-std::pow(m_sq, beta_ / 2.0));
                        
                        // 累加动态风险: gamma_d * P(I) * omega[k] * kernel
                        int idx = iy * grid_width_ + ix;
                        risk_grid_[idx] += gamma_d_ * intent_prob * time_weights_[k] * kernel;
                    }
                }
            }
        }
    }
    
    dynamic_updated_ = true;
}

double RiskMap25D::query(const Eigen::Vector3d& q) const
{
    if (!is_initialized_) return 0.0;
    
    // 2.5D查询：忽略z坐标，只查询(x,y)
    // 转换为栅格坐标
    double gx = (q(0) - grid_origin_(0)) / resolution_ - 0.5;
    double gy = (q(1) - grid_origin_(1)) / resolution_ - 0.5;
    
    // 边界检查
    if (gx < 0 || gx >= grid_width_ - 1 || gy < 0 || gy >= grid_height_ - 1) {
        return 0.0;
    }
    
    // 双线性插值
    return bilinearInterpolate(gx, gy);
}

double RiskMap25D::bilinearInterpolate(double gx, double gy) const
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 获取四个角点的栅格索引
    int ix0 = static_cast<int>(std::floor(gx));
    int iy0 = static_cast<int>(std::floor(gy));
    int ix1 = ix0 + 1;
    int iy1 = iy0 + 1;
    
    // 边界检查
    if (ix0 < 0 || ix1 >= grid_width_ || iy0 < 0 || iy1 >= grid_height_) {
        return 0.0;
    }
    
    // 插值权重
    double wx = gx - ix0;
    double wy = gy - iy0;
    
    // 获取四个角点的风险值
    double r00 = risk_grid_[iy0 * grid_width_ + ix0];
    double r10 = risk_grid_[iy0 * grid_width_ + ix1];
    double r01 = risk_grid_[iy1 * grid_width_ + ix0];
    double r11 = risk_grid_[iy1 * grid_width_ + ix1];
    
    // 双线性插值
    double r0 = r00 * (1.0 - wx) + r10 * wx;
    double r1 = r01 * (1.0 - wx) + r11 * wx;
    double risk = r0 * (1.0 - wy) + r1 * wy;
    
    return risk;
}

bool RiskMap25D::exportGridData(std::vector<double>& data) const
{
    if (!is_initialized_) return false;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    data = risk_grid_;
    return true;
}

void RiskMap25D::clear()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::fill(risk_grid_.begin(), risk_grid_.end(), 0.0);
    static_updated_ = false;
    dynamic_updated_ = false;
}

// ============ 调试/可视化 ============

void RiskMap25D::publishVisualization(const ros::Publisher& pub) const
{
    if (!pub) return;
    if (!is_initialized_) return;
    
    // TODO: 发布OccupancyGrid消息用于rviz可视化
    // 可以参考原来的RiskMap2D实现
}

} // namespace globalPlanner
