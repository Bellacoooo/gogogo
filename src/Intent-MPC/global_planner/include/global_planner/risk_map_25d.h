/**
 * @file risk_map_25d.h
 * @brief 2.5D融合风险地图：静态几何风险 + 多意图动态障碍物风险（单层版本）
 * 
 * 核心特性：
 * 1. 2.5D结构：单个固定高度层，维护一个2D风险栅格
 * 2. 静态风险：基于3D ESDF的距离场，使用安全裕度的二次惩罚
 * 3. 动态风险：多意图概率加权 + 时间衰减 + 高阶椭圆核
 * 4. 高效查询：忽略z坐标，直接查询2D栅格
 * 
 * @author Intent-MPC Team
 * @date 2026-01-30
 */

#ifndef RISK_MAP_25D_H
#define RISK_MAP_25D_H

#include <vector>
#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

// 前向声明
namespace mapManager {
    class occMap;
}

namespace globalPlanner {

/**
 * @brief 单个动态障碍物的单个意图的单个预测步的数据
 */
struct DynamicPredictionStep {
    Eigen::Vector2d mu;        ///< 预测均值位置 (x, y)
    double sigma_x;            ///< X方向标准差
    double sigma_y;            ///< Y方向标准差
    double probability;        ///< 该意图的概率
    double height_min;         ///< 障碍物高度范围最小值
    double height_max;         ///< 障碍物高度范围最大值
};

/**
 * @brief 单个动态障碍物的单个意图的完整预测轨迹
 */
struct DynamicIntentTrajectory {
    std::vector<DynamicPredictionStep> steps;  ///< N个预测步
};

/**
 * @brief 单个动态障碍物的所有意图预测
 */
struct ObstaclePrediction {
    int id;                                            ///< 障碍物ID
    double z_min;                                      ///< 高度范围最小值
    double z_max;                                      ///< 高度范围最大值
    std::vector<DynamicIntentTrajectory> intent_trajs; ///< 4个意图的预测轨迹
    std::vector<double> intent_probs;                  ///< 意图后验概率 P(I)，共4个：FORWARD, LEFT, RIGHT, STOP
};

/**
 * @brief 动态障碍物预测数据集合
 */
struct DynamicPredictions {
    std::vector<ObstaclePrediction> obstacles;
    ros::Time timestamp;
};

/**
 * @brief 2.5D融合风险地图（单层版本）
 * 
 * 实现原理：
 * - 高度固定：在fixed_height_高度维护一个2D风险栅格
 * - 静态风险：R_s(q) = max(0, d_s - d(q))^2，d(q)从ESDF查询
 * - 动态风险：R_d(q) = Σ_i Σ_I P_i(I) * Σ_k ω_k * exp(-(m_k)^β)
 *   其中 m_k = (dx/σx)² + (dy/σy)²（Mahalanobis距离平方）
 * - 融合：R(q) = γ_s * R_s(q) + γ_d * R_d(q)
 */
class RiskMap25D {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄（用于读取参数）
     */
    explicit RiskMap25D(const ros::NodeHandle& nh);

    /**
     * @brief 析构函数
     */
    ~RiskMap25D() = default;

    /**
     * @brief 设置地图指针（用于查询静态距离）
     * @param map 占位栅格地图指针
     */
    void setOccupancyMap(const std::shared_ptr<mapManager::occMap>& map);
    
    /**
     * @brief 更新静态风险（从内部occupancy map计算）
     * @note 内部方法，不加锁（由调用者保证线程安全）
     */
    void updateStaticRisk();
    
    /**
     * @brief 独立更新静态风险（带锁，可安全调用）
     */
    void updateStaticRiskOnly();

    /**
     * @brief 更新动态风险（基于意图预测）
     * @param predictions 动态障碍物预测数据
     */
    void updateDynamic(const std::vector<ObstaclePrediction>& predictions);

    /**
     * @brief 从OccupancyGrid消息更新动态风险（简化接口，用于兼容旧系统）
     * @param msg 风险地图消息（来自dynamic_predictor）
     */
    void updateFromDynamicMsg(const nav_msgs::OccupancyGrid& msg);

    /**
     * @brief 查询给定3D点的融合风险值（忽略z坐标）
     * @param q 查询点 (x, y, z)，z坐标被忽略
     * @return 风险值 [0, ∞)，0表示无风险
     */
    double query(const Eigen::Vector3d& q) const;

    /**
     * @brief 设置地图中心（通常是机器人当前位置）
     * @param center 地图中心位置
     */
    void setMapCenter(const Eigen::Vector3d& center);

    /**
     * @brief 检查地图是否已初始化
     * @return true如果地图已就绪
     */
    bool isValid() const { return is_initialized_; }

    /**
     * @brief 获取栅格信息（用于可视化）
     * @param resolution 输出分辨率
     * @param width 输出宽度
     * @param height 输出高度
     * @param origin 输出原点
     */
    void getGridInfo(double& resolution, int& width, int& height,
                     Eigen::Vector2d& origin) const {
        resolution = resolution_;
        width = grid_width_;
        height = grid_height_;
        origin = grid_origin_;
    }

    /**
     * @brief 清空风险地图
     */
    void clear();

    /**
     * @brief 导出2D风险栅格数据（用于可视化）
     * @param data 输出的栅格数据（行优先，大小=width*height）
     * @return true如果成功
     */
    bool exportGridData(std::vector<double>& data) const;

    /**
     * @brief 发布可视化消息（OccupancyGrid）
     * @param pub ROS发布器
     */
    void publishVisualization(const ros::Publisher& pub) const;

private:
    // ========== 配置参数 ==========
    
    /// 栅格参数
    double resolution_;           ///< 栅格分辨率 (m)
    int grid_width_;              ///< 栅格宽度（格子数）
    int grid_height_;             ///< 栅格高度（格子数）
    Eigen::Vector2d grid_origin_; ///< 栅格原点 (x, y)，世界坐标
    
    /// 高度参数（单层2.5D）
    double fixed_height_;         ///< 固定高度 (m)，默认1.0m（忽略z坐标查询）
    
    /// 静态风险参数
    double d_s_;                  ///< 安全距离 (m)
    double gamma_s_;              ///< 静态风险权重
    
    /// 动态风险参数
    double gamma_d_;              ///< 动态风险权重
    double beta_;                 ///< 椭圆核指数（m^beta）
    int num_pred_steps_;          ///< 预测步数 N
    std::vector<double> time_weights_;  ///< 时间权重 ω[k]，和为1
    double lambda_time_;          ///< 时间衰减系数（生成时间权重用）
    
    /// 数值稳定性参数
    double sigma_min_;            ///< 最小标准差（防止除零）
    double m_cut_;                ///< Mahalanobis距离截断阈值（超过此值风险忽略不计）
    double risk_epsilon_;         ///< 最小风险阈值（小于此值不光栅化）
    
    // ========== 数据存储 ==========
    
    /// 单层风险栅格：risk_grid_[y * width + x]
    std::vector<double> risk_grid_;
    
    /// 占位栅格地图指针（用于查询静态距离）
    std::shared_ptr<mapManager::occMap> occ_map_;
    
    /// 状态标志
    bool is_initialized_;
    bool static_updated_;
    bool dynamic_updated_;
    
    /// 地图中心（用于动态更新栅格原点）
    Eigen::Vector3d map_center_;
    
    /// 线程安全
    mutable std::mutex map_mutex_;
    
    // ========== 内部辅助函数 ==========
    
    /**
     * @brief 初始化栅格
     */
    void initializeGrids();
    
    /**
     * @brief 计算时间权重（指数衰减）
     */
    void computeTimeWeights();
    
    /**
     * @brief 预计算静态风险到栅格
     */
    void precomputeStaticRisk();
    
    /**
     * @brief 双线性插值查询2D风险（单层）
     * @param gx, gy 栅格坐标（连续值）
     * @return 风险值
     */
    double bilinearInterpolate(double gx, double gy) const;
};

} // namespace globalPlanner

#endif // RISK_MAP_25D_H
