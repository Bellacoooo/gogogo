/**
 * @file risk_map_25d_visualizer.cpp
 * @brief RiskMap25D可视化节点
 * 
 * 功能：
 * 1. 订阅动态预测数据
 * 2. 订阅ESDF地图
 * 3. 更新RiskMap25D
 * 4. 发布指定层的风险栅格可视化
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "global_planner/risk_map_25d.h"
#include "map_manager/ESDFMap.h"

using namespace globalPlanner;

class RiskMap25DVisualizer {
public:
    RiskMap25DVisualizer(const ros::NodeHandle& nh)
        : nh_(nh), nh_private_("~")
    {
        // 读取参数
        // 单层版本，无需指定层索引
        nh_private_.param("publish_rate", publish_rate_, 10.0);              // 10Hz
        nh_private_.param("use_color_map", use_color_map_, true);           // 使用彩色地图
        
        // 创建RiskMap25D
        risk_map_ = std::make_shared<RiskMap25D>(nh_private_);
        
        // 创建发布者
        risk_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/risk_map_25d/grid", 1);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/risk_map_25d/markers", 1);
        
        // 创建订阅者（需要根据您的系统调整话题名称）
        // 这里假设dynamic_predictor发布预测数据
        // 实际使用时需要创建转换器将现有数据格式转换为DynamicPredictions
        
        // 创建定时器
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                  &RiskMap25DVisualizer::timerCallback, this);
        
        ROS_INFO("[RiskMap25DVisualizer] Initialized, visualizing single-layer 2.5D risk map");
    }
    
    /**
     * @brief 手动更新动态预测（用于测试）
     */
    void updateDynamicTest() {
        // 创建测试数据：一个障碍物在原点附近移动
        DynamicPredictions predictions;
        predictions.timestamp = ros::Time::now();
        
        ObstaclePrediction obs;
        obs.id = 0;
        obs.z_min = 0.5;
        obs.z_max = 1.5;
        obs.intent_probs = {0.6, 0.2, 0.15, 0.05};  // FORWARD最高概率
        
        // 创建4个意图的轨迹
        for (int intent = 0; intent < 4; ++intent) {
            DynamicIntentTrajectory traj;
            
            // 生成10个预测步
            for (int k = 0; k < 10; ++k) {
                DynamicPredictionStep step;
                
                // 根据意图生成不同的轨迹
                double t = k * 0.1;  // 时间
                switch(intent) {
                    case 0:  // FORWARD
                        step.mu = Eigen::Vector2d(t, 0.0);
                        break;
                    case 1:  // LEFT
                        step.mu = Eigen::Vector2d(t * 0.7, t * 0.7);
                        break;
                    case 2:  // RIGHT
                        step.mu = Eigen::Vector2d(t * 0.7, -t * 0.7);
                        break;
                    case 3:  // STOP
                        step.mu = Eigen::Vector2d(0.0, 0.0);
                        break;
                }
                
                // 不确定性随时间增长
                step.sigma_x = 0.2 + 0.05 * k;
                step.sigma_y = 0.2 + 0.05 * k;
                step.height_min = 0.5;
                step.height_max = 1.5;
                
                traj.steps.push_back(step);
            }
            
            obs.intent_trajs.push_back(traj);
        }
        
        predictions.obstacles.push_back(obs);
        
        // 更新风险地图
        risk_map_->updateDynamic(predictions.obstacles);
    }
    
    /**
     * @brief 定时器回调：发布可视化
     */
    void timerCallback(const ros::TimerEvent&) {
        // 测试模式：生成测试数据
        static bool test_mode = true;
        if (test_mode) {
            updateDynamicTest();
        }
        
        // 发布风险栅格
        publishRiskGrid();
        
        // 发布标记
        publishMarkers();
    }
    
    /**
     * @brief 发布风险栅格（OccupancyGrid格式）
     */
    void publishRiskGrid() {
        if (risk_grid_pub_.getNumSubscribers() == 0) {
            return;
        }
        
        // 获取栅格参数
        double resolution;
        int width, height;
        Eigen::Vector2d origin;
        risk_map_->getGridInfo(resolution, width, height, origin);
        
        // 导出栅格数据
        std::vector<double> layer_data;
        if (!risk_map_->exportGridData(layer_data)) {
            ROS_WARN_THROTTLE(5.0, "[RiskMap25DVisualizer] Failed to export grid data");
            return;
        }
        
        // 创建OccupancyGrid消息
        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "map";
        
        grid_msg.info.resolution = resolution;
        grid_msg.info.width = width;
        grid_msg.info.height = height;
        grid_msg.info.origin.position.x = origin(0);
        grid_msg.info.origin.position.y = origin(1);
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        
        // 转换风险值到[0, 100]范围
        grid_msg.data.resize(layer_data.size());
        double max_risk = 0.0;
        for (double val : layer_data) {
            if (val > max_risk) max_risk = val;
        }
        
        if (max_risk > 1e-6) {
            for (size_t i = 0; i < layer_data.size(); ++i) {
                // 归一化到[0, 100]
                int8_t cell_value = static_cast<int8_t>(
                    std::min(100.0, (layer_data[i] / max_risk) * 100.0)
                );
                grid_msg.data[i] = cell_value;
            }
        } else {
            std::fill(grid_msg.data.begin(), grid_msg.data.end(), 0);
        }
        
        risk_grid_pub_.publish(grid_msg);
    }
    
    /**
     * @brief 发布标记（可视化层高度）
     */
    void publishMarkers() {
        if (markers_pub_.getNumSubscribers() == 0) {
            return;
        }
        
        visualization_msgs::MarkerArray marker_array;
        
        // 创建文本标记显示层信息
        visualization_msgs::Marker text_marker;
        text_marker.header.stamp = ros::Time::now();
        text_marker.header.frame_id = "map";
        text_marker.ns = "layer_info";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = 0.0;
        text_marker.pose.position.y = 0.0;
        text_marker.pose.position.z = 2.0;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.3;  // 文本大小
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        // 显示标题
        text_marker.text = "2.5D Risk Map (Single Layer)";
        
        marker_array.markers.push_back(text_marker);
        
        markers_pub_.publish(marker_array);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    std::shared_ptr<RiskMap25D> risk_map_;
    
    ros::Publisher risk_grid_pub_;
    ros::Publisher markers_pub_;
    
    ros::Timer timer_;
    
    double publish_rate_;
    bool use_color_map_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "risk_map_25d_visualizer");
    ros::NodeHandle nh;
    
    RiskMap25DVisualizer visualizer(nh);
    
    ROS_INFO("[RiskMap25DVisualizer] Node started");
    
    ros::spin();
    
    return 0;
}
