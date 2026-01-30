/**
 * @file test_risk_map_25d.cpp
 * @brief RiskMap25D单元测试
 */

#include <gtest/gtest.h>
#include "global_planner/risk_map_25d.h"
#include <ros/ros.h>

using namespace globalPlanner;

/**
 * @brief 测试fixture
 */
class RiskMap25DTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 创建NodeHandle
        ros::NodeHandle nh("~");
        
        // 设置测试参数
        nh.setParam("risk_map_25d/resolution", 0.1);
        nh.setParam("risk_map_25d/grid_width", 100);
        nh.setParam("risk_map_25d/grid_height", 100);
        nh.setParam("risk_map_25d/layer_min", 0.0);
        nh.setParam("risk_map_25d/layer_max", 2.0);
        nh.setParam("risk_map_25d/layer_thickness", 0.5);
        nh.setParam("risk_map_25d/d_s", 0.5);
        nh.setParam("risk_map_25d/gamma_s", 1.0);
        nh.setParam("risk_map_25d/gamma_d", 1.0);
        nh.setParam("risk_map_25d/beta", 1.0);
        nh.setParam("risk_map_25d/num_pred_steps", 10);
        nh.setParam("risk_map_25d/lambda_time", 0.1);
        nh.setParam("risk_map_25d/sigma_min", 0.1);
        nh.setParam("risk_map_25d/m_cut", 9.0);
        nh.setParam("risk_map_25d/risk_epsilon", 1e-4);
        
        // 创建RiskMap25D对象
        risk_map = std::make_shared<RiskMap25D>(nh);
        
        // 设置地图中心
        risk_map->setMapCenter(Eigen::Vector3d(0.0, 0.0, 1.0));
    }
    
    void TearDown() override {
        risk_map.reset();
    }
    
    std::shared_ptr<RiskMap25D> risk_map;
};

/**
 * @brief 测试1：初始化
 */
TEST_F(RiskMap25DTest, Initialization) {
    EXPECT_TRUE(risk_map->isValid());
    
    // 检查栅格参数
    double resolution;
    int width, height;
    Eigen::Vector2d origin;
    risk_map->getGridInfo(resolution, width, height, origin);
    
    EXPECT_DOUBLE_EQ(resolution, 0.1);
    EXPECT_EQ(width, 100);
    EXPECT_EQ(height, 100);
}

/**
 * @brief 测试2：动态核函数
 * 
 * 验证：
 * - 在均值处kernel最大（接近1）
 * - 随距离增大而衰减
 */
TEST_F(RiskMap25DTest, DynamicKernel) {
    // 创建简单的动态障碍物
    DynamicPredictions predictions;
    predictions.timestamp = ros::Time::now();
    
    DynamicObstacle obs;
    obs.obstacle_id = 0;
    obs.intent_probs = {1.0, 0.0, 0.0, 0.0};  // 100% FORWARD
    
    DynamicIntentTrajectory traj;
    DynamicPredictionStep step;
    step.mu = Eigen::Vector2d(0.0, 0.0);  // 在原点
    step.sigma_x = 0.5;
    step.sigma_y = 0.5;
    step.height_min = 0.5;
    step.height_max = 1.5;
    traj.steps.push_back(step);
    
    obs.intent_trajs = {traj, traj, traj, traj};  // 4个意图（只有第一个有概率）
    predictions.obstacles.push_back(obs);
    
    // 更新动态风险
    risk_map->updateDynamic(predictions);
    
    // 查询均值处的风险（应该最高）
    double risk_at_mu = risk_map->query(Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_GT(risk_at_mu, 0.0);
    
    // 查询1σ处的风险（应该衰减）
    double risk_at_1sigma = risk_map->query(Eigen::Vector3d(0.5, 0.0, 1.0));
    EXPECT_LT(risk_at_1sigma, risk_at_mu);
    EXPECT_GT(risk_at_1sigma, 0.0);
    
    // 查询3σ处的风险（应该接近0）
    double risk_at_3sigma = risk_map->query(Eigen::Vector3d(1.5, 0.0, 1.0));
    EXPECT_LT(risk_at_3sigma, risk_at_1sigma);
}

/**
 * @brief 测试3：多意图混合
 * 
 * 验证：
 * - 多个意图的风险加权求和
 */
TEST_F(RiskMap25DTest, MultiIntentMixture) {
    DynamicPredictions predictions;
    predictions.timestamp = ros::Time::now();
    
    DynamicObstacle obs;
    obs.obstacle_id = 0;
    // 两个意图各50%概率
    obs.intent_probs = {0.5, 0.5, 0.0, 0.0};
    
    // 第一个意图：在(1, 0)
    DynamicIntentTrajectory traj1;
    DynamicPredictionStep step1;
    step1.mu = Eigen::Vector2d(1.0, 0.0);
    step1.sigma_x = 0.5;
    step1.sigma_y = 0.5;
    step1.height_min = 0.5;
    step1.height_max = 1.5;
    traj1.steps.push_back(step1);
    
    // 第二个意图：在(-1, 0)
    DynamicIntentTrajectory traj2;
    DynamicPredictionStep step2;
    step2.mu = Eigen::Vector2d(-1.0, 0.0);
    step2.sigma_x = 0.5;
    step2.sigma_y = 0.5;
    step2.height_min = 0.5;
    step2.height_max = 1.5;
    traj2.steps.push_back(step2);
    
    DynamicIntentTrajectory empty_traj;
    obs.intent_trajs = {traj1, traj2, empty_traj, empty_traj};
    predictions.obstacles.push_back(obs);
    
    risk_map->updateDynamic(predictions);
    
    // 查询两个均值处的风险（应该大致相等）
    double risk1 = risk_map->query(Eigen::Vector3d(1.0, 0.0, 1.0));
    double risk2 = risk_map->query(Eigen::Vector3d(-1.0, 0.0, 1.0));
    
    EXPECT_GT(risk1, 0.0);
    EXPECT_GT(risk2, 0.0);
    EXPECT_NEAR(risk1, risk2, 0.1);  // 应该接近（因为概率相同）
}

/**
 * @brief 测试4：高度层选择
 * 
 * 验证：
 * - 查询不同高度时选择正确的层
 * - 障碍物高度范围外的风险应该小
 */
TEST_F(RiskMap25DTest, LayerSelection) {
    DynamicPredictions predictions;
    predictions.timestamp = ros::Time::now();
    
    DynamicObstacle obs;
    obs.obstacle_id = 0;
    obs.intent_probs = {1.0, 0.0, 0.0, 0.0};
    
    DynamicIntentTrajectory traj;
    DynamicPredictionStep step;
    step.mu = Eigen::Vector2d(0.0, 0.0);
    step.sigma_x = 0.5;
    step.sigma_y = 0.5;
    // 障碍物高度范围：[0.8, 1.2]m
    step.height_min = 0.8;
    step.height_max = 1.2;
    traj.steps.push_back(step);
    
    obs.intent_trajs = {traj, traj, traj, traj};
    predictions.obstacles.push_back(obs);
    
    risk_map->updateDynamic(predictions);
    
    // 查询障碍物高度范围内的风险（应该高）
    double risk_in_range = risk_map->query(Eigen::Vector3d(0.0, 0.0, 1.0));
    EXPECT_GT(risk_in_range, 0.0);
    
    // 查询障碍物高度范围外的风险（应该较低或为0）
    double risk_out_range = risk_map->query(Eigen::Vector3d(0.0, 0.0, 2.5));
    EXPECT_LE(risk_out_range, risk_in_range);
}

/**
 * @brief 测试5：时间衰减
 * 
 * 验证：
 * - 近期预测步的权重高于远期
 */
TEST_F(RiskMap25DTest, TimeDecay) {
    DynamicPredictions predictions;
    predictions.timestamp = ros::Time::now();
    
    DynamicObstacle obs;
    obs.obstacle_id = 0;
    obs.intent_probs = {1.0, 0.0, 0.0, 0.0};
    
    DynamicIntentTrajectory traj;
    
    // 第0步：在(0, 0)
    DynamicPredictionStep step0;
    step0.mu = Eigen::Vector2d(0.0, 0.0);
    step0.sigma_x = 0.5;
    step0.sigma_y = 0.5;
    step0.height_min = 0.5;
    step0.height_max = 1.5;
    traj.steps.push_back(step0);
    
    // 第5步：在(5, 0)（远期）
    DynamicPredictionStep step5;
    step5.mu = Eigen::Vector2d(5.0, 0.0);
    step5.sigma_x = 0.5;
    step5.sigma_y = 0.5;
    step5.height_min = 0.5;
    step5.height_max = 1.5;
    for (int i = 1; i < 5; ++i) {
        traj.steps.push_back(step5);  // 填充中间步
    }
    traj.steps.push_back(step5);
    
    obs.intent_trajs = {traj, traj, traj, traj};
    predictions.obstacles.push_back(obs);
    
    risk_map->updateDynamic(predictions);
    
    // 近期位置的风险应该高于远期
    double risk_near = risk_map->query(Eigen::Vector3d(0.0, 0.0, 1.0));
    double risk_far = risk_map->query(Eigen::Vector3d(5.0, 0.0, 1.0));
    
    EXPECT_GT(risk_near, 0.0);
    EXPECT_GT(risk_far, 0.0);
    EXPECT_GT(risk_near, risk_far);  // 近期风险更高
}

/**
 * @brief 测试6：导出层数据
 */
TEST_F(RiskMap25DTest, ExportLayerData) {
    // 创建简单的动态障碍物
    DynamicPredictions predictions;
    predictions.timestamp = ros::Time::now();
    
    DynamicObstacle obs;
    obs.obstacle_id = 0;
    obs.intent_probs = {1.0, 0.0, 0.0, 0.0};
    
    DynamicIntentTrajectory traj;
    DynamicPredictionStep step;
    step.mu = Eigen::Vector2d(0.0, 0.0);
    step.sigma_x = 0.5;
    step.sigma_y = 0.5;
    step.height_min = 0.5;
    step.height_max = 1.5;
    traj.steps.push_back(step);
    
    obs.intent_trajs = {traj, traj, traj, traj};
    predictions.obstacles.push_back(obs);
    
    risk_map->updateDynamic(predictions);
    
    // 导出第2层（z=1.0m）的数据
    std::vector<double> layer_data;
    bool success = risk_map->exportLayerData(2, layer_data);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(layer_data.size(), 100 * 100);  // 100x100栅格
    
    // 检查数据中至少有一些非零值
    int non_zero_count = 0;
    for (double val : layer_data) {
        if (val > 0.0) non_zero_count++;
    }
    EXPECT_GT(non_zero_count, 0);
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_risk_map_25d");
    ros::NodeHandle nh;
    
    return RUN_ALL_TESTS();
}
