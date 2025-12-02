/*
    FILE: dynamicPredictor.h
    ---------------------------------
    header file of dynamic obstacle predictor
*/

#ifndef DYNAMIC_PREDICTOR_H
#define DYNAMIC_PREDICTOR_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <onboard_detector/dynamicDetector.h>
#include <onboard_detector/fakeDetector.h>
#include <map_manager/dynamicMap.h>
#include <dynamic_predictor/utils.h>
// 在dynamicPredictor.h开头的#include区域新增一行（如果没有）
#include <std_msgs/Float32.h>
// 先添加头文件依赖
#include <chrono>



namespace dynamicPredictor{
    class predictor{
    private:
        std::string ns_;
        std::string hint_;

        // ROS
        ros::NodeHandle nh_;
        ros::Timer predTimer_;
        ros::Timer visTimer_;
        ros::Publisher historyTrajPub_;
        ros::Publisher predTrajPub_;
        ros::Publisher intentVisPub_;
        ros::Publisher varPointsPub_;
        ros::Publisher predBBoxPub_;

        ros::Publisher sValuePub_; // 新增：用于发布 s 值的 Publisher

        // Param
        Eigen::Vector3d robotSize_ = Eigen::Vector3d(0.0,0.0,0.0);
        int numPred_;
        double dt_;
        double minTurningTime_, maxTurningTime_;
        double zScore_;
        // bool useFakeDetector_ = false;
        int numIntent_ = 4;
        double frontAngle_;
        double stopVel_;
        double paramf_,paraml_,paramr_,params_; // Probablity
        double pscale_;
        bool useFakeDetector_ = false;

        bool mapReady_ = false;
        bool detectorReady_ = false;
        bool detectorGTReady_ = false;
        
        std::shared_ptr<onboardDetector::dynamicDetector> detector_; 
        std::shared_ptr<onboardDetector::fakeDetector> detectorGT_;
        std::shared_ptr<mapManager::dynamicMap> map_;       
        
        std::vector<std::vector<Eigen::Vector3d>> posHist_;
        std::vector<std::vector<Eigen::Vector3d>> velHist_;
        std::vector<std::vector<Eigen::Vector3d>> sizeHist_;
        std::vector<std::vector<Eigen::Vector3d>> accHist_;
        std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> allPredPoints_;
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> posPred_;
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> sizePred_;
        std::vector<Eigen::VectorXd> intentProb_;


        // 为了自适应新增的的
            // 新增：自适应方案所需变量
        std::vector<Eigen::Vector3d> accHistory_;  // 存储历史加速度，用于计算加加速度
        std::vector<Eigen::Vector3d> caPredHistory_; // 存储CA模型预测位置
        double gamma1_, gamma2_;  // M_t的权重系数
        double lambda1_, lambda2_; // D_t的敏感度参数
        double M_thresh_;         // 运动诊断阈值
        double s_max_;            // 最大权重（原pscale_的替代）
        size_t historyWindow_ = 1;   // 历史数据窗口大小（至少1）

        // 新增：核心指标计算函数
        double computeJerkNorm(const Eigen::Vector3d& currAcc);  // 计算加加速度幅值
        double computeNIS(const Eigen::Vector3d& currPos, const Eigen::Vector3d& caPredPos, const Eigen::Matrix3d& residualCov);  // 计算归一化残差平方
        double computeIntentEntropy(const Eigen::VectorXd& intentProb);  // 计算意图熵H_t
        double computeAdaptiveS(const double Ht, const double Mt);  // 计算自适应权重s_adaptive

        std::ofstream logFile_;  // 日志文件流

        // 新增：计时相关变量
        std::chrono::duration<double, std::milli> mainIntentTime_;    // 主意图生成耗时(ms)
        std::chrono::duration<double, std::milli> nonMainIntentTime_; // 非主意图总耗时(ms)
        std::chrono::duration<double, std::milli> genPointsTotalTime_;// genPoints总耗时(ms)
         // 新增：获取主意图索引
        std::vector<int> getMainIntents(const std::vector<Eigen::VectorXd>& intentProb);   

        
        


    public:
        predictor(const ros::NodeHandle& nh);

        void initParam();
        void registerPub();
        void registerCallback();  

        void setDetector(const std::shared_ptr<onboardDetector::dynamicDetector>& detector);
        void setDetector(const std::shared_ptr<onboardDetector::fakeDetector>& detector);
        void setMap(const std::shared_ptr<mapManager::dynamicMap>& map);

        // main function for prediction
        void predict();
        void intentProb(std::vector<Eigen::VectorXd> &intentProbTemp);
        // 添加了自适应的参数
        Eigen::MatrixXd genTransitionMatrix(const double &prevAngle, const double &currAngle, const Eigen::Vector3d &currVel, 
                                   const Eigen::Vector3d &currPos, const Eigen::Vector3d &currAcc);        Eigen::VectorXd genTransitionVector(const double &theta, const double &r, const Eigen::VectorXd &scale);
        //    新增 mainIntents 主意图参数
        void predTraj(std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> &allPredPointsTemp, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &posPredTemp, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &sizePredTemp   , const std::vector<int>& mainIntents);
        void genPoints(const int &intentType, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize);
        void genTraj(const std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predPos, std::vector<Eigen::Vector3d> &predSize);
        void modelForward(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize);
        void modelTurning(const int &intentType, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize);
        void modelStop(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize);
        void positionCorrection(std::vector<Eigen::Vector3d> &mean, const std::vector<std::vector<Eigen::Vector3d>> &predPoints);
        // callback 
        void visCB(const ros::TimerEvent&);  
        void predCB(const ros::TimerEvent&); 

        // visualization
        void publishVarPoints();
        void publishHistoryTraj();
        void publishPredTraj();
        void publishIntentVis();
        void publishPredBBox();
        

        // user function
        void getPrediction(std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predPos, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predSize, std::vector<Eigen::VectorXd> &intentProb);
        void getPrediction(std::vector<dynamicPredictor::obstacle> &predOb);

        void calculateAndPrintErrors(); // 新增声明

     

        ~predictor(); // 析构函数声明

    };
}

#endif