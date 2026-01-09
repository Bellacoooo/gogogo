/*
    FILE: dynamicPredictor.cpp
    ---------------------------------
    function implementation of dynamic osbtacle predictor
*/
#include <dynamic_predictor/dynamicPredictor.h>
#include <ctime>  // 需包含时间戳头文件
#include <iomanip>  // 用于格式化输出
#include <sstream>  // 用于字符串流
#include <fstream>  // 用于文件操作
#include <cstdlib>  // 用于 getenv
#include <sys/stat.h>  // 用于检查目录是否存在
#include <sys/types.h>  // 用于 mkdir
#include <errno.h>  // 用于 errno
#include <vector>  // 用于 vector
#include <unistd.h>  // 用于 access

namespace dynamicPredictor{

    // 内部工具函数：把一个 2D 高斯"盖章"到风险缓冲区上，同时更新高度地图
    static void stampGaussianToBuffer(
        std::vector<double>& risk_buffer,
        std::vector<double>& height_map,  // 新增：高度地图，存储每个栅格的最大障碍物高度
        const nav_msgs::MapMetaData& info,
        const Eigen::Vector2d& mu,
        const Eigen::Matrix2d& Sigma,
        double combined_weight,
        double obstacle_height = 0.0  // 新增：障碍物高度（z 尺寸）
    ){
        const double res = info.resolution;
        const unsigned int width  = info.width;
        const unsigned int height = info.height;

        if (width == 0 || height == 0 || res <= 0.0 || combined_weight <= 0.0) {
            return;
        }

        if (risk_buffer.size() != static_cast<std::size_t>(width * height)) {
            risk_buffer.assign(width * height, 0.0);
        }
        // 初始化高度地图（如果未初始化）
        if (height_map.size() != static_cast<std::size_t>(width * height)) {
            height_map.assign(width * height, 0.0);
        }

        // 使用对角线近似标准差
        double var_x = Sigma(0, 0);
        double var_y = Sigma(1, 1);
        if (var_x <= 1e-8) var_x = 1e-8;
        if (var_y <= 1e-8) var_y = 1e-8;
        double std_x = std::sqrt(var_x);
        double std_y = std::sqrt(var_y);

        const double k = 3.0; // 3σ 范围
        double min_x_world = mu(0) - k * std_x;
        double max_x_world = mu(0) + k * std_x;
        double min_y_world = mu(1) - k * std_y;
        double max_y_world = mu(1) + k * std_y;

        // 地图原点（世界坐标）
        double origin_x = info.origin.position.x;
        double origin_y = info.origin.position.y;

        auto worldToIndex = [&](double wx, double wy, int& ix, int& iy){
            ix = static_cast<int>(std::floor((wx - origin_x) / res));
            iy = static_cast<int>(std::floor((wy - origin_y) / res));
        };

        int min_ix, min_iy, max_ix, max_iy;
        worldToIndex(min_x_world, min_y_world, min_ix, min_iy);
        worldToIndex(max_x_world, max_y_world, max_ix, max_iy);

        if (min_ix < 0) min_ix = 0;
        if (min_iy < 0) min_iy = 0;
        if (max_ix >= static_cast<int>(width))  max_ix = static_cast<int>(width)  - 1;
        if (max_iy >= static_cast<int>(height)) max_iy = static_cast<int>(height) - 1;

        if (min_ix > max_ix || min_iy > max_iy) {
            return;
        }

        // 逆协方差矩阵
        Eigen::Matrix2d Sigma_safe = Sigma;
        Sigma_safe(0, 0) = std::max(Sigma_safe(0, 0), 1e-8);
        Sigma_safe(1, 1) = std::max(Sigma_safe(1, 1), 1e-8);
        Eigen::Matrix2d Sigma_inv = Sigma_safe.inverse();

        for (int iy = min_iy; iy <= max_iy; ++iy) {
            for (int ix = min_ix; ix <= max_ix; ++ix) {
                double wx = origin_x + (static_cast<double>(ix) + 0.5) * res;
                double wy = origin_y + (static_cast<double>(iy) + 0.5) * res;

                Eigen::Vector2d n(wx, wy);
                Eigen::Vector2d diff = n - mu;

                double mah_dist_sq = diff.transpose() * Sigma_inv * diff;
                double risk = std::exp(-0.5 * mah_dist_sq);
                double weighted_risk = combined_weight * risk;

                if (weighted_risk <= 0.0) continue;

                std::size_t idx = static_cast<std::size_t>(iy) * width + static_cast<std::size_t>(ix);
                risk_buffer[idx] += weighted_risk;
                // 更新高度地图：取最大值（多个障碍物重叠时，取最高的）
                if (obstacle_height > height_map[idx]) {
                    height_map[idx] = obstacle_height;
                }
            }
        }
    }
    predictor::predictor(const ros::NodeHandle& nh) : nh_(nh){
        this->ns_ = "dynamic_predictor";
        this->hint_ = "[predictor]";
        this->initParam();
        this->initRiskMapParam();
        this->registerPub();
        this->registerCallback();
        
        // 初始化CSV日志文件（带时间戳，保存到桌面）- 参考 traj_vis6.py 的逻辑
        std::time_t now = std::time(nullptr);
        std::tm* timeinfo = std::localtime(&now);
        std::stringstream ss;
        ss << std::put_time(timeinfo, "%Y%m%d_%H%M%S");
        std::string timestamp = ss.str();
        
        // 获取桌面路径（参考 Python 的 get_desktop_path 逻辑）
        std::string desktopPath = getDesktopPath();
        ROS_INFO_STREAM(this->hint_ << ": 检测到桌面路径: " << desktopPath);
        
        // 检查写入权限（参考 Python 的 os.access 逻辑）
        if (access(desktopPath.c_str(), W_OK) != 0) {
            ROS_WARN_STREAM(this->hint_ << ": 没有权限写入目录 " << desktopPath << "，将使用 /tmp");
            desktopPath = "/tmp";
        }
        
        // 构建完整的文件路径（参考 Python 的 os.path.abspath 逻辑）
        std::string filename = "intent_eval_" + timestamp + ".csv";
        std::string csvFilePath = desktopPath + "/" + filename;
        
        // 强制打开文件（使用绝对路径，确保文件被创建）
        logFile_.open(csvFilePath.c_str(), std::ios::out | std::ios::trunc);
        if (logFile_.is_open()) {
            // 立即写入CSV表头（确保文件被创建，即使没有数据）
            logFile_ << "时间戳,障碍物ID,障碍物X,障碍物Y,ADE(米),FDE(米),D_t,s_adaptive,最佳意图,有效步数,P_forward,P_left,P_right,P_stop\n";
            logFile_.flush();  // 确保数据写入缓冲区
            
            // 验证文件是否真的被创建
            struct stat fileInfo;
            if (stat(csvFilePath.c_str(), &fileInfo) == 0) {
                ROS_INFO_STREAM(this->hint_ << ": ✓ CSV log file created successfully at: " << csvFilePath);
                ROS_INFO_STREAM(this->hint_ << ":   File size: " << fileInfo.st_size << " bytes");
                // 额外输出到标准输出，确保用户能看到（参考 Python 的 print 逻辑）
                std::cout << "\n========================================" << std::endl;
                std::cout << "CSV LOG FILE CREATED: " << csvFilePath << std::endl;
                std::cout << "========================================\n" << std::endl;
            } else {
                ROS_WARN_STREAM(this->hint_ << ": File opened but stat() failed: " << csvFilePath);
            }
        } else {
            // 如果打开失败，尝试使用 /tmp 作为备选（参考 Python 的 fallback 逻辑）
            std::string fallbackPath = "/tmp/" + filename;
            ROS_WARN_STREAM(this->hint_ << ": Failed to open file at Desktop, trying fallback: " << fallbackPath);
            logFile_.open(fallbackPath.c_str(), std::ios::out | std::ios::trunc);
            if (logFile_.is_open()) {
                logFile_ << "时间戳,障碍物ID,障碍物X,障碍物Y,ADE(米),FDE(米),D_t,s_adaptive,最佳意图,有效步数,P_forward,P_left,P_right,P_stop\n";
                logFile_.flush();  // 确保数据写入缓冲区
                ROS_WARN_STREAM(this->hint_ << ": Using fallback location: " << fallbackPath);
                std::cout << "\n========================================" << std::endl;
                std::cout << "CSV LOG FILE CREATED (FALLBACK): " << fallbackPath << std::endl;
                std::cout << "========================================\n" << std::endl;
                csvFilePath = fallbackPath;  // 更新路径
            } else {
                ROS_ERROR_STREAM(this->hint_ << ": CRITICAL: Failed to create CSV log file even in /tmp!");
                std::cerr << "ERROR: Failed to create CSV log file!" << std::endl;
            }
        }
    }

    void predictor::initParam(){
        // prediction size
        if (not this->nh_.getParam(this->ns_ + "/prediction_size", this->numPred_)){
            this->numPred_ = 5;
            std::cout << this->hint_ << ": No prediction size parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The prediction size is set to: " << this->numPred_ << std::endl;
        }  

        // prediction time step
        if (not this->nh_.getParam(this->ns_ + "/prediction_time_step", this->dt_)){
            this->dt_ = 0.1;
            std::cout << this->hint_ << ": No prediction time step parameter found. Use default: 0.1." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The prediction time step is set to: " << this->dt_ << std::endl;
        }  

        // prediction confidence level
        if (not this->nh_.getParam(this->ns_ + "/prediction_z_score", this->zScore_)){
            this->zScore_ = 1.645;
            std::cout << this->hint_ << ": No prediction z score parameter found. Use default: 1.645." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The prediction z score is set to: " << this->zScore_ << std::endl;
        } 

        // minimum turning time
        if (not this->nh_.getParam(this->ns_ + "/min_turning_time", this->minTurningTime_)){
            this->minTurningTime_ = 2.0;
            std::cout << this->hint_ << ": No minimum turning time parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The minimum turning time is set to: " << this->minTurningTime_ << std::endl;
        }  

        // maximum turning time
        if (not this->nh_.getParam(this->ns_ + "/max_turning_time", this->maxTurningTime_)){
            this->maxTurningTime_ = 3.0;
            std::cout << this->hint_ << ": No maximum turning time parameter found. Use default: 3.0." << std::endl;
        }
        else{
            if (this->maxTurningTime_ < this->minTurningTime_){
                this->maxTurningTime_ = this->minTurningTime_+1.0;
            }
            std::cout << this->hint_ << ": The maximum turning time is set to: " << this->maxTurningTime_ << std::endl;
        }

        // max front prob param
        double maxFrontProb;
        if (not this->nh_.getParam(this->ns_ + "/max_front_prob", maxFrontProb)){
            maxFrontProb = 0.5;
            this->paramr_ = (1-maxFrontProb) / (3*maxFrontProb-1);
            this->paraml_ = (1-maxFrontProb) / (3*maxFrontProb-1);
            std::cout << this->hint_ << ": No max front prob param. Use default: 0.5." << std::endl;
        }
        else{
            this->paramr_ = (1-maxFrontProb) / (3*maxFrontProb-1);
            this->paraml_ = (1-maxFrontProb) / (3*maxFrontProb-1);
            std::cout << this->hint_ << ": Max front prob param is set to: " << maxFrontProb << std::endl;
        } 

        // front angle param
        if (not this->nh_.getParam(this->ns_ + "/front_angle", this->frontAngle_)){
            this->frontAngle_ = 1/6*M_PI;
            this->paramf_ = sqrt(pow(this->frontAngle_,2)/(-2*log(this->paraml_*(1+sin(this->frontAngle_))-this->paraml_)));
            std::cout << this->hint_ << ": No front angle param. Use default: 30 degree." << std::endl;
        }
        else{
            this->frontAngle_ = this->frontAngle_*M_PI/180;
            this->paramf_ = sqrt(pow(this->frontAngle_,2)/(-2*log(this->paraml_*(1+sin(this->frontAngle_))-this->paraml_)));
            std::cout << this->hint_ << ": Front angle param is set to: " << this->frontAngle_ << std::endl;
        } 

        // stop velocity param
        if (not this->nh_.getParam(this->ns_ + "/stop_velocity_thereshold", this->stopVel_)){
            this->stopVel_ = 0.2;
            this->params_ = atanh(0.5)/this->stopVel_;
            std::cout << this->hint_ << ": No stop velocity thereshold param. Use default: 0.2." << std::endl;
        }
        else{
            this->params_ = atanh(0.5)/this->stopVel_;
            std::cout << this->hint_ << ": Stop velocity thereshold param is set to: " << this->stopVel_ << std::endl;
        } 

        std::cout << this->hint_ << ": Front param is set to: " << this->paramf_ << std::endl;
        std::cout << this->hint_ << ": Left param is set to: " << this->paraml_ << std::endl;
        std::cout << this->hint_ << ": Right param is set to: " << this->paramr_ << std::endl;
        std::cout << this->hint_ << ": Stop param is set to: " << this->params_ << std::endl;

        // prob scale param
        if (not this->nh_.getParam(this->ns_ + "/prob_scale_param", this->pscale_)){
            this->pscale_ = 5;
            std::cout << this->hint_ << ": No prob scale param. Use default: 1.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Prob scale param is set to: " << this->pscale_ << std::endl;
        } 

        // ==================== 自适应意图惯性方案参数 ====================
        // 多维特征融合权重 (加加速度、角加速度、预测误差)
        if (not this->nh_.getParam(this->ns_ + "/w1", this->w1_)){
            this->w1_ = 0.4;  // 加加速度权重
            std::cout << this->hint_ << ": No w1 parameter found. Use default: 0.4." << std::endl;
        } else {
            std::cout << this->hint_ << ": The w1 (jerk weight) is set to: " << this->w1_ << std::endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/w2", this->w2_)){
            this->w2_ = 0.3;  // 角加速度权重
            std::cout << this->hint_ << ": No w2 parameter found. Use default: 0.3." << std::endl;
        } else {
            std::cout << this->hint_ << ": The w2 (angular accel weight) is set to: " << this->w2_ << std::endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/w3", this->w3_)){
            this->w3_ = 0.3;  // 预测误差权重
            std::cout << this->hint_ << ": No w3 parameter found. Use default: 0.3." << std::endl;
        } else {
            std::cout << this->hint_ << ": The w3 (prediction error weight) is set to: " << this->w3_ << std::endl;
        }

        // 归一化上限参数
        if (not this->nh_.getParam(this->ns_ + "/j_max", this->j_max_)){
            this->j_max_ = 10.0;  // 加加速度上限 (m/s^3)，行人典型值
            std::cout << this->hint_ << ": No j_max parameter found. Use default: 10.0 m/s^3." << std::endl;
        } else {
            std::cout << this->hint_ << ": The j_max (jerk limit) is set to: " << this->j_max_ << std::endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/alpha_max", this->alpha_max_)){
            this->alpha_max_ = M_PI;  // 角加速度上限 (rad/s^2)
            std::cout << this->hint_ << ": No alpha_max parameter found. Use default: π rad/s^2." << std::endl;
        } else {
            std::cout << this->hint_ << ": The alpha_max (angular accel limit) is set to: " << this->alpha_max_ << std::endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/e_max", this->e_max_)){
            this->e_max_ = 0.3;  // 预测误差上限 (m)
            std::cout << this->hint_ << ": No e_max parameter found. Use default: 0.3 m." << std::endl;
        } else {
            std::cout << this->hint_ << ": The e_max (prediction error limit) is set to: " << this->e_max_ << std::endl;
        }

        // Sigmoid映射参数
        if (not this->nh_.getParam(this->ns_ + "/s_min", this->s_min_)){
            this->s_min_ = 1.2;  // 最小自适应权重
            std::cout << this->hint_ << ": No s_min parameter found. Use default: 1.2." << std::endl;
        } else {
            std::cout << this->hint_ << ": The s_min is set to: " << this->s_min_ << std::endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/s_max", this->s_max_)){
            this->s_max_ = 5.0;  // 最大自适应权重
            std::cout << this->hint_ << ": No s_max parameter found. Use default: 5.0." << std::endl;
        } else {
            std::cout << this->hint_ << ": The s_max is set to: " << this->s_max_ << std::endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/sigmoid_k", this->sigmoid_k_)){
            this->sigmoid_k_ = 10.0;  // Sigmoid陡峭度因子
            std::cout << this->hint_ << ": No sigmoid_k parameter found. Use default: 10.0." << std::endl;
        } else {
            std::cout << this->hint_ << ": The sigmoid_k is set to: " << this->sigmoid_k_ << std::endl;
        }

        if (not this->nh_.getParam(this->ns_ + "/sigmoid_mu", this->sigmoid_mu_)){
            this->sigmoid_mu_ = 0.5;  // Sigmoid中心偏移量
            std::cout << this->hint_ << ": No sigmoid_mu parameter found. Use default: 0.5." << std::endl;
        } else {
            std::cout << this->hint_ << ": The sigmoid_mu is set to: " << this->sigmoid_mu_ << std::endl;
        }

        // 新增：自适应方案参数 - history_window（历史窗口大小）
        int temp_history_window;  // 用int临时接收参数
        if (not this->nh_.getParam(this->ns_ + "/history_window", temp_history_window)){
            this->historyWindow_ = 1;  // 默认值
            std::cout << this->hint_ << ": No history_window parameter found. Use default: 1." << std::endl;
        }
        else{
            this->historyWindow_ = static_cast<size_t>(temp_history_window);  // 转换为size_t
            std::cout << this->hint_ << ": The history_window is set to: " << this->historyWindow_ << std::endl;
        }

        // 历史数据现在由 intentProb 循环逐帧传递，不再需要全局历史容器初始化

        // 注释：日志文件已在构造函数中初始化（带时间戳的intent_eval_<timestamp>.csv）
        // 不要在这里重新打开，否则会覆盖构造函数中创建的文件
        // std::string log_filename = "/tmp/adaptive_mdp_log.csv"; // 可写路径
        // logFile_.open(log_filename, std::ios::out | std::ios::app);
        // if (logFile_.is_open()) {
        //     // 写入表头（仅在文件新建时写入，通过判断文件大小实现）
        //     logFile_.seekp(0, std::ios::end);
        //     if (logFile_.tellp() == 0) {
        //         logFile_ << "Time(s),S_Adaptive,Ht,Mt,NIS,Jerk,P_Forward,P_Left,P_Right,P_Stop\n";
        //     }
        //     std::cout << this->hint_ << ": Log file initialized at " << log_filename << std::endl;
        // } else {
        //     std::cerr << this->hint_ << "Error: Failed to open log file!" << std::endl;
        // }

    }

// 动态检测器
    void predictor::setDetector(const std::shared_ptr<onboardDetector::dynamicDetector>& detector){
        this->detector_ = detector;
        this->detectorReady_ = true;
    }

// 假检测器
    void predictor::setDetector(const std::shared_ptr<onboardDetector::fakeDetector>& detector){
        this->detectorGT_ = detector;
        this->useFakeDetector_ = true;
        this->detectorGTReady_ = true;
    }

    void predictor::setMap(const std::shared_ptr<mapManager::dynamicMap>& map){
        this->map_ = map;
        this->map_->getRobotSize(this->robotSize_);
        this->setDetector(this->map_->getDetector());
        this->mapReady_ = true;
    }
    
    void predictor::registerPub(){
        // history trajectory pub
        this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/history_trajectories", 10);
        this->predTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/predict_trajectories", 10);
        this->intentVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/intent_probability", 10);
        this->varPointsPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/var_points", 10);
        this->predBBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/pred_bbox", 10);
        this->predInfoPub_ = this->nh_.advertise<dynamic_predictor::PredictedObstacles>(this->ns_ + "/predicted_obstacles", 10);
        this->riskMapPub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>(this->ns_ + "/dynamic_risk_map", 1);
        this->riskHeightMapPub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>(this->ns_ + "/dynamic_risk_height_map", 1);
        this->sValuePub_ = this->nh_.advertise<std_msgs::Float32>(this->ns_ + "/current_s_value", 1);
        this->adaptiveMetricsPub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(this->ns_ + "/adaptive_metrics", 10);
    }


// 使用ros定时器功能，定期调用两个回调函数，预测和可视化
    void predictor::registerCallback(){
        this->predTimer_= this->nh_.createTimer(ros::Duration(0.033), &predictor::predCB, this);
        this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &predictor::visCB, this);
    }

    void predictor::visCB(const ros::TimerEvent&){
        this->publishHistoryTraj();
        this->publishPredTraj();
        this->publishIntentVis();
        this->publishVarPoints();
        this->publishPredBBox();
        this->publishAdaptiveMetrics(); // 发布自适应指标
        this->publishRiskMap();         // 发布当前的动态风险地图（目前为空白，占位框架）
    }

    void predictor::predCB(const ros::TimerEvent&){    
        this->predict();
    }

    // main function for prediction
    void predictor::predict(){ 
        // get history
        if (this->useFakeDetector_ and this->detectorGTReady_ and this->mapReady_){
            this->detectorGT_->getDynamicObstaclesHist(this->posHist_, this->velHist_, this->accHist_, this->sizeHist_, this->obsIds_, this->robotSize_);
        }
        else if (not this->useFakeDetector_ and this->detectorReady_ and this->mapReady_){
            this->detector_->getDynamicObstaclesHist(this->posHist_, this->velHist_, this->accHist_, this->sizeHist_, this->obsIds_, this->robotSize_);
        }
        if (this->posHist_.size()){
            // 原有的代码，我要加主意图的确定和耗时打印，就修改了这部分
            // if (this->posHist_[0].size()){
            //     // intent prediction
            //     std::vector<Eigen::VectorXd> intentProbTemp;
            //     this->intentProb(intentProbTemp);

            //     // trajectory prediction
            //     std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> allPredPointsTemp;
            //     std::vector<std::vector<std::vector<Eigen::Vector3d>>> posPredTemp;
            //     std::vector<std::vector<std::vector<Eigen::Vector3d>>> sizePredTemp;
            //     this->predTraj(allPredPointsTemp, posPredTemp, sizePredTemp);

            //     this->intentProb_ = intentProbTemp;
            //     this->posPred_ = posPredTemp;
            //     this->sizePred_ = sizePredTemp;
            //     this->allPredPoints_ = allPredPointsTemp;
            // }
                if (this->posHist_.size() && this->posHist_[0].size()) {
                    // 意图预测
                    std::vector<Eigen::VectorXd> intentProbTemp;
                    this->intentProb(intentProbTemp);

                    // 新增：确定主意图
                    std::vector<int> mainIntents = getMainIntents(intentProbTemp);

                    // 轨迹预测（修改调用方式，传入主意图信息）
                    std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> allPredPointsTemp;
                    std::vector<std::vector<std::vector<Eigen::Vector3d>>> posPredTemp;
                    std::vector<std::vector<std::vector<Eigen::Vector3d>>> sizePredTemp;
                    std::vector<std::vector<std::vector<Eigen::Vector3d>>> varPredTemp;

                    // 新增：重置计时变量
                    mainIntentTime_ = std::chrono::duration<double, std::milli>::zero();
                    nonMainIntentTime_ = std::chrono::duration<double, std::milli>::zero();
                    genPointsTotalTime_ = std::chrono::duration<double, std::milli>::zero();

                    // 调用predTraj时传入主意图，用于内部计时
                    this->predTraj(allPredPointsTemp, posPredTemp, sizePredTemp, varPredTemp, mainIntents);

                    // 新增：打印耗时统计
                    std::cout << "\n===== 轨迹生成耗时统计 =====" << std::endl;
                    std::cout << "genPoints总耗时: " << genPointsTotalTime_.count() << " ms" << std::endl;
                    std::cout << "主意图生成耗时: " << mainIntentTime_.count() << " ms" << std::endl;
                    std::cout << "非主意图总耗时: " << nonMainIntentTime_.count() << " ms" << std::endl;
                    std::cout << "===========================\n" << std::endl;


                    this->intentProb_ = intentProbTemp;
                    this->posPred_ = posPredTemp;
                    this->sizePred_ = sizePredTemp;
                    this->varPred_ = varPredTemp;
                    this->allPredPoints_ = allPredPointsTemp;
                    this->publishPredictedObstacles();
        
    }
        }
        else{
            this->intentProb_.clear();
            this->allPredPoints_.clear();
            this->posPred_.clear();
            this->sizePred_.clear();
            this->varPred_.clear();
            this->obsIds_.clear();
        }

        // 计算ADE、FDE
        calculateAndPrintErrors();



    }



// ==================== 自适应意图惯性机制：核心指标计算 ====================

    /**
     * @brief 计算归一化加加速度特征 f_jerk(t)
     * @param currAcc 当前时刻的加速度 (m/s^2)
     * @param prevAcc 前一时刻的加速度 (m/s^2)
     * @return 归一化的加加速度特征，范围 [0, 1]
     * 
     * 物理意义：加加速度反映了"加速度的变化率"，是运动突变的直接体现。
     * 稳定运动（如匀速直行）对应加加速度接近0；
     * 意图切换（如从"直行"变为"转向"或"停止"）导致加加速度显著增大。
     * 
     * 数学定义：
     *   jerk_t = (a_t - a_{t-1}) / Δt
     *   f_jerk(t) = min(||jerk_t|| / j_max, 1.0)
     */
    double predictor::computeJerkFeature(const Eigen::Vector3d& currAcc, const Eigen::Vector3d& prevAcc) {
        // 计算加加速度向量 (m/s^3)
        Eigen::Vector3d jerk = (currAcc - prevAcc) / dt_;
        double jerkNorm = jerk.norm();
        
        // 归一化到 [0, 1]
        double f_jerk = std::min(jerkNorm / j_max_, 1.0);
        return f_jerk;
    }

    /**
     * @brief 计算归一化角加速度特征 f_angular(t)
     * @param currAngle 当前时刻的朝向角 (rad)
     * @param prevAngle 前一时刻的朝向角 (rad)
     * @return 归一化的角加速度特征，范围 [0, 1]
     * 
     * 注意：此函数已废弃，角加速度计算现在直接在 genTransitionMatrix 中完成，
     * 以确保每个历史帧使用正确的前一帧数据。保留此函数仅为兼容性。
     */
    double predictor::computeAngularAccelFeature(double currAngle, double prevAngle) {
        // 此函数已不再使用，计算逻辑已移至 genTransitionMatrix
        return 0.0;
    }

    /**
     * @brief 计算归一化预测-观测误差特征 f_error(t)
     * @param currPos 当前时刻的实际观测位置 (m)
     * @param prevPos 前一时刻的位置 (m)
     * @param prevVel 前一时刻的速度 (m/s)
     * @return 归一化的预测误差特征，范围 [0, 1]
     * 
     * 物理意义：基于牛顿第一定律的推论——若障碍物无外力作用（意图未变），
     * 其运动应符合惯性假设（Constant Velocity Model）。
     * 当预测误差较小时，表明障碍物运动符合惯性假设，意图稳定；
     * 当预测误差显著增大时，说明存在显著的控制输入（加速、制动或转向），预示潜在的意图改变。
     * 
     * 数学定义：
     *   p̂_t = p_{t-1} + v_{t-1} * Δt  (惯性预测位置)
     *   e_t = ||p_t - p̂_t||  (预测-观测误差)
     *   f_error(t) = min(e_t / e_max, 1.0)
     */
    double predictor::computePredictionErrorFeature(const Eigen::Vector3d& currPos, 
                                                     const Eigen::Vector3d& prevPos, 
                                                     const Eigen::Vector3d& prevVel) {
        // 基于恒速模型（Constant Velocity）进行惯性预测
        Eigen::Vector3d predictedPos = prevPos + prevVel * dt_;
        
        // 计算预测位置与实际观测位置的欧氏距离 (m)
        double predictionError = (currPos - predictedPos).norm();
        
        // 归一化到 [0, 1]
        double f_error = std::min(predictionError / e_max_, 1.0);
        return f_error;
    }

    /**
     * @brief 通过Sigmoid函数计算自适应意图惯性参数 s_adaptive
     * @param Mt 综合运动变化指标，范围 [0, 1]
     * @return 自适应权重 s_adaptive，范围 [s_min, s_max]
     * 
     * 设计原则：
     * 1. 负相关性：s_adaptive 随 Mt 单调递减
     * 2. 平滑性：映射函数连续可微，避免参数跳变
     * 3. 值域约束：s_adaptive ∈ [s_min, s_max]
     * 
     * 数学定义：
     *   s_adaptive(Mt) = s_min + (s_max - s_min) * σ(-k(Mt - μ))
     *   其中 σ(x) = 1 / (1 + e^(-x)) 为标准Logistic函数
     *   
     * 特性分析：
     * - 当障碍物处于稳定运动状态（Mt → 0）时，s_adaptive → s_max，系统表现出强意图惯性
     * - 当障碍物发生剧烈机动（Mt → 1）时，s_adaptive → s_min，系统迅速降低对历史意图的信任
     * - 在过渡区域（Mt ≈ μ = 0.5），函数具有较大的斜率，保证了意图识别的敏捷性
     */
    double predictor::computeAdaptiveS(double Mt) {
        // 标准Sigmoid函数：σ(x) = 1 / (1 + e^(-x))
        // 使用负号实现负相关性：Mt增大 → s_adaptive减小
        double sigmoid_input = -sigmoid_k_ * (Mt - sigmoid_mu_);
        double sigmoid_output = 1.0 / (1.0 + std::exp(-sigmoid_input));
        
        // 线性映射到 [s_min, s_max]
        double s_adaptive = s_min_ + (s_max_ - s_min_) * sigmoid_output;
        
        // 安全截断（防止数值误差）
        return std::max(s_min_, std::min(s_adaptive, s_max_));
    }






    void predictor::intentProb(std::vector<Eigen::VectorXd> &intentProbTemp){
        int numOb = this->posHist_.size();  //当前跟踪的目标数量
        
        intentProbTemp.resize(numOb);   //intentProbTemp用来存储每个目标的意图概率（输出结果）
        
        // 初始化自适应指标存储（每个障碍物一个）
        currentDt_.resize(numOb, 0.0);
        currentSAdaptive_.resize(numOb, 1.0);
        currentKAdaptive_.resize(numOb, -1);
        
        for (int i=0; i<numOb; ++i){
            // init state prob P
            // 初始化状态概率（均匀分布，4种意图：前、左、右、停）
            Eigen::VectorXd P;
            P.resize(this->numIntent_);
            P.setConstant(1.0/this->numIntent_);
            int numHist = this->posHist_[i].size();
            
            // 遍历历史帧，从过去到现在迭代更新意图概率
            for (int j=2; j<numHist; ++j){
                // transition matrix 
                // 获取三个连续帧的数据（用于计算运动变化指标）
                // 索引说明：numHist-j 是"前前帧"，numHist-j-1 是"前一帧"，numHist-j-2 是"当前帧"
                
                // 前前帧数据（用于计算前一帧的角速度）
                Eigen::Vector3d prevPrevPos = this->posHist_[i][numHist-j];
                
                // 前一帧数据
                Eigen::Vector3d prevPos = this->posHist_[i][numHist-j-1];
                Eigen::Vector3d prevVel = this->velHist_[i][numHist-j-1];
                Eigen::Vector3d prevAcc = this->accHist_[i][numHist-j-1];
                
                // 当前帧数据
                Eigen::Vector3d currPos = this->posHist_[i][numHist-j-2];
                Eigen::Vector3d currVel = this->velHist_[i][numHist-j-2];
                Eigen::Vector3d currAcc = this->accHist_[i][numHist-j-2];
                
                // 计算角度（朝向）
                double prevPrevAngle = atan2(prevPos(1) - prevPrevPos(1), prevPos(0) - prevPrevPos(0));
                double prevAngle = atan2(currPos(1) - prevPos(1), currPos(0) - prevPos(0));
                // 修正：currAngle应该基于当前速度方向，而不是位置差
                double currAngle = atan2(currVel(1), currVel(0));
                
                // 判断是否有有效的前一帧数据（第一帧j=2时没有有效的前一帧）
                bool hasValidPrevFrame = (j > 2);
                
                // 调用 genTransitionMatrix，传入完整的历史帧数据
                Eigen::MatrixXd transMat = this->genTransitionMatrix(
                    prevAngle, currAngle, currVel, currPos, currAcc,
                    prevAcc, prevPos, prevVel, prevPrevAngle,
                    hasValidPrevFrame, i
                );

                Eigen::VectorXd newP = transMat * P;
                P = newP;
            }
            intentProbTemp[i] = P;
            
        }
    }

    void predictor::initRiskMapParam(){
        // risk map resolution
        if (!this->nh_.getParam(this->ns_ + "/risk_map_resolution", this->riskMapResolution_)){
            this->riskMapResolution_ = 0.2;
            std::cout << this->hint_ << ": No risk_map_resolution param. Use default: 0.2." << std::endl;
        } else {
            std::cout << this->hint_ << ": risk_map_resolution set to: " << this->riskMapResolution_ << std::endl;
        }

        // risk map width (cells)
        if (!this->nh_.getParam(this->ns_ + "/risk_map_width", this->riskMapWidth_)){
            this->riskMapWidth_ = 100;
            std::cout << this->hint_ << ": No risk_map_width param. Use default: 100." << std::endl;
        } else {
            std::cout << this->hint_ << ": risk_map_width set to: " << this->riskMapWidth_ << std::endl;
        }

        // risk map height (cells)
        if (!this->nh_.getParam(this->ns_ + "/risk_map_height", this->riskMapHeight_)){
            this->riskMapHeight_ = 100;
            std::cout << this->hint_ << ": No risk_map_height param. Use default: 100." << std::endl;
        } else {
            std::cout << this->hint_ << ": risk_map_height set to: " << this->riskMapHeight_ << std::endl;
        }

        // origin offset relative to robot (meters)
        // 若未显式设置，则自动让机器人位于风险图中心
        double default_offset_x = -0.5 * this->riskMapWidth_  * this->riskMapResolution_;
        double default_offset_y = -0.5 * this->riskMapHeight_ * this->riskMapResolution_;

        if (!this->nh_.getParam(this->ns_ + "/risk_origin_offset_x", this->riskOriginOffsetX_)){
            this->riskOriginOffsetX_ = default_offset_x;
            std::cout << this->hint_ << ": No risk_origin_offset_x param. Use default(centered): "
                      << this->riskOriginOffsetX_ << std::endl;
        }
        if (!this->nh_.getParam(this->ns_ + "/risk_origin_offset_y", this->riskOriginOffsetY_)){
            this->riskOriginOffsetY_ = default_offset_y;
            std::cout << this->hint_ << ": No risk_origin_offset_y param. Use default(centered): "
                      << this->riskOriginOffsetY_ << std::endl;
        }
    }

    //据角度和速度，计算"意图转移概率矩阵"
    // Eigen::MatrixXd predictor::genTransitionMatrix(const Eigen::Vector3d &prevPos, const Eigen::Vector3d &currPos, const Eigen::Vector3d &prevVel, const Eigen::Vector3d &currVel){
    Eigen::MatrixXd predictor::genTransitionMatrix(
        const double &prevAngle, 
        const double &currAngle, 
        const Eigen::Vector3d &currVel, 
        const Eigen::Vector3d &currPos, 
        const Eigen::Vector3d &currAcc,
        const Eigen::Vector3d &prevAcc,
        const Eigen::Vector3d &prevPos,
        const Eigen::Vector3d &prevVel,
        const double &prevPrevAngle,
        bool hasValidPrevFrame,
        int obsIdx
    ){
        // Initialize transMat
        Eigen::MatrixXd transMat;
        Eigen::VectorXd probVec;        
        probVec.resize(this->numIntent_);
        transMat.resize(this->numIntent_, this->numIntent_);

        //确保角度范围在 -π 到 π 之间（比如 +190° 会换成 -170°）。
        // // double theta = atan2(currPos(1)-prevPos(1), currPos(0)-prevPos(0)) -  atan2(currVel(1), currVel(0));
        double theta =  currAngle - prevAngle;
        
        if (theta > M_PI){
            theta = theta - 2*M_PI;
        }
        else if (theta <= -M_PI){
            theta = theta + 2*M_PI;
        }
        
        // 表示当前速度的模长（未使用，注释掉以避免警告）
        // double r = sqrt(pow(currVel(0), 2) + pow(currVel(1), 2));   

    // ==================== 自适应意图惯性机制：指标计算与权重映射 ====================
        
        // 3. 计算三个核心物理指标
        // 注意：只有当 hasValidPrevFrame=true 时才计算有效指标，否则使用默认值
        
        // 3.1 加加速度特征 f_jerk(t)
        double f_jerk = 0.0;
        if (hasValidPrevFrame) {
            f_jerk = computeJerkFeature(currAcc, prevAcc);
        }

        // 3.2 角加速度特征 f_angular(t)
        // 需要前前帧角度(prevPrevAngle)来计算前一帧角速度
        double f_angular = 0.0;
        if (hasValidPrevFrame) {
            // 计算前一帧的角速度
            double prevAngularVel = (prevAngle - prevPrevAngle) / dt_;
            // 角度归一化到 [-π, π]
            while (prevAngularVel > M_PI) prevAngularVel -= 2.0 * M_PI;
            while (prevAngularVel <= -M_PI) prevAngularVel += 2.0 * M_PI;
            
            // 计算当前帧的角速度
            double currAngularVel = (currAngle - prevAngle) / dt_;
            while (currAngularVel > M_PI) currAngularVel -= 2.0 * M_PI;
            while (currAngularVel <= -M_PI) currAngularVel += 2.0 * M_PI;
            
            // 计算角加速度
            double angularAccel = (currAngularVel - prevAngularVel) / dt_;
            f_angular = std::min(std::abs(angularAccel) / alpha_max_, 1.0);
        }

        // 3.3 预测-观测误差特征 f_error(t)
        double f_error = 0.0;
        if (hasValidPrevFrame) {
            f_error = computePredictionErrorFeature(currPos, prevPos, prevVel);
        }

        // 4. 多维特征融合：综合运动变化指标 M_t
        // M_t = w1 * f_jerk + w2 * f_angular + w3 * f_error
        double Mt = w1_ * f_jerk + w2_ * f_angular + w3_ * f_error;
        // 安全截断到 [0, 1]（防止权重设置不当导致越界）
        Mt = std::max(0.0, std::min(Mt, 1.0));

        // 5. Sigmoid映射：计算自适应意图惯性参数 s_adaptive
        double s_adaptive = computeAdaptiveS(Mt);
        
        // 调试输出：监控自适应参数变化
        // ROS_INFO_THROTTLE(2.0, "Obs %d: f_jerk=%.3f, f_angular=%.3f, f_error=%.3f => Mt=%.3f => s_adaptive=%.3f",
        //                   obsIdx, f_jerk, f_angular, f_error, Mt, s_adaptive);
        
        // 6. 保存当前障碍物的自适应指标（用于可视化和日志）
        if (obsIdx >= 0 && obsIdx < static_cast<int>(currentSAdaptive_.size())) {
            currentSAdaptive_[obsIdx] = s_adaptive;
            // 存储 Mt 用于分析（复用 currentDt_ 变量名，实际存储 Mt）
            if (obsIdx < static_cast<int>(currentDt_.size())) {
                currentDt_[obsIdx] = Mt;
            }
        }
        
        // 7. 生成转移矩阵的每一列
        // 对每种意图（前进、左、右、停）都生成一个"转移概率列"
        // 每列表示：从这个意图出发，到别的意图的概率
        // 使用自适应权重 s_adaptive 来调节意图惯性
        double r = sqrt(pow(currVel(0), 2) + pow(currVel(1), 2));  // 速度模长
        for (int i = 0; i < this->numIntent_; ++i) {
            Eigen::VectorXd scale = Eigen::VectorXd::Ones(this->numIntent_);
            // 关键修改：用自适应权重替代固定的 pscale_
            scale(i) = s_adaptive;
            // 调用原论文的转移概率生成函数
            Eigen::VectorXd probVec = this->genTransitionVector(theta, r, scale);
            transMat.block(0, i, this->numIntent_, 1) = probVec;
        }

        // 8. 发布当前的 s_adaptive 值（用于实时监控）
        std_msgs::Float32 s_msg;
        s_msg.data = s_adaptive;
        this->sValuePub_.publish(s_msg);

        return transMat;
    }

    Eigen::VectorXd predictor::genTransitionVector(const double &theta, const double &r, const Eigen::VectorXd &scale){
        Eigen::VectorXd probVec;        
        probVec.resize(this->numIntent_);
        double ps, pf, pr, pl;

        pf = scale(0)*(exp(-0.5*pow(theta/this->paramf_,2))+this->paraml_); //guassian distribution
        pl = scale(1)*(this->paraml_*(1+sin(theta)));
        pr = scale(2)*(this->paramr_*(1-sin(theta)));
        ps = ((1-tanh(this->params_/scale(3)*r)));
        double sum = pr+pl+pf;
        pr = (1-ps)*pr/sum;
        pl = (1-ps)*pl/sum;
        pf = (1-ps)*pf/sum;

        probVec(FORWARD) = pf;
        probVec(LEFT) = pl;
        probVec(RIGHT) = pr;
        probVec(STOP) = ps;

        return probVec;
    }


// 为每个障碍物的每种意图生成预测轨迹，包括预测点、位置和大小。关键逻辑genPoints（）、genTraj（）
    void predictor::predTraj(std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> &allPredPointsTemp,
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> &posPredTemp,
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> &sizePredTemp,
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> &varPredTemp,
        const std::vector<int>& mainIntents){     //这里输入的参数新加了一个主意图的参数

        // 清空并重置输出容器，确保不包含旧数据
        posPredTemp.clear();
        posPredTemp.resize(this->posHist_.size());
        sizePredTemp.clear();
        sizePredTemp.resize(this->sizeHist_.size());
        varPredTemp.clear();
        varPredTemp.resize(this->sizeHist_.size());
        allPredPointsTemp.clear();
        allPredPointsTemp.resize(this->posHist_.size());
        
        // predict each obstacle遍历每个障碍物
        for (int i=0; i < int(this->posHist_.size()); i++){
            posPredTemp[i].resize(this->numIntent_);
            sizePredTemp[i].resize(this->numIntent_);
            varPredTemp[i].resize(this->numIntent_);
            allPredPointsTemp[i].resize(this->numIntent_);








            // predict for each number of intent
            for (int j=FORWARD; j<=STOP; ++j){
                std::vector<std::vector<Eigen::Vector3d>> predPoints;   //存储当前意图的预测点
                std::vector<Eigen::Vector3d> predSize;                  //存储当前意图的预测大小
                // 根据障碍物当前的状态（位置、速度、加速度、大小）和意图，生成预测点和大小
                this->genPoints(j, this->posHist_[i][0], this->velHist_[i][0], this->accHist_[i][0], this->sizeHist_[i][0], predPoints, predSize);
                if (predPoints.size()){
                    std::vector<Eigen::Vector3d> predPos;
                    std::vector<Eigen::Vector3d> varPred;
                    this->genTraj(predPoints, predPos, predSize, varPred);
                    posPredTemp[i][j] = predPos;
                    sizePredTemp[i][j] = predSize;
                    varPredTemp[i][j] = varPred;
                    allPredPointsTemp[i][j] = predPoints;
                }
                // 如果未生成预测点，则使用历史状态生成默认预测轨迹
                else{
                    Eigen::Vector3d size = this->sizeHist_[i][0];
                    Eigen::Vector3d currVel = this->velHist_[i][0];
                    Eigen::Vector3d currPos = this->posHist_[i][0];
                    std::vector<Eigen::Vector3d> predPos;
                    std::vector<Eigen::Vector3d> varPred;
                    // 计算速度的模长
                    double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2)); 
                    for (int i=0;i<this->numPred_+1;i++){
                        predPos.push_back(currPos);
                        predSize.push_back(size);
                        varPred.push_back(Eigen::Vector3d::Zero());
                        size(0) += 2*min(vel,this->stopVel_)*this->dt_;
                        size(1) += 2*min(vel,this->stopVel_)*this->dt_;
                    }
                    posPredTemp[i][j] = predPos;
                    sizePredTemp[i][j] = predSize;
                    varPredTemp[i][j] = varPred;
                }
            }
        }
	}

    void predictor::genPoints(const int &intentType, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
        predPoints.clear();
        predSize.clear();
        double vel =  sqrt(pow(currVel(0),2)+pow(currVel(1),2));
        if (vel <= this->stopVel_){
           this->modelStop(currPos, currVel, currSize, predPoints, predSize);
        }
        else{
            if (intentType==FORWARD){
                this->modelForward(currPos, currVel, currAcc, currSize, predPoints, predSize);
            }
            else if (intentType==LEFT or intentType==RIGHT){
                this->modelTurning(intentType, currPos, currVel, currAcc, currSize, predPoints, predSize);
            }
            else if (intentType==STOP){
                this->modelStop(currPos, currVel,currSize, predPoints, predSize);
            }
        }
    }

    void predictor::modelForward(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
        predPoints.clear();
        predSize.clear();
        double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2));
        double angleInit = atan2(currVel(1), currVel(0)); // facing direction
        double minVel, maxVel;
        double minAngle, maxAngle;
        minVel = vel-vel;     //采样最小速度为0
        maxVel = vel+vel;     //采样最大速度为2倍当前速度
        minAngle = angleInit - this->frontAngle_;    //frontAngle_一个采样范围参数
        maxAngle = angleInit + this->frontAngle_;
        bool isValid = true;

        // const velocity
        for (double i=minAngle; i<maxAngle; i+=0.1){
            for (double j=minVel; j<maxVel; j+=0.1){
                std::vector<Eigen::Vector3d> predPointTemp;
                Eigen::VectorXd currState(4);
                currState<<currPos(0), currPos(1), j*cos(i), j*sin(i);
                predPointTemp.clear();
                predPointTemp.push_back(currPos);
                for (int k=0; k<this->numPred_;k++){
                    // TODO: check const acc model
                    Eigen::MatrixXd model;
                    model = MatrixXd::Identity(4,4);
                    model.block(0,2,2,2) = Eigen::MatrixXd::Identity(2,2)*this->dt_;
                    Eigen::VectorXd nextState = model*currState;
                    Eigen::Vector3d p;
                    p << nextState(0), nextState(1), currPos(2);
                    if (this->map_->isInflatedOccupied(p)){
                        isValid = false;
                        break;
                    }
                    else{
                        predPointTemp.push_back(p);
                    }
                    currState = nextState;
                }
                if (isValid){
                    predPoints.push_back(predPointTemp);
                }
                else{
                    isValid = true;
                    break;
                }
            }
        }

        for (int i=0;i<this->numPred_+1;i++){
            predSize.push_back(currSize);
        }
    }

    void predictor::modelTurning(const int & intentType, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
        predPoints.clear();
        predSize.clear();
        // double acc = sqrt(pow(currAcc(0),2)+pow(currAcc(1),2));
        double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2));
        double angleInit = atan2(currVel(1), currVel(0));
        double minVel, maxVel;
        double angle;
        minVel = vel-vel;
        maxVel = vel+vel;
        // double minAcc, maxAcc;
        // minAcc = acc-acc;
        // maxAcc = acc+acc;
        double endMin, endMax;

        if (intentType != LEFT and intentType != RIGHT){
            cout << this->hint_ << ": Please enter the correct intent!!!" << endl;
        }
        
        double minAngVel, maxAngVel;
        if(intentType == LEFT){
            endMin = this->frontAngle_+angleInit;
            endMax = (M_PI-this->frontAngle_)+angleInit;
            minAngVel = (M_PI/2)/this->maxTurningTime_;
            maxAngVel = (M_PI/2)/this->minTurningTime_;
        }
        else{
            endMin = -(M_PI-this->frontAngle_)+angleInit;
            endMax = -this->frontAngle_+angleInit;
            minAngVel = (-M_PI/2)/this->minTurningTime_;
            maxAngVel = (-M_PI/2)/this->maxTurningTime_;
        }
        bool isValid = true;

        for (double i = minVel; i<maxVel;i+=0.2){
            for (double j = minAngVel;j<maxAngVel;j+=0.2){
                for (double endAngle = endMin;endAngle<endMax;endAngle+=0.2){
                    std::vector<Eigen::Vector3d> predPointTemp;
                    Eigen::VectorXd currState(4);
                    angle = angleInit;
                    currState<<currPos(0), currPos(1), i*cos(angle), i*sin(angle);
                    predPointTemp.clear();
                    predPointTemp.push_back(currPos);
                    for (int k=0; k<this->numPred_;k++){
                        Eigen::MatrixXd model;
                        model = MatrixXd::Identity(4,4);
                        model.block(0,2,2,2) = Eigen::MatrixXd::Identity(2,2)*this->dt_;
                        Eigen::VectorXd nextState = model*currState;
                        Eigen::Vector3d p;
                        p << nextState(0), nextState(1), currPos(2);
                        if (this->map_->isInflatedOccupied(p)){
                            isValid = false;
                            break;
                        }
                        else{
                            predPointTemp.push_back(p);
                        }
                        currState = nextState;
                        angle += j*this->dt_;
                        if(intentType == LEFT){
                            angle  = min(angle, endAngle);
                        }
                        else if (intentType == RIGHT){
                            angle  = max(angle, endAngle);
                        }
                        double v = sqrt(pow(currState(2),2)+pow(currState(3),2));
                        currState(2) = v*cos(angle);
                        currState(3) = v*sin(angle);
                    }
                    if (isValid){
                        predPoints.push_back(predPointTemp);
                    }
                    else{
                        isValid = true;
                    }
                }
                    
                }
            }
        for (int i=0;i<this->numPred_+1;i++){
            predSize.push_back(currSize);
        }
    }
    
    void predictor::modelStop(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
        predPoints.clear();
        predSize.clear();
        std::vector<Eigen::Vector3d> predPointTemp;
        Eigen::Vector3d size = currSize;
        double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2)); 
        for (int i=0;i<this->numPred_+1;i++){
            predPointTemp.push_back(currPos);
            predSize.push_back(size);
            size(0) += 2*min(vel,this->stopVel_)*this->dt_;
            size(1) += 2*min(vel,this->stopVel_)*this->dt_;
        }
        predPoints.push_back(predPointTemp);
    }

// 得到均值和方差
    void predictor::genTraj(const std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predPos, std::vector<Eigen::Vector3d> &predSize, std::vector<Eigen::Vector3d> &varPred){
        predPos.clear();
        varPred.clear();
        for (int i=0;i<this->numPred_+1;i++){
            double meanx, meany;
            double variancex, variancey;
            double sumx = 0 , sumy = 0;
            double sumVarx = 0, sumVary = 0;
            int counter = 0;
            for (int j=0; j<int(predPoints.size());j++){
                if (i < int(predPoints[j].size())){
                    sumx += predPoints[j][i](0);
                    sumy += predPoints[j][i](1);
                    counter += 1;
                }
            }
            if (counter){
                meanx = sumx/counter;
                meany = sumy/counter;
                for (int j=0; j<int(predPoints.size()); j++){
                    sumVarx += pow(predPoints[j][i](0)-meanx,2);
                    sumVary += pow(predPoints[j][i](1)-meany,2);
                }
                variancex = sumVarx/counter;
                variancey = sumVary/counter;
                Eigen::Vector3d p;
                p<<meanx, meany, predPoints[0][0](2);
                predPos.push_back(p);
                Eigen::Vector3d var;
                var<<variancex, variancey, 0.0;
                varPred.push_back(var);
                predSize[i](0) += 2*sqrt(variancex)*this->zScore_; // confidence level under gaussian
                predSize[i](1) += 2*sqrt(variancey)*this->zScore_;  //置信区间的计算
            }
            else{
                break;
            }
        }
        this->positionCorrection(predPos, predPoints); // if mean trajectory has collision, find the closest. Otherwise, use the mean.
    }

// 检查平均轨迹是否安全，如果平均轨迹会碰撞，就从候选轨迹中选择一条最接近平均的安全轨迹作为替代。
    void predictor::positionCorrection(std::vector<Eigen::Vector3d> &mean, const std::vector<std::vector<Eigen::Vector3d>> &predPoints){
        bool isCollide = false;
        for (int i=0; i<int(mean.size()); i++){
            if (this->map_->isInflatedOccupied(mean[i])){
                isCollide = true;
                break;
            }
        }
        double sum = 0;
        double minSum = INFINITY;
        int minIdx = -1;
        if (isCollide){
            for (int i=0; i<int(predPoints.size()); i++){
                sum = 0;
                for (int j=0; j<int(mean.size()); j++){
                    sum += sqrt(pow(predPoints[i][j](0)-mean[j](0),2)+pow(predPoints[i][j](1)-mean[j](1),2));
                    if (sum > minSum){
                        break;
                    }
                }
                if (sum < minSum){
                    minSum = sum;
                    minIdx = i;
                }
            }
            mean = predPoints[minIdx];
        }
    }

    void predictor::publishVarPoints(){
        visualization_msgs::MarkerArray trajMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->allPredPoints_.size(); ++i){     //这个allPredPoints_是预测出来的所有点，是一个四维数组
            visualization_msgs::Marker traj;
            traj.header.frame_id = "map";            //参考坐标系
            traj.header.stamp = ros::Time::now();
            traj.ns = "predictor";
            traj.id = countMarker;                   //每个marker的唯一ID
            traj.type = visualization_msgs::Marker::POINTS;
            traj.scale.x = 0.03;
            traj.scale.y = 0.03;
            traj.scale.z = 0.03;
            traj.color.a = 1.0; // Don't forget to set the alpha!
            traj.color.r = 0.0;
            traj.color.g = 0.0;
            traj.color.b = 1.0;
            traj.lifetime = ros::Duration(0.1);    
            for (size_t j=0; j<this->allPredPoints_[i].size(); ++j){
                for (size_t k=0; k<this->allPredPoints_[i][j].size(); k++){
                    for (size_t l=0; l<this->allPredPoints_[i][j][k].size();l++){
                        geometry_msgs::Point p;
                        Eigen::Vector3d pos = this->allPredPoints_[i][j][k][l];   //四维索引 [i][j][k][l] 用于定位到具体的三维点
                        p.x = pos(0); p.y = pos(1); p.z = pos(2);
                        double meanx, meany;
                        double stdx, stdy;
                        meanx = this->posPred_[i][j][l](0);
                        meany = this->posPred_[i][j][l](1);
                        stdx = this->sizePred_[i][j][l](0)-this->sizeHist_[i][0](0);
                        stdy = this->sizePred_[i][j][l](1)-this->sizeHist_[i][0](1);
                        if (p.x <= meanx+stdx/2 and p.x >= meanx-stdx/2){
                            if (p.y <= meany+stdy/2 and p.y >= meany-stdy/2){
                                traj.points.push_back(p);
                            }
                        }
                    }
                    
                }
            }

            ++countMarker;
            trajMsg.markers.push_back(traj);
        }
        this->varPointsPub_.publish(trajMsg);
    }

    void predictor::publishHistoryTraj(){
        visualization_msgs::MarkerArray trajMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->posHist_.size(); ++i){
            visualization_msgs::Marker traj;
            traj.header.frame_id = "map";
            traj.header.stamp = ros::Time::now();
            traj.ns = "predictor";
            traj.id = countMarker;
            traj.type = visualization_msgs::Marker::LINE_STRIP;
            traj.scale.x = 0.03;
            traj.scale.y = 0.03;
            traj.scale.z = 0.03;
            traj.color.a = 1.0; // Don't forget to set the alpha!
            traj.color.r = 0.0;
            traj.color.g = 1.0;
            traj.color.b = 0.0;
            traj.lifetime = ros::Duration(0.1);
            for (size_t j=0; j<this->posHist_[i].size(); ++j){
                geometry_msgs::Point p;
                Eigen::Vector3d pos = this->posHist_[i][j];
                p.x = pos(0); p.y = pos(1); p.z = pos(2);
                traj.points.push_back(p);                 
            }

            ++countMarker;
            trajMsg.markers.push_back(traj);   
        }
        this->historyTrajPub_.publish(trajMsg);
    }

    void predictor::publishPredTraj(){
		if (this->posPred_.size() != 0){
            visualization_msgs::MarkerArray trajMsg;
			for (int i=0; i<int(this->posPred_.size()); ++i){
                for (int j=0; j<int(this->posPred_[i].size());j++){
                    visualization_msgs::Marker traj;
                    traj.header.frame_id = "map";
                    traj.header.stamp = ros::Time::now();
                    traj.ns = "predictor";
                    // 编码：obstacle_id * numIntent_ + intent_type，方便 Python 解析
                    traj.id = i * this->numIntent_ + j;
                    traj.type = visualization_msgs::Marker::LINE_STRIP;  //线段
                    traj.scale.x = 0.1;             //线段粗细和颜色
                    traj.scale.y = 0.1;
                    traj.scale.z = 0.1;
                    traj.color.a = 1.0;
                    traj.color.r = 1.0;
                    traj.color.g = 0.0;
                    traj.color.b = 0.0;
                    traj.lifetime = ros::Duration(0.1);    //表示标记在发布后会自动过期。
                    for (int k=0; k<int(this->posPred_[i][j].size()); ++k){
                        geometry_msgs::Point p;
                        Eigen::Vector3d pos = this->posPred_[i][j][k];
                        p.x = pos(0); p.y = pos(1); p.z = pos(2);
                        traj.points.push_back(p);                    //每个轨迹都统计点，但是不知道用来干嘛
                    }
                    trajMsg.markers.push_back(traj);
                }
			}
			this->predTrajPub_.publish(trajMsg);
		}
	}

    void predictor::publishIntentVis(){ 
        visualization_msgs::MarkerArray intentVisMsg;
        int countMarker = 0;
        for (int i=0; i<int(this->posHist_.size()); ++i){
            visualization_msgs::Marker intentMarker;
            intentMarker.header.frame_id = "map";
            intentMarker.header.stamp = ros::Time::now();
            intentMarker.ns = "predictor";
            intentMarker.id =  countMarker;
            intentMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            intentMarker.pose.position.x = this->posHist_[i][0](0);
            intentMarker.pose.position.y = this->posHist_[i][0](1);
            intentMarker.pose.position.z = this->posHist_[i][0](2) + this->sizeHist_[i][0](2)/2. + 0.3;
            intentMarker.scale.x = 0.35;
            intentMarker.scale.y = 0.35;
            intentMarker.scale.z = 0.35;
            intentMarker.color.a = 1.0;
            intentMarker.color.r = 1.0;
            intentMarker.color.g = 0.0;
            intentMarker.color.b = 0.0;
            intentMarker.lifetime = ros::Duration(0.1);
            // std::string intentText = "Front: " + std::to_string(this->intentProb_[i](0)) 
            //                         + "Left: " + std::to_string(this->intentProb_[i](1)) 
            //                         + "Right: " + std::to_string(this->intentProb_[i](2))
            //                         + "Stop: " + std::to_string(this->intentProb_[i](3));
            // intentMarker.text = intentText;
            std::vector<std::string> intentText(4);
            intentText[FORWARD] = "Front: "+ std::to_string(this->intentProb_[i](FORWARD));
            intentText[LEFT] = "Left: " + std::to_string(this->intentProb_[i](LEFT)) ;
            intentText[RIGHT] = "Right: " + std::to_string(this->intentProb_[i](RIGHT));
            intentText[STOP] = "Stop: " + std::to_string(this->intentProb_[i](STOP));
            int maxIdx = 0;
            double max = 0;
            for (int j=0; j<this->numIntent_;j++){
                if (this->intentProb_[i](j)>max){
                    maxIdx = j;
                    max = this->intentProb_[i](j);
                }
            }
            intentMarker.text = intentText[maxIdx];
            intentVisMsg.markers.push_back(intentMarker);
            ++countMarker;
        }
        this->intentVisPub_.publish(intentVisMsg);
    }

    void predictor::publishPredBBox(){
        if (this->posPred_.size() == 0) return;
        // publish top N intent future bounding box with the inflate size
        visualization_msgs::MarkerArray predBBoxMsg;
        visualization_msgs::Marker line;
        line.header.frame_id = "map";
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.ns = "box3D";  
        line.scale.x = 0.06;
        line.color.r = 0;
        line.color.g = 1;
        line.color.b = 0;
        line.color.a = 1.0;
        line.lifetime = ros::Duration(0.1);

        visualization_msgs::Marker range;
        range.header.frame_id = "map";
        range.header.stamp = ros::Time::now();
        range.ns = "pred_ob_size";
        range.id = 0;
        range.type = visualization_msgs::Marker::SPHERE;
        range.action = visualization_msgs::Marker::ADD;
        range.color.a = 0.4;
        range.color.r = 0.0;
        range.color.g = 0.0;
        range.color.b = 1.0;
        range.lifetime = ros::Duration(0.1);
        for (int i=0; i<int(this->intentProb_.size()); ++i){
            std::vector<std::pair<double, int>> intentProb;
            for (int j=0; j<this->numIntent_; ++j){
                intentProb.push_back({this->intentProb_[i](j), j});
            }
            std::sort(intentProb.begin(), intentProb.end(), 
            [](const std::pair<double, int> &left, const std::pair<double, int> &right) 
            {return left.first > right.first;});

            for (int n=0; n<1; ++n){ // top N intent
                int intentIdx = intentProb[n].second;
                std::vector<Eigen::Vector3d> predTraj = this->posPred_[i][intentIdx];
                std::vector<Eigen::Vector3d> predSize = this->sizePred_[i][intentIdx];
                for (int t=10; t<int(predTraj.size()); t+=10){
                    Eigen::Vector3d obPos = predTraj[t];
                    Eigen::Vector3d predObSize = predSize[t];
                    Eigen::Vector3d obSize = this->sizeHist_[i][0];

                    double x = obPos(0); 
                    double y = obPos(1); 
                    double z = (obPos(2)+obSize(2)/2)/2; 

                    double x_width = obSize(0);
                    double y_width = obSize(1);
                    double z_width = 2*obPos(2);

                    
                    vector<geometry_msgs::Point> verts;
                    geometry_msgs::Point p;
                    // vertice 0
                    p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
                    verts.push_back(p);

                    // vertice 1
                    p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
                    verts.push_back(p);

                    // vertice 2
                    p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
                    verts.push_back(p);

                    // vertice 3
                    p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
                    verts.push_back(p);

                    // vertice 4
                    p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
                    verts.push_back(p);

                    // vertice 5
                    p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
                    verts.push_back(p);

                    // vertice 6
                    p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
                    verts.push_back(p);

                    // vertice 7
                    p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
                    verts.push_back(p);
                    
                    int vert_idx[12][2] = {
                        {0,1},
                        {1,2},
                        {2,3},
                        {0,3},
                        {0,4},
                        {1,5},
                        {3,7},
                        {2,6},
                        {4,5},
                        {5,6},
                        {4,7},
                        {6,7}
                    };
                    
                    for (size_t i=0;i<12;i++){
                        line.points.push_back(verts[vert_idx[i][0]]);
                        line.points.push_back(verts[vert_idx[i][1]]);
                    }
                    
                    predBBoxMsg.markers.push_back(line);
                    
                    line.id++;


                    range.pose.position.x = obPos(0);
                    range.pose.position.y = obPos(1);
                    range.pose.position.z = obPos(2);
                    range.scale.x = predObSize(0);
                    range.scale.y = predObSize(1);
                    range.scale.z = 0.1;
                    range.id++;
                    predBBoxMsg.markers.push_back(range);
                }
            }
        }
        this->predBBoxPub_.publish(predBBoxMsg);
    }

    void predictor::publishRiskMap(){
        if (!this->mapReady_){
            return;
        }

        Eigen::Vector3d robotPos;
        this->map_->getPosition(robotPos);

        nav_msgs::OccupancyGrid riskMsg;
        riskMsg.header.frame_id = "map";
        riskMsg.header.stamp = ros::Time::now();

        riskMsg.info.resolution = this->riskMapResolution_;
        riskMsg.info.width = this->riskMapWidth_;
        riskMsg.info.height = this->riskMapHeight_;

        // origin: anchor to robot, apply offset so robot is at a fixed relative location in the grid
        riskMsg.info.origin.position.x = robotPos(0) + this->riskOriginOffsetX_;
        riskMsg.info.origin.position.y = robotPos(1) + this->riskOriginOffsetY_;
        riskMsg.info.origin.position.z = 0.0;

        riskMsg.info.origin.orientation.w = 1.0;
        riskMsg.info.origin.orientation.x = 0.0;
        riskMsg.info.origin.orientation.y = 0.0;
        riskMsg.info.origin.orientation.z = 0.0;

        // a) 浮点风险缓冲区（累加所有障碍物、多模态、全时间步）
        std::vector<double> float_risk_data(this->riskMapWidth_ * this->riskMapHeight_, 0.0);
        // 高度地图：存储每个栅格的最大障碍物高度
        std::vector<double> height_map(this->riskMapWidth_ * this->riskMapHeight_, 0.0);

        // 参数：时间衰减因子 gamma
        const double gamma = 0.98;

        // 诊断：检查数据源
        static int call_count = 0;
        call_count++;
        bool should_log = (call_count % 30 == 0);  // 每30次调用打印一次（约1秒）
        if (should_log) {
            ROS_WARN("[RISK-MAP] call_count=%d allPredPoints_.size()=%zu posPred_.size()=%zu posHist_.size()=%zu intentProb_.size()=%zu",
                     call_count, this->allPredPoints_.size(), this->posPred_.size(), this->posHist_.size(), this->intentProb_.size());
        }

        // 修改：使用 posPred_ 作为主要数据源（更稳定，不会被清空）
        // 即使 allPredPoints_ 被清空，posPred_ 仍然有数据
        int processed_obstacles = 0;
        int processed_intents = 0;
        int processed_timesteps = 0;
        int skipped_obstacles = 0;
        int skipped_intents = 0;
        int skipped_timesteps = 0;
        
        // 遍历 posPred_（更稳定的数据源）
        for (size_t i = 0; i < this->posPred_.size(); ++i) {
            // 检查必要的数据是否存在
            if (i >= this->intentProb_.size()) {
                skipped_obstacles++;
                continue;
            }
            if (i >= this->posHist_.size() || this->posHist_[i].empty()) {
                skipped_obstacles++;
                continue;
            }
            
            const auto& intents = this->intentProb_[i];
            if (intents.size() == 0) {
                skipped_obstacles++;
                continue;
            }
            
            // 遍历每个意图模态
            for (int j = 0; j < this->numIntent_ && j < static_cast<int>(intents.size()); ++j) {
                double omega = intents(j);  // 该模态概率
                if (omega <= 1e-4) {
                    skipped_intents++;
                    continue;  // 概率太小略过
                }
                
                if (i >= this->posPred_.size() || j >= static_cast<int>(this->posPred_[i].size())) {
                    skipped_intents++;
                    continue;
                }
                
                const auto& traj = this->posPred_[i][j];  // 该模态的轨迹
                if (traj.empty()) {
                    skipped_intents++;
                    continue;
                }

                // 遍历每个时间步（使用 posPred_ 的时间步数，与 publishVarPoints 中的 l 对应）
                for (size_t l = 0; l < traj.size(); ++l) {
                    processed_timesteps++;
                    
                    // 获取均值和标准差（与 publishVarPoints 完全一致）
                    // publishVarPoints 中使用：meanx = posPred_[i][j][l](0)
                    double meanx = traj[l](0);
                    double meany = traj[l](1);
                    double stdx = 0.0;
                    double stdy = 0.0;
                    
                    // 计算标准差（与 publishVarPoints 完全一致）
                    // publishVarPoints 中使用：stdx = sizePred_[i][j][l](0) - sizeHist_[i][0](0)
                    if (i < this->sizePred_.size() && j < static_cast<int>(this->sizePred_[i].size()) && 
                        l < this->sizePred_[i][j].size() && i < this->sizeHist_.size() && 
                        !this->sizeHist_[i].empty()) {
                        stdx = this->sizePred_[i][j][l](0) - this->sizeHist_[i][0](0);
                        stdy = this->sizePred_[i][j][l](1) - this->sizeHist_[i][0](1);
                } else {
                        // 如果没有尺寸信息，使用默认值
                        stdx = 0.2;
                        stdy = 0.2;
                    }
                    
                    // 确保标准差为正
                    stdx = std::max(stdx, 0.1);
                    stdy = std::max(stdy, 0.1);
                    
                    // 获取障碍物高度
                    double obstacle_height = 0.0;
                    if (i < this->sizeHist_.size() && !this->sizeHist_[i].empty()) {
                        obstacle_height = this->sizeHist_[i][0](2);
                    }
                        
                        // 时间衰减
                    double time_decay = std::pow(gamma, static_cast<double>(l));
                    double combined_weight = omega * time_decay;
                    if (combined_weight <= 0.0) {
                        skipped_timesteps++;
                        continue;
                    }
                    
                    // 构建协方差矩阵（使用标准差）
                    Eigen::Vector2d mu(meanx, meany);
                    Eigen::Matrix2d Sigma = Eigen::Matrix2d::Zero();
                    Sigma(0, 0) = stdx * stdx;
                    Sigma(1, 1) = stdy * stdy;
                    
                    // 将风险圈（椭圆）添加到风险地图
                        stampGaussianToBuffer(
                            float_risk_data,
                            height_map,
                            riskMsg.info,
                        mu,
                            Sigma,
                        combined_weight,
                        obstacle_height
                        );
                    }
                processed_intents++;
            }
            processed_obstacles++;
        }
        
        // 详细诊断日志
        if (should_log) {
            ROS_WARN("[RISK-MAP] processed: obs=%d (skipped=%d) intents=%d (skipped=%d) timesteps=%d (skipped=%d)",
                     processed_obstacles, skipped_obstacles, processed_intents, skipped_intents, 
                     processed_timesteps, skipped_timesteps);
        }

        // b) 将浮点风险映射到 [0,100] 的 int8 OccupancyGrid
        riskMsg.data.resize(this->riskMapWidth_ * this->riskMapHeight_);

        double max_risk = 0.0;
        double mean_risk = 0.0;
        int non_zero_count = 0;
        for (double v : float_risk_data) {
            if (v > max_risk) max_risk = v;
            if (v > 1e-8) {
                mean_risk += v;
                non_zero_count++;
            }
        }
        if (non_zero_count > 0) {
            mean_risk /= non_zero_count;
        }

        // 诊断日志（合并到 should_log）
        if (should_log) {
            ROS_WARN("[RISK-MAP] stats: max_risk=%.6f mean_risk=%.6f non_zero=%d/%zu (%.2f%%)",
                     max_risk, mean_risk, non_zero_count, float_risk_data.size(), 
                     100.0 * non_zero_count / float_risk_data.size());
        }

        if (max_risk <= 1e-8) {
            // 没有任何风险，直接全部设为 0
            std::fill(riskMsg.data.begin(), riskMsg.data.end(), 0);
            if (should_log) {
                ROS_WARN("[RISK-MAP] WARNING: max_risk=0, risk map is empty! Check if allPredPoints_ has data.");
            }
        } else {
            for (std::size_t i = 0; i < float_risk_data.size(); ++i) {
                double norm = float_risk_data[i] / max_risk;   // 0~1
                if (norm < 0.0) norm = 0.0;
                if (norm > 1.0) norm = 1.0;
                // 使用 ceil 向上取整，确保微小的风险值（如 0.5%）不会被截断为 0
                // 这样可以保留障碍物边缘的梯度信息
                double scaled = norm * 100.0;
                riskMsg.data[i] = static_cast<int8_t>(std::min(100, static_cast<int>(std::ceil(scaled))));
            }
            if (should_log) {
                // 统计发布的数据
                int non_zero_msg = 0;
                for (int8_t v : riskMsg.data) {
                    if (v > 0) non_zero_msg++;
                }
                ROS_WARN("[RISK-MAP] published: non_zero_msg=%d/%zu (%.2f%%)",
                         non_zero_msg, riskMsg.data.size(), 100.0 * non_zero_msg / riskMsg.data.size());
            }
        }

        // 数据保留机制：如果新数据为空或数据量太少，使用保留的旧数据
        ros::Time current_time = ros::Time::now();
        
        // 检查新数据是否有效
        int non_zero_new = 0;
        for (int8_t v : riskMsg.data) {
            if (v > 0) non_zero_new++;
        }
        bool new_data_valid = (non_zero_new > 100) && (max_risk > 1e-8);  // 至少100个非零栅格
        
        // 如果新数据无效，且存在保留的旧数据，且旧数据未过期，则使用旧数据
        if (!new_data_valid && !lastValidRiskMap_.data.empty()) {
            double time_since_last = (current_time - lastValidRiskMapTime_).toSec();
            if (time_since_last < riskMapRetentionTime_) {
                riskMsg = lastValidRiskMap_;
                riskMsg.header.stamp = current_time;  // 更新时间戳
                if (should_log) {
                    ROS_WARN("[RISK-MAP] Using retained data (new data invalid: non_zero=%d, max_risk=%.6f, time_since_last=%.2fs)",
                             non_zero_new, max_risk, time_since_last);
                }
            } else {
                if (should_log) {
                    ROS_WARN("[RISK-MAP] Retained data expired (time_since_last=%.2fs > %.2fs), publishing empty map",
                             time_since_last, riskMapRetentionTime_);
                }
            }
        }
        
        // 如果新数据有效，更新保留数据
        if (new_data_valid) {
            lastValidRiskMap_ = riskMsg;
            lastValidRiskMapTime_ = current_time;
            if (should_log) {
                ROS_WARN("[RISK-MAP] Updated retained data (non_zero=%d, max_risk=%.6f)", non_zero_new, max_risk);
            }
        }
        
        // 确保总是发布风险地图（即使全为0）
        this->riskMapPub_.publish(riskMsg);

        // 发布高度地图（使用 OccupancyGrid 格式，但存储的是高度值，单位：米）
        nav_msgs::OccupancyGrid heightMsg;
        heightMsg.header = riskMsg.header;
        heightMsg.info = riskMsg.info;
        heightMsg.data.resize(this->riskMapWidth_ * this->riskMapHeight_);
        
        // 将高度值（米）转换为 int8（0-100 表示 0-10 米，分辨率 0.1 米）
        const double height_scale = 10.0;  // 最大高度 10 米
        for (std::size_t i = 0; i < height_map.size(); ++i) {
            double h = height_map[i];
            if (h < 0.0) h = 0.0;
            if (h > height_scale) h = height_scale;
            // 将高度映射到 0-100：h / height_scale * 100
            heightMsg.data[i] = static_cast<int8_t>(std::round((h / height_scale) * 100.0));
        }
        
        this->riskHeightMapPub_.publish(heightMsg);
    }

    void predictor::publishPredictedObstacles(){
        if (this->posPred_.empty() || this->intentProb_.empty()){
            return;
        }
        dynamic_predictor::PredictedObstacles msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        for (size_t i = 0; i < this->posPred_.size(); ++i){
            dynamic_predictor::PredictedObstacle ob;
            ob.id = (i < this->obsIds_.size()) ? this->obsIds_[i] : static_cast<int>(i);

            if (i < this->posHist_.size() && !this->posHist_[i].empty()){
                ob.current_position.x = this->posHist_[i][0](0);
                ob.current_position.y = this->posHist_[i][0](1);
                ob.current_position.z = this->posHist_[i][0](2);
            }
            if (i < this->velHist_.size() && !this->velHist_[i].empty()){
                ob.current_velocity.x = this->velHist_[i][0](0);
                ob.current_velocity.y = this->velHist_[i][0](1);
                ob.current_velocity.z = this->velHist_[i][0](2);
            }

            for (int m = 0; m < this->numIntent_ && m < static_cast<int>(this->posPred_[i].size()); ++m){
                dynamic_predictor::PredictedTrajectory traj;
                if (i < this->intentProb_.size() && this->intentProb_[i].size() > m){
                    traj.probability = this->intentProb_[i](m);
                } else {
                    traj.probability = 0.0;
                }

                const std::vector<Eigen::Vector3d>& meanList = this->posPred_[i][m];
                const std::vector<Eigen::Vector3d>* varListPtr = nullptr;
                if (i < this->varPred_.size() && m < static_cast<int>(this->varPred_[i].size())){
                    varListPtr = &this->varPred_[i][m];
                }
                for (size_t t = 0; t < meanList.size(); ++t){
                    geometry_msgs::Point mean;
                    mean.x = meanList[t](0);
                    mean.y = meanList[t](1);
                    mean.z = meanList[t](2);
                    traj.mean.push_back(mean);

                    geometry_msgs::Vector3 var;
                    if (varListPtr && t < varListPtr->size()){
                        var.x = (*varListPtr)[t](0);
                        var.y = (*varListPtr)[t](1);
                        var.z = (*varListPtr)[t](2);
                    } else {
                        var.x = var.y = var.z = 0.0;
                    }
                    traj.variance.push_back(var);
                }

                if (!traj.mean.empty()){
                    ob.modes.push_back(traj);
                }
            }

            msg.obstacles.push_back(ob);
        }

        this->predInfoPub_.publish(msg);
    }

    void predictor::publishAdaptiveMetrics(){
        // 发布每个障碍物的 D_t 和 s_adaptive
        // 格式：对于 N 个障碍物，数组长度为 2*N，每个障碍物有两个值：[D_t, s_adaptive]
        std_msgs::Float64MultiArray metricsMsg;
        metricsMsg.layout.dim.resize(1);
        metricsMsg.layout.dim[0].label = "adaptive_metrics";
        metricsMsg.layout.dim[0].size = currentDt_.size() * 2; // 每个障碍物2个值
        metricsMsg.layout.dim[0].stride = 2;
        metricsMsg.layout.data_offset = 0;
        
        // 填充数据：对于每个障碍物，先 D_t，再 s_adaptive
        for (size_t i = 0; i < currentDt_.size(); ++i) {
            double Dt = (i < currentDt_.size()) ? currentDt_[i] : 0.0;
            double s_adap = (i < currentSAdaptive_.size()) ? currentSAdaptive_[i] : 1.0;
            metricsMsg.data.push_back(Dt);
            metricsMsg.data.push_back(s_adap);
        }
        
        this->adaptiveMetricsPub_.publish(metricsMsg);
    }

// 直接返回预测的位置、尺寸和意图概率，适合需要单独处理这些数据的情况
    void predictor::getPrediction(std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predPos, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predSize, std::vector<Eigen::VectorXd> &intentProb){
        if (this->sizePred_.size()){
            predPos = this->posPred_;
            predSize = this->sizePred_;
            intentProb = this->intentProb_;
        }
        else{
            predPos.clear();
            predSize.clear();
            intentProb.clear();
        }
    }

// 将预测结果封装到 dynamicPredictor::obstacle 结构体中返回，封装为obstacle对象的形式，适合需要以对象为单位处理数据的场景
    void predictor::getPrediction(std::vector<dynamicPredictor::obstacle> &predOb){
        for (int i=0;i<int(this->posPred_.size());i++){
            dynamicPredictor::obstacle ob;
            ob.posPred = this->posPred_[i];
            ob.sizePred = this->sizePred_[i];
            ob.intentProb = this->intentProb_[i];
            predOb.push_back(ob);
        }
    }





    // 新增成员函数：计算并打印误差
    void predictor::calculateAndPrintErrors() {
        if (posHist_.empty() || posPred_.empty()) {
            // ROS_WARN("No reference or prediction data available, skip error calculation.");
            return;
        }

        // 预测总时长3s，计算时间步数N
        const int totalPredSteps = static_cast<int>(3.0 / dt_); // 3s内的时间步数
        if (totalPredSteps <= 0) {
            ROS_ERROR("Invalid dt_ (time interval), cannot calculate steps.");
            return;
        }

        // 遍历每个障碍物
        for (size_t obsIdx = 0; obsIdx < posHist_.size(); ++obsIdx) {
            const auto& refTraj = posHist_[obsIdx]; // 该障碍物的历史轨迹（从过去到现在）
            const auto& predTrajs = posPred_[obsIdx]; // 该障碍物的所有意图的预测轨迹（从现在到未来）

            // 检查参考轨迹是否有效
            if (refTraj.empty()) {
                ROS_WARN("Obstacle %zu has no reference trajectory, skip.", obsIdx);
                continue;
            }

            // 说明：posHist_是历史轨迹，最后一个点是当前时刻 t0
            // posPred_是从当前时刻 t0 开始的未来预测轨迹（t0+1, t0+2, ..., t0+totalPredSteps）
            // 要计算ADE/FDE，需要未来的真实轨迹（t0+1 到 t0+totalPredSteps），但在实时系统中我们没有
            
            // 回测方案：用历史轨迹的最后 totalPredSteps 个点作为"未来"参考
            // 这意味着：假设在 t0 - totalPredSteps 时刻做了预测，预测未来 totalPredSteps 步
            // 然后用 t0 - totalPredSteps + 1 到 t0 的历史轨迹作为"未来"参考
            // 注意：这需要历史轨迹足够长，且假设运动模式在短时间内相对稳定
            
            const int histSize = static_cast<int>(refTraj.size());
            if (histSize < totalPredSteps + 1) {
                // 历史轨迹需要至少 totalPredSteps + 1 个点才能进行回测
                // （需要 totalPredSteps 个点作为"未来"参考）
                ROS_WARN("Obstacle %zu: history size (%d) < required steps (%d + 1), skip error calculation.", 
                        obsIdx, histSize, totalPredSteps);
                continue;
            }

            // 回测：用历史轨迹的最后 totalPredSteps 个点作为"未来"参考
            // 历史轨迹索引范围：[0, histSize-1]，最后 totalPredSteps 个点的索引是 [histSize - totalPredSteps, histSize - 1]
            // 假设在 histSize - totalPredSteps - 1 时刻做了预测，预测未来 totalPredSteps 步
            // 预测的时刻应该是 [histSize - totalPredSteps, histSize - 1]（共 totalPredSteps 个点）
            const int refStartIdx = histSize - totalPredSteps; // 参考轨迹起始索引
            const int validSteps = std::min(totalPredSteps, histSize - refStartIdx); // 确保不超过历史轨迹长度
            
            // 注意：当前 posPred_ 是在当前时刻 t0（histSize-1）做的预测，不是 predStartIdx 时刻
            // 这里用当前预测作为近似（假设运动模式稳定）
            // 理想情况下，应该在 predStartIdx 时刻保存预测结果

            // 遍历所有意图的预测轨迹，找最小ADE和FDE
            double minADE = INFINITY;
            double minFDE = INFINITY;
            int bestIntent = -1;

            for (size_t intentIdx = 0; intentIdx < predTrajs.size(); ++intentIdx) {
                const auto& predTraj = predTrajs[intentIdx]; // 该意图的预测轨迹

                // 检查预测轨迹时间步是否足够
                if (predTraj.size() < static_cast<size_t>(validSteps)) {
                    ROS_WARN("Obstacle %zu, intent %zu: prediction steps (%zu) < valid steps (%d), skip.",
                            obsIdx, intentIdx, predTraj.size(), validSteps);
                    continue;
                }

                // 计算该意图的ADE和FDE
                double ade = 0.0;
                double fde = 0.0;
                int actualSteps = 0; // 实际计算的有效步数

                for (int t = 0; t < validSteps; ++t) {
                    // 修复2：时间对齐
                    // 回测场景：用历史轨迹的最后 totalPredSteps 个点作为"未来"参考
                    // 历史轨迹的最后 totalPredSteps 个点的索引范围：[refStartIdx, histSize - 1]
                    // 即：[histSize - totalPredSteps, histSize - 1]
                    // 预测轨迹的 t 时刻对应参考轨迹的 refStartIdx + t
                    const int refIdx = refStartIdx + t;
                    if (refIdx >= histSize) {
                        // 索引越界，提前退出
                        break;
                    }

                    // 计算t时刻的位置误差（欧氏距离）
                    Eigen::Vector3d error = predTraj[t] - refTraj[refIdx];
                    
                    // ADE和FDE都使用2D距离（忽略z分量），保持一致性
                    double dist = error.head<2>().norm(); // 使用2D距离（x, y）
                    ade += dist;
                    actualSteps++;

                    // 记录最后一步误差（FDE）- 每次更新，确保是真正的最后一步
                    fde = dist;
                }

                // 计算平均ADE（使用实际有效步数）
                if (actualSteps > 0) {
                    ade /= actualSteps;
                    // FDE已经在循环中设置为最后一步的误差，不需要额外处理
                } else {
                    // 如果没有有效步数，跳过这个意图
                    continue;
                }

                // 更新最小ADE和FDE
                if (ade < minADE) {
                    minADE = ade;
                    minFDE = fde;
                    bestIntent = static_cast<int>(intentIdx);
                }
            }

            // 打印结果（只打印有有效预测的障碍物）
            if (bestIntent != -1) {
                ROS_INFO_THROTTLE(1.0, "Obstacle %zu | Min ADE: %.4f | Min FDE: %.4f | Best Intent: %d | Valid Steps: %d",
                            obsIdx, minADE, minFDE, bestIntent, validSteps);
                
                // 写入CSV文件
                if (logFile_.is_open()) {
                    // 使用当前时间的字符串格式 YYYY-MM-DD HH:MM:SS
                    std::time_t now = std::time(nullptr);
                    std::tm* timeinfo = std::localtime(&now);
                    std::stringstream time_ss;
                    time_ss << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S");
                    std::string timestamp_str = time_ss.str();
                    
                    // 获取当前意图概率分布
                    Eigen::VectorXd intentProbVec;
                    if (static_cast<size_t>(obsIdx) < intentProb_.size() && intentProb_[obsIdx].size() > 0) {
                        intentProbVec = intentProb_[obsIdx];
                        ROS_INFO_THROTTLE(5.0, "CSV写入：Obs %zu, intentProb_.size()=%zu, intentProbVec.size()=%ld", 
                                          obsIdx, intentProb_.size(), intentProbVec.size());
                    } else {
                        intentProbVec = Eigen::VectorXd::Constant(numIntent_, 1.0 / numIntent_);
                        ROS_WARN_THROTTLE(5.0, "CSV写入：Obs %zu 使用默认概率分布", obsIdx);
                    }
                    
                    // 按照枚举顺序提取概率：FORWARD=0, LEFT=1, RIGHT=2, STOP=3
                    double P_forward = (intentProbVec.size() > FORWARD) ? intentProbVec(FORWARD) : 0.25;
                    double P_left = (intentProbVec.size() > LEFT) ? intentProbVec(LEFT) : 0.25;
                    double P_right = (intentProbVec.size() > RIGHT) ? intentProbVec(RIGHT) : 0.25;
                    double P_stop = (intentProbVec.size() > STOP) ? intentProbVec(STOP) : 0.25;
                    
                    // 调试输出：打印意图概率
                    // ROS_INFO_THROTTLE(5.0, "Obs %zu Intent Prob: Forward=%.3f, Left=%.3f, Right=%.3f, Stop=%.3f", 
                    //                   obsIdx, P_forward, P_left, P_right, P_stop);
                    
                    // 获取自适应指标
                    double Dt = (static_cast<size_t>(obsIdx) < currentDt_.size()) ? currentDt_[obsIdx] : 0.0;
                    double s_adap = (static_cast<size_t>(obsIdx) < currentSAdaptive_.size()) ? currentSAdaptive_[obsIdx] : 1.0;
                    // 确保 bestIntent 在有效范围内 (0-3)
                    int k_adap = (bestIntent >= 0 && bestIntent < numIntent_) ? bestIntent : -1;
                    
                    // 获取障碍物当前位置（当前时刻的位置，即历史轨迹的最后一个点）
                    double obstacle_x = 0.0;
                    double obstacle_y = 0.0;
                    if (static_cast<size_t>(obsIdx) < posHist_.size() && !posHist_[obsIdx].empty()) {
                        obstacle_x = posHist_[obsIdx][0](0);  // 当前时刻的x坐标
                        obstacle_y = posHist_[obsIdx][0](1);  // 当前时刻的y坐标
                        ROS_INFO_THROTTLE(5.0, "CSV写入：Obs %zu 位置=(%.3f, %.3f), P=[%.3f,%.3f,%.3f,%.3f]", 
                                          obsIdx, obstacle_x, obstacle_y, P_forward, P_left, P_right, P_stop);
                    } else {
                        ROS_WARN_THROTTLE(5.0, "CSV写入：Obs %zu 位置数据不可用", obsIdx);
                    }
                    
                    // 写入CSV行（按表头顺序：时间戳,障碍物ID,障碍物X,障碍物Y,ADE(米),FDE(米),D_t,s_adaptive,最佳意图,有效步数,P_forward,P_left,P_right,P_stop）
                    logFile_ << std::fixed << std::setprecision(6)
                             << timestamp_str << ","
                             << obsIdx << ","
                             << obstacle_x << ","
                             << obstacle_y << ","
                             << minADE << ","
                             << minFDE << ","
                             << Dt << ","
                             << s_adap << ","
                             << k_adap << ","
                             << validSteps << ","
                             << P_forward << ","
                             << P_left << ","
                             << P_right << ","
                             << P_stop << "\n";
                    logFile_.flush();  // 立即刷新到磁盘
                    ROS_INFO_THROTTLE(5.0, "CSV数据已写入并刷新");
                } else {
                    ROS_ERROR_THROTTLE(5.0, "CSV文件未打开！无法写入数据");
                }
            }
        }
    }



    // 新增：获取每个障碍物的主意图（最高概率对应的意图索引）
    std::vector<int> predictor::getMainIntents(const std::vector<Eigen::VectorXd>& intentProb) {
        std::vector<int> mainIntents;
        for (const auto& prob : intentProb) {
            int mainIntent;
            prob.maxCoeff(&mainIntent);  // 获取最高概率对应的索引
            mainIntents.push_back(mainIntent);
        }
        return mainIntents;
    }









    // 获取桌面路径（参考 traj_vis6.py 的 get_desktop_path 逻辑）
    std::string predictor::getDesktopPath() {
        const char* home = std::getenv("HOME");
        if (!home) {
            ROS_WARN_STREAM(this->hint_ << ": HOME environment variable not set, using /tmp");
            return "/tmp";
        }
        
        std::string homeStr = std::string(home);
        
        // 优先检测中文桌面路径（参考 Python: os.path.join(home, "桌面")）
        std::string chineseDesktop = homeStr + "/桌面";
        struct stat info;
        if (stat(chineseDesktop.c_str(), &info) == 0 && (info.st_mode & S_IFDIR)) {
            return chineseDesktop;
        }
        
        // 再检测英文桌面路径（参考 Python: os.path.join(home, "Desktop")）
        std::string englishDesktop = homeStr + "/Desktop";
        if (stat(englishDesktop.c_str(), &info) == 0 && (info.st_mode & S_IFDIR)) {
            return englishDesktop;
        }
        
        // 都不存在时使用家目录（保底方案，参考 Python 的 return home）
        ROS_WARN_STREAM(this->hint_ << ": 未找到桌面目录，将使用家目录保存文件");
        return homeStr;
    }

    // 日志，添加了析构函数以确保文件正确关闭
    predictor::~predictor() {
        if (logFile_.is_open()) {
            logFile_.flush();  // 确保所有数据都写入缓冲区
            logFile_.close();  // 关闭文件，确保数据写入磁盘
            std::cout << "\n========================================" << std::endl;
            std::cout << hint_ << ": CSV log file closed and saved." << std::endl;
            std::cout << "========================================\n" << std::endl;
        } else {
            std::cout << hint_ << ": Warning - log file was not open." << std::endl;
        }
    }
}