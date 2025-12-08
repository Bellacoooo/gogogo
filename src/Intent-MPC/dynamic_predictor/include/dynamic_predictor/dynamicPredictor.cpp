/*
    FILE: dynamicPredictor.cpp
    ---------------------------------
    function implementation of dynamic osbtacle predictor
*/
#include <dynamic_predictor/dynamicPredictor.h>
#include <ctime>  // 需包含时间戳头文件

namespace dynamicPredictor{
    predictor::predictor(const ros::NodeHandle& nh) : nh_(nh){
        this->ns_ = "dynamic_predictor";
        this->hint_ = "[predictor]";
        this->initParam();
        this->registerPub();
        this->registerCallback();
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
            this->pscale_ = 1.5;
            std::cout << this->hint_ << ": No prob scale param. Use default: 1.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Prob scale param is set to: " << this->pscale_ << std::endl;
        } 

        // 自适应方案参数
            // 新增：自适应方案参数 - gamma1（NIS权重）
        if (not this->nh_.getParam(this->ns_ + "/gamma1", this->gamma1_)){
            this->gamma1_ = 0.5;  // 默认值
            std::cout << this->hint_ << ": No gamma1 parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The gamma1 is set to: " << this->gamma1_ << std::endl;
        }

        // 新增：自适应方案参数 - gamma2（加加速度权重）
        if (not this->nh_.getParam(this->ns_ + "/gamma2", this->gamma2_)){
            this->gamma2_ = 0.5;  // 默认值
            std::cout << this->hint_ << ": No gamma2 parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The gamma2 is set to: " << this->gamma2_ << std::endl;
        }

        // 新增：自适应方案参数 - lambda1（熵敏感度）
        if (not this->nh_.getParam(this->ns_ + "/lambda1", this->lambda1_)){
            this->lambda1_ = 1.0;  // 默认值
            std::cout << this->hint_ << ": No lambda1 parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The lambda1 is set to: " << this->lambda1_ << std::endl;
        }

        // 新增：自适应方案参数 - lambda2（运动诊断敏感度）
        if (not this->nh_.getParam(this->ns_ + "/lambda2", this->lambda2_)){
            this->lambda2_ = 2.0;  // 默认值
            std::cout << this->hint_ << ": No lambda2 parameter found. Use default: 2.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The lambda2 is set to: " << this->lambda2_ << std::endl;
        }

        // 新增：自适应方案参数 - M_thresh（运动突变阈值）
        if (not this->nh_.getParam(this->ns_ + "/M_thresh", this->M_thresh_)){
            this->M_thresh_ = 1.0;  // 默认值
            std::cout << this->hint_ << ": No M_thresh parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The M_thresh is set to: " << this->M_thresh_ << std::endl;
        }

        // 新增：自适应方案参数 - s_max（最大自适应权重）
        if (not this->nh_.getParam(this->ns_ + "/s_max", this->s_max_)){
            this->s_max_ = 2.0;  // 默认值
            std::cout << this->hint_ << ": No s_max parameter found. Use default: 2.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The s_max is set to: " << this->s_max_ << std::endl;
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

        // 初始化历史数据容器（放在参数加载之后，确保使用正确的 historyWindow_ 值）
        this->accHistory_.reserve(this->historyWindow_ + 1);
        this->caPredHistory_.reserve(this->historyWindow_ + 1);

        // 初始化日志文件
        std::string log_filename = "/tmp/adaptive_mdp_log.csv"; // 可写路径
        logFile_.open(log_filename, std::ios::out | std::ios::app);
        if (logFile_.is_open()) {
            // 写入表头（仅在文件新建时写入，通过判断文件大小实现）
            logFile_.seekp(0, std::ios::end);
            if (logFile_.tellp() == 0) {
                logFile_ << "Time(s),S_Adaptive,Ht,Mt,NIS,Jerk,P_Forward,P_Left,P_Right,P_Stop\n";
            }
            std::cout << this->hint_ << ": Log file initialized at " << log_filename << std::endl;
        } else {
            std::cerr << this->hint_ << "Error: Failed to open log file!" << std::endl;
        }

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
        this->sValuePub_ = this->nh_.advertise<std_msgs::Float32>(this->ns_ + "/current_s_value", 1);
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
    }

    void predictor::predCB(const ros::TimerEvent&){    
        this->predict();
    }

    // main function for prediction
    void predictor::predict(){ 
        // get history
        if (this->useFakeDetector_ and this->detectorGTReady_ and this->mapReady_){
            this->detectorGT_->getDynamicObstaclesHist(this->posHist_, this->velHist_, this->accHist_, this->sizeHist_, this->robotSize_);
        }
        else if (not this->useFakeDetector_ and this->detectorReady_ and this->mapReady_){
            this->detector_->getDynamicObstaclesHist(this->posHist_, this->velHist_, this->accHist_, this->sizeHist_, this->robotSize_);
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

                    // 新增：重置计时变量
                    mainIntentTime_ = std::chrono::duration<double, std::milli>::zero();
                    nonMainIntentTime_ = std::chrono::duration<double, std::milli>::zero();
                    genPointsTotalTime_ = std::chrono::duration<double, std::milli>::zero();

                    // 调用predTraj时传入主意图，用于内部计时
                    this->predTraj(allPredPointsTemp, posPredTemp, sizePredTemp, mainIntents);

                    // 新增：打印耗时统计
                    std::cout << "\n===== 轨迹生成耗时统计 =====" << std::endl;
                    std::cout << "genPoints总耗时: " << genPointsTotalTime_.count() << " ms" << std::endl;
                    std::cout << "主意图生成耗时: " << mainIntentTime_.count() << " ms" << std::endl;
                    std::cout << "非主意图总耗时: " << nonMainIntentTime_.count() << " ms" << std::endl;
                    std::cout << "===========================\n" << std::endl;


                    this->intentProb_ = intentProbTemp;
                    this->posPred_ = posPredTemp;
                    this->sizePred_ = sizePredTemp;
                    this->allPredPoints_ = allPredPointsTemp;
        
    }
        }
        else{
            this->intentProb_.clear();
            this->allPredPoints_.clear();
            this->posPred_.clear();
            this->sizePred_.clear();
        }

        // 计算ADE、FDE
        calculateAndPrintErrors();



    }



// 自己加的指标
    // 自适应的指标，和计算。四个函数
    // 1. 计算加加速度幅值 ||j_t||
    double predictor::computeJerkNorm(const Eigen::Vector3d& currAcc) {
        if (accHistory_.size() < historyWindow_) {
            accHistory_.push_back(currAcc);
            return 0.0;  // 历史不足时返回0
        }
        // 取最近一次历史加速度计算加加速度
        Eigen::Vector3d prevAcc = accHistory_.back();
        double jerkNorm = (currAcc - prevAcc).norm() / dt_;  // dt_为预测时间步长（原有参数）
        // 更新历史（保持窗口大小）
        accHistory_.erase(accHistory_.begin());
        accHistory_.push_back(currAcc);
        return jerkNorm;
    }

    // 2. 计算归一化残差平方 NIS_t（假设残差协方差由跟踪器提供，此处简化为单位矩阵）
    double predictor::computeNIS(const Eigen::Vector3d& currPos, const Eigen::Vector3d& caPredPos, const Eigen::Matrix3d& residualCov) {
        Eigen::Vector3d residual = currPos - caPredPos;
        // 避免协方差矩阵奇异，添加微小扰动
        Eigen::Matrix3d cov = residualCov + Eigen::Matrix3d::Identity() * 1e-6;
        double nis = residual.transpose() * cov.inverse() * residual;
        // 存储CA模型预测位置用于后续计算
        if (caPredHistory_.size() >= historyWindow_) {
            caPredHistory_.erase(caPredHistory_.begin());
        }
        caPredHistory_.push_back(caPredPos);
        return nis;
    }

    // 3. 计算意图熵 H_t
    double predictor::computeIntentEntropy(const Eigen::VectorXd& intentProb) {
        double entropy = 0.0;
        for (int i = 0; i < intentProb.size(); ++i) {
            double p = intentProb(i);
            if (p > 1e-6) {  // 避免log(0)
                entropy -= p * log(p);
            }
        }
        return entropy;
    }

    // 4. 计算自适应权重 s_adaptive
    double predictor::computeAdaptiveS(const double Ht, const double Mt) {
        // 计算熵约束项：e^(-λ1*Ht)
        double entropyTerm = exp(-lambda1_ * Ht);
        // 计算运动诊断约束项：1/(1 + e^(λ2*(Mt - M_thresh)))
        double motionTerm = 1.0 / (1.0 + exp(lambda2_ * (Mt - M_thresh_)));
        // 衰减因子 D_t
        double Dt = entropyTerm * motionTerm;
        // 自适应权重（限制在[1, s_max_]）
        double s_adaptive = 1.0 + (s_max_ - 1.0) * Dt;
        return std::max(1.0, std::min(s_adaptive, s_max_));  // 截断到有效范围
    }






    void predictor::intentProb(std::vector<Eigen::VectorXd> &intentProbTemp){
        int numOb = this->posHist_.size();  //当前跟踪的目标数量
        
        intentProbTemp.resize(numOb);   //intentProbTemp用来存储每个目标的意图概率（输出结果）
        for (int i=0; i<numOb; ++i){
            // init state prob P
            // 初始化状态概率（均匀分布，4种意图：前、左、右、停）
            Eigen::VectorXd P;
            P.resize(this->numIntent_);
            P.setConstant(1.0/this->numIntent_);
            int numHist = this->posHist_[i].size();
            for (int j=2; j<numHist; ++j){
                // transition matrix 
                 // 获取历史位置和速度
                Eigen::Vector3d prevPos, prevVel;
                Eigen::Vector3d currPos, currVel;
                prevPos = this->posHist_[i][numHist-j-1];
                prevVel = this->velHist_[i][numHist-j-1];
                currPos = this->posHist_[i][numHist-j-2];
                currVel = this->velHist_[i][numHist-j-2];
                double prevAngle = atan2(prevPos(1)-this->posHist_[i][numHist-j](1), prevPos(0)-this->posHist_[i][numHist-j](0));
                double currAngle = atan2(currPos(1)-prevPos(1), currPos(0)-prevPos(0));
                // currVel = this->posHist_[i][numHist-j-2];
                // Eigen::MatrixXd transMat = this->genTransitionMatrix(prevPos, currPos, prevVel, currVel);
                // 原论文用的是这个函数
                // Eigen::MatrixXd transMat = this->genTransitionMatrix(prevAngle, currAngle, currVel);
                // 现在改为自适应改为下面的两行，增加了输入
                Eigen::Vector3d currAcc = this->accHist_[i][numHist-j-2];  // 与pos/vel获取逻辑一致
                Eigen::MatrixXd transMat = this->genTransitionMatrix(prevAngle, currAngle, currVel, currPos, currAcc);

                Eigen::VectorXd newP= transMat*P;
                P = newP;
            }
            intentProbTemp[i] = P;
            
        }
    }

    //据角度和速度，计算“意图转移概率矩阵”
    // Eigen::MatrixXd predictor::genTransitionMatrix(const Eigen::Vector3d &prevPos, const Eigen::Vector3d &currPos, const Eigen::Vector3d &prevVel, const Eigen::Vector3d &currVel){
    Eigen::MatrixXd predictor::genTransitionMatrix(const double &prevAngle, const double &currAngle, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currAcc){
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
        
        // 表示当前速度的模长
        double r = sqrt(pow(currVel(0), 2) + pow(currVel(1), 2));   

    // 3. 新增：计算自适应权重所需指标（根据你的扩展逻辑）
        // 3. 计算自适应权重所需指标
        // 3.1 加加速度 norm（使用传入的 currAcc）
        double jerkNorm = computeJerkNorm(currAcc);  // 调用已有计算函数，无需再用 .norm()

        // 3.2 NIS 计算（使用传入的 currPos 和 CA 模型预测）
        Eigen::Vector3d caPredPos;
        if (!caPredHistory_.empty()) {
            caPredPos = caPredHistory_.back();  // 从历史获取最近的 CA 预测位置
        } else {
            caPredPos = currPos;  // 无历史时用当前位置作为默认
        }
        Eigen::Matrix3d residualCov = Eigen::Matrix3d::Identity() * 0.1;  // 残差协方差（根据实际情况调整）
        double nis = computeNIS(currPos, caPredPos, residualCov);  // 调用已有 NIS 计算函数

        // 3.3 意图熵（从当前意图概率计算，若未初始化则用均匀分布）
        Eigen::VectorXd currIntentProb;
        if (!this->intentProb_.empty()) {
            currIntentProb = this->intentProb_.back();  // 从历史获取
        } else {
            currIntentProb = Eigen::VectorXd::Constant(this->numIntent_, 1.0 / this->numIntent_);  // 均匀分布初始化
        }
        double Ht = - (currIntentProb.array() * currIntentProb.array().log()).sum();  // 熵计算公式

        // 3.4 运动诊断指标
        double Mt = gamma1_ * nis + gamma2_ * jerkNorm;

        // 3.5 自适应权重（使用正确的 computeAdaptiveS 函数）
        double s_adaptive = computeAdaptiveS(Ht, Mt);
        // 4. 生成转移矩阵的每一列
        for (int i = 0; i < this->numIntent_; ++i) {
            Eigen::VectorXd scale = Eigen::VectorXd::Ones(this->numIntent_);
            scale(i) = s_adaptive;  // 用自适应权重
            transMat.col(i) = this->genTransitionVector(theta, r, scale);
        }        

        // //对每种意图（前进、左、右、停）都生成一个“转移概率列”；
        // // 每列表示：从这个意图出发，到别的意图的概率；
        // // 这些概率是通过 genTransitionVector() 算出来的。
        // // fill in every column of transition matrix
        // for (int i=0; i<this->numIntent_;i++){
        //     Eigen::VectorXd scale;
        //     scale.setOnes(this->numIntent_);
        //     scale(i) = this->pscale_;
        //     Eigen::VectorXd probVec = this->genTransitionVector(theta, r, scale);
        //     transMat.block(0,i,this->numIntent_,1) = probVec;
        // }        


            // 写入日志数据
        if (logFile_.is_open()) {
        // 记录时间戳（ROS时间，精确到秒）
        logFile_ << ros::Time::now().toSec() << ",";
        // 记录核心变量
        logFile_ << s_adaptive << "," 
                << Ht << "," 
                << Mt << "," 
                << nis << "," 
                << jerkNorm << ",";
        // 记录4种意图概率（FORWARD/LEFT/RIGHT/STOP）
        logFile_ << currIntentProb(FORWARD) << "," 
                << currIntentProb(LEFT) << "," 
                << currIntentProb(RIGHT) << "," 
                << currIntentProb(STOP) << "\n";
        }

            // 【新增】发布 S_Adaptive 值
        std_msgs::Float32 s_msg;
        s_msg.data = s_adaptive; // 假设 s_adaptive 在这里是可见的
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
        const std::vector<int>& mainIntents){     //这里输入的参数新加了一个主意图的参数

        // 清空并重置输出容器，确保不包含旧数据
        posPredTemp.clear();
        posPredTemp.resize(this->posHist_.size());
        sizePredTemp.clear();
        sizePredTemp.resize(this->sizeHist_.size());
        allPredPointsTemp.clear();
        allPredPointsTemp.resize(this->posHist_.size());
        
        // predict each obstacle遍历每个障碍物
        for (int i=0; i < int(this->posHist_.size()); i++){
            posPredTemp[i].resize(this->numIntent_);
            sizePredTemp[i].resize(this->numIntent_);
            allPredPointsTemp[i].resize(this->numIntent_);








            // predict for each number of intent
            for (int j=FORWARD; j<=STOP; ++j){
                std::vector<std::vector<Eigen::Vector3d>> predPoints;   //存储当前意图的预测点
                std::vector<Eigen::Vector3d> predSize;                  //存储当前意图的预测大小
                // 根据障碍物当前的状态（位置、速度、加速度、大小）和意图，生成预测点和大小
                this->genPoints(j, this->posHist_[i][0], this->velHist_[i][0], this->accHist_[i][0], this->sizeHist_[i][0], predPoints, predSize);
                if (predPoints.size()){
                    std::vector<Eigen::Vector3d> predPos;
                    this->genTraj(predPoints, predPos, predSize);
                    posPredTemp[i][j] = predPos;
                    sizePredTemp[i][j] = predSize;
                    allPredPointsTemp[i][j] = predPoints;
                }
                // 如果未生成预测点，则使用历史状态生成默认预测轨迹
                else{
                    Eigen::Vector3d size = this->sizeHist_[i][0];
                    Eigen::Vector3d currVel = this->velHist_[i][0];
                    Eigen::Vector3d currPos = this->posHist_[i][0];
                    std::vector<Eigen::Vector3d> predPos;
                    // 计算速度的模长
                    double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2)); 
                    for (int i=0;i<this->numPred_+1;i++){
                        predPos.push_back(currPos);
                        predSize.push_back(size);
                        size(0) += 2*min(vel,this->stopVel_)*this->dt_;
                        size(1) += 2*min(vel,this->stopVel_)*this->dt_;
                    }
                    posPredTemp[i][j] = predPos;
                    sizePredTemp[i][j] = predSize;
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
    void predictor::genTraj(const std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predPos, std::vector<Eigen::Vector3d> &predSize){
        predPos.clear();
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
            int countMarker = 0;
			for (int i=0; i<int(this->posPred_.size()); ++i){
                for (int j=0; j<int(this->posPred_[i].size());j++){
                    visualization_msgs::Marker traj;
                    traj.header.frame_id = "map";
                    traj.header.stamp = ros::Time::now();
                    traj.ns = "predictor";
                    traj.id = countMarker;
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
                    ++countMarker;
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
            ROS_WARN("No reference or prediction data available, skip error calculation.");
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
                ROS_INFO_STREAM("Obstacle " << obsIdx 
                            << " | Min ADE: " << minADE 
                            << " | Min FDE: " << minFDE 
                            << " | Best Intent: " << bestIntent 
                            << " | Valid Steps: " << validSteps);
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









    // 日志，添加了析构函数以确保文件正确关闭
    predictor::~predictor() {
    if (logFile_.is_open()) {
        logFile_.close();
        std::cout << hint_ << ": Log file closed." << std::endl;
    }
}
}