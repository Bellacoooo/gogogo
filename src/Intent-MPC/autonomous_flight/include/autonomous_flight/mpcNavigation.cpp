/*
	FILE: mpcNavigation.cpp
	------------------------
	dynamic navigation implementation file
*/
#include <autonomous_flight/mpcNavigation.h>

namespace AutoFlight{
	mpcNavigation::mpcNavigation(const ros::NodeHandle& nh) : flightBase(nh){
		this->initParam();
		this->initModules();
		this->registerPub();		
	}

	void mpcNavigation::initParam(){
    	// parameters    
    	// use simulation detector	
		if (not this->nh_.getParam("autonomous_flight/use_fake_detector", this->useFakeDetector_)){
			this->useFakeDetector_ = false;
			cout << "[AutoFlight]: No use fake detector param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use fake detector is set to: " << this->useFakeDetector_ << "." << endl;
		}

		// use predictor	
		if (not this->nh_.getParam("autonomous_flight/use_predictor", this->usePredictor_)){
			this->usePredictor_ = false;
			cout << "[AutoFlight]: No use predictor param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Use predictor is set to: " << this->usePredictor_ << "." << endl;
		}


    	// use global planner or not	
		if (not this->nh_.getParam("autonomous_flight/use_global_planner", this->useGlobalPlanner_)){
			this->useGlobalPlanner_ = false;
			cout << "[AutoFlight]: No use global planner param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Global planner use is set to: " << this->useGlobalPlanner_ << "." << endl;
		}

		// global planner type (rrt / astar)
		if (not this->nh_.getParam("autonomous_flight/global_planner_type", this->globalPlannerType_)){
			this->globalPlannerType_ = "rrt";
			cout << "[AutoFlight]: No global planner type param found. Use default: rrt." << endl;
		}
		else{
			cout << "[AutoFlight]: Global planner type: " << this->globalPlannerType_ << "." << endl;
		}

		// No turning of yaw
		if (not this->nh_.getParam("autonomous_flight/no_yaw_turning", this->noYawTurning_)){
			this->noYawTurning_ = false;
			cout << "[AutoFlight]: No yaw turning param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Yaw turning use is set to: " << this->noYawTurning_ << "." << endl;
		}	

		// full state control (yaw)
		if (not this->nh_.getParam("autonomous_flight/use_yaw_control", this->useYawControl_)){
			this->useYawControl_ = false;
			cout << "[AutoFlight]: No yaw control param found. Use default: false." << endl;
		}
		else{
			cout << "[AutoFlight]: Yaw control use is set to: " << this->useYawControl_ << "." << endl;
		}		

    	// desired linear velocity    	
		if (not this->nh_.getParam("autonomous_flight/desired_velocity", this->desiredVel_)){
			this->desiredVel_ = 1.0;
			cout << "[AutoFlight]: No desired velocity param found. Use default: 1.0 m/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}

		// desired acceleration
		if (not this->nh_.getParam("autonomous_flight/desired_acceleration", this->desiredAcc_)){
			this->desiredAcc_ = 1.0;
			cout << "[AutoFlight]: No desired acceleration param found. Use default: 1.0 m/s^2." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired acceleration is set to: " << this->desiredAcc_ << "m/s^2." << endl;
		}

    	// desired angular velocity    	
		if (not this->nh_.getParam("autonomous_flight/desired_angular_velocity", this->desiredAngularVel_)){
			this->desiredAngularVel_ = 1.0;
			cout << "[AutoFlight]: No desired angular velocity param found. Use default: 0.5 rad/s." << endl;
		}
		else{
			cout << "[AutoFlight]: Desired angular velocity is set to: " << this->desiredAngularVel_ << "rad/s." << endl;
		}	

		// whether or not to use predefined goal
		if (not this->nh_.getParam("autonomous_flight/use_predefined_goal", this->usePredefinedGoal_)){
			this->usePredefinedGoal_ = false;
			cout << "[AutoFlight]: No use predefined goal param found. Use default: false." << endl;
		} 
		else{
			cout << "[AutoFlight]: Use predefined goal is set to: " << this->usePredefinedGoal_ << "." << endl;
		}

		if (not this->nh_.getParam("autonomous_flight/predefined_goal_directory", this->refTrajPath_)){
			this->refTrajPath_ = "None";
			cout << "[AutoFlight]: No predefined goal directory directory." << endl;
		} 
		else{
			std::string autoFlightPkgPath = ros::package::getPath("autonomous_flight");
			this->refTrajPath_ = autoFlightPkgPath + this->refTrajPath_;
			cout << "[AutoFlight]: Predefined goal directory is set to: " << this->refTrajPath_ << "." << endl;
		}	

		if (this->usePredefinedGoal_){
			this->goalIdx_ = 0;
			this->predefinedGoal_ = this->getRefTraj();
			this->goal_ = this->predefinedGoal_.poses[this->goalIdx_];
		}

		// whether or not to repeat tracking predefined path
		if (not this->nh_.getParam("autonomous_flight/execute_path_times", this->repeatPathNum_)){
			this->repeatPathNum_ = 1;
			cout << "[AutoFlight]: No execute path number of times param found. Use default: 1." << endl;
		} 
		else{
			cout << "[AutoFlight]: Execute path number of times is set to: " << this->repeatPathNum_ << "." << endl;
		}	

		// realtime replanning parameters
		if (not this->nh_.getParam("autonomous_flight/enable_realtime_replan", this->enableRealtimeReplan_)){
			this->enableRealtimeReplan_ = true;  // 默认启用实时重规划
		}
		if (not this->nh_.getParam("autonomous_flight/global_replan_interval", this->globalReplanInterval_)){
			this->globalReplanInterval_ = 0.5;  // 降低到0.5秒，提高重规划频率
		}
		if (not this->nh_.getParam("autonomous_flight/path_deviation_threshold", this->pathDeviationThreshold_)){
			this->pathDeviationThreshold_ = 0.5;
		}
		this->lastGlobalReplanTime_ = ros::Time::now();

	}

	void mpcNavigation::initModules(){
		// initialize map
		if (this->useFakeDetector_){
			// initialize fake detector
			this->detector_.reset(new onboardDetector::fakeDetector (this->nh_));	
			this->map_.reset(new mapManager::dynamicMap (this->nh_, false));
		}
		else{
			this->map_.reset(new mapManager::dynamicMap (this->nh_));
		}

		// initialize predictor
		if (this->usePredictor_){
			this->predictor_.reset(new dynamicPredictor::predictor (this->nh_));
			this->predictor_->setMap(this->map_);
			if (this->useFakeDetector_){
				this->predictor_->setDetector(this->detector_);
			}
		}

		// initialize global planner (rrt / astar)
		if (this->globalPlannerType_ == "astar"){
			this->aStarPlanner_.reset(new globalPlanner::AStarOccMap (this->nh_));
			this->aStarPlanner_->setMap(this->map_);
		}
		// SIPP 已删除
		else{ // rrt by default
			this->rrtPlanner_.reset(new globalPlanner::rrtOccMap<3> (this->nh_));
			this->rrtPlanner_->setMap(this->map_);
		}

		// initialize polynomial trajectory planner
		this->polyTraj_.reset(new trajPlanner::polyTrajOccMap (this->nh_));
		this->polyTraj_->setMap(this->map_);
		this->polyTraj_->updateDesiredVel(this->desiredVel_);
		this->polyTraj_->updateDesiredAcc(this->desiredAcc_);

		// initialize piecewise linear trajectory planner
		this->pwlTraj_.reset(new trajPlanner::pwlTraj (this->nh_));

		this->mpc_.reset(new trajPlanner::mpcPlanner (this->nh_));
		this->mpc_->updateMaxVel(this->desiredVel_*1.5);
		this->mpc_->updateMaxAcc(this->desiredAcc_);
		this->mpc_->setMap(this->map_);
	}

	void mpcNavigation::registerPub(){
		this->rrtPathPub_ = this->nh_.advertise<nav_msgs::Path>("mpcNavigation/rrt_path", 10);
		this->polyTrajPub_ = this->nh_.advertise<nav_msgs::Path>("mpcNavigation/poly_traj", 10);
		this->pwlTrajPub_ = this->nh_.advertise<nav_msgs::Path>("mpcNavigation/pwl_trajectory", 10);
		this->mpcTrajPub_ = this->nh_.advertise<nav_msgs::Path>("mpcNavigation/mpc_trajectory", 10);
		this->inputTrajPub_ = this->nh_.advertise<nav_msgs::Path>("mpcNavigation/input_trajectory", 10);
		this->goalPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("mpcNavigation/goal", 10);
		this->mpcStatusPub_ = this->nh_.advertise<std_msgs::Bool>("mpcNavigation/infeasible", 10);
		this->riskMap25DPub_ = this->nh_.advertise<nav_msgs::OccupancyGrid>("/risk_map_25d/grid", 1);
		
		// 订阅风险地图（保留旧的，向后兼容）
		this->riskMapSub_ = this->nh_.subscribe("dynamic_predictor/dynamic_risk_map", 1, 
		                                         &mpcNavigation::riskMapCB, this);
		
		// 初始化风险地图对象
		this->riskMap2D_.reset(new globalPlanner::RiskMap2D());
		
		// 初始化2.5D风险地图
		this->riskMap25D_.reset(new globalPlanner::RiskMap25D(this->nh_));
		
		// 设置occupancy map（用于查询静态距离）
		if (this->map_) {
			this->riskMap25D_->setOccupancyMap(this->map_);
		}
		
		// 启动定时器：定期更新RiskMap25D
		this->riskMapUpdateTimer_ = this->nh_.createTimer(ros::Duration(0.1), &mpcNavigation::updateRiskMap25DCB, this);
		
		ROS_INFO("[MPC-Nav] RiskMap25D initialized");
	}

	void mpcNavigation::registerCallback(){
		this->mpcWorker_ = std::thread(&mpcNavigation::mpcCB, this);
		this->mpcWorker_.detach();

		// collision check callback
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.01), &mpcNavigation::replanCheckCB, this);

		// trajectory execution callback
		this->trajExeTimer_ = this->nh_.createTimer(ros::Duration(0.01), &mpcNavigation::trajExeCB, this);

		// visualization callback
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &mpcNavigation::visCB, this);
	}

	nav_msgs::Path mpcNavigation::getRefTraj(){
		nav_msgs::Path path_msg;
		path_msg.header.frame_id = "map"; // Change according to your TF frame
		path_msg.header.stamp = ros::Time::now();
		
		std::ifstream file(this->refTrajPath_);
		
		if (!file.is_open()) {
			ROS_ERROR("Failed to open trajectory file!");
		}
	
		std::string line;
		while (std::getline(file, line)) {
			std::istringstream iss(line);
			double dt, x, y, z;
			if (!(iss >> dt >> x >> y >> z)) break;
	
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "map";
			pose.header.stamp = ros::Time::now();
			pose.pose.position.x = x;
			pose.pose.position.y = y;
			pose.pose.position.z = z;
			pose.pose.orientation.w = 1.0;  // No rotation
	
			path_msg.poses.push_back(pose);
		}
	
		file.close();
		return path_msg;
	}

	void mpcNavigation::mpcCB(){
		ros::Rate r(10);
		while (ros::ok()){
			if (this->mpcReplan_){
				this->replanning_ = true;
				if (not this->refTrajReady_){
					if (this->usePredefinedGoal_){			
						double dt = 0.1; 
						nav_msgs::Path mpcInputTraj = this->predefinedGoal_;
						this->mpc_->updatePath(mpcInputTraj, dt);
						this->inputTrajMsg_ = mpcInputTraj;
						this->mpcFirstTime_ = true;
						this->repeatPathNum_ -= 1;
						this->refTrajReady_ = true;
						this->trackingStartTime_ = ros::Time::now();
					}
					else{
						if (this->useGlobalPlanner_){
							nav_msgs::Path rrtPathMsgTemp;
							if (this->globalPlannerType_ == "astar" && this->aStarPlanner_){
								ROS_WARN("[MPC-A*-CALL] ========== USING A* PLANNER ==========");
								ROS_WARN("[MPC-A*-CALL] Before updateStart: odom=(%.3f,%.3f,%.3f)", 
								         this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
								try {
								this->aStarPlanner_->updateStart(this->odom_.pose.pose);
									ROS_WARN("[MPC-A*-CALL] updateStart completed");
								} catch (const std::exception& e) {
									ROS_ERROR("[MPC-A*-CALL] Exception in updateStart: %s", e.what());
									throw;
								}
								ROS_WARN("[MPC-A*-CALL] Before updateGoal: goal=(%.3f,%.3f,%.3f)", 
								         this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
								try {
								this->aStarPlanner_->updateGoal(this->goal_.pose);
									ROS_WARN("[MPC-A*-CALL] updateGoal completed");
								} catch (const std::exception& e) {
									ROS_ERROR("[MPC-A*-CALL] Exception in updateGoal: %s", e.what());
									throw;
								}
								// 🔧 设置风险地图（优先使用2.5D）
								if (use_risk_map_25d_ && this->riskMap25D_ && this->riskMap25D_->isValid()) {
									// ✨ 使用新版2.5D风险地图
									ROS_INFO_THROTTLE(5.0, "[MPC-A*-CALL] Using RiskMap25D (2.5D fused)");
									try {
										// 更新地图中心为当前位置
										this->riskMap25D_->setMapCenter(this->currPos_);
										
										// TODO: 更新动态预测数据（需要从predictor获取）
										// 暂时先使用静态风险地图功能
										
										// 设置到A*
										this->aStarPlanner_->setRiskMap25D(this->riskMap25D_);
										ROS_INFO_THROTTLE(10.0, "[MPC-A*-CALL] ✅ RiskMap25D set successfully");
									} catch (const std::exception& e) {
										ROS_ERROR("[MPC-A*-CALL] Exception in setRiskMap25D: %s", e.what());
										// 回退到旧版2D
										if (this->riskMap2D_ && this->riskMap2D_->isValid()) {
											this->aStarPlanner_->setRiskMap(this->riskMap2D_);
										}
									}
								} else if (this->riskMap2D_ && this->riskMap2D_->isValid()) {
									// 回退到旧版2D风险地图
									ROS_WARN_THROTTLE(5.0, "[MPC-A*-CALL] Using RiskMap2D (legacy 2D)");
									try {
										this->aStarPlanner_->setRiskMap(this->riskMap2D_);
									} catch (const std::exception& e) {
										ROS_ERROR("[MPC-A*-CALL] Exception in setRiskMap: %s", e.what());
										throw;
									}
								} else {
									ROS_WARN("[MPC-A*-CALL] Risk map not set: riskMap2D_=%p, isValid()=%s", 
									         this->riskMap2D_.get(), 
									         (this->riskMap2D_ && this->riskMap2D_->isValid()) ? "true" : "false");
								}
								
								// 设置动态障碍物方框（硬约束）
								std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
								Eigen::Vector3d robotSize;
								try {
									this->map_->getRobotSize(robotSize);
									this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize, robotSize);
									if (!obstaclesPos.empty() && obstaclesPos.size() == obstaclesSize.size()) {
										ROS_WARN("[MPC-A*-CALL] Setting %zu dynamic obstacle boxes", obstaclesPos.size());
										try {
											this->aStarPlanner_->setDynamicObstacleBoxes(obstaclesPos, obstaclesSize);
											ROS_WARN("[MPC-A*-CALL] setDynamicObstacleBoxes completed");
										} catch (const std::exception& e) {
											ROS_ERROR("[MPC-A*-CALL] Exception in setDynamicObstacleBoxes: %s", e.what());
											// 继续执行，不抛出异常
										} catch (...) {
											ROS_ERROR("[MPC-A*-CALL] Unknown exception in setDynamicObstacleBoxes");
											// 继续执行，不抛出异常
										}
									} else {
										ROS_WARN("[MPC-A*-CALL] No dynamic obstacles or size mismatch (pos=%zu, size=%zu)",
										         obstaclesPos.size(), obstaclesSize.size());
									}
								} catch (const std::exception& e) {
									ROS_ERROR("[MPC-A*-CALL] Exception in getDynamicObstacles: %s", e.what());
									// 继续执行，不抛出异常
								} catch (...) {
									ROS_ERROR("[MPC-A*-CALL] Unknown exception in getDynamicObstacles");
									// 继续执行，不抛出异常
								}
								ROS_WARN("[MPC-A*-CALL] Before makePlan call");
								try {
									this->aStarPlanner_->makePlan(rrtPathMsgTemp);
									ROS_WARN("[MPC-A*-CALL] makePlan completed, returned %zu waypoints", rrtPathMsgTemp.poses.size());
									
									// 检查路径是否有效
									if (rrtPathMsgTemp.poses.empty()) {
										ROS_ERROR("[MPC-A*-CALL] A* returned empty path! This may cause issues.");
										// 不抛出异常，继续执行，让系统尝试其他方法
									}
								} catch (const std::exception& e) {
									ROS_ERROR("[MPC-A*-CALL] Exception in makePlan: %s", e.what());
									ROS_ERROR("[MPC-A*-CALL] Continuing without throwing to prevent crash...");
									// 不抛出异常，避免整个系统崩溃
									rrtPathMsgTemp.poses.clear();  // 清空路径，让系统知道规划失败
								} catch (...) {
									ROS_ERROR("[MPC-A*-CALL] Unknown exception in makePlan");
									ROS_ERROR("[MPC-A*-CALL] Continuing without throwing to prevent crash...");
									// 不抛出异常，避免整个系统崩溃
									rrtPathMsgTemp.poses.clear();  // 清空路径，让系统知道规划失败
								}
								ROS_INFO("[MPC] A* returned %zu waypoints (RAW, before smoothing)", rrtPathMsgTemp.poses.size());
							}
							// SIPP 已删除
							else if (this->rrtPlanner_){
								this->rrtPlanner_->updateStart(this->odom_.pose.pose);
								this->rrtPlanner_->updateGoal(this->goal_.pose);
								this->rrtPlanner_->makePlan(rrtPathMsgTemp);
							}

							// 记录全局规划得到的路径长度，便于调试
							ROS_INFO("[MPC] Global planner (%s) returned path with %zu poses (RAW A* output).",
							         this->globalPlannerType_.c_str(),
							         rrtPathMsgTemp.poses.size());

							// 若路径点过少（<2），认为本次全局规划失败，直接跳过多项式/MPC更新，防止后续崩溃
							if (rrtPathMsgTemp.poses.size() < 2){
								ROS_WARN("[MPC] Global planner path has less than 2 points. Skip polyTraj/MPC update this cycle.");
								this->refTrajReady_ = false;
								this->mpcFirstTime_ = true;
								// 不更新 rrtPathMsg_，保留上一次的可视化路径
							}
							else{
								// 更新可视化用的 RRT/A* 路径
								{
									std::lock_guard<std::mutex> lock(this->rrtPathMutex_);
									this->rrtPathMsg_ = rrtPathMsgTemp;
								}

								// 只有在路径合法时，才进行 polyTraj 与 MPC 路径更新
							Eigen::Vector3d startVel (0, 0, 0);
							Eigen::Vector3d startAcc (0, 0, 0);
							Eigen::Vector3d endVel (0, 0, 0);
							Eigen::Vector3d endAcc (0, 0, 0);
							std::vector<Eigen::Vector3d> startEndConditions {startVel, startAcc, endVel, endAcc};

							ROS_INFO("[MPC] A* raw path will be smoothed by polyTraj (polynomial trajectory planner)");
							this->polyTraj_->updatePath(rrtPathMsgTemp, startEndConditions);
							this->polyTraj_->makePlan(this->polyTrajMsg_); // include corridor constraint		
							
							
							double dt = 0.1;
							nav_msgs::Path mpcInputTraj = this->polyTraj_->getTrajectory(dt);
							ROS_INFO("[MPC] After polyTraj smoothing: %zu waypoints (smooth trajectory)", mpcInputTraj.poses.size());
							this->mpc_->updatePath(mpcInputTraj,dt);
							this->inputTrajMsg_ = mpcInputTraj;
							this->refTrajReady_ = true;
							this->mpcFirstTime_ = true;
							this->trackingStartTime_ = ros::Time::now();
							}
						}
						else{
							nav_msgs::Path waypoints, polyTrajTemp;
							geometry_msgs::PoseStamped start, goal;
							start.pose = this->odom_.pose.pose; goal = this->goal_;
							waypoints.poses = std::vector<geometry_msgs::PoseStamped> {start, goal};					
							
							Eigen::Vector3d startVel (0, 0, 0);
							Eigen::Vector3d startAcc (0, 0, 0);
							Eigen::Vector3d endVel (0, 0, 0);
							Eigen::Vector3d endAcc (0, 0, 0);
							std::vector<Eigen::Vector3d> startEndConditions {startVel, startAcc, endVel, endAcc};

							this->polyTraj_->updatePath(waypoints, startEndConditions);
							this->polyTraj_->makePlan(this->polyTrajMsg_); // include corridor constraint

							double dt = 0.1;
							nav_msgs::Path mpcInputTraj = this->polyTraj_->getTrajectory(dt);
							this->mpc_->updatePath(mpcInputTraj,dt);
							this->inputTrajMsg_ = mpcInputTraj;
							this->refTrajReady_ = true;
							this->mpcFirstTime_ = true;
							this->trackingStartTime_ = ros::Time::now();
						}
					}
				}
				else if (this->refTrajReady_){
					Eigen::Vector3d currPos = this->currPos_;
					Eigen::Vector3d currVel = this->currVel_;
	
					this->mpc_->updateCurrStates(currPos, currVel);
					if (this->usePredictor_){
						std::vector<std::vector<std::vector<Eigen::Vector3d>>> predPos, predSize;
						std::vector<Eigen::VectorXd> intentProb;
						this->predictor_->getPrediction(predPos, predSize, intentProb);
						this->mpc_->updatePredObstacles(predPos, predSize, intentProb);
					}
					else{
						std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
						if (this->useFakeDetector_){
							Eigen::Vector3d robotSize;
							this->map_->getRobotSize(robotSize);
							this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize, robotSize);
						}
						else{ 
							this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
						}
						this->mpc_->updateDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
					}

				ros::Time trajStartTime = ros::Time::now();
				bool newTrajReturn;
				if (this->usePredictor_){
					// makePlan with predictor
					newTrajReturn = this->mpc_->makePlanWithPred();
				}
				else{
					newTrajReturn = this->mpc_->makePlan();
				}
				
				// 发布 MPC 求解状态（仅发布，不改变逻辑）
				std_msgs::Bool mpcStatusMsg;
				mpcStatusMsg.data = !newTrajReturn;  // true = infeasible, false = success
				this->mpcStatusPub_.publish(mpcStatusMsg);
				
				nav_msgs::Path mpcTraj;	
				
				if (newTrajReturn){
						this->trajStartTime_ = trajStartTime;
						if (this->mpcHasCollision() or this->hasDynamicCollision()){
							this->mpcTrajectoryReady_ = false;
							this->stop();
						}
						else{
							this->mpc_->getTrajectory(mpcTraj);
							this->mpcTrajMsg_ = mpcTraj;
							this->mpcTrajectoryReady_ = true;
							this->mpcFirstTime_ = false;
						}
					}
					else if (not this->mpcFirstTime_){
						if (this->mpcHasCollision() or this->hasDynamicCollision()){
							this->mpcTrajectoryReady_ = false;
							this->stop();
						}
						else{
							this->mpcTrajectoryReady_ = true;
						}
					}
					else{
						this->mpcTrajectoryReady_ = false;
						this->stop();	
					}
				}
			}
			this->replanning_ = false;
			r.sleep();
		}
	}

	void mpcNavigation::replanCheckCB(const ros::TimerEvent&){
		if (this->usePredefinedGoal_){
			if (not this->refTrajReady_ and this->predefinedGoal_.poses.size()>0){
				this->mpcReplan_ = false;
				this->refTrajReady_ = false;
				this->goal_ = this->predefinedGoal_.poses.back();
				this->mpcReplan_ = true;
				return;
			}
		}
		else if (this->goalReceived_){
			this->mpcReplan_ = false;
			this->mpcTrajectoryReady_ = false;
			ros::Rate r(200);
			while(ros::ok() and this->replanning_){
				r.sleep();
			}
			this->mpcTrajectoryReady_ = false;
			if(this->goalHasCollision()){
				this->refTrajReady_ = false;
				this->goalReceived_ = false;
				cout << "[AutoFlight]: Invalid goal position, please assign a new goal." << endl; 
				return;
			}
			else{
				this->refTrajReady_ = false;
				if (not this->noYawTurning_ ){
					double yaw = atan2(this->goal_.pose.position.y - this->odom_.pose.pose.position.y, this->goal_.pose.position.x - this->odom_.pose.pose.position.x);
					this->facingYaw_ = yaw;
					this->moveToOrientation(yaw, this->desiredAngularVel_);
				}
				this->firstTimeSave_ = true;
				this->mpcReplan_ = true;
				this->goalReceived_ = false;
				if (this->useGlobalPlanner_){
					cout << "[AutoFlight]: Start global planning." << endl;
				}

				cout << "[AutoFlight]: Replan for new goal position." << endl; 
				return;
			}
		}
		if (this->mpcTrajectoryReady_){
			// realtime replan: time-based & deviation-based (only when using global planner)
			if (this->enableRealtimeReplan_ && this->useGlobalPlanner_ && this->refTrajReady_){
				ros::Time currTime = ros::Time::now();
				double timeSinceLast = (currTime - this->lastGlobalReplanTime_).toSec();
				bool timeTriggered = timeSinceLast >= this->globalReplanInterval_;

				// compute min distance from current pos to path segments
				bool deviationTriggered = false;
				nav_msgs::Path pathCopy;
				{
					std::lock_guard<std::mutex> lock(this->rrtPathMutex_);
					pathCopy = this->rrtPathMsg_;
				}
				if (pathCopy.poses.size() >= 2){
					Eigen::Vector3d currPos(this->odom_.pose.pose.position.x,
											this->odom_.pose.pose.position.y,
											this->odom_.pose.pose.position.z);
					double minDist = std::numeric_limits<double>::max();
					for (size_t i=0; i+1<pathCopy.poses.size(); ++i){
						Eigen::Vector3d p1(pathCopy.poses[i].pose.position.x,
										   pathCopy.poses[i].pose.position.y,
										   pathCopy.poses[i].pose.position.z);
						Eigen::Vector3d p2(pathCopy.poses[i+1].pose.position.x,
										   pathCopy.poses[i+1].pose.position.y,
										   pathCopy.poses[i+1].pose.position.z);
						Eigen::Vector3d v = p2 - p1;
						Eigen::Vector3d w = currPos - p1;
						double c1 = w.dot(v);
						if (c1 <= 0){
							minDist = std::min(minDist, (currPos - p1).norm());
						}else{
							double c2 = v.dot(v);
							if (c2 <= c1){
								minDist = std::min(minDist, (currPos - p2).norm());
							}else{
								double b = c1 / c2;
								Eigen::Vector3d pb = p1 + b*v;
								minDist = std::min(minDist, (currPos - pb).norm());
							}
						}
					}
					deviationTriggered = minDist > this->pathDeviationThreshold_;
				}

				if (timeTriggered || deviationTriggered){
					this->mpcTrajectoryReady_ = false;
					this->refTrajReady_ = false;
					this->mpcReplan_ = true;
					this->lastGlobalReplanTime_ = currTime;
					if (timeTriggered){
						cout << "[AutoFlight]: Time-based realtime replan (interval " << timeSinceLast << " s)." << endl;
					}
					if (deviationTriggered){
						cout << "[AutoFlight]: Path deviation realtime replan." << endl;
					}
					return;
				}
			}

			if (this->usePredefinedGoal_){
				ros::Time currTime = ros::Time::now();
				if (this->mpcHasCollision() or this->hasDynamicCollision()){ 
					this->stop();
					this->mpcTrajectoryReady_ = false;
					this->mpcReplan_ = true;
					cout << "[AutoFlight]: Collision detected. MPC replan." << endl;
					return;
				}
				else if (AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) <= 0.3 and 
					(currTime-this->trackingStartTime_ ).toSec() >= 3){
					if (this->repeatPathNum_ == 0){
						this->mpcReplan_ = false;
						this->mpcTrajectoryReady_ = false;
						ros::Rate r(200);
						while(ros::ok() and this->replanning_){
							r.sleep();
						}
						this->mpcTrajectoryReady_ = false;
						this->stop();
						this->predefinedGoal_.poses.clear();
						this->refTrajReady_ = false;
						cout << "[AutoFlight]: Goal reached. MPC Stop replan." << endl;
					}
					else{
						double dt = 0.1;
						nav_msgs::Path mpcInputTraj = this->predefinedGoal_;
						this->mpc_->updatePath(mpcInputTraj, dt);
						this->inputTrajMsg_ = mpcInputTraj;
						this->mpcFirstTime_ = true;
						if (this->repeatPathNum_ > 1){
							cout << "[AutoFlight]: Goal reached. " << this->repeatPathNum_ << " rounds left." << endl;
						}
						else{
							cout << "[AutoFlight]: Goal reached. " << this->repeatPathNum_ << " round left." << endl;
						}
						this->repeatPathNum_ -= 1;
						this->refTrajReady_ = true;
						this->trackingStartTime_ = ros::Time::now();
						this->mpcReplan_ = true;
					}
					return;
				}

			}
			else{					
				if (this->goalHasCollision()){
					this->mpcReplan_ = false;
					this->mpcTrajectoryReady_ = false;
					ros::Rate r(200);
					while(ros::ok() and this->replanning_){
						r.sleep();
					}
					this->mpcTrajectoryReady_ = false;
					this->stop();
					this->refTrajReady_ = false;
					this->mpcFirstTime_ = true;
					cout<<"[AutoFlight]: Invalid goal. Stop!" << endl;
					return;
				}
				else if (this->mpcHasCollision() or this->hasDynamicCollision()){ 
					this->stop();
					this->mpcTrajectoryReady_ = false;
					this->mpcReplan_ = true;
					cout << "[AutoFlight]: Collision detected. MPC replan." << endl;
					return;
				}
				else if(AutoFlight::getPoseDistance(this->odom_.pose.pose, this->goal_.pose) <= 0.3){
					this->mpcReplan_ = false;
					this->mpcTrajectoryReady_ = false;
					ros::Rate r(200);
					while(ros::ok() and this->replanning_){
						r.sleep();
					}
					this->stop();
					this->refTrajReady_ = false;
					this->mpcTrajectoryReady_ = false;
					this->mpcFirstTime_ = true;
					cout << "[AutoFlight]: Goal reached. MPC Stop replan." << endl;
					return;
				}
			}
		}
	}

	void mpcNavigation::trajExeCB(const ros::TimerEvent&){
		bool trajectoryReady;
		trajectoryReady = this->mpcTrajectoryReady_;
		if (trajectoryReady){
			ros::Time currTime = ros::Time::now();
			double realTime = (currTime - this->trajStartTime_).toSec();
			Eigen::Vector3d pos, vel, acc;
			Eigen::Vector3d refPos;
			double endTime;
			endTime = this->mpc_->getHorizon() * this->mpc_->getTs();
			pos = this->mpc_->getPos(realTime);
			vel = this->mpc_->getVel(realTime);
			acc = this->mpc_->getAcc(realTime);
			refPos = this->mpc_->getRef(realTime);

			double leftTime = endTime - realTime; 
			tracking_controller::Target target;
			if (leftTime <= 0.0){ // zero vel and zero acc if close to
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = 0.0;
				target.velocity.y = 0.0;
				target.velocity.z = 0.0;
				target.acceleration.x = 0.0;
				target.acceleration.y = 0.0;
				target.acceleration.z = 0.0;
				target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				this->updateTargetWithState(target);						
			}
			else{
				if (not this->useYawControl_){
					target.yaw = this->facingYaw_;
				}
				else if (this->noYawTurning_){
					target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
				}
				else{
					// smoothing yaw angle
					double forwardDist = 1.0;
					double dt = this->mpc_->getTs();
					bool noYawChange = true;
					for (double t=realTime; t<=endTime; t+=dt){
						// Eigen::Vector3d p = this->mpc_->getPos(t);
						Eigen::Vector3d p = this->mpc_->getRef(t);
						if ((p - refPos).norm() >= forwardDist){
							target.yaw = atan2(p(1) - refPos(1), p(0) - refPos(0));
							noYawChange = false;
							break;
						}
					}

					if (noYawChange){
						target.yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
					}
				}				
				target.position.x = pos(0);
				target.position.y = pos(1);
				target.position.z = pos(2);
				target.velocity.x = vel(0);
				target.velocity.y = vel(1);
				target.velocity.z = vel(2);
				target.acceleration.x = acc(0);
				target.acceleration.y = acc(1);
				target.acceleration.z = acc(2);
				this->updateTargetWithState(target);						
			}
		}
	}

	void mpcNavigation::visCB(const ros::TimerEvent&){
		if (this->rrtPathMsg_.poses.size() != 0){
			this->rrtPathPub_.publish(this->rrtPathMsg_);
		}
		if (this->polyTrajMsg_.poses.size() != 0){
			this->polyTrajPub_.publish(this->polyTrajMsg_);
		}
		if (this->pwlTrajMsg_.poses.size() != 0){
			this->pwlTrajPub_.publish(this->pwlTrajMsg_);
		}
		if (this->mpcTrajMsg_.poses.size() != 0){
			this->mpcTrajPub_.publish(this->mpcTrajMsg_);
		}
		if (this->inputTrajMsg_.poses.size() != 0){
			this->inputTrajPub_.publish(this->inputTrajMsg_);
		}
		this->publishGoal();
	}

	void mpcNavigation::freeMapCB(const ros::TimerEvent&){
		std::vector<onboardDetector::box3D> obstacles;
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
		this->detector_->getObstacles(obstacles);
		// 正常相机FOV：75度 (约1.31弧度)，介于60-90度之间
		double fov = 75.0 * M_PI / 180.0;  // 约1.30899694
		for (onboardDetector::box3D ob: obstacles){
			if (this->detector_->isObstacleInSensorRange(ob, fov)){
				Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z-ob.z_width/2-0.3);
				Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width/2+0.3);
				freeRegions.push_back(std::make_pair(lowerBound, upperBound));
			}
		}
		this->map_->updateFreeRegions(freeRegions);
		this->map_->freeRegions(freeRegions);
	}


// 这个是原本的run函数，不知道为什么在跑圆圈，有可能是重写了，现在我要它停止
	void mpcNavigation::run(){
		// take off the drone
		this->takeoff();

		// register timer callback
		this->registerCallback();
	}


// 	void mpcNavigation::run(){
//     // 起飞
//     this->takeoff();

//     // 改为：起飞后直接停止，不执行圆形轨迹
//     this->stop();
    
//     cout << "[AutoFlight]: Drone is holding position after takeoff." << endl;

//     // 注册回调函数（保持其他功能）
//     this->registerCallback();
    
// }

	bool mpcNavigation::goalHasCollision(){
		Eigen::Vector3d p;
		double r = 0.5;//radius for goal collision check
		for (double i=-r; i<=r;i+=0.1){
			for(double j=-r;j<=r;j+=0.1){
				for (double k = -r; k<=r; k+=0.1){
					p(0) = this->goal_.pose.position.x+i;
					p(1) = this->goal_.pose.position.y+j;
					p(2) = this->goal_.pose.position.z+k;
					if (this->map_->isInflatedOccupied(p)){
						return true;

					}
				}
			}
		}
		return false;
	}

	bool mpcNavigation::mpcHasCollision(){
		if (this->mpcTrajectoryReady_){
			double dt = this->mpc_->getTs();
			ros::Time currTime = ros::Time::now();
			double startTime = std::min(1.0, (currTime-this->trajStartTime_).toSec());
			double endTime = std::min(startTime+2.0, this->mpc_->getHorizon()*dt);
			for (double t=startTime; t<=endTime; t+=dt){
				Eigen::Vector3d p = this->mpc_->getPos(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					cout<<"[AutoFlight]: MPC collision detected!"<<endl;
					return true;
				}
			}
		}
		return false;
	}

	bool mpcNavigation::hasCollision(){
		bool trajectoryReady;
		trajectoryReady = this->mpcTrajectoryReady_;
		if (trajectoryReady){
			double dt = this->mpc_->getTs();
			ros::Time currTime = ros::Time::now();
			double startTime = std::min(1.0, (currTime-this->trajStartTime_).toSec());
			double endTime = std::min(startTime+2.0, this->mpc_->getHorizon()*dt);
			for (double t=startTime; t<=endTime; t+=dt){
				Eigen::Vector3d p = this->mpc_->getPos(t);
				bool hasCollision = this->map_->isInflatedOccupied(p);
				if (hasCollision){
					cout<<"[AutoFlight]: MPC collision detected!"<<endl;
					return true;
				}
			}
		}
		return false;
	}

	bool mpcNavigation::hasDynamicCollision(){
		bool trajectoryReady;
		trajectoryReady = this->mpcTrajectoryReady_;

		if (trajectoryReady){
			std::vector<Eigen::Vector3d> obstaclesPos, obstaclesVel, obstaclesSize;
			if (this->useFakeDetector_){
				Eigen::Vector3d robotSize;
				this->map_->getRobotSize(robotSize);
				this->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize, robotSize);
			}
			else{ 
				this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
			}
			double dt = this->mpc_->getTs();
			ros::Time currTime = ros::Time::now();
			double startTime = std::min(1.0, (currTime-this->trajStartTime_).toSec());
			double endTime = std::min(startTime+1.0, this->mpc_->getHorizon()*dt);
			for (double t=startTime; t<=endTime; t+=dt){
				Eigen::Vector3d p = this->mpc_->getPos(t);
				for (size_t i=0; i<obstaclesPos.size(); ++i){
					Eigen::Vector3d ob = obstaclesPos[i];
					Eigen::Vector3d size = obstaclesSize[i];
					Eigen::Vector3d lowerBound = ob - size/2;
					Eigen::Vector3d upperBound = ob + size/2;
					if (p(0) >= lowerBound(0) and p(0) <= upperBound(0) and
						p(1) >= lowerBound(1) and p(1) <= upperBound(1) and
						p(2) >= lowerBound(2) and p(2) <= upperBound(2)){
						return true;
					}					
				}
			}
		}
		return false;
	}

	void mpcNavigation::getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize, const Eigen::Vector3d &robotSize){
		std::vector<onboardDetector::box3D> obstacles;
		if (this->useFakeDetector_){
		// 正常相机FOV：75度 (约1.31弧度)，只获取视野内的障碍物
		double camera_fov = 75.0 * M_PI / 180.0;  // 约1.30899694
		this->detector_->getObstaclesInSensorRange(camera_fov, obstacles, robotSize);
		for (onboardDetector::box3D ob : obstacles){
			Eigen::Vector3d pos (ob.x, ob.y, ob.z);
			Eigen::Vector3d vel (ob.Vx, ob.Vy, 0.0);
			Eigen::Vector3d size (ob.x_width, ob.y_width, ob.z_width);
			obstaclesPos.push_back(pos);
			obstaclesVel.push_back(vel);
			obstaclesSize.push_back(size);
			}
		}
		else{ 
			this->map_->getDynamicObstacles(obstaclesPos, obstaclesVel, obstaclesSize);
		}
	}

	void mpcNavigation::publishGoal(){
		nav_msgs::Path goal;
		goal.poses = std::vector<geometry_msgs::PoseStamped> {this->goal_};

		if (goal.poses.size() != 0){
			visualization_msgs::MarkerArray msg;
			std::vector<visualization_msgs::Marker> pointVec;
			visualization_msgs::Marker point;
			int pointCount = 0;
			for (int i=0; i<int(goal.poses.size()); ++i){
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "input_traj_points";
				point.id = pointCount;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = goal.poses[i].pose.position.x;
				point.pose.position.y = goal.poses[i].pose.position.y;
				point.pose.position.z = goal.poses[i].pose.position.z;
				point.lifetime = ros::Duration(0.5);
				point.scale.x = 0.3;
				point.scale.y = 0.3;
				point.scale.z = 0.3;
				point.color.a = 1.0;
				point.color.r = 1;
				point.color.g = 0;
				point.color.b = 0;
				pointVec.push_back(point);
				++pointCount;			
			}
			msg.markers = pointVec;	
			this->goalPub_.publish(msg);
		}
	}

	void mpcNavigation::riskMapCB(const nav_msgs::OccupancyGridConstPtr& msg){
		// ROS_WARN("[MPC-RISK-CB] riskMapCB called: riskMap2D_=%p, aStarPlanner_=%p, msg->width=%u, msg->height=%u",
		//          this->riskMap2D_.get(), this->aStarPlanner_.get(), msg->info.width, msg->info.height);
		
		// 更新旧版2D风险地图（向后兼容）
		if (this->riskMap2D_) {
			// ROS_WARN("[MPC-RISK-CB] Calling updateFromMsg...");
			try {
				this->riskMap2D_->updateFromMsg(*msg);
				// ROS_WARN("[MPC-RISK-CB] updateFromMsg completed");
			} catch (const std::exception& e) {
				ROS_ERROR("[MPC-RISK-CB] Exception in updateFromMsg: %s", e.what());
				return;
			} catch (...) {
				ROS_ERROR("[MPC-RISK-CB] Unknown exception in updateFromMsg");
				return;
			}
			
			// 如果 A* 规划器已初始化，立即更新风险地图
			if (this->aStarPlanner_) {
				// ROS_WARN("[MPC-RISK-CB] Calling aStarPlanner_->setRiskMap...");
				try {
					this->aStarPlanner_->setRiskMap(this->riskMap2D_);
					// ROS_WARN("[MPC-RISK-CB] setRiskMap completed");
				} catch (const std::exception& e) {
					ROS_ERROR("[MPC-RISK-CB] Exception in setRiskMap: %s", e.what());
				} catch (...) {
					ROS_ERROR("[MPC-RISK-CB] Unknown exception in setRiskMap");
				}
			} else {
				// ROS_WARN("[MPC-RISK-CB] aStarPlanner_ is NULL, skipping setRiskMap");
			}
		} else {
			// ROS_WARN("[MPC-RISK-CB] riskMap2D_ is NULL, skipping update");
		}
		
		// 🔧 更新新版2.5D风险地图的动态部分
		if (this->riskMap25D_) {
			try {
				this->riskMap25D_->updateFromDynamicMsg(*msg);
				ROS_INFO_THROTTLE(10.0, "[MPC-RISK-CB] RiskMap25D dynamic risk updated");
				
				// 发布风险地图用于可视化
				publishRiskMap25D();
			} catch (const std::exception& e) {
				ROS_ERROR_THROTTLE(5.0, "[MPC-RISK-CB] Exception updating RiskMap25D: %s", e.what());
			}
		}
		
		// ROS_WARN("[MPC-RISK-CB] riskMapCB completed");
	}

	void mpcNavigation::updateRiskMap25DCB(const ros::TimerEvent&){
		if (!this->riskMap25D_ || !this->map_) {
			return;
		}

		// 1. 设置地图中心（机器人当前位置）
		this->riskMap25D_->setMapCenter(this->currPos_);
		
		// 注意：
		// - 静态风险从occupancy map计算（通过getDistance接口）
		// - 动态风险通过订阅 dynamic_predictor/dynamic_risk_map 更新（在riskMapCB中）
		// 这里我们将静态风险计算集成到riskMapCB中，与动态风险一起更新
	}

	void mpcNavigation::publishRiskMap25D(){
		if (!this->riskMap25D_ || !this->riskMap25D_->isValid()) {
			return;
		}

		// 导出栅格数据
		std::vector<double> grid_data;
		if (!this->riskMap25D_->exportGridData(grid_data)) {
			ROS_WARN_THROTTLE(5.0, "[MPC-Nav] Failed to export RiskMap25D grid data");
			return;
		}

		// 创建OccupancyGrid消息
		nav_msgs::OccupancyGrid grid_msg;
		grid_msg.header.stamp = ros::Time::now();
		grid_msg.header.frame_id = "map";

		// 获取栅格信息
		double resolution;
		int width, height;
		Eigen::Vector2d origin;
		this->riskMap25D_->getGridInfo(resolution, width, height, origin);

		grid_msg.info.resolution = resolution;
		grid_msg.info.width = width;
		grid_msg.info.height = height;
		grid_msg.info.origin.position.x = origin(0);
		grid_msg.info.origin.position.y = origin(1);
		grid_msg.info.origin.position.z = 0.0;
		grid_msg.info.origin.orientation.w = 1.0;

		// 转换数据：double (0-inf) -> int8 (0-100)
		grid_msg.data.resize(grid_data.size());
		
		// 找到最大风险值用于归一化
		double max_risk = 0.0;
		for (double risk : grid_data) {
			if (risk > max_risk) max_risk = risk;
		}

		if (max_risk > 1e-8) {
			for (size_t i = 0; i < grid_data.size(); ++i) {
				double norm = grid_data[i] / max_risk;  // 0~1
				grid_msg.data[i] = static_cast<int8_t>(std::min(100, static_cast<int>(norm * 100.0)));
			}
		} else {
			// 全是0
			std::fill(grid_msg.data.begin(), grid_msg.data.end(), 0);
		}

		// 发布
		this->riskMap25DPub_.publish(grid_msg);
		
		ROS_INFO_THROTTLE(10.0, "[MPC-Nav] Published RiskMap25D (max_risk=%.4f)", max_risk);
	}
}