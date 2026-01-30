/*
	FILE: mpcNavigation.h
	------------------------
	dynamic navigation header file
*/

#ifndef AUTOFLIGHT_MPC_NAVIGATION_H
#define AUTOFLIGHT_MPC_NAVIGATION_H

#include <ros/package.h>
#include <autonomous_flight/flightBase.h>
#include <map_manager/dynamicMap.h>
#include <onboard_detector/fakeDetector.h>
#include <dynamic_predictor/dynamicPredictor.h>
#include <global_planner/rrtOccMap.h>
#include <global_planner/a_star_occ.h>
// #include <global_planner/sipp_occ_map.h>  // SIPP已删除
#include <global_planner/risk_map_2d.h>
#include <global_planner/risk_map_25d.h>  // 🔧 新增：2.5D风险地图
#include <nav_msgs/OccupancyGrid.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/piecewiseLinearTraj.h>
#include <trajectory_planner/bsplineTraj.h>
#include <trajectory_planner/mpcPlanner.h>
#include <std_msgs/Bool.h>
#include <mutex>

namespace AutoFlight{

	class mpcNavigation : public flightBase{
	private:
		std::shared_ptr<mapManager::dynamicMap> map_;
		std::shared_ptr<onboardDetector::fakeDetector> detector_;  // only used when useFakeDetector_=true
		std::shared_ptr<dynamicPredictor::predictor> predictor_;
		std::shared_ptr<globalPlanner::rrtOccMap<3>> rrtPlanner_;
		std::shared_ptr<globalPlanner::AStarOccMap> aStarPlanner_;
		// std::shared_ptr<globalPlanner::SippOccMap> sippPlanner_;  // SIPP已删除
		std::shared_ptr<globalPlanner::RiskMap2D> riskMap2D_;    // 风险地图（旧版2D）
		std::shared_ptr<globalPlanner::RiskMap25D> riskMap25D_;  // 🔧 风险地图（新版2.5D）
		bool use_risk_map_25d_{true};  // 🔧 默认使用2.5D风险地图
		std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj_;
		std::shared_ptr<trajPlanner::pwlTraj> pwlTraj_;
		std::shared_ptr<trajPlanner::mpcPlanner> mpc_;

		ros::Timer replanCheckTimer_;
		ros::Timer trajExeTimer_;
		ros::Timer visTimer_;
		ros::Timer freeMapTimer_;

	ros::Publisher rrtPathPub_;
	ros::Publisher polyTrajPub_;
	ros::Publisher pwlTrajPub_;
	ros::Publisher mpcTrajPub_;
	ros::Publisher inputTrajPub_;
	ros::Publisher goalPub_;
	ros::Publisher mpcStatusPub_;  // 发布 MPC 求解状态（用于数据记录）
	
	ros::Subscriber riskMapSub_;  // 订阅风险地图

		std::thread mpcWorker_;

		// parameters
		bool useFakeDetector_;
		bool usePredictor_;
		bool useGlobalPlanner_;
		std::string globalPlannerType_ = "rrt"; // rrt / astar (sipp已删除)
		bool noYawTurning_;
		bool useYawControl_;
		bool usePredefinedGoal_;
		double desiredVel_;
		double desiredAcc_;
		double desiredAngularVel_;
		std::string refTrajPath_;
		nav_msgs::Path predefinedGoal_;
		int goalIdx_ = 0;
		int repeatPathNum_;

		// navigation data
		bool mpcReplan_ = false;
		bool replanning_ = false;
		bool needGlobalPlan_ = false;
		bool globalPlanReady_ = false;
		bool refTrajReady_ = false;
		bool mpcFirstTime_ = false;
		// realtime replanning params
		bool enableRealtimeReplan_ = false;
		double globalReplanInterval_ = 2.0;   // seconds
		double pathDeviationThreshold_ = 0.5; // meters
		ros::Time lastGlobalReplanTime_;

		nav_msgs::Path rrtPathMsg_;
		std::mutex rrtPathMutex_; // protect rrtPathMsg_ access across threads
		nav_msgs::Path polyTrajMsg_;
		nav_msgs::Path pwlTrajMsg_;
		nav_msgs::Path mpcTrajMsg_;
		nav_msgs::Path inputTrajMsg_;
		bool mpcTrajectoryReady_ = false;
		ros::Time trajStartTime_;
		ros::Time trackingStartTime_;
		double trajTime_; // current trajectory time
		double prevInputTrajTime_ = 0.0;
		trajPlanner::bspline trajectory_; // trajectory data for tracking
		double facingYaw_;
		bool firstTimeSave_ = false;
		bool lastDynamicObstacle_ = false;
		ros::Time lastDynamicObstacleTime_;
		
	public:
		mpcNavigation(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void riskMapCB(const nav_msgs::OccupancyGridConstPtr& msg);  // 风险地图回调

		void mpcCB();
		void staticPlannerCB(const ros::TimerEvent&);
		void replanCheckCB(const ros::TimerEvent&);
		void trajExeCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void freeMapCB(const ros::TimerEvent&); // using fake detector

		void run();	
		bool goalHasCollision();
		bool mpcHasCollision();
		bool hasCollision();
		bool hasDynamicCollision();
		nav_msgs::Path getRefTraj();
		nav_msgs::Path getCurrentTraj(double dt);
		nav_msgs::Path getRestGlobalPath();
		void getDynamicObstacles(std::vector<Eigen::Vector3d>& obstaclesPos, std::vector<Eigen::Vector3d>& obstaclesVel, std::vector<Eigen::Vector3d>& obstaclesSize, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0, 0.0, 0.0));
		void publishGoal();
	};
}

#endif