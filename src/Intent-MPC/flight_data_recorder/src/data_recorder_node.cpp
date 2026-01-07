/*
    FILE: data_recorder_node.cpp
    -------------------------------
    Record UAV flight data for MPC navigation experiments
    Records: time, UAV position, min distance to obstacles, collision flag, infeasible flag, path length
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <onboard_detector/GetDynamicObstacles.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>

class DataRecorder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odomSub_;
    ros::Subscriber mpcInfeasibleSub_;
    ros::ServiceClient obstacleClient_;
    ros::Timer recordTimer_;
    
    // Data storage
    std::ofstream csvFile_;
    std::string csvFilePath_;
    
    // UAV state
    bool odomReceived_;
    Eigen::Vector3d currentPos_;
    Eigen::Vector3d previousPos_;
    double pathLength_;
    
    // Obstacle data
    std::vector<Eigen::Vector3d> obstaclePositions_;
    std::vector<Eigen::Vector3d> obstacleSizes_;
    
    // Flags
    bool mpcInfeasible_;
    bool collisionDetected_;
    
    // Parameters
    double collisionThreshold_;  // meters, distance < threshold means collision
    double recordRate_;  // Hz
    double detectionRange_;  // meters, range to query obstacles
    bool recordingStarted_;
    double startTime_;
    
public:
    DataRecorder(const ros::NodeHandle& nh) : nh_(nh), odomReceived_(false), 
        pathLength_(0.0), mpcInfeasible_(false), collisionDetected_(false),
        recordingStarted_(false), startTime_(0.0) {
        
        // Get parameters
        nh_.param("data_recorder/collision_threshold", collisionThreshold_, 0.4);
        nh_.param("data_recorder/record_rate", recordRate_, 10.0);  // 10Hz = 0.1s
        nh_.param("data_recorder/detection_range", detectionRange_, 50.0);
        
        ROS_INFO("[DataRecorder]: Collision threshold: %.2f m", collisionThreshold_);
        ROS_INFO("[DataRecorder]: Record rate: %.1f Hz", recordRate_);
        
        // Setup CSV file
        setupCSVFile();
        
        // Setup subscribers
        odomSub_ = nh_.subscribe("/CERLAB/quadcopter/odom", 10, &DataRecorder::odomCallback, this);
        mpcInfeasibleSub_ = nh_.subscribe("/mpcNavigation/infeasible", 10, &DataRecorder::mpcStatusCallback, this);
        
        // Setup service client for obstacles
        // Actual service name: /fake_detector/getDynamicObstacles (not /fake_detector_node/getDynamicObstacles)
        std::string serviceName = "/fake_detector/getDynamicObstacles";
        obstacleClient_ = nh_.serviceClient<onboard_detector::GetDynamicObstacles>(serviceName);
        
        // Wait for service to be available
        ROS_INFO("[DataRecorder]: Waiting for obstacle service: %s", serviceName.c_str());
        if (ros::service::waitForService(serviceName, ros::Duration(5.0))) {
            ROS_INFO("[DataRecorder]: ✓ Service %s is available!", serviceName.c_str());
        } else {
            ROS_WARN("[DataRecorder]: ✗ Service %s not available after 5s, will keep trying...", serviceName.c_str());
        }
        
        // Setup timer for recording
        recordTimer_ = nh_.createTimer(ros::Duration(1.0 / recordRate_), &DataRecorder::recordCallback, this);
        
        ROS_INFO("[DataRecorder]: Data recorder initialized. Waiting for odometry...");
    }
    
    ~DataRecorder() {
        if (csvFile_.is_open()) {
            csvFile_.close();
            ROS_INFO("[DataRecorder]: CSV file closed: %s", csvFilePath_.c_str());
        }
    }
    
    void setupCSVFile() {
        // Create filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        
        std::string packagePath = ros::package::getPath("flight_data_recorder");
        if (packagePath.empty()) {
            ROS_WARN("[DataRecorder]: Cannot find package path, using /tmp");
            packagePath = "/tmp";
        }
        
        csvFilePath_ = packagePath + "/flight_data_" + ss.str() + ".csv";
        csvFile_.open(csvFilePath_);
        
        if (!csvFile_.is_open()) {
            ROS_ERROR("[DataRecorder]: Failed to open CSV file: %s", csvFilePath_.c_str());
            return;
        }
        
        // Write header
        csvFile_ << "time,uav_x,uav_y,uav_z,dist_min,path_length,infeasible_flag,collision_flag\n";
        csvFile_.flush();
        
        ROS_INFO("[DataRecorder]: CSV file created: %s", csvFilePath_.c_str());
    }
    
    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        if (!odomReceived_) {
            previousPos_ = Eigen::Vector3d(msg->pose.pose.position.x, 
                                          msg->pose.pose.position.y, 
                                          msg->pose.pose.position.z);
            odomReceived_ = true;
            recordingStarted_ = true;
            startTime_ = ros::Time::now().toSec();
            ROS_INFO("[DataRecorder]: First odometry received. Starting recording...");
        }
        
        currentPos_ = Eigen::Vector3d(msg->pose.pose.position.x, 
                                      msg->pose.pose.position.y, 
                                      msg->pose.pose.position.z);
        
        // Update path length
        Eigen::Vector3d displacement = currentPos_ - previousPos_;
        pathLength_ += displacement.norm();
        previousPos_ = currentPos_;
    }
    
    void mpcStatusCallback(const std_msgs::BoolConstPtr& msg) {
        // msg->data = true 表示 infeasible（求解失败）
        // msg->data = false 表示 success（求解成功）
        mpcInfeasible_ = msg->data;
    }
    
    void recordCallback(const ros::TimerEvent&) {
        if (!odomReceived_ || !recordingStarted_) {
            return;
        }
        
        // Query obstacles
        queryObstacles();
        
        // Calculate minimum distance to obstacles
        double minDist = calculateMinDistance();
        
        // Check collision
        collisionDetected_ = (minDist < collisionThreshold_);
        
        // Get current time relative to start
        double currentTime = ros::Time::now().toSec() - startTime_;
        
        // Write data to CSV
        if (csvFile_.is_open()) {
            csvFile_ << std::fixed << std::setprecision(2) << currentTime << ","
                     << std::setprecision(3) << currentPos_.x() << ","
                     << currentPos_.y() << ","
                     << currentPos_.z() << ","
                     << std::setprecision(2) << minDist << ","
                     << pathLength_ << ","
                     << (mpcInfeasible_ ? 1 : 0) << ","
                     << (collisionDetected_ ? 1 : 0) << "\n";
            csvFile_.flush();
        }
        
        // Print status
        if (collisionDetected_) {
            ROS_WARN("[DataRecorder] COLLISION! Min distance: %.2f m", minDist);
        }
    }
    
    void queryObstacles() {
        obstaclePositions_.clear();
        obstacleSizes_.clear();
        
        static int callCount = 0;
        static int successCount = 0;
        static int failCount = 0;
        callCount++;
        
        // 检查服务是否存在
        if (!obstacleClient_.exists()) {
            if (callCount % 25 == 0) {  // 每5秒打印一次（5Hz）
                ROS_ERROR("[DataRecorder-ERROR]: Service does not exist: /fake_detector/getDynamicObstacles");
                ROS_ERROR("[DataRecorder-ERROR]: Please check if fake_detector_node is running!");
            }
            return;
        }
        
        onboard_detector::GetDynamicObstacles srv;
        srv.request.current_position.x = currentPos_.x();
        srv.request.current_position.y = currentPos_.y();
        srv.request.current_position.z = currentPos_.z();
        srv.request.range = detectionRange_;
        
        // 调用服务
        bool callSuccess = obstacleClient_.call(srv);
        
        if (callSuccess) {
            successCount++;
            for (size_t i = 0; i < srv.response.position.size(); ++i) {
                Eigen::Vector3d pos(srv.response.position[i].x,
                                   srv.response.position[i].y,
                                   srv.response.position[i].z);
                Eigen::Vector3d size(srv.response.size[i].x,
                                    srv.response.size[i].y,
                                    srv.response.size[i].z);
                obstaclePositions_.push_back(pos);
                obstacleSizes_.push_back(size);
            }
            
            // 每5秒打印一次状态
            if (callCount % 25 == 0) {
                ROS_INFO("[DataRecorder-STATUS]: Service call SUCCESS! Found %lu obstacles at (%.2f, %.2f, %.2f)", 
                         obstaclePositions_.size(), currentPos_.x(), currentPos_.y(), currentPos_.z());
                if (obstaclePositions_.size() > 0) {
                    ROS_INFO("[DataRecorder-STATUS]: First obstacle at (%.2f, %.2f, %.2f), size=(%.2f, %.2f, %.2f)",
                             obstaclePositions_[0].x(), obstaclePositions_[0].y(), obstaclePositions_[0].z(),
                             obstacleSizes_[0].x(), obstacleSizes_[0].y(), obstacleSizes_[0].z());
                }
            }
        }
        else {
            failCount++;
            if (callCount % 25 == 0) {  // 每5秒打印一次
                ROS_ERROR("[DataRecorder-ERROR]: Service call FAILED! (Success: %d, Failed: %d, Total: %d)", 
                         successCount, failCount, callCount);
                ROS_ERROR("[DataRecorder-ERROR]: Current position: (%.2f, %.2f, %.2f)", 
                         currentPos_.x(), currentPos_.y(), currentPos_.z());
            }
        }
    }
    
    double calculateMinDistance() {
        if (obstaclePositions_.empty()) {
            return 999.0;  // No obstacles nearby
        }
        
        double minDist = 999.0;
        for (size_t i = 0; i < obstaclePositions_.size(); ++i) {
            // Calculate distance from UAV to obstacle bounding box
            Eigen::Vector3d obstaclePos = obstaclePositions_[i];
            Eigen::Vector3d obstacleSize = obstacleSizes_[i];
            
            // Distance to obstacle center (simple approximation)
            Eigen::Vector3d diff = currentPos_ - obstaclePos;
            
            // Subtract half of obstacle size to get distance to surface
            double distToCenter = diff.norm();
            double obstacleRadius = obstacleSize.norm() / 2.0;
            double distToSurface = std::max(0.0, distToCenter - obstacleRadius);
            
            minDist = std::min(minDist, distToSurface);
        }
        
        return minDist;
    }
    
    void printSummary() {
        ROS_INFO("========================================");
        ROS_INFO("[DataRecorder] Flight Summary:");
        ROS_INFO("  Total path length: %.2f m", pathLength_);
        ROS_INFO("  CSV file saved: %s", csvFilePath_.c_str());
        ROS_INFO("========================================");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_recorder_node");
    ros::NodeHandle nh;
    
    DataRecorder recorder(nh);
    
    ros::spin();
    
    recorder.printSummary();
    
    return 0;
}

