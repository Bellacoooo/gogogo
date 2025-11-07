#include <ros/ros.h>
#include <autonomous_flight/mpcNavigation.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "mpc_navigation_node");
	ros::NodeHandle nh;

	AutoFlight::mpcNavigation navigator (nh);
	navigator.run();
	ros::spin();
	return 0;
}