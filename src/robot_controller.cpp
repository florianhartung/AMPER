#include "ros/ros.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_controller");

	ROS_INFO("Hello world from controller");

	ros::spin();
	return 0;
}
