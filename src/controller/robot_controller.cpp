#include "ros/ros.h"
#include "path_finding.hpp"


const std::tuple<size_t, size_t> START_POS(0, 0);
const std::tuple<size_t, size_t> END_POS(1, 1);


int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_controller");
	ROS_INFO("Started controller");


	const Path path = findPath(LABYRINTH, START_POS, END_POS);


	ros::spin();
	return 0;
}
