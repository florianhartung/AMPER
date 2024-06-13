#include "ros/ros.h"
#include "path_finding.hpp"
#include "const_labyrinth.hpp"
#include "AMPER/MoveAction.h"
#include "actionlib/client/simple_action_client.h"


const std::tuple<size_t, size_t> START_POS(0, 0);
const std::tuple<size_t, size_t> END_POS(1, 1);


int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_controller");
	ROS_INFO("Started controller");

    actionlib::SimpleActionClient<AMPER::MoveAction> client("MoveServer", true);
    client.waitForServer();

    AMPER::MoveGoal goal;
    goal.distance = 10.0;
    goal.is_turn_action = false;
    goal.angle = -90.0;
    client.sendGoal(goal);

    // 100 Sekunden wartet er, ansonsten time out, wenn kein argument, dann warter er unendlich lang
    bool finished = client.waitForResult(ros::Duration(100));
    if (finished) {
        bool success = client.getResult()->success;
        ROS_INFO("Finished goal with success: %d", success);
    } else {
        ROS_ERROR("Timeout! Server taking too long! Cancelling goal...");
        client.cancelGoal();
    }

	const Path path = findPath(LABYRINTH, START_POS, END_POS);

	ros::spin();
	return 0;
}
