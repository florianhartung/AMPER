#include "ros/ros.h"
#include "path_finding.hpp"
#include "const_labyrinth.hpp"
#include "AMPER/MoveAction.h"
#include "actionlib/client/simple_action_client.h"


const std::tuple<size_t, size_t> START_POS(1, 1);
const std::tuple<size_t, size_t> END_POS(8, 3);

void sendMoveGoal(actionlib::SimpleActionClient<AMPER::MoveAction>& client, AMPER::MoveGoal goal) {
    std::stringstream ss;
    ss << "Controller: Sending goal to navigation server: (is_turn=" << (goal.is_turn_action ? '1' : '0') <<", dis=" << goal.distance << ", angle=" << goal.angle << ")";
    std::string s = ss.str();
    const char* cs = s.c_str();
    ROS_INFO(cs);

    client.sendGoal(goal);

    bool finished = client.waitForResult(ros::Duration(100)); // arbitrary timeout

    if (!finished) {
        throw std::runtime_error("Timeout during navigation server action call");
    }

    if (!client.getResult()->success) {
        throw std::runtime_error("Navigation server returned error");
    }
}

void moveForward(actionlib::SimpleActionClient<AMPER::MoveAction>& client) {
    AMPER::MoveGoal goal;
    goal.is_turn_action = false;
    goal.distance = 1.0;
    goal.angle = 0.0;

    sendMoveGoal(client, goal);
}

// If left==false, then it will turn right
void turn(actionlib::SimpleActionClient<AMPER::MoveAction>& client, bool left) {
    AMPER::MoveGoal goal;
    goal.is_turn_action = true;
    goal.distance = 0.0;

    if (left) {
        goal.angle = 90.0;
    } else {
        goal.angle = -90.0;
    }

    sendMoveGoal(client, goal);
}

enum Direction {
    X_POS = 0,
    Y_NEG = 1,
    X_NEG = 2,
    Y_POS = 3,
};

void turnToDirection(actionlib::SimpleActionClient<AMPER::MoveAction>& client, Direction currentDirection, Direction targetDirection) {
    if (currentDirection == targetDirection) {
        return;
    }



    if (static_cast<Direction>((currentDirection + 1) % 4) == targetDirection) {
        turn(client, false); // turn right
    } else if (static_cast<Direction>(currentDirection == (targetDirection + 1) % 4)) {
        turn(client, true); // turn left
    } else {
        throw new std::runtime_error("Cannot make U-turn");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    ROS_INFO("Started controller");

    actionlib::SimpleActionClient<AMPER::MoveAction> client("MoveServer", true);
    client.waitForServer();


    if (START_POS == END_POS) {
        ROS_WARN("Cannot calculate path because robot start and end positions are equal");
        return 0;
    }

    Path path = findPath(LABYRINTH, START_POS, END_POS);

    PathNode startNode = path.popNode();

    std::tuple<size_t, size_t> currentPosition = {startNode.x, startNode.y};
    Direction currentDirection = Direction::X_POS;

    while (path.size() > 0) {
        PathNode next = path.popNode();

        std::stringstream ss;
        ss << "Next path node: (" << next.x << ',' << next.y << ')';
        std::string s = ss.str();
        const char* cs = s.c_str();
        ROS_INFO(cs);

        size_t currentX = std::get<0>(currentPosition);
        size_t currentY = std::get<1>(currentPosition);

        Direction targetDirection;
        if (currentX == next.x && currentY == next.y + 1) {
            targetDirection = Direction::Y_NEG;
        } else if (currentX == next.x && currentY + 1 == next.y) {
            targetDirection = Direction::Y_POS;
        } else if (currentY == next.y && currentX == next.x + 1){
            targetDirection = Direction::X_NEG;
        } else if (currentY == next.y && currentX + 1 == next.x){
            targetDirection = Direction::X_POS;
        } else {
            throw std::runtime_error("Invalid path");
        }

        turnToDirection(client, currentDirection, targetDirection);
        currentDirection = targetDirection;
	currentPosition = {next.x, next.y};

        moveForward(client);
    }

    ros::spin();
    return 0;
}

