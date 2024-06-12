#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <actionlib/server/simple_action_server.h>
#include <AMPER/MoveForwardAction.h>
#include <AMPER/TurnRightAction.h>
#include <cmath>
#include <iostream>

class Vector2 {
private:
    double x;
    double y;
public:
    Vector2(double x, double y):
            x(x), y(y)
    {

    }

    double getX() {
        return x;
    }

    double getY() {
        return y;
    }
};

class Navigator {
public:
    enum robotState {
        TURNING_RIGHT,
        MOVING_FORWARD,
        IDLE
    };

    Navigator(): move_forward_server(nh, "MoveForwardServer", boost::bind(&Navigator::moveForwardCB, this, _1), false),
                 turn_right_server(nh, "TurnRightServer", boost::bind(&Navigator::turnRightCB, this, _1), false)
    {
        move_forward_server.start();
        turn_right_server.start();
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/AMPER/navigation/cmd_vel", 10);
        pid_control_timer = nh.createTimer(ros::Duration(1.0 / 60), &Navigator::pidControlCallback, this);
        odom_sub = nh.subscribe("/AMPER/navigation/odom", 10, &Navigator::odomCallback, this);
        currentState = IDLE;
    }


    void moveForwardCB(const AMPER::MoveForwardGoalConstPtr &goal) {
        currentState = MOVING_FORWARD;
    }


    void turnRightCB(const AMPER::TurnRightGoalConstPtr &goal) {
        currentState = TURNING_RIGHT;
    }

    void pidControlCallback(const ros::TimerEvent &event) {
        switch(currentState) {
            case TURNING_RIGHT:
                break;
            case MOVING_FORWARD:
                break;
            case IDLE:
                break;
        }

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

        currentPos = Vector2(msg->pose.pose.position.x, msg->pose.pose.position.y);
        currentHeading = tf::getYaw(msg->pose.pose.orientation);

    }

protected:
    ros::NodeHandle nh;

    // Create the ROS publisher for the cmd_vel topic
    ros::Publisher cmd_vel_pub;

    // Create the ROS timer for the PID control loop
    ros::Timer pid_control_timer;

    // Create the ROS subscriber for the odometry topic
    ros::Subscriber odom_sub;
    Navigator::robotState currentState;

    actionlib::SimpleActionServer<AMPER::MoveForwardAction> move_forward_server;
    actionlib::SimpleActionServer<AMPER::TurnRightAction> turn_right_server;
private:
    Vector2 currentPos{0, 0};
    double currentHeading;
};


int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "navigator");

  Navigator nav;

  // Start the ROS node main loop
  ros::spin();

  return 0;
}