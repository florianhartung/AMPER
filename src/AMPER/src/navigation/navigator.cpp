#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <actionlib/server/simple_action_server.h>
#include <AMPER/MoveAction.h>
#include <cmath>
#include <iostream>

class Vector3 {
private:
    double x;
    double y;
    double angle;
public:
    Vector3(double x, double y, double angle):
            x(x), y(y), angle(angle)
    {

    }

    double getX() {
        return x;
    }

    double getY() {
        return y;
    }

    double getAngle() {
        return angle;
    }

    Vector3 subtractPos(Vector3 v2) {
        Vector3 newVec{x - v2.getX(), y - v2.getY(), 0.0};
        return newVec;
    }

    Vector3 addDistance(double distance) {

        double newX = x + distance * cos(angle);
        double newY = y + distance * sin(angle);

        return Vector3(newX, newY, angle);
    }

    Vector3 setAngle(double new_angle) {

        return Vector3(x, y, new_angle);
    }

    double lengthPos() {
        return std::sqrt(x*x + y*y);
    }
};

class Navigator {
public:
    enum robotState {
        TURNING,
        MOVING_FORWARD,
        IDLE
    };

    Navigator(): moveServer(nh, "MoveServer", false)
    {

        moveServer.registerGoalCallback(boost::bind(&Navigator::moveGoalCB, this));
        moveServer.start();
        cmdVelPub = nh.advertise<geometry_msgs::Twist>("/AMPER/navigation/cmd_vel", 10);
        pidControlTimer = nh.createTimer(ros::Duration(1.0 / 60), &Navigator::pidControlCallback, this);
        odomSub = nh.subscribe("/AMPER/navigation/odom", 10, &Navigator::odomCallback, this);
        currentState = IDLE;
    }


    void moveGoalCB() {
        ROS_INFO("Goal Received!");
        AMPER::MoveGoal goal = *moveServer.acceptNewGoal();
        if (goal.is_turn_action) {
            // translate angle to standard definition angle
            double angle = PI * goal.angle / 180.0;

            goalPos = currentPos.setAngle(angle);

            currentState = TURNING;
        } else {
            goalPos = currentPos.addDistance(goal.distance);
            currentState = MOVING_FORWARD;
        }
    }

    void reset() {
        geometry_msgs::Twist emptyCmdMsg;

        currentState = IDLE;
        errorIntegral = 0.0;
        prevError = 0.0;
        cmdVelPub.publish(emptyCmdMsg);
    }

    void succeedGoal() {
        result.success = true;
        moveServer.setSucceeded(result);
    }

    double getDist(Vector3 v1, Vector3 v2) {
        Vector3 subVec = v1.subtractPos(v2);

        return subVec.lengthPos();
    }

    double getAngleDist(double a1, double a2) {
        double diff = a1 - a2;
        diff = 180.0 * diff / PI;
        diff = std::fmod((diff + 180.0), 360.0) - 180.0;
        return PI * diff / 180.0;
    }

    void turn(double dt) {
        //ROS_INFO("TURNING");
        geometry_msgs::Twist cmdMsg;
        double desiredAngle = goalPos.getAngle();
        double angleDistance = getAngleDist(desiredAngle, currentPos.getAngle());

        int sign = angleDistance > 0 ? 1 : -1;
        double angularVelocity = sign * std::min(maxAngularVel, angleDistance);

        cmdMsg.angular.z = angularVelocity;

        feedback.angle_rotated = angleDistance;
        feedback.distance_moved = 0;
        moveServer.publishFeedback(feedback);

        cmdVelPub.publish(cmdMsg);
    }

    void move(double dt) {
        ROS_INFO("currentPos %f %f goalPos %f %f", currentPos.getX(), currentPos.getY(), goalPos.getX(), goalPos.getY());
        geometry_msgs::Twist cmdMsg;
        double distanceRemaining = getDist(goalPos, currentPos);
        ROS_INFO("dist %f", distanceRemaining);
        double pTerm = kp * distanceRemaining;

        errorIntegral += dt * distanceRemaining;
        double iTerm = ki * errorIntegral;
        double dTerm = kd * (distanceRemaining - prevError) / dt;

        prevError = distanceRemaining;

        ROS_INFO("pid %f", pTerm + iTerm + dTerm);

        double velocity = std::min(maxVel, pTerm + iTerm + dTerm);
        ROS_INFO("velocity %f", velocity);

        cmdMsg.linear.x = velocity;

        feedback.distance_moved = distanceRemaining;
        moveServer.publishFeedback(feedback);

        cmdVelPub.publish(cmdMsg);
    }


    void pidControlCallback(const ros::TimerEvent &event) {
        if (!moveServer.isActive()) {
            ROS_INFO("WAITING FOR GOAL...");
            return;
        }

        double time = event.current_real.toSec();
        double dt = time - lastTime;
        lastTime = time;

        if (lastTime == 0)  {
            return;
        }

        geometry_msgs::Twist emptyCmdMsg;

        switch(currentState) {
            case TURNING:
                // checks if robot can rotate closer, if angle distance smaller than 0.5 degree stops
                if (std::abs(getAngleDist(goalPos.getAngle(), currentPos.getAngle())) < PI / 360) {
                    reset();
                    succeedGoal();
                    break;
                }
                turn(dt);
                break;
            case MOVING_FORWARD:
                // checks if robot can move closer, if distance smaller than 0.01 m stops
                ROS_INFO("DIST %f", getDist(goalPos, currentPos));
                if (getDist(goalPos, currentPos) < 0.01) {
                    reset();
                    succeedGoal();
                    break;
                }
                move(dt);
                break;
            case IDLE:
                errorIntegral = 0;
                prevError = 0;
                cmdVelPub.publish(emptyCmdMsg);
                break;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        double currentHeading = tf::getYaw(msg->pose.pose.orientation);
        currentPos = Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, currentHeading);
        //ROS_INFO("position + angle: %f %f %f", currentPos.getX(), currentPos.getY(), currentPos.getAngle());
    }

protected:
    ros::NodeHandle nh;

    // Create the ROS publisher for the cmd_vel topic
    ros::Publisher cmdVelPub;

    // Create the ROS timer for the PID control loop
    ros::Timer pidControlTimer;

    // Create the ROS subscriber for the odometry topic
    ros::Subscriber odomSub;
    Navigator::robotState currentState;
    AMPER::MoveResult result;
    AMPER::MoveFeedback feedback;

    actionlib::SimpleActionServer<AMPER::MoveAction> moveServer;

private:
    Vector3 currentPos{0.0, 0.0, 0.0};
    Vector3 goalPos{0.0, 0.0, 0.0};

    // Velocity limits
    const double PI = 3.141592653589;
    const double maxAngularVel = PI / 2;
    const double maxVel = 0.8;

    // PID control gains
    const double kp = 0.1;
    const double ki = 0.05;
    const double kd = 0.05;

    // Variables for PID control
    double errorIntegral = 0.0;
    double prevError = 0.0;

    double lastTime = 0;
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