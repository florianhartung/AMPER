# AMPER: Autonomous Maze Pathfinding and Exploration Robot [WIP]
AMPER is a robot project in ROS.
The robot can can navigate mazes by solving for a path, then traversing the path to the target position.

## How to build and run
```sh
source /opt/ros/noetic/setup.sh
rm -rf build devel
catkin_make
source devel/setup.sh
roslaunch AMPER simulation.launch
```

## How to set new start and end positions
TODO

## Future plans
- Use of a LIDAR sensor for navigation
- Currently the robot can only move one block forward or turn 90 degrees at a time. In the future the robot should calculate and traverse the fastest route without making unnecessary stops.
