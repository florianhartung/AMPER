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

## How to generate a new labyrinth
TODO

## How to set new start and end positions
1. Open `src/AMPER/src/controller/const_labyrinth.hpp` to view the current labyrinth. `false` means there is a solid block, `true` means a block is air and is thus viable for a start and end position.
2. Note the new start and end indices. For example (1, 4) and (1, 1).
3. Edit the `START_POS` and `END_POS` constants in `src/AMPER/src/controller/robot_controller.cpp`.
4. Now set the robot starting position in the world. Do this by opening the launch configuration `src/AMPER/launch/simulation.launch`. Then edit the values of the `x` and `y` arg elements while adding an offset of 0.5 to every position like this:
```xml
...
<arg name="x" default="1.5"/>
<arg name="y" default="4.5"/>
...
```

## Future plans
- Use of a LIDAR sensor for navigation
- Currently the robot can only move one block forward or turn 90 degrees at a time. In the future the robot should calculate and traverse the fastest route without making unnecessary stops.
