# AMPER: Autonomous Maze Pathfinding and Exploration Robot
AMPER is a robot project in ROS. The robot can navigate mazes by first finding a path for a given hardcoded labyrinth, then traverse the path from its start position to the target position.

## Setup
```sh
chmod u+x run.sh
chmod u+x generate_and_run.sh
source devel/setup.bash
```
## How to run
### Monte Carlo Simulation
It creates a new random labyrinth, sets a random start and end position, and runs the robot to solve the maze. You can set the
size of the maze with width and height arguments and with an optional minimum distance between the start and end positions (it defaults to 5). 
Because the minimum distance is 5, please set the width and height to at least 10.
```sh  
./generate_and_run.sh <width> <height> (min_diff)
```
### Generate random maze and set your own start and end positions
Generate a random maze with the given width and height, look at const_labyrinth.hpp in src/AMPER/src/controller then set the start and end positions to the given coordinates.
```sh
python3 src/AMPER/scripts/world_generation.py <width> <height>
catkin_make
./run.sh <start_x> <start_y> <end_x> <end_y>
```

## How it works
### System Architecture

![alt text](src/AMPER/docs/figures/architecture.drawio_new.png)

### Maze Generation
For the maze generation, we used Prim's algorithm.
1. Initialize a grid where all cells are considered as walls.
2. Select a single cell to be the current cell and mark it as part of the maze.
3. Add the walls of the current cell to a list of walls.
4. Enter a loop that continues as long as there are walls in the list.
5. Select a random wall from the list.
6. Check the cell on the opposite side of the selected wall. If it's not part of the maze yet:
   - Remove the selected wall.
   - Mark the cell on the opposite side as part of the maze.
   - Add the walls of the cell on the opposite side to the wall list.
7. Repeat the loop (go to step 5) until there are no more walls in the list.
8. After the loop ends, add walls around the entire perimeter of the maze to prevent the robot from going out of bounds.

The implementation was done in Python under src/AMPER/scripts/maze_gen.py.

After we got our array in the form of a 2d array, where true means air and false means wall, we converted it to a C++ header file, which will be used by the pathfinding algorithm and a world file
for gazebo to render the labyrinth and for the robot to move in.

The implementation was also done in Python under src/AMPER/scripts/world_generation.py.
### Pathfinding
For the pathfinding, we used the A* algorithm. A* is a graph traversal and path search algorithm that finds the shortest path between the start node and the end node. 
It uses a heuristic function to estimate the cost of the cheapest path through a node. 

Implementation can be found in src/AMPER/src/controller/pathfinding.cpp.

### Robot Movement
For the movement we make use of ROS services. We have a controller node, which is responsible for telling the robot, where to move next.
The controller node gets the start position and uses the pathfinding algorithm to calculate the path to the end position. 

Then we have the Navigator server, which is responsible for the actual movement of the robot. It tries to execute a current goal, which is either a distance that needs
to be moved or an angle to which the robot needs to turn. It has four possible STATES: IDLE, MOVING_FORWARD, TURNING and CORRECTING.
The robot can only be in one of these states at any given time. The states should be self-explanatory, but the CORRECTING state is used when the robot needs to move forward, but has an offset from the center of the path and if it keeps moving forward it will hit a wall, sooner of later.
In CORRECTING the robot turns a little bit so that it when it moves forward it will reach the center of the path again. CORRECTING can only be entered from the MOVING_FORWARD state.

The controller has a client that sends the next goal to the navigator, which gets computed by looking at the next node (grid cell) in the path seeing if the robot has to move forward or turn. It
has the current optimization that when next node are perfectly aligned in a straight line it will add their distances together and skip the intermediate nodes. After a goal is created it is sent as a message and request to the navigator server, which will then execute the goal.
The controller now waits for the navigator to finish the goal and then sends the next goal. This process repeats until the robot reaches the end position.

Implementation can be found in src/AMPER/src/controller/robot_controller.cpp and src/AMPER/src/navigation/navigator.cpp.