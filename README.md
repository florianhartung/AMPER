# AMPER: Autonomous Maze Pathfinding and Exploration Robot [WIP]
AMPER is a robot project in ROS.
The robot can can navigate mazes by solving for a path, then traversing the path to the target position.

## Setup
```sh
chmod u+x run.sh
chmod u+x generate_and_run.sh
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
./run.sh <start_x> <start_y> <end_x> <end_y>
```

## How it works
