#!/usr/bin/env python
import sys
import world_generation
import maze_gen
import random
import os
import yaml


config_path = os.path.join(os.path.dirname(__file__), '../../../config.yaml')

def distance(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

if len(sys.argv) != 3 and len(sys.argv) != 4:
    print("Usage: python world-launch.py <width> <height> (min_dist)")
    sys.exit(1)

def generate_config_file(start_x, start_y, end_x, end_y):
    config = {

        "start": {
            "x": start_x,
            "y": start_y
        },
        "end": {
            "x": end_x,
            "y": end_y
        }

    }

    with open(config_path, 'w') as f:
        yaml.dump(config, f)

width, height = int(sys.argv[1]), int(sys.argv[2])
min_dist = 5
if len(sys.argv) == 4:
    min_dist = float(sys.argv[3])

array = maze_gen.generate_maze(width, height)
print("Generated maze")

world_generation.generate_world_file_and_hpp_file(array)
print("Generated world file and hpp file")

start_x, start_y = 0, 0
while array[start_y][start_x] != 1:
    start_x, start_y = random.randint(1, 10), random.randint(1, 10)

print("Start position in grid: ", start_x, start_y)
print("Start position in gazebo: ", start_x + 0.5, start_y + 0.5)

end_x, end_y = 0, 0
while not (array[end_y][end_x] == 1 and distance((start_x, start_y), (end_x, end_y)) < min_dist):
    end_x, end_y = random.randint(1, 10), random.randint(1, 10)

print("End position in grid: ", end_x, end_y)

generate_config_file(start_x, start_y, end_x, end_y)