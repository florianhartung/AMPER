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

valid_positions = [(x, y) for y in range(height) for x in range(width) if array[y][x] == 1]

# Randomly select a start position from the list of valid positions
start_x, start_y = random.choice(valid_positions)

print("Start position in grid: ", start_x, start_y)
print("Start position in gazebo: ", start_x + 0.5, start_y + 0.5)

# Get a list of valid end positions that are at least min_dist away from the start position
valid_end_positions = [(x, y) for x, y in valid_positions if distance((start_x, start_y), (x, y)) >= min_dist]

end_x, end_y = random.choice(valid_end_positions)

print("End position in grid: ", end_x, end_y)

generate_config_file(start_x, start_y, end_x, end_y)