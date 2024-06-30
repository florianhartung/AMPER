import random

def init_maze(w: int, h: int):
    """
    Initializes a maze with the given width and height
    :param w: width
    :param h: height
    :return:
    """
    maze = [[False for _ in range(w)] for _ in range(h)]
    return maze

def add_walls(coordinates, wall_list, maze):
    """
    Adds walls to the wall list if they are not already in the list
    :param coordinates: x, y
    :param wall_list:
    :param maze:
    :return:
    """
    x, y = coordinates
    if x+1 < len(maze[0]) and maze[y][x+1] is False:
        wall_list.append((x+1, y))
    if x-1 >= 0 and maze[y][x-1] is False:
        wall_list.append((x-1, y))
    if y+1 < len(maze) and maze[y+1][x] is False:
        wall_list.append((x, y+1))
    if y-1 >= 0 and maze[y-1][x] is False:
        wall_list.append((x, y-1))

def get_neighbours(coordinates, maze):
    """
    Returns the neighbouring walls of the given coordinates
    :param coordinates: x, y
    :param maze:
    :return:
    """
    x, y = coordinates
    neighbouring_walls = []
    if x+1 < len(maze[0]):
        neighbouring_walls.append((x+1, y))
    if x-1 >= 0:
        neighbouring_walls.append((x-1, y))
    if y+1 < len(maze):
        neighbouring_walls.append((x, y+1))
    if y-1 >= 0:
        neighbouring_walls.append((x, y-1))
    return neighbouring_walls


def generate_maze(w: int, h: int):
    """
    Generates a maze with the given width and height using Prim's algorithm
    :param w: width
    :param h: height
    :return: returns a maze of size w-2 x h-2
    """
    maze = init_maze(w-2, h-2)
    wall_list = []
    start_cell = (random.randint(0, w-3), random.randint(0, h-3))
    maze[start_cell[1]][start_cell[0]] = True
    add_walls(start_cell, wall_list, maze)
    while len(wall_list) > 0:
        random_wall = random.choice(wall_list)
        neighbours = get_neighbours(random_wall, maze)
        sum_of_neighbours = 0
        for nx, ny in neighbours:
            sum_of_neighbours += maze[ny][nx]
        if sum_of_neighbours == 1:
            maze[random_wall[1]][random_wall[0]] = True
            add_walls(random_wall, wall_list, maze)
        wall_list.remove(random_wall)
    # ummauern
    full_wall = [False for _ in range(w)]
    for line in maze:
        line.insert(0, False)
        line.append(False)
    maze.insert(0, full_wall)
    maze.append(full_wall)
    
    return maze
