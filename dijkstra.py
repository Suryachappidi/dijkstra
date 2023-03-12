import numpy as np

# Dimensions of the map
width = 600
height = 250

# return change in x , y and returns cost

def Actionmove_right():
    return 1, 0, 1

def Actionmove_left():
    return -1, 0, 1

def Actionmove_up():
    return 0, 1, 1

def Actionmove_down():
    return 0, -1, 1

def Actionmove_upright():
    return 1, 1, 1.4

def Actionmove_upleft():
    return -1, 1, 1.4

def Actionmove_downright():
    return 1, -1, 1.4

def Actionmove_downleft():
    return -1, -1, 1.4

# Calculating all possible neighbours for robot using available actions

def compute_neighbours(map, node):
    width, height = map.shape
    x, y = node[1], node[0]
    neighbours = []
    for dx, dy, cost in [Actionmove_right(), Actionmove_left(), Actionmove_up(), Actionmove_down(),
                         Actionmove_upright(), Actionmove_upleft(), Actionmove_downright(), Actionmove_downleft()]:
        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < height and 0 <= new_y < width and map[new_y, new_x] == 0:
            neighbours.append((new_y, new_x))
    return neighbours

def ObstacleMap(width, height):
    map = np.full((height, width), 0)
    for y in range(height):
        for x in range(width):

            # Box obstacle
            l1 = (x-5) - 150
            l2 = (x+5) - 100
            l3 = y-5 - 100
            l4 = y+5 - 150

            if(l1<0 and l2>0):
                map[y, x] = 1

            if(l3 >0 and l4 < 0):
                map[y, x] = 1

            # Box Robot clearance
            l1_c = (x) - 150
            l2_c = (x) - 100
            l3_c = y - 100
            l4_c = y - 150

            if (l1_c < 0 and l2_c > 0):
                map[y, x] = 2

            if (l3 > 0 and l4 < 0):
                map[y, x] = 0

            #triangle clearance
            t1_c = (x+5) - 460
            t2_c = (y) + 2 * x - 1156.1803
            t3_c = (y) - 2 * x + 906.1803
            if (t1_c > 0 and t2_c < 0 and t3_c > 0):
                map[y, x] = 1

            # traiangle obstacle
            t1 = (x) - 460
            t2 = (y) + 2 * (x) - 1145
            t3 = (y) - 2 * (x) + 895

            if (t1 > 0 and t2 < 0 and t3 > 0):
                map[y, x] = 2

            # Hexagon Obstacle (clearance)
            h1_c = y - 0.577 * x + 123.21 + 5.7726
            h2_c = x - 364.95 - 5.7726
            h3_c = y + 0.577 * x - 373.21 - 5.7726
            h4_c = y - 0.577 * x - 26.92 - 5.7726
            h5_c = x - 235 + 5.7726
            h6_c = y + 0.577 * x - 223.08 + 5.7726

            if (h2_c < 0 and h5_c > 0 and h1_c > 0 and h3_c < 0 and h4_c < 0 and h6_c > 0):
                map[y, x] = 1

            #Hexagon Obstacle
            h1 = y - 0.577 * x + 123.21
            h2 = x - 364.95
            h3 = y + 0.577 * x - 373.21
            h4 = y - 0.577 * x - 26.92
            h5 = x - 235
            h6 = y + 0.577 * x - 223.08

            if (h2 < 0 and  h5 > 0 and h1 > 0 and h3 < 0 and h4 < 0 and h6 > 0):
                map[y, x] = 2

    # Map Surrrounding Clearnce
    map[:5, :width] = 1
    map[height - 5:height, :width] = 1
    map[:height, :5] = 1
    map[:height, width - 5:width] = 1

    return map

def is_valid(x, y, obstacle_map):
    return (0 <= x < obstacle_map.shape[1] and
            0 <= y < obstacle_map.shape[0] and
            obstacle_map[y][x] not in [1, 2])


# Creating the Obstacle map
obstacle_map = ObstacleMap(width, height)

# Taking valid Goal and Start Nodes
start_coord = input("Enter start coordinates as x,y: ")
start_x, start_y = start_coord.split(',')
start_x = int(start_x)
start_y = int(start_y)
if is_valid(start_x, start_y, obstacle_map):
    start = (start_y, start_x)
else:
    print("Invalid start node or Node is in Obstacle space")
    exit(-1)

goal_coordinates = input("Enter goal coordinates as x,y: ")
goal_x, goal_y = goal_coordinates.split(',')
goal_x = int(goal_x)
goal_y = int(goal_y)
if not is_valid(goal_x, goal_y, obstacle_map):
    goal = (goal_y, goal_x)
else:
    print("Invalid goal node or Node is in Obstacle space")
    exit(-1)
