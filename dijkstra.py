import numpy as np

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