from software.Graph import Tilemap, AStar

import math

def startWorld():

    tm = Tilemap('floor_plan.png')
    maze = tm.get_tiles()

    astar = AStar(maze)
    start = (0,0)
    end = (99,84)

    path = astar.gen_path(start, end)
    astar.draw_path('floor_plan.png', path)

    print(path)
    run_directions_from_path(path)

def run_directions_from_path(path):
    current_point = (0, 0)
    margin = 0.1

    straight_dist_traveled = 0

    for point in path[1:]:

        s1 = point[0] - current_point[0]
        s2 = point[1] - current_point[1]

        dist = math.sqrt(s1 ** 2 + s2 ** 2)
        if (s1 == 0):
            angle = 0
        else:
            angle = math.tan(s2 / s1)

        if (angle < margin):
            straight_dist_traveled += dist
            continue

        if (straight_dist_traveled != 0):
            # robot.move('straight', straight_dist_traveled, 0.5)
            straight_dist_traveled = 0

        # robot.move('turn', angle, 0.5)
        print ('turned ' , angle , ' radians')
        # robot.move('straight', dist, 0.5)
        print ('moved ' , dist , ' units')

        current_point = point
startWorld()