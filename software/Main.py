from software.Graph import Tilemap, AStar

import math

def startWorld():

    tm = Tilemap('floor_plan.png')
    maze = tm.get_tiles()

    astar = AStar(maze)
    start = (10,0)
    end = (10,10)

    path = astar.gen_path(start, end)
    astar.draw_path('floor_plan.png', path)

    print(path)
    run_directions_from_path(path)

def run_directions_from_path(path):
    current_point = path[0]
    current_heading = -math.pi / 2

    margin = 0.2

    straight_dist_traveled = 0

    for point in path[1:]:

        s1 = point[0] - current_point[0]
        s2 = point[1] - current_point[1]

        dist = math.sqrt(s1 ** 2 + s2 ** 2)

        if (s1 == 0):
            angle = 0
        else:
            angle = math.atan(s2 / s1)


        if (abs(angle) < margin):
            straight_dist_traveled += dist
            continue

        if (straight_dist_traveled != 0):
            # robot.move('straight', straight_dist_traveled, 0.5)
            straight_dist_traveled = 0


        # robot.move('turn', angle, 0.5)
        print ('turned ' , angle * (180/math.pi) , ' degrees')
        # robot.move('straight', dist, 0.5)
        print ('moved ' , dist , ' units')

        current_point = point

    if (straight_dist_traveled != 0):
            # robot.move('straight', straight_dist_traveled, 0.5)
        print ('move ', straight_dist_traveled)

startWorld()