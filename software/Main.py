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
    current_heading = 3 * math.pi / 2

    margin = 0.2

    straight_dist_traveled = 0

    for point in path[1:]:

        curr_head_degr = current_heading * 180 / math.pi

        s1 = point[0] - current_point[0]
        s2 = -(point[1] - current_point[1])

        dist = math.sqrt(s1 ** 2 + s2 ** 2)

        if (s1 == 0):
            if (s2 > 0):
                theta = math.pi / 2
            else:
                theta = 3 * math.pi / 2

        elif s1 < 0:
            if s2 > 0:
                theta = -math.atan(s2/s1) + math.pi / 2
            else:
                theta = math.atan(s2/s1) + math.pi
        else:
            theta = (math.atan(s2 / s1) + 2 * math.pi)

        theta = (theta + (2 * math.pi)) % (2 * math.pi)

        angle = min(current_heading - theta + 2 * math.pi, theta - current_heading, key=abs)


        current_heading = theta
        current_point = point

        if (abs(angle) < margin):
            straight_dist_traveled += dist
            continue

        if (straight_dist_traveled != 0):
            # robot.move('straight', straight_dist_traveled, 0.5)
            print ('moving straight distance ', straight_dist_traveled)
            straight_dist_traveled = 0

        # robot.move('turn', angle, 0.5)
        print ('turned ' , round(angle * (180/math.pi), 3) , ' degrees')
        # robot.move('straight', dist, 0.5)
        print ('moved ' , dist , ' units')


    if (straight_dist_traveled != 0):
            # robot.move('straight', straight_dist_traveled, 0.5)
        print ('move ', straight_dist_traveled)

startWorld()