#Thomas Devlin 2019
import math
import subprocess
import time

mouse_process = subprocess.Popen(['python3', 'mouse.py']) #change to mouse.py for actual thing
def move(dist, tol):
    """ moves the robot forward until it has moved required distance
            to a tolerance (tol). Reads position data from mouse.
            Outputs text packets to arduino via serial port
    """
    error = dist
    vel_prev = 0
    coeff = [0,0,0]                             #tune PID coefficients
    time_prev = time.time()
    while error > tol:
        error = dist - get_position()[1]        #calculate error from ypos
        vel_prev = vel
        vel = pid(error, vel, coeff)
        time_curr = time.time()
        angle = angle(vel, vel_prev, time_curr - time_prev)
        time_prev = time_curr
    return error

def pid(error, vel, coeff=[0,0,0]):
    p = (error/dist) * coeff[0]
    i = ((dist-error)/dist) * coeff[1]
    d = vel * coeff[2]
    return (p + i + d) / 3 #range should be -1 to +1

def angle(vel, vel_prev, time):
    """ return angle from vertical in degrees
    """
    accel = (vel - vel_prev) / time
    return (math.atan(accel/9.81)) * (180/math.pi)

def get_position():
    """ return position in mm
    """
    data = open('position.txt', 'r')
    values = data.read()
    data.close()
    return values.split(' ')
