#
# Thomas Devlin 2019
#

from math import *
import serial
import time
import struct
import threading

ser = serial.Serial('/dev/ttyUSB0')

ARM_A = 0.035               #primary linkage arm length (m)
ARM_B = 0.06                #secondary linkage arm length (m)
RAD_C = 0.15621838453       #platform radius (m)
DIST_D = 0.048733971724     #vertical distance from input pivot to platform pivot (m)
TURN_COEFF = [0, 0, 0]      #PID coeff for a turn
STRAIGHT_COEFF = [0, 0, 0]  #PID coeff for a straight move
MOUSE_RADIUS = 10           #mouse radius from turn center, also ensure mouse axes are normal to radius
MOUSE_DPI = 1000


def move(movetype, dist, tol=0.5):
    """ moves the robot until it has moved required distance
            to a tolerance tol. Reads position data from mouse.
            Outputs text packets to arduino via serial port.
        movetype = 'straight' or 'turn'
        dist = desired distance in mm or angle in radians
        tol = tolerance in mm or degrees from desired distance or angle
    """
    assert(movetype == 'straight' or movetype == 'turn')
    if movetype == 'turn':
        tol = tol * MOUSE_RADIUS
        dist = dist * MOUSE_RADIUS
        pid = pid_add(TURN_COEFF)
    else:
        pid = pid_add(STRAIGHT_COEFF)
    error = dist
    tol = min(tol, MOUSE_DPI / 25.4)            #limit tolerance to mouse precision (should still use a much larger tolerance than minimum)
    time_prev = time.time()
    vel_prev = 0
    mouse = Mouse(MOUSE_DPI)

    while error > tol:                          #PID loop
        error = dist - get_position(mouse)[1]        #calculate error from mouse ypos
        vel = pid(error, vel_prev)              #pid
        vel_prev = vel
        time_curr = time.time()
        angle = angle(vel, vel_prev, time_curr - time_prev)
        time_prev = time_curr
        text_packet(movetype, vel, angle)
    mouse.stop()
    if movetype == 'straight':
        return error
    else:
        return error / MOUSE_RADIUS

def pid_add(coeff=[0,0,0]):
    """ Generates a pid function with given coefficients
    """
    def pid(error, vel):
        p = (error/dist) * coeff[0]
        i = ((dist-error)/dist) * coeff[1]
        d = vel * coeff[2]
        return (p + i + d) / 3 #range should be -1 to +1
    return pid

def angle(vel, vel_prev, time):
    """ return angle from vertical in degrees
    """
    accel = (vel - vel_prev) / time
    platform_angle_rad = atan(accel / 9.81)
    fn = add_forward_kinematics(ARM_A, ARM_B, RAD_C, DIST_D)
    input_angle_rad = newton_raphson(platform_angle_rad, 1, fn, 0.0001)
    return input_angle_rad * (180/pi)

def add_forward_kinematics(arm_a, arm_b, rad_c, dist_d):
    """ generates a forward kinematics function for a linkage of
        given arm lengths
    """
    def forward_kinematics(x):
        num = arm_a * cos(x) + sqrt(pow(arm_b, 2) - (pow(arm_a, 2) * pow(sin(x), 2))) - dist_d
        denom = rad_c
        return asin(num / denom)
    return forward_kinematics

def forward_approx(x, fn, step_size):
    """ returns a forward approximation of the
        derivative of fn at x using step size
        step_size
    """
    return (fn(x+step_size) - fn(x)) / step_size

def get_position(mouse):
    """ return (x, y) position in mm from an instance
        of the Mouse class
    """
    return [mouse.get_x(), mouse.get_y()]

def text_packet(movetype, vel, angle):  #vel is vel if movetype straight, vel is omega if movetype turn
    """ generate text packet and send via Serial
        to arduino
    """
    packet = "{} {} {}".format(movetype, str(vel), str(angle))
    print(packet)
    ser.write(bytes(packet))

def newton_raphson(goal, x0, fn, tol):
    """ approximate root of fn using the Newton-Raphson
        method
    """
    while abs(fn(x0) - goal) > tol:
        x0 = x0 - (fn(x0) - goal) / forward_approx(x0, fn, 0.0001)
    return x0

class Mouse():

    def __init__(self, mouse_dpi = 1000):
        self.dpi = mouse_dpi
        self.stop = False
        self.x = 0
        self.y = 0
        self.mouse_data = open('/dev/input/mice', 'rb')
        t1 = threading.Thread(target = update_loop, args = (self))
        t1.start()

    def update_loop(self):
        while not self.stop:
            self.update()

    def update(self):
        buf = self.mouse_data.read(3)
        a, b = struct.unpack('bb', buf[1:])
        self.x += a
        self.y += b

    def stop(self):
        self.stop = True

    def get_x(self):
        return (self.x / self.dpi) * 25.4

    def get_y(self):
        return (self.y / self.dpi) * 25.4
