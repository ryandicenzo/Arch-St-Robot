#
# Thomas Devlin 2019
#

from math import *
import serial
import time
import struct
import threading

ser = serial.Serial('/dev/ttyUSB0')

class Robot():
    ARM_A = 0.035               #primary linkage arm length (m)
    ARM_B = 0.06                #secondary linkage arm length (m)
    RAD_C = 0.15621838453       #platform radius (m)
    DIST_D = 0.048733971724     #vertical distance from input pivot to platform pivot (m)
    TURN_COEFF = [1, 0, 0]      #PID coeff for a turn
    STRAIGHT_COEFF = [1, 0, 0]  #PID coeff for a straight move
    MOUSE_RADIUS = 10           #mouse radius from turn center, also ensure mouse axes are normal to radius
    MOUSE_DPI = 1000

    def __init__(self):
        self.heading = 0
        self.xpos = 0
        self.ypos = 0
        self.mouse = Mouse(self.MOUSE_DPI)
        self.vel = 0

    def move(self, movetype, dist, tol=0.5):
        """ moves the robot until it has moved required distance
                to a tolerance tol. Reads position data from mouse.
                Outputs text packets to arduino via serial port.
            movetype = 'straight' or 'turn'
            dist = desired distance in mm or angle in radians
            tol = tolerance in mm or degrees from desired distance or angle
        """
        assert(movetype == 'straight' or movetype == 'turn')
        if movetype == 'turn':
            tol = tol * self.MOUSE_RADIUS
            dist = dist * self.MOUSE_RADIUS
            pid = self.pid_add(dist, self.TURN_COEFF)
        else:
            pid = self.pid_add(dist, self.STRAIGHT_COEFF)
        error = dist
        tol = min(tol, self.MOUSE_DPI / 25.4)            #limit tolerance to mouse precision (should still use a much larger tolerance than minimum)
        time_prev = time.time()
        vel_prev = 0
        self.mouse = Mouse(self.MOUSE_DPI)

        while error > tol:                          #PID loop
            error = dist - self.get_position()[1]   #calculate error from mouse ypos
            vel = pid(error, vel_prev)              #pid
            time_curr = time.time()
            deltat = max(time_curr - time_prev, 0.0000001)
            angle = self.angle(vel, vel_prev, deltat)
            vel_prev = vel
            time_prev = time_curr
            self.text_packet(movetype, vel, angle)

        self.mouse.halt()
        if movetype == 'straight':
            self.xpos += (dist - error) * cos(self.heading)
            self.ypos += (dist - error) * sin(self.heading)
            return [self.xpos, self.ypos, self.heading, error]
        else:
            self.heading = (self.heading + (dist - (error / self.MOUSE_RADIUS))) % (2 * pi)
            return [self.xpos, self.ypos, self.heading, error / self.MOUSE_RADIUS]

    def pid_add(self, dist, coeff):
        """ Generates a pid function with given coefficients
        """
        def pid(error, vel):
            p = (error/dist) * coeff[0]
            i = ((dist-error)/dist) * coeff[1]
            d = vel * coeff[2]
            return (p + i + d) / 3 #range should be -1 to +1
        return pid

    def angle(self, vel, vel_prev, time):
        """ return angle from vertical in degrees
        """
        accel = (vel - vel_prev) / time
        platform_angle_rad = atan(accel / 9810)
        fn = self.add_forward_kinematics(self.ARM_A, self.ARM_B, self.RAD_C, self.DIST_D)
        input_angle_rad = self.newton_raphson(platform_angle_rad, 1, fn, 0.01)
        return input_angle_rad * (180/pi)

    def add_forward_kinematics(self, arm_a, arm_b, rad_c, dist_d):
        """ generates a forward kinematics function for a linkage of
            given arm lengths
        """
        def forward_kinematics(x):
            num = arm_a * cos(x) + sqrt(pow(arm_b, 2) - (pow(arm_a, 2) * pow(sin(x), 2))) - dist_d
            denom = rad_c
            return asin(num / denom)
        return forward_kinematics

    def forward_approx(self, x, fn, step_size):
        """ returns a forward approximation of the
            derivative of fn at x using step size
            step_size
        """
        return (fn(x + step_size) - fn(x)) / step_size

    def get_position(self):
        """ return (x, y) position in mm from an instance
            of the Mouse class
        """
        return [self.mouse.get_x(), self.mouse.get_y()]

    def text_packet(self, movetype, vel, angle):  #vel is vel if movetype straight, vel is omega if movetype turn
        """ generate text packet and send via Serial
            to arduino
        """
        packet = "{} {} {}".format(movetype, str(vel), str(angle))
        ser.write(packet)

    def newton_raphson(self, goal, x0, fn, tol):
        """ approximate root of fn using the Newton-Raphson
            method
        """
        while abs(fn(x0) - goal) > tol:
            x0 = x0 - (fn(x0) - goal) / self.forward_approx(x0, fn, 0.0001)
        return x0

    def getxpos(self):
        return self.xpos

    def getypos(self):
        return self.ypos

    def getheading(self):
        return self.heading

class Mouse():

    def __init__(self, mouse_dpi = 1000):
        self.dpi = mouse_dpi
        self.stop = False
        self.x = 0
        self.y = 0
        self.mouse_data = open('/dev/input/mice', 'rb')
        self.t1 = threading.Thread(target = Mouse.update_loop, args = (self,))
        self.t1.daemon = True
        self.t1.start()

    def update_loop(self):
        while not self.stop:
            self.update()

    def update(self):
        buf = self.mouse_data.read(3)
        a, b = struct.unpack('bb', buf[1:])
        self.x += a
        self.y += b

    def halt(self):
        self.stop = True

    def get_x(self):
        return (float(self.x) / self.dpi) * 25.4

    def get_y(self):
        return (float(self.y) / self.dpi) * 25.4


r = Robot()
