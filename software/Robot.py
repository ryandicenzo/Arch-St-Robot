import math

from software.Node import Point
from pathlib import Path

class Robot:
    name = ""
    position = Point(0, 0)
    heading = 0

    def __init__(self, x):
        self.name = x

    def calcDistanceTo(self, p):
        return math.sqrt(math.pow(p.x - self.position.x, 2) + math.pow(p.y - self.position.y, 2))

    def moveToPoint(self, p):
        angle = math.atan(p.y / p.x)

        if angle != self.heading:
            self.rotateTo(angle)

        distReq = self.calcDistanceTo(p)
        self.moveForward(distReq)

    def moveForward(self, p):
        xd = math.cos(self.heading) * p
        yd = math.sin(self.heading) * p

        self.position.x += xd
        self.position.y += yd

    def rotateTo(self, p):
        self.heading = p

    # For Raspberry PI
    def saveToCSV(self, x):
        p = Path("./data.txt")
        p.write_text(x)
