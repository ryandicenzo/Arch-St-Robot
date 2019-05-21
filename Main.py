from Robot import Robot
from Point import Point

def startWorld():
    grigometh = Robot("grigo")
    
    p = Point(10, 10)
    grigometh.moveToPoint(p)

    print (grigometh.position.x)
    print (grigometh.position.y)

startWorld()