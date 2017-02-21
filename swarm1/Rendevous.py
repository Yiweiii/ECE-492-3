import sys
import cmath
import math
from robot_structure import Robot

timestamp = .1 # Timestamp will be removed, using for testing, will be based on robot speed.

def rendezvous(robot1, robot2, robot3):
    dist1 = .1  # set to zer0 or small value for rendevous
    dist2 = .1  # set to zer0 or small value for rendevous

    x1 = robot1.xpos
    x2 = robot2.xpos
    x3 = robot3.xpos
    y1 = robot1.ypos
    y2 = robot2.ypos
    y3 = robot3.ypos
    angle = robot1.dir

    xa = x1 - x2
    xb = x1 - x3
    ya = y1 - y2
    yb = y1 - y3

    (maga, phasea) = cmath.polar((complex(xa, ya)))
    (magb, phaseb) = cmath.polar((complex(xb, yb)))

    movea = timestamp * (maga - dist1)
    moveb = timestamp * (magb - dist2)




    xx = -(movea * math.cos(phasea) + moveb * math.cos(phaseb))
    yy = -(movea * math.sin(phasea) + moveb * math.sin(phaseb))

    (mag, phase) = cmath.polar((complex(xx,yy)))

    phasedeg = phase * 180 / cmath.pi - 180

    rotateangle = angle - phasedeg


   # print("Move dist", mag)
   # print("angle to move to ", rotateangle)

    return mag, phase, rotateangle








