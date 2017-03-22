import sys
import cmath
import math
from robot_structure import Robot

timestamp = .1 #Controls step size 

def rendezvous(robot1, robot2, robot3):
    dist1 = 0  # set to zer0 or small value for rendevous
    dist2 = 0  # set to zer0 or small value for rendevous

    x1 = robot1.xpos
    x2 = robot2.xpos
    x3 = robot3.xpos
    y1 = robot1.ypos
    y2 = robot2.ypos
    y3 = robot3.ypos


    ya = -(x1 - x2)
    yb = -(x1 - x3)
    xa = (y1 - y2)
    xb = (y1 - y3)

    maga = math.sqrt(xa * xa + ya * ya)
    magb = math.sqrt(xb * xb + yb * yb)

    phasea = math.atan2(ya, xa)
    phaseb = math.atan2(yb, xb)

    movea = timestamp * (maga - dist1)
    moveb = timestamp * (magb - dist2)
    #print("movea", movea)
    #print("moveb", moveb)


    yy = (movea * math.cos(phasea) + moveb * math.cos(phaseb))
    xx = (movea * math.sin(phasea) + moveb * math.sin(phaseb))


    #print(xx, yy)
    finalx = robot1.xpos - xx
    finaly = robot1.ypos - yy

    ##(mag, phase) = cmath.polar((complex(xx,yy)))

    phase = math.atan2(yy, xx)

    phasedeg = ((phase * 180 / cmath.pi) + 360 ) %360
    ##rotateangle = angle - phasedeg


    ##print("Move dist", mag)
    #print("angle to move to ", rotateangle)
    ### we will be moving to a set X, Y location and rotating untill a certain angle We may want to return X,Y angle.
    return finalx, finaly, phasedeg








