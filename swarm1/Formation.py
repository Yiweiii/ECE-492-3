import sys
import cmath
import math
from robot_structure import Robot

timestamp = .5 #Controls step size

def Formation(robot1, robot2, robot3, robot4):
    dist1 = 100  # set to zer0 or small value for rendevous
    dist2 = 200  # set to zer0 or small value for rendevous
    dist3 = 223.6

    y1 = -robot1.xpos
    y2 = -robot2.xpos
    y3 = -robot3.xpos
    y4 = -robot4.xpos
    x1 = robot1.ypos
    x2 = robot2.ypos
    x3 = robot3.ypos
    x4 = robot4.ypos


    xa = (x1 - x2)
    xb = (x1 - x3)
    xc = (x1 - x4)
    ya = (y1 - y2)
    yb = (y1 - y3)
    yc = (y1 - y4)

    maga = math.sqrt(xa * xa + ya * ya)
    magb = math.sqrt(xb * xb + yb * yb)
    magc = math.sqrt(xc * xc + yc * yc)

    phasea = math.atan2(ya, xa)
    phaseb = math.atan2(yb, xb)
    phasec = math.atan2(yc, xc)

    movea = timestamp * (maga - dist1)
    moveb = timestamp * (magb - dist2)
    movec = timestamp * (magc - dist3)
    #print("movea", movea)
    #print("moveb", moveb)


    yy = (movea * math.cos(phasea) + moveb * math.cos(phaseb) + movec * math.cos(phasec))
    xx = (movea * math.sin(phasea) + moveb * math.sin(phaseb) + movec * math.sin(phasec))


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








