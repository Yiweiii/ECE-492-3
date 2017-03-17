import sys
import cmath
import math
from robot_structure import Robot

timestamp = 1 # Timestamp will be removed, using for testing, will be based on robot speed.

def rendezvous(robot1, robot2, robot3):
    dist1 = 0  # set to zer0 or small value for rendevous
    dist2 = 0  # set to zer0 or small value for rendevous

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

    maga = math.sqrt(xa*xa +ya*ya)
    magb = math.sqrt(xb*xb +yb*yb)
    phasemovea = math.atan2(ya,xa)
    phasemoveb = math.atan2(yb,xb)
    if ya > 0:
        phasea = math.atan2(-ya, -xa)
    else:
        phasea = math.atan2(-ya, xa)
    if yb > 0:
        phaseb = math.atan2(-yb, -xb)
    else:
        phaseb = math.atan2(-yb, xb)		
	#phasea = math.atan2(ya, xa)
    #phaseb = math.atan2(yb, xb)
	#(maga, phasea) = cmath.polar((complex(xa, ya)))
    #(magb, phaseb) = cmath.polar((complex(xb, yb)))
    #print(maga, " mag ", magb)

    movea = timestamp * (maga - dist1)
    moveb = timestamp * (magb - dist2)
    #print("movea", movea)
    #print("moveb", moveb)


    xx = (movea * math.cos(phasemovea) + moveb * math.cos(phasemoveb))
    yy = (movea * math.sin(phasemovea) + moveb * math.sin(phasemoveb))
    #print(xx, yy)
    finalx = robot1.xpos - xx
    finaly = robot1.ypos - yy
    #print(finalx,finaly)

    #(mag, phase) = cmath.polar((complex(xx,yy)))

    #phase = math.atan2(yy,xx)

    if yy > 0:
        phase = math.atan2(-yy, -xx)
    else:
        phase = math.atan2(-yy, xx)	
    phasedeg = (phase * (180 / cmath.pi) )
    phasedeg = (phasedeg + 360)%360
    rotateangle = angle - phasedeg


    ##print("Move dist", mag)
    #print("angle to move to ", rotateangle)
    ### we will be moving to a set X, Y location and rotating untill a certain angle We may want to return X,Y angle.
    return finalx, finaly, phasedeg








