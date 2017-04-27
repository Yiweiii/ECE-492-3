from socket import *
from robot_structure import Robot
from Rendevous import rendezvous
import sys
import math
import camera_actions as ca
import cv2
import time


def messagepackage(robot, xpos, ypos, angle):
    x1 = abs(float(robot.xpos) / 314)
    y1 = abs(float(robot.ypos) / 314)
    xx1 = abs(xpos / 314)
    yy1 = abs(ypos / 314)
    x1str = str("%.2f" % round(x1, 2))
    y1str = str("%.2f" % round(y1, 2))
    angle1str = str("%03d" % angle)
    anglerobot1str = str("%03d" % robot.dir)
    xx1str = str("%.2f" % round(xx1, 2))
    yy1str = str("%.2f" % round(yy1, 2))

    message = ('c ' + x1str + ',' + y1str + ',' + anglerobot1str + ',' + xx1str + ',' + yy1str + ',' + angle1str)


    return message
	
def messagepackage2(robot, xpos, ypos, angle):
    x1 = abs(float(robot.xpos) / 314)
    y1 = abs(float(robot.ypos) / 314)
    xx1 = abs(xpos / 314)
    yy1 = abs(ypos / 314)
    x1str = str("%.2f" % round(x1, 2))
    y1str = str("%.2f" % round(y1, 2))
    angle1str = str("%03d" % angle)
    anglerobot1str = str("%03d" % robot.dir)
    xx1str = str("%.2f" % round(xx1, 2))
    yy1str = str("%.2f" % round(yy1, 2))

    message = ('D ' + x1str + ',' + y1str + ',' + anglerobot1str + ',' + xx1str + ',' + yy1str + ',' + angle1str)


    return message