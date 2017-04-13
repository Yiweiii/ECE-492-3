from socket import *
from robot_structure import Robot
from Rendevous import rendezvous
import sys
import math
import camera_actions as ca
import cv2
import time


def direction(robot, xpos, ypos, angle, rotcount, fwdcount, a):

    controlrange = (abs(angle - robot.dir)/7)

    lower_range = angle - 25
    upper_range = angle + 25

    if lower_range < 0:
        lower_range = 0
    if upper_range > 360:
        upper_range = 360

    xpos_lower_range = xpos - 10
    xpos_upper_range = xpos + 10
    ypos_lower_range = ypos - 10
    ypos_upper_range = ypos + 10

    mag = math.sqrt(((xpos - robot.xpos)*(xpos-robot.xpos)) + ((ypos - robot.ypos)*(ypos-robot.ypos)))
    mag = int(50 + 1.5*mag)
    if mag >= 200:
        mag = 200

    rotmag = abs(angle - robot.dir)
    rotmag = int(50 + 5*rotmag/6)
    if rotmag >= 200:
        rotmag = 200

    if (robot.xpos > xpos_lower_range and robot.xpos < xpos_upper_range) and (
            robot.ypos > ypos_lower_range and robot.ypos < ypos_upper_range):
        message = 'stop'
        fwdcount = 1
        rotcount = 1
    elif (robot.dir > lower_range and robot.dir < upper_range):
        rotcount = 1
        if fwdcount == 1:
            message = 's'
            fwdcount = fwdcount + 1
        else:
            message = 'f' + str(mag)
    else:
        if rotcount == 1:
            a = (angle - robot.dir + 360) % 360
            rotcount = 2
        if a <= 180:
            message = 'a' + str(rotmag)
        else:
            message = 'A' + str(rotmag)



    return message, rotcount, fwdcount, a