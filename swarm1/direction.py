from socket import *
from robot_structure import Robot
from Rendevous import rendezvous
import sys
import math
import camera_actions as ca
import cv2
import time


def direction(robot, xpos, ypos, angle, rotcount, fwdcount, a):

    lower_range = angle - 25
    upper_range = angle + 25

    if lower_range < 0:
        lower_range = 0
    if upper_range > 360:
        upper_range = 360

    xpos_lower_range = xpos - 2
    xpos_upper_range = xpos + 2
    ypos_lower_range = ypos - 2
    ypos_upper_range = ypos + 2

    if (robot.dir < lower_range or robot.dir > upper_range):  ## rotate untill rotate is good
        if rotcount == 1:
            a = (angle - robot.dir + 360) % 360
            rotcount = 2
        if a <= 180:
            message = 'a'
        else:
            message = 'A'
    elif (robot.xpos < xpos_lower_range or robot.xpos > xpos_upper_range) or (
            robot.ypos < ypos_lower_range or robot.ypos > ypos_upper_range):  ## Move untill xpos is good
        if fwdcount == 1:
            message = 's'
            fwdcount = fwdcount + 1
        else:
            message = 'f'
    else:
        message = 'stop'
        fwdcount = 1
        rotcount = 1

    return message, rotcount, fwdcount, a