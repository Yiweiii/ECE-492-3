from socket import *
from robot_structure import Robot
from Rendevous import rendezvous
import sys
import math


##Create four robots
robot1 = Robot(1)
robot2 = Robot(2)
robot3 = Robot(3)
robot4 = Robot(4)

##set positions will be from video
robot1.setPos(10, 10, 0)
robot2.setPos(0, 0, 0)
robot3.setPos(0, 0, 0)
robot4.setPos(0, 0, 0)

HOST = '192.168.1.4'
PORT = 2390
BUFSIZE = 1024
FLAG = 0

ADDR = (HOST, PORT)

udpSerSock = socket(AF_INET, SOCK_DGRAM)

angle1 = -90
xpos1 = -10
while True:

    if robot1.dir != angle1:  ## rotate untill rotate is good
        MESSAGE = "A"
        angle1 = angle1 + 10;
    elif robot1.xpos != xpos1: ## Move untill xpos is good
       MESSAGE = 'a'
       xpos1 = xpos1 + 1;
    else:
        MESSAGE = 'stop'
        ##rerun Rendezvous function

    ##MESSAGE = raw_input('>')
    print "send message: ", MESSAGE

    udpSerSock.sendto(MESSAGE, ADDR)
    try:
        data, ADDR = udpSerSock.recvfrom(BUFSIZE)
        print(data)
    except:
        data = "FAIL"

    if data == "ACK0":
        print "ACK received"
        print "batterylow"
    elif data == "ACK1":
        print "batteryhigh"
    elif data == "FAIL":
        print "Robot not connected"
    else:
        print "Message not recieved by robot, re-send"

udpSerSock.close()

