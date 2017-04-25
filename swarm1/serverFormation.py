from socket import *
from robot_structure import Robot
from Rendevous import rendezvous
from direction import direction
from messagepackage import messagepackage
from camera_actions import *
import sys
import math
import cv2
import time

cap = cv2.VideoCapture(1)

ROBOTS = []

# fourcc = cv2.cv.CV_FOURCC('X','V','I','D')
video_writer = cv2.VideoWriter("output3.avi", -1, 20, (640, 480))

# Create four robots
robot1 = Robot(BLUE, BLUE, BLUE)  # Blue
robot2 = Robot(BLUE, YELLOW, YELLOW)  # Green
robot3 = Robot(YELLOW, BLUE, BLUE)  # Red
robot4 = Robot(YELLOW, YELLOW, YELLOW)  # Yellow

# Current map
# Robot1 F8:F0:05:F1:D6:1C  - .6 - blue
# Robot2 F8:F0:05:F7:FF:F9  - .2 - red red blue
# Robot3 F8:F0:05:F7:FF:F1  - .4 - red
# Robot4 F8:F0:05:F7:FF:F2  - .5 - yellow

robot1.HOST = '192.168.1.6'  # blue robot
robot2.HOST = '192.168.1.4'  # red red blue
robot3.HOST = '192.168.1.2'  # red robot
robot4.HOST = '192.168.1.7'  # Yellow Robot

ROBOTS.append(robot1)
ROBOTS.append(robot2)
ROBOTS.append(robot3)
ROBOTS.append(robot4)

PORT = 2390
BUFSIZE = 1024
FLAG = 0

udpSerSock = socket(AF_INET, SOCK_DGRAM)

i = 0
count = 1
MESSAGE1 = "q"
currMESSAGE1 = "q"
MESSAGE2 = "q"
currMESSAGE2 = "q"
MESSAGE3 = "q"
currMESSAGE3 = "q"
MESSAGE4 = "q"
currMESSAGE4 = "q"
fwdcount1 = 1
rotcount1 = 1
fwdcount2 = 1
rotcount2 = 1
rotcount3 = 1
fwdcount3 = 1
rotcount4 = 1
fwdcount4 = 1
a1 = 1
a2 = 1
a3 = 1
a4 = 1

ADDR = []
for i in range(0, len(ROBOTS)):
    address = (ROBOTS[i].HOST, PORT)
    ADDR.append(address)


def update_locations(bgr_image):
    orig_image = bgr_image.copy()
    blur_image = cv2.medianBlur(bgr_image, 3)
    hsv_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
    for i in range(0, len(ROBOTS)):
        ROBOTS[i].inview = find_robot(hsv_image, orig_image, ROBOTS[i])
    return


while True:
    ret, bgr_image = cap.read()
    cv2.imshow("cam_image", bgr_image)

    update_locations(bgr_image)

    video_writer.write(bgr_image)

    print "iteration :%d" % i

    if i == 25:
        print "Iterations - done!"
        MESSAGE = "stop"
        for x in range(0, len(ROBOTS)):
            ADDR = (ROBOTS[x].HOST, PORT)
            udpSerSock.sendto(MESSAGE, ADDR)
        print "Sending message : ", MESSAGE
        cap.release()
        video_writer.release()
        cv2.destroyAllWindows()
        udpSerSock.close()
        exit(0)
    # initialize locations so that all robots have a starting location before sending move commands.
    if i > 10:
        (xpos1, ypos1, angle1) = Formation(ROBOTS[0], ROBOTS[1], ROBOTS[2], ROBOTS[3]) #blue
        (xpos2, ypos2, angle2) = Formation(ROBOTS[1], ROBOTS[0], ROBOTS[3], ROBOTS[2]) #green
        (xpos3, ypos3, angle3) = Formation(ROBOTS[2], ROBOTS[3], ROBOTS[0], ROBOTS[1]) #red
        (xpos4, ypos4, angle4) = Formation(ROBOTS[3], ROBOTS[2], ROBOTS[1], ROBOTS[0])  # Yellow

        MESSAGE1 = messagepackage(robot1, xpos1, ypos1, angle1)
        MESSAGE2 = messagepackage(robot2, xpos2, ypos2, angle2)
        MESSAGE3 = messagepackage(robot3, xpos3, ypos3, angle3)
        MESSAGE4 = messagepackage(robot4, xpos4, ypos4, angle4)

        # print("robot1blue", MESSAGE1, robot1.xpos, robot1.ypos, xpos1, ypos1, robot1.dir, angle1)
        # print("robot2blueyellow", MESSAGE2, robot2.xpos, robot2.ypos, xpos2, ypos2, robot2.dir, angle2)
        # print("robot3yellowblueblue", MESSAGE3, robot3.xpos, robot3.ypos, xpos3, ypos3, robot3.dir, angle3)
        # print("robot4Yellow", MESSAGE4, robot4.xpos, robot4.ypos, xpos4, ypos4, robot4.dir, angle4)

    i = i + 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
video_writer.release()
cv2.destroyAllWindows()
udpSerSock.close()
