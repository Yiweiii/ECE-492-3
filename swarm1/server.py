from socket import *
from robot_structure import Robot
from Rendevous import rendezvous
import sys
import math
import camera_actions as ca
import cv2
import time

cap = cv2.VideoCapture(0)

#fourcc = cv2.cv.CV_FOURCC('X','V','I','D')
video_writer = cv2.VideoWriter("output3.avi", -1, 20, (640, 480))

##Create four robots
robot1 = Robot(ca.BLUE)  # Blue
robot2 = Robot(ca.GREEN)  # Green
robot3 = Robot(ca.RED)

<<<<<<< HEAD
#Blue Robot
HOST1 = '192.168.1.7' # blue robot
HOST2 = '192.168.1.3' # green robot
HOST3 = '192.168.1.8' # red robot
=======
# robot2.setPos(40,40,0)
# robot3.setPos(40,40,0)

HOST = '192.168.1.3'
>>>>>>> parent of 7c858e0... Add files via upload
PORT = 2390
BUFSIZE = 1024
FLAG = 0

ADDR1 = (HOST1, PORT)   # blue robot
ADDR2 = (HOST2, PORT) # green robot
ADDR3 = (HOST3, PORT) # red robot

udpSerSock = socket(AF_INET, SOCK_DGRAM)

<<<<<<< HEAD
#Red robot

i = 0
count = 1
MESSAGE1 = "q"
currMESSAGE1 = "q"
MESSAGE2 = "q"
currMESSAGE2 = "q"
MESSAGE3 = "q"
currMESSAGE3 = "q"
fwdcount1 = 1
rotcount1 = 1
fwdcount2 = 1
rotcount2 = 1
rotcount3 = 1
fwdcount3 = 1
a1 = 1
a2 = 1
a3 = 1

=======

i = 0
count = 1
rotcount = 1
MESSAGE = "q"
currMESSAGE = "q"
fwdcount = 1
>>>>>>> parent of 7c858e0... Add files via upload
while True:
    ret, bgr_image = cap.read()
    cv2.imshow("cam_image", bgr_image)
    orig_image = bgr_image.copy()
    bgr_image = cv2.medianBlur(bgr_image, 3)
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    hue_image = ca.ID_hue_image(hsv_image, robot1.ID, orig_image)
    #cv2.imshow("hue",hue_image)
    hue_image2 = ca.ID_hue_image(hsv_image, robot2.ID, orig_image)
    hue_image3 = ca.ID_hue_image(hsv_image, robot3.ID, orig_image)
    robot_in_view = ca.acquire_locations(hue_image, robot1)
    ca.acquire_locations(hue_image2, robot2)
    ca.acquire_locations(hue_image3, robot3)
    video_writer.write(bgr_image)

    if robot_in_view:

        print(i)
        print "Robot 1 x:%d y:%d dir:%d" % (robot1.xpos, robot1.ypos, robot1.dir)
        print "Robot 2 x:%d y:%d dir:%d" % (robot2.xpos, robot2.ypos, robot2.dir)
        print "Robot 3 x:%d y:%d dir:%d" % (robot3.xpos, robot3.ypos, robot3.dir)

        (xpos1, ypos1, angle1) = rendezvous(robot1, robot2, robot3)
        if i == 500:
            MESSAGE = 'stop'
<<<<<<< HEAD
            udpSerSock.sendto(MESSAGE, ADDR1)
            udpSerSock.sendto(MESSAGE, ADDR2)
            udpSerSock.sendto(MESSAGE, ADDR3)
=======
            udpSerSock.sendto(MESSAGE, ADDR)
>>>>>>> parent of 7c858e0... Add files via upload
            cap.release()
            cv2.destroyAllWindows()
            udpSerSock.close()
            video_writer.release()
            exit(0)

<<<<<<< HEAD
        (xpos1, ypos1, angle1) = rendezvous(robot1, robot2, robot3) #blue
        (xpos2, ypos2, angle2) = rendezvous(robot2, robot1, robot3) #green
        (xpos3, ypos3, angle3) = rendezvous(robot3, robot2, robot1) #red
        (MESSAGE1, rotcount1, fwdcount1, a1) = direction(robot1, xpos1, ypos1, angle1, rotcount1, fwdcount1, a1)
        (MESSAGE2, rotcount2, fwdcount2, a2) = direction(robot2, xpos2, ypos2, angle2, rotcount2, fwdcount2, a2)
        (MESSAGE3, rotcount3, fwdcount3, a3) = direction(robot3, xpos3, ypos3, angle3, rotcount3, fwdcount3, a3)
        print("robot1blue", MESSAGE1, robot1.xpos, robot1.ypos, xpos1, ypos1, robot1.dir, angle1)
        print("robot2green", MESSAGE2, robot2.xpos, robot2.ypos, xpos2, ypos2, robot2.dir, angle2)
        print("robot3red", MESSAGE3, robot3.xpos, robot3.ypos, xpos3, ypos3, robot3.dir, angle3)

        if (MESSAGE1 != currMESSAGE1):
            udpSerSock.sendto(MESSAGE1, ADDR1)
            currMESSAGE1 = MESSAGE1
        if (MESSAGE2 != currMESSAGE2):
            udpSerSock.sendto(MESSAGE2, ADDR2)
            currMESSAGE2 = MESSAGE2
        if (MESSAGE3 != currMESSAGE3):
            udpSerSock.sendto(MESSAGE3, ADDR3)
            currMESSAGE3 = MESSAGE3
=======
        lower_range = angle1 - 25
        upper_range = angle1 + 25
		
        if lower_range < 0:
           lower_range = 0
        if upper_range > 360:
           upper_range = 360

        xpos_lower_range = xpos1 - 2
        xpos_upper_range = xpos1 + 2
        ypos_lower_range = ypos1 - 2
        ypos_upper_range = ypos1 + 2
        print "angle 1: %d" % angle1
        print "xpos1 : %d " % xpos1
        print "ypos1 : %d " % ypos1


        if (robot1.dir < lower_range or robot1.dir > upper_range) and (count == 1):  ## rotate untill rotate is good
            if rotcount == 1:
                a = (angle1 - robot1.dir +360 )% 360
                rotcount = 2
            if a <= 180:
                MESSAGE = 'a'
            else:
                MESSAGE = 'A'
            print "a : %d" % a
            print "Robot 1: x:%d y:%d dir:%d " % (robot1.xpos, robot1.ypos, robot1.dir)
        elif (robot1.xpos < xpos_lower_range or robot1.xpos > xpos_upper_range) or (robot1.ypos < ypos_lower_range or robot1.ypos > ypos_upper_range) :  ## Move untill xpos is good
            if fwdcount == 1:
                MESSAGE = 's'
                print(MESSAGE)
                fwdcount = fwdcount + 1
            else:
                MESSAGE = 'f'
                print(MESSAGE)
        else:
            MESSAGE = 'stop'
            (xpos1, ypos1, angle1) = rendezvous(robot1, robot2, robot3)
            fwdcount = 1
            rotcount = 1

        ##MESSAGE = raw_input('>')
        # print "send message: ", MESSAGE
        # i = i + 1

        # print(currMESSAGE)
        # print(MESSAGE)
        if (MESSAGE != currMESSAGE):
            udpSerSock.sendto(MESSAGE, ADDR)
            print "send message: ", MESSAGE
            currMESSAGE = MESSAGE
>>>>>>> parent of 7c858e0... Add files via upload
        i = i + 1
    # try:
    # print(data)
    # data, ADDR = udpSerSock.recvfrom(BUFSIZE)
    # except:
    # print "FAIL"
    # data = "FAIL"

    # if data == "ACK0":
    # print "ACK received"
    # print "batterylow"
    # elif data == "ACK1":
    # print "batteryhigh"
    # elif data == "FAIL":
    # print "Robot not connected"
    # else:
    # print "Message not recieved by robot, re-send"

    else:
        print "Robot not in view - stopping robot"
        MESSAGE = "stop"
<<<<<<< HEAD
        udpSerSock.sendto(MESSAGE, ADDR1)
        udpSerSock.sendto(MESSAGE, ADDR2)
        udpSerSock.sendto(MESSAGE, ADDR3)
=======
        udpSerSock.sendto(MESSAGE, ADDR)
>>>>>>> parent of 7c858e0... Add files via upload
        print "send message : ", MESSAGE
        #exit(0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

        # time.sleep(1)

cap.release()
video_writer.release()
cv2.destroyAllWindows()
udpSerSock.close()
