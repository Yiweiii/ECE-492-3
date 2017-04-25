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

cap = cv2.VideoCapture(0)

ROBOTS = []

# fourcc = cv2.cv.CV_FOURCC('X','V','I','D')
video_writer = cv2.VideoWriter("output3.avi", -1, 20, (640, 480))

# Create four robots
robot1 = Robot(BLUE, BLUE, BLUE) # Blue
robot2 = Robot(BLUE, YELLOW, YELLOW ) # Green
robot3 = Robot(RED, RED, RED) # Red
robot4 = Robot(YELLOW, BLUE, BLUE) # Yellow

# Current map
# Robot1 F8:F0:05:F1:D6:1C  - .6 - blue
# Robot2 F8:F0:05:F7:FF:F9  - .2 - red red blue
# Robot3 F8:F0:05:F7:FF:F1  - .4 - red
# Robot4 F8:F0:05:F7:FF:F2  - .5 - yellow

robot1.HOST = '192.168.1.7' # blue robot
robot2.HOST = '192.168.1.4' # red red blue
robot3.HOST = '192.168.1.2' # red robot
robot4.HOST = '192.168.1.6' # Yellow Robot

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
	return hsv_image
	
def get_angle(start, next):
	(x, y) = start
	results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1),(x+1, y+1),(x-1, y-1),(x-1, y+1),(x+1, y-1)]
	if results[0] == next:
		return 0.0
	elif results[1] == next:
		return 90.0
	elif results[2] == next:
		return 180.0
	elif results[3] == next:
		return 270.0
	elif results[4] == next:
		return 45.0
	elif results[5] == next:
		return 135.0
	elif results[6] == next:
		return 215.0
	elif results[7] == next:
		return 315.0
	else:
		return 0.0
		print "error"
		
while True:
    ret, bgr_image = cap.read()
    cv2.imshow("cam_image", bgr_image)
    height, width, channels = bgr_image.shape
    robotwidth = 80.0
    hsv_image = update_locations(bgr_image)
	
    video_writer.write(bgr_image)

    print "iteration :%d" % i
    Map = GridWithWeights(int(round(width)/robotwidth),int(round(height)/robotwidth))
    acquire_obstacles(hsv_image, Map, robotwidth)
    ROBOTS[3].displayRobot()
	


    if i == 100:
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
    if i > 10:
        (xpos1, ypos1, angle1) = rendezvous(ROBOTS[0], ROBOTS[1], ROBOTS[2]) #blue
        (xpos2, ypos2, angle2) = rendezvous(ROBOTS[1], ROBOTS[0], ROBOTS[3]) #green
        (xpos3, ypos3, angle3) = rendezvous(ROBOTS[2], ROBOTS[3], ROBOTS[0]) #red
        (xpos4, ypos4, angle4) = rendezvous(ROBOTS[3], ROBOTS[2], ROBOTS[1]) #yellow
		
        Start = (int(round(ROBOTS[3].getX()/robotwidth)),int(round(ROBOTS[3].getY()/robotwidth)))
        print "Robot start position:(%d,%d)" %(Start[0],Start[1])
        Goal = (3,3)
        if Start != Goal :
			red_rover_path, red_rover_cost = a_star_search(Map,Start,Goal)
			draw_grid(Map, width=3, path=reconstruct_path(red_rover_path, start=Start,goal=Goal))
			print "\n"
			p = Goal
			path = []
			for i in range(0, red_rover_cost[Goal]):
				path.append(p)
				p = red_rover_path[p]
			
			print(path)
			Next = (path[red_rover_cost[Goal]-1][0],path[red_rover_cost[Goal]-1][1])

			xpos4 = (Next[0]) * (robotwidth/2)
			ypos4 = (Next[1]) * (robotwidth/2)
			angle4 = get_angle(Start, Next)
        else:
			xpos4 = ROBOTS[3].xpos
			ypos4 = ROBOTS[3].ypos
			angle4 = ROBOTS[3].dir
			MESSAGE4 = "s"
			udpSerSock.sendto(MESSAGE4, ADDR[3])
			cap.release()
			video_writer.release()
			cv2.destroyAllWindows()
			udpSerSock.close()
		
        #xpos4 = 300
        #ypos4 = 300
        #angle4 = 180
		
        print "Robot xgoal: %d, ygoal:%d, agoal:%d" %(xpos4, ypos4, angle4)

        MESSAGE1 = messagepackage(robot1, xpos1, ypos1, angle1)
        MESSAGE2 = messagepackage(robot2, xpos2, ypos2, angle2)
        MESSAGE3 = messagepackage(robot3, xpos3, ypos3, angle3)
        MESSAGE4 = messagepackage(robot4, xpos4, ypos4, angle4)
        udpSerSock.sendto(MESSAGE1, ADDR[0])
        # currMESSAGE1 = MESSAGE1
        udpSerSock.sendto(MESSAGE2, ADDR[1])
        #   currMESSAGE2 = MESSAGE2
        udpSerSock.sendto(MESSAGE3, ADDR[2])
        #    currMESSAGE3 = MESSAGE3
        udpSerSock.sendto(MESSAGE4, ADDR[3])
        #   currMESSAGE4 = MESSAGE4

        print("robot1blue", MESSAGE1, robot1.xpos, robot1.ypos, xpos1, ypos1, robot1.dir, angle1)
        print("robot2blueyellow", MESSAGE2, robot2.xpos, robot2.ypos, xpos2, ypos2, robot2.dir, angle2)
        print("robot3red", MESSAGE3, robot3.xpos, robot3.ypos, xpos3, ypos3, robot3.dir, angle3)
        print("robot4Yellow", MESSAGE4, robot4.xpos, robot4.ypos, xpos4, ypos4, robot4.dir, angle4)

    i = i + 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
video_writer.release()
cv2.destroyAllWindows()
udpSerSock.close()
