from socket import *
from robot_structure import Robot
from Rendevous import rendezvous
from direction import direction
from messagepackage import*
from Formation import Formation
from camera_actions import *
import sys
import math
import cv2
import time

RENDEZVOUS = 0
FORMATION = 1
FOLLOW = 2
PATHFIND = 3
PATHFOLLOW = 4

ROBOT_MODE = PATHFOLLOW

cap = cv2.VideoCapture(0)

ROBOTS = []

# fourcc = cv2.cv.CV_FOURCC('X','V','I','D')
video_writer = cv2.VideoWriter("output3.avi", -1, 20, (640, 480))

# Create four robots
robot1 = Robot(BLUE, BLUE, BLUE)
robot2 = Robot(VIOLET, VIOLET, VIOLET)
robot3 = Robot(RED, RED, RED)
robot4 = Robot(YELLOW, YELLOW, YELLOW)

# Current map
# Robot1 F8:F0:05:F1:D6:1C  - .6 - blue
# Robot2 F8:F0:05:F7:FF:F9  - .2 - red red blue
# Robot3 F8:F0:05:F7:FF:F1  - .4 - red
# Robot4 F8:F0:05:F7:FF:F2  - .5 - yellow

robot1.HOST = '192.168.1.6' # blue robot
robot2.HOST = '192.168.1.3' # red red blue
robot3.HOST = '192.168.1.5' # red robot
robot4.HOST = '192.168.1.4' # Yellow Robot

LEAD_ROBOT = 2

ROBOTS.append(robot1)
ROBOTS.append(robot2)
ROBOTS.append(robot3)

PORT = 2390
BUFSIZE = 1024
FLAG = 0

udpSerSock = socket(AF_INET, SOCK_DGRAM)

ADDR = []
MESSAGES = []
XPOS = []
YPOS = []
ANGLE = []
for i in range(0, len(ROBOTS)):
	address = (ROBOTS[i].HOST, PORT)
	message = "s"
	xpos = 0.0
	ypos = 0.0
	angle = 0.0
	ADDR.append(address)
	MESSAGES.append(message)
	XPOS.append(xpos)
	YPOS.append(ypos)
	ANGLE.append(angle)

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

i = 0
while True:
	ret, bgr_image = cap.read()
	cv2.imshow("cam_image", bgr_image)
	height, width, channels = bgr_image.shape
	robotwidth = 85.0
	hsv_image = update_locations(bgr_image)

	video_writer.write(bgr_image)

	print "iteration :%d" % i
	
	if ROBOT_MODE == PATHFIND or ROBOT_MODE == PATHFOLLOW:
		Map = GridWithWeights(int(round(width)/robotwidth),int(round(height)/robotwidth))
		acquire_obstacles(hsv_image, Map, robotwidth)

	if i == 1000:
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
	
		if ROBOT_MODE == RENDEZVOUS:
			(XPOS[0], YPOS[0], ANGLE[0]) = rendezvous(ROBOTS[0], ROBOTS[1], ROBOTS[2])
			(XPOS[1], YPOS[1], ANGLE[1]) = rendezvous(ROBOTS[1], ROBOTS[0], ROBOTS[3])
			(XPOS[2], YPOS[2], ANGLE[2]) = rendezvous(ROBOTS[2], ROBOTS[3], ROBOTS[0])
			(XPOS[3], YPOS[3], ANGLE[3]) = rendezvous(ROBOTS[3], ROBOTS[2], ROBOTS[1])
		elif ROBOT_MODE == FORMATION:
			(XPOS[0], YPOS[0], ANGLE[0]) = Formation(ROBOTS[0], ROBOTS[1], ROBOTS[2], ROBOTS[3])
			(XPOS[1], YPOS[1], ANGLE[1]) = Formation(ROBOTS[1], ROBOTS[0], ROBOTS[3], ROBOTS[2])
			(XPOS[2], YPOS[2], ANGLE[2]) = Formation(ROBOTS[2], ROBOTS[3], ROBOTS[0], ROBOTS[1])
			(XPOS[3], YPOS[3], ANGLE[3]) = Formation(ROBOTS[3], ROBOTS[2], ROBOTS[1], ROBOTS[0])
		elif ROBOT_MODE == FOLLOW:
			(XPOS[0], YPOS[0], ANGLE[0]) = rendezvous(ROBOTS[0], ROBOTS[1], ROBOTS[1])
			(XPOS[1], YPOS[1], ANGLE[1]) = rendezvous(ROBOTS[1], ROBOTS[2], ROBOTS[2])
			(XPOS[2], YPOS[2], ANGLE[2]) = rendezvous(ROBOTS[2], ROBOTS[3], ROBOTS[3])
		elif ROBOT_MODE == PATHFIND:
			for k in range(0, len(ROBOTS)):
				Start = (int((ROBOTS[k].getX()/robotwidth)),int((ROBOTS[k].getY())/robotwidth))
				print "Robot start position:(%d,%d)" %(Start[0],Start[1])
		
				Goal = (1,0)
		
				if Start != Goal :
					red_rover_path, red_rover_cost = a_star_search(Map,Start,Goal)
					draw_grid(Map, width=3, path=reconstruct_path(red_rover_path, start=Start,goal=Goal))
					print "\n"
			
					p = Goal
					path = []
					for i in range(0, red_rover_cost[Goal]):
						path.append(p)
						p = red_rover_path[p]
			
					Next = (path[red_rover_cost[Goal]-1][0],path[red_rover_cost[Goal]-1][1])

					XPOS[k] = (Next[0]) * (robotwidth/2)
					YPOS[k] = (Next[1]) * (robotwidth/2)
					ANGLE[k] = get_angle(Start, Next)
			
				else:
					XPOS[k] = ROBOTS[k].xpos
					YPOS[k] = ROBOTS[k].ypos
					ANGLE[k] = ROBOTS[k].dir
					udpSerSock.sendto("s",ADDR[k])
					ADDR[k] = (robot4.HOST, PORT)
		elif ROBOT_MODE == PATHFOLLOW:
			Start = (int((ROBOTS[LEAD_ROBOT].getX()/robotwidth)),int((ROBOTS[LEAD_ROBOT].getY())/robotwidth))
			print "Robot start position:(%d,%d)" %(Start[0],Start[1])
	
			Goal = (1,0)
	
			if Start != Goal :
				red_rover_path, red_rover_cost = a_star_search(Map,Start,Goal)
				draw_grid(Map, width=3, path=reconstruct_path(red_rover_path, start=Start,goal=Goal))
				print "\n"
		
				p = Goal
				path = []
				for i in range(0, red_rover_cost[Goal]):
					path.append(p)
					p = red_rover_path[p]
			
				Next = (path[red_rover_cost[Goal]-1][0],path[red_rover_cost[Goal]-1][1])
				
				print "Robot next position: (%d,%d)" %(Next[0],Next[1])

				XPOS[LEAD_ROBOT] = (Next[0]) * (robotwidth)
				YPOS[LEAD_ROBOT] = (Next[1]) * (robotwidth)
				ANGLE[LEAD_ROBOT] = get_angle(Start, Next)
		
			else:
				XPOS[LEAD_ROBOT] = ROBOTS[LEAD_ROBOT].xpos
				YPOS[LEAD_ROBOT] = ROBOTS[LEAD_ROBOT].ypos
				ANGLE[LEAD_ROBOT] = ROBOTS[LEAD_ROBOT].dir
				udpSerSock.sendto("s",ADDR[LEAD_ROBOT])
				ADDR[LEAD_ROBOT] = (robot4.HOST, PORT)

			XPOS[0], YPOS[0], ANGLE[0] = rendezvous(ROBOTS[0], ROBOTS[LEAD_ROBOT], ROBOTS[LEAD_ROBOT])
			XPOS[1], YPOS[1], ANGLE[1] = rendezvous(ROBOTS[1], ROBOTS[0],ROBOTS[0])
		else:
			print "ERROR OCCURED"
			exit(0)
			
		for k in range(0, len(ROBOTS)):
			MESSAGES[k] = messagepackage2(ROBOTS[k],XPOS[k],YPOS[k],ANGLE[k])
			
		for k in range(0, len(ROBOTS)):
			udpSerSock.sendto(MESSAGES[k], ADDR[k])
		
		for k in range(0, len(ROBOTS)):
			print(k, "MESSAGE:", MESSAGES[k], "Robot xpos:", ROBOTS[k].xpos, "Robot ypos:", ROBOTS[k].ypos, "Robot dir:", ROBOTS[k].dir,
			"Goal xpos:", XPOS[k], "Goal ypos:", YPOS[k], "Goal angle:", ANGLE[k])

	i = i + 1

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
video_writer.release()
cv2.destroyAllWindows()
udpSerSock.close()
