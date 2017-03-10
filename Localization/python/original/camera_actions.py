import numpy as np
import cv2
import robot_structure
from implementation import*

num_active_robots = 1

red_lower = np.array([100,0,0],dtype="uint8")
red_upper = np.array([255,30,30],dtype="uint8")

green_lower = np.array([0,100,0],dtype="uint8")
green_upper = np.array([30,255,30],dtype="uint8")

blue_lower = np.array([0,0,100],dtype="uint8")
blue_upper = np.array([30,30,255],dtype="uint8")

yellow_lower = np.array([100,100,0],dtype="uint8")
yellow_upper = np.array([255,255,30],dtype="uint8")

def acquire_locations(img, robot):
	if(robot.ID is 1):
		lower = red_lower
		upper = red_upper
	elif(robot.ID is 2):
		lower = green_lower
		upper = green_upper
	elif(robot.ID is 3):
		lower = blue_lower
		upper = blue_upper
	elif(robot.ID is 4):
		lower = yellow_lower
		upper = yellow_upper
	else:
		print "ERROR Robot does not exist"
		
	mask = cv2.inRange(img, lower, upper)
	
	cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	
	if len(cnts) > 0:
		c = max(cnts,key=cv2.contourArea)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		dir = 0.0
		robot.setPos(center[0],center[1], dir)
		print "Robot center %d %d" %(center[0],center[1])
	else:
		print "Robot not in camera-view"

def path_finding(Map, robot, goals):
	cost = np.zeros(len(goals),dtype="int64")
	path = [{} for _ in range(len(goals))]
	for i in range(0, len(goals)):
		temp_path, temp_cost= dijkstra_search(Map,robot.getPosXY(),goals[i])
		cost[i] = temp_cost[goals[i]]
		path[i] = temp_path
	min = cost.min()
	return path[cost.argmin()], cost.argmin()
		