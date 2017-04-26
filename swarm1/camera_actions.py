import numpy as np
import cv2
import robot_structure as rs
from implementation import*
from kalman import *
import sys
import math
import operator
import time

RED = 1
GREEN = 2
BLUE = 3
YELLOW = 4
ORANGE = 5
VIOLET = 6

red_lower_a = np.array([0,180,100])
red_upper_a = np.array([10,255,255])
red_lower_b = np.array([170, 100, 100])
red_upper_b = np.array([180,255,255])

green_lower = np.array([37,81,158])
green_upper = np.array([83,119,247])

blue_lower = np.array([90,50,255])
blue_upper = np.array([110,255,255])

yellow_lower = np.array([25,100,100])
yellow_upper = np.array([35,255,255])

violet_lower = np.array([115,100,160])
violet_upper = np.array([125,255,255])

orange_lower = np.array([10, 100, 180])
orange_upper = np.array([80,255,255])

Robotx_est = np.asmatrix(np.zeros((3,1)))
Robotp_est = np.asmatrix(np.zeros((3,3)))

Robotx_pre = np.asmatrix(np.zeros((3,1)))
Robotp_pre = np.asmatrix(np.zeros((3,3)))

class triangle:
	def __init__(self, x1, x2, x3):
		self.x1 = x1
		self.x2 = x2
		self.x3 = x3
		self.d1 = 1000
		self.d2 = 1000
		self.perimeter = 0

	def get_perimeter(self):
		return self.d1 + self.d2
		
def isLeft(a, b, c):
	return ((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0])) > 0;

def ID_hue_image(img, ID, orig):
	if(ID is RED):
		lower_red_hue_range = cv2.inRange(img, red_lower_a, red_upper_a)
		upper_red_hue_range = cv2.inRange(img, red_lower_b, red_upper_b)
		mask = cv2.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0)
		hue_image = cv2.GaussianBlur(mask, (9,9), 2, 2)
		return hue_image
	elif(ID is GREEN):
		lower = green_lower
		upper = green_upper
	elif(ID is BLUE):
		lower = blue_lower
		upper = blue_upper
	elif(ID is YELLOW):
		lower = yellow_lower
		upper = yellow_upper
	elif ID is ORANGE:
		lower = orange_lower
		upper = orange_upper
	elif ID is VIOLET:
		lower = violet_lower
		upper = violet_upper
	else:
		print "ERROR Robot does not exist"
		
	mask = cv2.inRange(img, lower, upper)
	hue_image = cv2.GaussianBlur(mask, (9,9), 2, 2)
	
	output = cv2.bitwise_and(orig,orig, mask=mask)
	#cv2.imshow("out",output)
	#cv2.waitKey(0)
	return hue_image
	
def main():
	if len(sys.argv) == 3:
		print "File mode"
		image_path = sys.argv[1]
		bgr_image = cv2.imread(image_path)
		
		orig_image = bgr_image.copy()
			
		bgr_image = cv2.medianBlur(bgr_image, 3)
		hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
	else:
		print "Active mode"
		cap = cv2.VideoCapture(0)
		while(True):
			ret, bgr_image = cap.read()
			cv2.imshow("cam_image", bgr_image)
			#width, height = frame.shape

			orig_image = bgr_image.copy()
			bgr_image = cv2.medianBlur(bgr_image, 3)
			hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
				
			Robot1 = rs.Robot(YELLOW, BLUE, BLUE)
			Robot2 = rs.Robot(BLUE, YELLOW, YELLOW)
			Robot3 = rs.Robot(VIOLET, VIOLET, VIOLET)
			Robot4 = rs.Robot(RED, RED, RED)

			print "\nfind YELLOW BLUE BLUE"
			find_robot(hsv_image,orig_image,Robot1)
			
			print "\nfind BLUE YELLOW YELLOW"
			find_robot(hsv_image,orig_image,Robot2)
			
			print "\nfind VIOLET VIOLET VIOLET"
			find_robot(hsv_image,orig_image,Robot3)
			
			print "\nfind RED RED RED"
			find_robot(hsv_image,orig_image,Robot4)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

		cap.release()
		cv2.destroyAllWindows()

def acquire_obstacles(img, Map, max_height):
	Walls = []
	hue_image = ID_hue_image(img, GREEN, img)
	#cv2.imshow("img", hue_image)
	#cv2.waitKey(0)
	cnts = cv2.findContours(hue_image.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	#print "\nlen(cnts) : %d" % len(cnts)
	for i in range(0,len(cnts)):
		c = cnts[i]
		x,y,w,h = cv2.boundingRect(c)
		# "x : %d y : %d w : %d h : %d" % (x,y,w,h)
		
		j = int(round(w/max_height))
		k = int(round(h/max_height))
		l = int(round(x/max_height))
		m = int(round(y/max_height))
		#print(m)
		
		for z in range(0, j):
			p = (l+z,m)
			Walls.append(p)
		for q in range(0, k):
			p = (l,m+q)
			Walls.append(p)
	#print(Walls)
	Map.walls = Walls
	
	
def find_robot(hsv_image, orig_image, robot):
	color1 = robot.c1
	color2 = robot.c2
	color3 = robot.c3
	found = False
	hue_image_1 = ID_hue_image(hsv_image, color1, orig_image)
	cnts = cv2.findContours(hue_image_1.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	points_array_1 = []
	final_coordinates = []
	for i in range(0, len(cnts)):
		c = cnts[i]
		M = cv2.moments(c)
		a = int(M["m10"] / M["m00"])
		b = int(M["m01"] / M["m00"])
		points_array_1.append((a,b))
		T = triangle((a,b),[0,0],[0,0])
		final_coordinates.append(T)
	if (color1 == color2 and color2 == color3):
		cv2.imshow("hue", hue_image_1)
		if len(cnts) < 3:
			print "Error less than 3 cnts"
			return
	#print (points_array_1)
	hue_image_2 = ID_hue_image(hsv_image, color2, orig_image)
	cnts = cv2.findContours(hue_image_2.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	points_array_2 = []
	for i in range(0, len(cnts)):
		c = cnts[i]
		M = cv2.moments(c)
		a = int(M["m10"] / M["m00"])
		b = int(M["m01"] / M["m00"])
		points_array_2.append((a,b))
	#print (points_array_2)
	hue_image_3 = ID_hue_image(hsv_image, color3, orig_image)
	cnts = cv2.findContours(hue_image_3.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	#cv2.imshow("hue", hue_image_3)
	#cv2.waitKey(0)
	points_array_3 = []
	for i in range(0, len(cnts)):
		c = cnts[i]
		M = cv2.moments(c)
		a = int(M["m10"] / M["m00"])
		b = int(M["m01"] / M["m00"])
		points_array_3.append((a,b))
	#print (points_array_3)
		
	#print(len(points_array_2))
	for i in range(0,len(points_array_1)):
		d = 0
		distances_1 = []
		if (color1 == color2):
			d = d + 1
			points_array_2.remove(points_array_1[i])
		#print(points_array_2)
		for j in range(0, len(points_array_2)):
			a = (points_array_1[i][0], points_array_1[i][1])
			b = (points_array_2[j][0], points_array_2[j][1])
			x_d = math.sqrt((a[0]-b[0])**2 + (a[1] - b[1])**2)
			distances_1.append(x_d)
		if len(distances_1) == 0:
			print "error occured in localization"
			return False
		min_index, min_value = min(enumerate(distances_1), key=operator.itemgetter(1))
		if min_value < 30 and len(distances_1)>1:
			#print(min_value)
			distances_1.remove(min_value)
			min_index, min_value = min(enumerate(distances_1), key=operator.itemgetter(1))
		#print(i)
		#print "min_index : %d" %min_index
		if min_index < i :
			#print "here"
			index = min_index
		else:
			index = min_index + d
		#print "index : %d" % index
		if (color1 == color2):
			points_array_2.insert(i,points_array_1[i])
		final_coordinates[i].x2 = points_array_2[index]
		final_coordinates[i].d1 = min_value

	#print(len(points_array_3))
	for i in range(0,len(points_array_1)):
		d = 0
		distances_2 = []
		if (color1 == color3):
			d = d + 1
			points_array_3.remove(points_array_1[i])
		if (color2 == color3):
			#print(final_coordinates[i].x2)
			points_array_3.remove(final_coordinates[i].x2)
			d = d + 1
		for j in range(0, len(points_array_3)):
			a = (points_array_1[i][0], points_array_1[i][1])
			b = (points_array_3[j][0], points_array_3[j][1])
			x_d = math.sqrt((a[0]-b[0])**2 + (a[1] - b[1])**2)
			distances_2.append(x_d)
		if len(distances_2) == 0:
			print "Error occured in localization"
			return False
		min_index, min_value = min(enumerate(distances_2), key=operator.itemgetter(1))
		#print(points_array_3)
		if min_index < i :
			index = min_index
		else:
			index = min_index + d
		if (color1 == color3):
			points_array_3.insert(i,points_array_1[i])
		if (color2 == color3):
			points_array_3.insert(i,final_coordinates[i].x2)
		final_coordinates[i].x3 = points_array_3[index]
		final_coordinates[i].d2 = min_value
	
	final_perims = []
	check = True
	for i in range(0,len(points_array_1)):
		final_perims.append(final_coordinates[i].get_perimeter())
		X = (final_coordinates[i].x1,final_coordinates[i].x2, final_coordinates[i].x3)
		d1 = final_coordinates[i].d1
		d2 = final_coordinates[i].d2
		x = (final_coordinates[i].x2[0] + final_coordinates[i].x3[0])/2
		y = (final_coordinates[i].x2[1] + final_coordinates[i].x3[1])/2
		x1 = (x,y)
		#print "d1: %d d2: %d" % (d1,d2)
		if (color2 != color3):
			check = isLeft(final_coordinates[i].x1,x1,final_coordinates[i].x2)
		if ((abs(d1 - d2) < 5) and check and d1>30 and d2>30 and d1<40 and d2<40):
			#print ("\nRobot found")
			print "d1: %d d2: %d" % (d1,d2)
			#print(X)
			found = True
			Robotx, Roboty, Robotdir = thetacalc(final_coordinates[i].x1,final_coordinates[i].x2, final_coordinates[i].x3)
			robot.setPos(Robotx, Roboty, Robotdir)
			print "Robot x: %d y: %d dir : %d" %(robot.xpos, robot.ypos, robot.dir)
			new_sample = np.mat([[Robotx],[Roboty],[Robotdir]])
			kal_setup(robot.velocity, robot.ang_velocity, robot.dir)
			Robotx_pre, Robotp_pre = kal_predict(robot.x_est, robot.p_est)
			(Robotx_est, Robotp_est) = kal_predict(new_sample,Robotx_pre, Robotp_pre)
			robot.setKF(Robotx_est, Robotp_est)
		return True
		
	if found == False:
		Robotx_pre, Robotp_pre = kal_predict(robot.x_est, robot.p_est)
		Robotx = Robotx_pre[0,0]
		Roboty = Robotx_pre[1,0]
		Robotdir = Robotx_pre[2,0]
		robot.setPos(Robotx, Roboty, Robotdir)
		kal_setup(robot.velocity, robot.ang_velocity, robot.dir)
		new_sample = np.mat([[Robotx],[Roboty],[Robotdir]])
		(Robotx_est, Robotp_est) = kal_predict(new_sample,Robotx_pre, Robotp_pre)
		robot.setKF(Robotx_est, Robotp_est)
		print "\nRobot not found"
	return found
	
def thetacalc_n(a,b,c):
	final_x = a[0]
	final_y = a[1]
	x = (b[0] + c[0])/2
	y = (b[1] + c[1])/2
	x1 = (x,y)
	x2 = (a[0],a[1])
	x3 = (x2[0],y)
	dis_x = x3[0] - x1[0]
	dis_y = x3[1] - x2[1]
	dir = (math.atan2(dis_y,dis_x) * 180/3.14159)
	degrees = (dir + 360) % 360
	return final_x, final_y, degrees
	
def thetacalc(a, b, c):
	final_x = 0
	final_y = 0
	t1 = (a[0],a[1])
	t2 = (b[0],b[1])
	t3 = (c[0],c[1])
	t = [t1, t2, t3]
	##print(t)
	coord = [a, b, c]
	a_b = math.sqrt((a[0]-b[0])**2 + (a[1] - b[1])**2)
	a_c = math.sqrt((a[0]-c[0])**2 + (a[1] - c[1])**2)
	b_c = math.sqrt((b[0]-c[0])**2 + (b[1] - c[1])**2)
	values = [a_b, a_c, b_c]
	##print(values)
	min_index, min_value = min(enumerate(values), key=operator.itemgetter(1))
	if min_index is 0:
		##print ("a_b")
		x = (coord[0][0] + coord[1][0])/2
		y = (coord[0][1] + coord[1][1])/2
		x1 = (x,y)
		x2 = (coord[2][0],coord[2][1])
		x3 = (x2[0], y)
		#print (x1)
		#print (x2)
		#print (x3)
		dis_x = x3[0] - x1[0]
		dis_y = x3[1] - x2[1]
		final_x = (x + x2[0])/2;
		final_y = (y + x2[1])/2;
		#final_x = x2[0]
		#final_y = x2[1]
	elif min_index is 1:
		##print("a_c")
		x = (coord[0][0] + coord[2][0])/2
		y = (coord[0][1] + coord[2][1])/2
		x1 = (x,y)
		x2 = (coord[1][0],coord[1][1])
		x3 = (x2[0], y)
		#print (x1)
		#print (x2)
		#print (x3)
		dis_x = x3[0] - x1[0]
		dis_y = x3[1] - x2[1]
		final_x = (x + x2[0])/2;
		final_y = (y + x2[1])/2;
		#final_x = x2[0]
		#final_y = x2[1]
	else:
		##print("b_c")
		x = (coord[1][0] + coord[2][0])/2
		y = (coord[1][1] + coord[2][1])/2
		x1 = (x,y)
		x2 = (coord[0][0],coord[0][1])
		x3 = (x2[0], y)
		#print (x1)
		#print (x2)
		#print (x3)
		dis_x = x3[0] - x1[0]
		dis_y = x3[1] - x2[1]
		final_x = (x + x2[0])/2;
		final_y = (y + x2[1])/2;
		#final_x = x2[0]
		#final_y = x2[1]
		
	##print "disy : %d disx : %d" % (dis_y, dis_x)
	dir = (math.atan2(dis_y,dis_x) * 180/3.14159)
	degrees = (dir + 360) % 360
	##print "dir %d" %dir
	
	return final_x, final_y, degrees
	
def acquire_locations(img, robot):
	coordinates = [[0 for x in range(2)] for y in range(3)] 
	#cnts = cv2.findContours(img.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	cnts = cv2.findContours(img.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
	#print "len(cnts) : %d" % len(cnts)
	if len(cnts) is 3:
		#c = max(cnts,key=cv2.contourArea)
		for i in range(0, 3):
			c = cnts[i]
			#((x,y),radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			a = int(M["m10"] / M["m00"])
			b = int(M["m01"] / M["m00"])
			coordinates[i][0] = a
			coordinates[i][1] = b
		
		#Robotx = (coordinates[0][0] + coordinates[1][0])/2
		#Roboty = (coordinates[0][1] + coordinates[1][1])/2
		#Robotk = float((coordinates[0][1] - coordinates[1][1])) / (coordinates[0][0] - coordinates[1][0])
		#Robotdir = abs(math.atan(Robotk) * 180 / 3.14159)
		#print(coordinates)
		Robotx, Roboty, Robotdir = thetacalc(coordinates[0], coordinates[1], coordinates[2])
		robot.setPos(Robotx, Roboty, Robotdir)
		
		return True
		
	else:
		print "Robot :%d not in camera-view" % robot.ID
		return False

def path_finding(Map, robot, goals):
	cost = np.zeros(len(goals),dtype="int64")
	path = [{} for _ in range(len(goals))]
	for i in range(0, len(goals)):
		temp_path, temp_cost= dijkstra_search(Map,robot.getPosXY(),goals[i])
		cost[i] = temp_cost[goals[i]]
		path[i] = temp_path
	min = cost.min()
	return path[cost.argmin()], cost.argmin()
	
if __name__ == '__main__':
	main()	