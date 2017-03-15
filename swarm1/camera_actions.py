import numpy as np
import cv2
import robot_structure as rs
from implementation import*
import sys
import math
import operator
import time

red_lower = np.array([0,100,100])
red_upper = np.array([10,255,255])

#red_lower = np.array([150,100,100])
#red_upper = np.array([179,255,255])

green_lower = np.array([37,81,158])
green_upper = np.array([83,119,247])

blue_lower = np.array([70,100,255])
blue_upper = np.array([140,255,255])

yellow_lower = np.array([25,110,190])
yellow_upper = np.array([90,255,255])

violet_lower = np.array([120,60,95])
violet_upper = np.array([150,255,255])

orange_lower = np.array([10, 100, 180])
orange_upper = np.array([80,255,255])

def ID_hue_image(img, ID, orig):
	if(ID is 1):
		lower = red_lower
		upper = red_upper
	elif(ID is 2):
		lower = green_lower
		upper = green_upper
	elif(ID is 3):
		lower = blue_lower
		upper = blue_upper
	elif(ID is 4):
		lower = yellow_lower
		upper = yellow_upper
	elif ID is 5:
		lower = orange_lower
		upper = orange_upper
	elif ID is 6:
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
		
			
		Robot1 = rs.Robot(int(sys.argv[2]))
		print "Robot ID: %d" % (Robot1.ID)
		
		hue_image = ID_hue_image(hsv_image, Robot1.ID, orig_image)
				
		track_robot(hue_image, orig_image, Robot1)
		acquire_locations(hue_image, Robot1)
	else:
		print "Active mode"
		cap = cv2.VideoCapture(0)
		while(True):
			ret, bgr_image = cap.read()
			#cv2.imshow("cam_image", bgr_image)
			#width, height = frame.shape

			orig_image = bgr_image.copy()
				
			bgr_image = cv2.medianBlur(bgr_image, 3)
			hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
			
			#cv2.imshow("orginal", orig_image)
			#cv2.imshow("bgr", bgr_image)
			#cv2.imshow("hsv", hsv_image)
				
			Robot1 = rs.Robot(3)
			print "Robot ID: %d" % (Robot1.ID)
				
			#lower_red_hue_range = cv2.inRange(hsv_image,cv2.cv.Scalar(0,100,100),cv2.cv.Scalar(10,255,255))
			#upper_red_hue_range = cv2.inRange(hsv_image,cv2.cv.Scalar(160,100,100),cv2.cv.Scalar(180,255,255))
			#red_hue_image = cv2.addWeighted(lower_red_hue_range,1.0,upper_red_hue_range,1.0,0.0)
			
			hue_image = ID_hue_image(hsv_image, Robot1.ID, orig_image)
					
			#track_robot(hue_image, orig_image)
			#cv2.imshow("hue",hue_image)
			acquire_locations(hue_image, Robot1)
			
			#time.sleep(1)
			  
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

		cap.release()
		cv2.destroyAllWindows()


def track_robot(img, orig_image, robot):
	circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,1,15,
								param1=10,param2=27, minRadius=0,maxRadius=0)
	output = img.copy()
	coordinates = [[0 for x in range(2)] for y in range(3)] 
	if circles is not None:
		print "Robot detected"
		circles = np.round(circles[0,:]).astype("int")
		i = 0
		if len(circles) is 3:
			for (x,y,r) in circles:
				#print "Circle %d: %d y: %d r: %d" %(i,x,y,r)
				coordinates[i][0] = x
				coordinates[i][1] = y
				#cv2.circle(orig_image,(x,y),r,(0,255,0),4)
				#cv2.rectangle(orig_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
				i = i + 1
			
			Robotx, Roboty, Robotdir = thetacalc(coordinates[0], coordinates[1], coordinates[2])
			print "Robot x: %d y: %d dir : %d" %(Robotx, Roboty, Robotdir)
			robot.setPos(Robotx,Roboty,Robotdir)
		else:
			print "Error occured"
	else:
		print "Circles not found"
		
	#cv2.imshow("output", orig_image)
	#cv2.waitKey(0)
	
def thetacalc(a, b, c):
	t1 = (a[0],a[1])
	t2 = (b[0],b[1])
	t3 = (c[0],c[1])
	t = [t1, t2, t3]
	#print(t)
	coord = [a, b, c]
	a_b = math.sqrt((a[0]-b[0])**2 + (a[1] - b[1])**2)
	a_c = math.sqrt((a[0]-c[0])**2 + (a[1] - c[1])**2)
	b_c = math.sqrt((b[0]-c[0])**2 + (b[1] - c[1])**2)
	values = [a_b, a_c, b_c]
	#print(values)
	min_index, min_value = min(enumerate(values), key=operator.itemgetter(1))
	if min_index is 0:
		#print ("a_b")
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
	elif min_index is 1:
		#print("a_c")
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
	else:
		#print("b_c")
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
	
	#print "disy : %d disx : %d" % (dis_y, dis_x)
	dir = (math.atan2(dis_y,dis_x) * 180/3.14159)
	degrees = (dir + 360) % 360
	#print "dir %d" %dir
	return x, y, degrees
	
	

def acquire_locations(img, robot):
	coordinates = [[0 for x in range(2)] for y in range(3)] 
	cnts = cv2.findContours(img.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	print len(cnts)
	if len(cnts) is 3:
		#c = max(cnts,key=cv2.contourArea)
		for i in range(0, 3):
			c = cnts[i]
			((x,y),radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			a = int(M["m10"] / M["m00"])
			b = int(M["m01"] / M["m00"])
			coordinates[i][0] = a
			coordinates[i][1] = b
		
		#Robotx = (coordinates[0][0] + coordinates[1][0])/2
		#Roboty = (coordinates[0][1] + coordinates[1][1])/2
		#Robotk = float((coordinates[0][1] - coordinates[1][1])) / (coordinates[0][0] - coordinates[1][0])
		#Robotdir = abs(math.atan(Robotk) * 180 / 3.14159)
		
		Robotx, Roboty, Robotdir = thetacalc(coordinates[0], coordinates[1], coordinates[2])
	
		robot.setPos(Robotx,Roboty,Robotdir)
		print "Robot x: %d y: %d dir : %d" %(robot.xpos, robot.ypos, robot.dir)
		
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
	
if __name__ == '__main__':
	main()	