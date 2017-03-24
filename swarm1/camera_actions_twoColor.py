from __future__ import division
import numpy as np
import cv2
import robot_structure as rs
from implementation import*
import sys
import math
import operator
import time
#import matplotlib.pyplot as plt
import random

#red_lower = np.array([0,100,100])
#red_upper = np.array([80,255,255])

red_lower = np.array([110,100,100])
red_upper = np.array([179,255,255])

green_lower = np.array([37,81,158])
green_upper = np.array([83,119,247])

#blue_lower = np.array([70,100,255])
#blue_upper = np.array([140,255,255])

blue_lower = np.array([110,50,50])
blue_upper = np.array([130,255,255])

yellow_lower = np.array([25,110,190])
yellow_upper = np.array([90,255,255])

violet_lower = np.array([120,60,95])
violet_upper = np.array([150,255,255])

orange_lower = np.array([10, 100, 180])
orange_upper = np.array([80,255,255])

X_estimate = np.asmatrix(np.zeros((1,4)))
X_predict = np.asmatrix(np.zeros((1,4)))
A = np.mat([[1,1,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]])
H = np.mat([[1,0,0,0],[0,0,1,0]])
I = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
R = np.mat([[10000,0],[0,10000]])
P_estimate = np.mat([[10000,5000,0,0],[5000,5000,0,0],[0,0,10000,5000],[0,0,5000,5000]])
P_predict = np.asmatrix(np.zeros((4,4)))
new_sample = np.mat([[0],[0]])
kg = np.asmatrix(np.zeros((4,2)))
x_a = []
y_a = []

robot_num = 4
group_bot = np.ones((robot_num,4))
record = np.zeros((robot_num,1))

for i in range(2):
    x_a.append(i)
    y_a.append(i)
	
x_cor = 0
y_cor = 0

x_array1 = []
y_array1 = []

x_array2 = []
y_array2 = []

#X_predict = np.mat([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])
#P_estimate = np.mat([[1,1,0,0],[1,1,0,0],[1,1,0,0],[1,1,0,0]])
#X_estimate[k-2,:] = X_est;
#u[k-2] = u1;
k = 2
i = 0
coeff = 0.8

Llower_thresh = 100
Lupper_thresh = 150
Slower_thresh = 60
Supper_thresh = 75
def kal_predict():
    #u(k-1) = u_trans
	global A,X_estimate,X_predict,P_estimate,P_predict
	X_predict = (A*X_estimate.transpose()).transpose()
	P_predict = A*P_estimate*(A.transpose())
	
def kal_update(new_sample):
	global A,H,I,Kg,X_estimate,X_predict,P_estimate,P_predict
	Kg = P_predict*(H.transpose())*(np.linalg.inv(H*P_predict*(H.transpose()) + R))
	X_estimate = (X_predict.transpose() + Kg*(new_sample - H*(X_predict.transpose()))).transpose()
	P_estimate = (I - Kg*H)*P_predict
	#new_message[:,k] = new_sample - np.dot(H,X_predict[k,:].transpose())
	#new_deviation = np.dot(H,np.dot(P_predict,H.transpose())) + R
	#delta(k) = np.dot(np.dot(new_message[:,k].transpose(),np.linalg.inv(new_deviation)),new_message[:,k])
	#u(k) = coeff * u(k-1) + delta(k)
	#u_trans = u(k)
	

#while (i <= 80):
	#x.append(i + random.uniform(1,5))
	#y.append(4*i + random.uniform(1,5))
	#i = i + 1
count = 0
def kalman(x,y):
	global count
	if (count == 0):
		x_a[0] = x
		y_a[0] = y
		count = count + 1
	elif (count == 1):
		x_a[1] = x
		y_a[1] = y
		X_estimate[0,0] = x_a[1]
		X_estimate[0,1] = (x_a[1] - x_a[0])/2
		X_estimate[0,2] = y_a[1]
		X_estimate[0,3] = (y_a[1] - y_a[0])/2
		count = count + 1
	else:
		new_sample[0] = x
		new_sample[1] = y
		kal_predict()
		kal_update(new_sample)
	
	x_cor = X_estimate[0,0]
	y_cor = X_estimate[0,2]
	
	return x_cor, y_cor


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
	
	output = cv2.bitwise_and(img, img, mask=mask)
	#cv2.imshow("out",output)
	#cv2.waitKey(0)
	return hue_image

q = 0	
def main():
	global q
	if len(sys.argv) == 3:
		#print "File mode"
		image_path = sys.argv[1]
		bgr_image = cv2.imread(image_path)
		
		orig_image = bgr_image.copy()
		
		bgr_image = cv2.medianBlur(bgr_image, 3)
		hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
		hue_image = ID_hue_image(hsv_image, 1, orig_image)
		coordinates_Prime = acquire_locations(hue_image)
		coordinates_Secondary = color_select(hsv_image, orig_image, 3)
		Robot1 = rs.Robot(int(sys.argv[2]))
		print "Robot ID: %d" % (Robot1.ID)
		group_robot(coordinates_Prime)
		rebuild(coordinates_Prime, coordinates_Secondary)
		row = 0
		while (row < robot_num):
			if (group_bot[row,3] == Robot1.ID):
				x_A, y_A, x_B, y_B, x_C, y_C = get_pos(coordinates_Prime, row)
				print "x =", x_A, " ", "y =", y_A
				print "x =", x_B, " ", "y =", y_B
				print "x =", x_C, " ", "y =", y_C
				break
			row = row + 1
		print group_bot
	else:
		#print "Active mode"
		cap = cv2.VideoCapture(0)
		while(True):
			ret, bgr_image = cap.read()
			#print(q)
			#q = q + 1
			cv2.imshow("cam_image", bgr_image)
			#width, height = frame.shape

			orig_image = bgr_image.copy()
				
			bgr_image = cv2.medianBlur(bgr_image, 3)
			hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
			
			#cv2.imshow("orginal", orig_image)
			#cv2.imshow("bgr", bgr_image)
			#cv2.imshow("hsv", hsv_image)
				
			#Robot1 = rs.Robot(3)
			Robot2 = rs.Robot(1)
			#print "Robot ID: %d" % (Robot1.ID)
				
			#lower_red_hue_range = cv2.inRange(hsv_image,cv2.cv.Scalar(0,100,100),cv2.cv.Scalar(10,255,255))
			#upper_red_hue_range = cv2.inRange(hsv_image,cv2.cv.Scalar(160,100,100),cv2.cv.Scalar(180,255,255))
			#red_hue_image = cv2.addWeighted(lower_red_hue_range,1.0,upper_red_hue_range,1.0,0.0)
			
			#hue_image = ID_hue_image(hsv_image, Robot1.ID, orig_image)
			hue_image2 = ID_hue_image(hsv_image, Robot2.ID, orig_image)
					
			#track_robot(hue_image, orig_image)
			#cv2.imshow("hue",hue_image)
			#acquire_locations(hue_image, Robot1)
			acquire_locations(hue_image2, Robot2)
			#time.sleep(1)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

		cap.release()
		cv2.destroyAllWindows()



def color_select(hsv_image, orig_image, color_ID):
	hue_image = ID_hue_image(hsv_image, color_ID, orig_image)
	coordinates_Secondary = acquire_locations(hue_image)
	return coordinates_Secondary


#def track_robot(img, orig_image, robot):

		
def acquire_locations(img):
	cnts = cv2.findContours(img.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	coordinates = np.zeros((len(cnts),2))
		#c = max(cnts,key=cv2.contourArea)
	for i in range(0, len(cnts)):
		c = cnts[i]
		((x,y),radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		a = int(M["m10"] / M["m00"])
		b = int(M["m01"] / M["m00"])
		coordinates[i,0] = a
		coordinates[i,1] = b
	return coordinates

def get_pos(coordinates_Prime, row):
	x_A = coordinates_Prime[int(group_bot[row,0]),0]
	y_A = coordinates_Prime[int(group_bot[row,0]),1]
	x_B = coordinates_Prime[int(group_bot[row,1]),0]
	y_B = coordinates_Prime[int(group_bot[row,1]),1]
	x_C = coordinates_Prime[int(group_bot[row,2]),0]
	y_C = coordinates_Prime[int(group_bot[row,2]),1]
	return x_A, y_A, x_B, y_B, x_C, y_C

def distance(x1, y1, x2, y2):
	dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
	return dist
	
def info_update(sub_color):
	global group_bot, record

	i = 0
	while(i < len(sub_color)):
		row = 0
		while (row < robot_num):
			if (group_bot[row,0] == sub_color[i]):
				record[row,0] = 0
				group_bot[row,3] = group_bot[row,3] + 1
			elif (group_bot[row,1] == sub_color[i]):
				record[row,0] = 1
				group_bot[row,3] = group_bot[row,3] + 1
			elif (group_bot[row,2] == sub_color[i]):
				record[row,0] = 2
				group_bot[row,3] = group_bot[row,3] + 1
			else:
				group_bot[row,3] = group_bot[row,3]
			row = row + 1
		i = i + 1

#future hash table	
def rebuild(coordinates_Prime, coordinates_Secondary):
	set_num = 2
	sub_color = []
	i = 0
	while (i < len(coordinates_Secondary)):
		j = i
		while (j < len(coordinates_Prime)):
			if (abs(coordinates_Secondary[i,0] - coordinates_Prime[j,0]) < 2) and (abs(coordinates_Secondary[i,1] - coordinates_Prime[j,1]) < 2):
				sub_color.append(j)
			j = j + 1 
		i = i + 1

	info_update(sub_color)

	row = 0
	while (row < robot_num):
		if (group_bot[row,3] == 2):
			second_pos = record[row,0]
			x_A, y_A, x_B, y_B, x_C, y_C = get_pos(coordinates_Prime, row)
			d1 = distance(x_A, y_A, x_B, y_B)
			d2 = distance(x_A, y_A, x_C, y_C)
			d3 = distance(x_B, y_B, x_C, y_C)
			if (d1 > d2):      #d2 min
				if (second_pos == 0):
					set_num = cal_mid(x_A, y_A, x_C, y_C, x_B, y_B)
				elif (second_pos == 2):
					set_num = cal_mid(x_C, y_C, x_A, y_A, x_B, y_B)
			elif (d1 > d3):    #d3 min
				if (second_pos == 1):
					set_num = cal_mid(x_B, y_B, x_C, y_C, x_A, y_A)
				elif (second_pos == 2):
					set_num = cal_mid(x_C, y_C, x_B, y_B, x_A, y_A)
			else:
				if (second_pos == 0):
					set_num = cal_mid(x_A, y_A, x_B, y_B, x_C, y_C)
				elif (second_pos == 1):
					set_num = cal_mid(x_B, y_B, x_A, y_A, x_C, y_C)
			group_bot[row,3] = set_num
		elif (group_bot[row,3] == 3):
			group_bot[row,3] = 4
		
		row = row + 1
			
def cal_mid(x_1, y_1, x_2, y_2, x_3, y_3):
	set_num = 2
	x_m = (x_1 + x_2)/2
	y_m = (y_1 + y_2)/2
	angle = math.atan2(y_3 - y_m,x_3 - x_m) * 180/3.14159
	if (angle >= -90 and angle <= 90):
		if (y_1 >= y_2):
			set_num = 3
	elif (angle >= 90 and angle <= 180) or (angle >= -180 and angle <= -90):
		if (y_1 <= y_2):
			set_num = 3
	else:
		set_num = 2
	return set_num

def group_robot(coordinates_Prime):
	global group_bot
	i = 0
	num = 0
	copy_coordinates = np.zeros((1,len(coordinates_Prime)))
	#temp_robot = np.zeros((4,2))
	#temp_robot = np.zeros((3,6))
	while(i < len(coordinates_Prime)):
		flag = 0
		x1 = coordinates_Prime[i,0]
		y1 = coordinates_Prime[i,1]
		if (copy_coordinates[0,i] == 0):
			copy_coordinates[0,i] = 1
			j = i + 1
			while (j < len(coordinates_Prime)):
				x2 = coordinates_Prime[j,0]
				y2 = coordinates_Prime[j,1]
				if ((Llower_thresh <= distance(x1, y1, x2, y2) and distance(x1, y1, x2, y2) <= Lupper_thresh)) or ((Slower_thresh <= distance(x1, y1, x2, y2) and distance(x1, y1, x2, y2) <= Supper_thresh)):
					group_bot[num,0] = i
					copy_coordinates[0,j] = 1
					if (flag == 0):
						group_bot[num,1] = j
						flag = 1
					else:
						group_bot[num,2] = j
				j = j + 1
			num = num + 1
		i = i + 1
	
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