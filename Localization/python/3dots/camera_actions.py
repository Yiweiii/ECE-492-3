import numpy as np
import cv2
import robot_structure
from implementation import*
import sys
import math

def main():
	if len(sys.argv) == 2:
		image_path = sys.argv[1]
		bgr_image = cv2.imread(image_path)
		
		orig_image = bgr_image.copy()
		
		bgr_image = cv2.medianBlur(bgr_image,3)
		
		hsv_image = cv2.cvtColor(bgr_image,cv2.COLOR_BGR2HSV)
		
		lower_red_hue_range = cv2.inRange(hsv_image,cv2.cv.Scalar(0,100,100),cv2.cv.Scalar(10,255,255))
		upper_red_hue_range = cv2.inRange(hsv_image,cv2.cv.Scalar(160,100,100),cv2.cv.Scalar(179,255,255))

		
		red_hue_image = cv2.addWeighted(lower_red_hue_range,1.0,upper_red_hue_range,1.0,0.0)
		red_hue_image = cv2.GaussianBlur(red_hue_image,(9,9),2,2)
		
		track_robot(red_hue_image,orig_image)
		acquire_locations(bgr_image)
	else:
		print "Usage: python camera_actions image_path"

def track_robot(img, orig_image):
	circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,1,15,
								param1=10,param2=27, minRadius=0,maxRadius=0)
	output = img.copy()
	#circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT, 1,20)
	if circles is not None:
		print "Circles found"
		circles = np.round(circles[0,:]).astype("int")
		for (x,y,r) in circles:
			print "Robot x: %d y: %d r: %d" %(x,y,r)
			cv2.circle(orig_image,(x,y),r,(0,255,0),4)
			cv2.rectangle(orig_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
	else:
		print "Circles not found"
		
	cv2.imshow("output", orig_image)
	cv2.waitKey(0)

def acquire_locations(img):	
	mask = cv2.inRange(img, red_lower, red_upper)
	#cv2.imshow("mask",mask)
	#cv2.waitKey(0)
	cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
	
	if len(cnts) > 0:
		print len(cnts)
		c = max(cnts,key=cv2.contourArea)
		((x,y),radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		dir = 0.0
		cv2.circle(img, (int(x),int(y)),5,(0,0,0),2)
		#robot.setPos(center[0],center[1], dir)
		print "Robot center %d %d" %(center[0],center[1])
	else:
		print "Robot not in camera-view"
		
	#cv2.imshow("locations",img)
	#cv2.waitKey(0)

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