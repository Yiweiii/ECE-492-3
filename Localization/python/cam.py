import numpy as np
import cv2
import robot_structure as rs

# Get video feed from overhead camera
cap = cv2.VideoCapture(0)

# width and height of image frame
height = 480
width = 640

# pixel height and width of robot
r_h = 48
r_w = 64

# create 2d map of image view
def createArray():
   h = height/r_h
   w = width/r_w
   Map = [[0 for x in range(w)] for y in range(h)]
   print Map

def createRobot():
	r1 = rs.Robot(5,5,4,4,0.4)

createArray()
createRobot()

while(True):
   ret, frame = cap.read()
   
   #width, height = frame.shape
   cv2.imshow('frame',frame)
   
   if cv2.waitKey(1) & 0xFF == ord('q'):
      break

cap.release()
cv2.destroyAllWindows()
