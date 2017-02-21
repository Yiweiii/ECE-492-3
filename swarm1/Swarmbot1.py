import sys
import cmath
from robot_structure import Robot

robot1 = Robot(1)

Dist1 = float(raw_input('Dist1>')) # set to zer0 or small value for rendevous
Dist2 = float(raw_input('Dist2>')) # set to zer0 or small value for rendevous

Robot1X = float(raw_input('Robot1X>'))
Robot1Y = float(raw_input('Robot1Y>'))
Robot2X = float(raw_input('Robot2X>'))
Robot2Y = float(raw_input('Robot2Y>'))

X1 = (robot1.xpos - Robot1X)
Y1 = (robot1.ypos - Robot1Y)
X2 = (robot1.xpos - Robot2X)
Y2 = (robot1.ypos - Robot2Y)
X = X1 + X2
Y = Y1 + Y2

print(X, "   ",  Y) # target location
(mag1, phase1) = cmath.polar((complex(X,Y)))

phase1 = phase1 * 180 / cmath.pi + 180

rotateangle = robot1.dir - (phase1)

finalmag = (mag1)/2

print("angle", finalmag)
print("angle to move to ", rotateangle)
print("mag1", mag1)








