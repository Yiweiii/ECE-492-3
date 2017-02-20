import sys
import cmath


SelfX = float(raw_input('SelfX>'))
SelfY = float(raw_input('SelfY>'))
SelfAng = float(raw_input('SelfAng>'))

Dist1 = float(raw_input('Dist1>')) # set to zer0 or small value for rendevous
Dist2 = float(raw_input('Dist2>')) # set to zer0 or small value for rendevous

Robot1X = float(raw_input('Robot1X>'))
Robot1Y = float(raw_input('Robot1Y>'))
Robot2X = float(raw_input('Robot2X>'))
Robot2Y = float(raw_input('Robot2Y>'))

X1 = (SelfX - Robot1X)
Y1 = (SelfY - Robot1Y)
X2 = (SelfX - Robot2X)
Y2 = (SelfY - Robot2Y)
X = X1 + X2
Y = Y1 + Y2

print(X, "   ",  Y) # target location
(mag1, phase1) = cmath.polar((complex(X,Y)))

phase1 = phase1 * 180 / cmath.pi + 180
#phase2 = phase2 * 180 / cmath.pi + 180

rotateangle = SelfAng - (phase1)


finalmag = (mag1)/2

print("angle", finalmag)
print("angle to move to ", rotateangle)
print("mag1", mag1)








