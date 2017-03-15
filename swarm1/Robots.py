import sys
import math
from robot_structure import Robot
from Rendevous import rendezvous

##Create four robots
robot1 = Robot(1)
robot2 = Robot(2)
robot3 = Robot(3)
robot4 = Robot(4)

## SET ROBOT Values  --This will be from the camera function

robot1.setPos(10, 10, 90)
robot2.setPos(0, 0, 0)
robot3.setPos(0, 0, 0)
robot4.setPos(0, 0, 0)

##Direction outputs

while True:
    (mag1, angle1, rangle1) = rendezvous(robot1, robot2, robot3)
    (mag2, angle2, rangle2) = rendezvous(robot2, robot1, robot4)
    (mag3, angle3, rangle3) = rendezvous(robot3, robot4, robot1)
    (mag4, angle4, rangle4) = rendezvous(robot4, robot3, robot2)

    #This will be done by camera function
    robot1.setPos(robot1.xpos + mag1 * math.cos(angle1), robot1.ypos + mag1 * math.sin(angle1), angle1)
    robot2.setPos(robot2.xpos + mag2 * math.cos(angle2), robot2.ypos + mag2 * math.sin(angle2), angle2)
    robot3.setPos(robot3.xpos + mag3 * math.cos(angle3), robot3.ypos + mag3 * math.sin(angle3), angle3)
    robot4.setPos(robot4.xpos + mag4 * math.cos(angle4), robot4.ypos + mag4 * math.sin(angle4), angle4)
    ## Print out for testing
    print(robot1.xpos, "robot1", robot1.ypos)
    print(robot2.xpos, "robot2", robot2.ypos)
    print(robot3.xpos, "robot3", robot3.ypos)
    print(robot3.xpos, "robot4", robot4.ypos)

## Calculate and output pwm requests



