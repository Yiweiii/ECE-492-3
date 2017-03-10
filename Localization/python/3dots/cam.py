import cv2
import robot_structure as rs
from implementation import*
import camera_actions as ca

# Get video feed from overhead camera
cap = cv2.VideoCapture(0)

# width and height of image frame
height = 480
width = 640

Map = GridWithWeights(16,12)
Map.walls = [(6,8)]

came_from, cost_so_far = dijkstra_search(diagram4, (1, 4), (7, 8))
draw_grid(diagram4, width=3, point_to=came_from, start=(1, 4), goal=(7, 8))
print()
draw_grid(diagram4, width=3, number=cost_so_far, start=(1, 4), goal=(7, 8))
print()
draw_grid(diagram4, width=3, path=reconstruct_path(came_from, start=(1, 4), goal=(7, 8)))

while(True):
   ret, frame = cap.read()
   
   #width, height = frame.shape
   cv2.imshow('frame',frame)
   
   # Create Map
   Map = GridWithWeights(16,12)
   
   # Create robots with different color schemes
   red_Rover = rs.Robot(1)
   #green_Rover = rs.Robot(2)
   #blue_Rover = rs.Robot(3)
   #yellow_Rover = rs.Robot(4)
   
   ca.acquire_locations(frame, red_Rover)
   #ca.acquire_locations(frame, green_Rover)
   #ca.acquire_locations(frame, blue_Rover)
   #ca.acquire_locations(frame, yellow_Rover)
   
   Goal = (8,6)
   
   red_rover_path, red_rover_cost = dijkstra_search(Map,red_Rover.getPosXY(),Goal)
   draw_grid(Map, width=3, path=reconstruct_path(red_rover_path, start=red_Rover.getPosXY(),goal=Goal))
   print "\n"
   
   Goals = [(8,6), (0,10), (2,4)]
   red_rover_path2, index = ca.path_finding(Map,red_Rover,Goals)
   draw_grid(Map, width=3, path=reconstruct_path(red_rover_path2, start=red_Rover.getPosXY(),goal=Goals[index]))
   print "\n"
   
   if cv2.waitKey(1) & 0xFF == ord('q'):
      break

cap.release()
cv2.destroyAllWindows()
