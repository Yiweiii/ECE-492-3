class Robot:
	robotCount = 0

	def __init__(self, x,y,x1,y1,angle):
		self.x = x
		self.y = y
		self.idnum = Robot.robotCount
		self.claw = self.claw(x1,y1,angle)
		Robot.robotCount += 1

	def displayCount(self):
		print "Total robot count %d" % Robot.robotCount

	def displayRobot(self):
		print "Robot #: ",self.idnum, ", x: ", self.x, ", y: ", self.y

	def setPosition(self,x,y):
		self.x = x
		self.y = y
		
	class claw:
		def __init__(self, x,y,angle):
			self.x = x
			self.y = y
			self.orientation = angle
			
		def displayClaw(self):
			print "x: ", self.x, ", y: ", self.y
			
		def setPosition(self,x,y):
			self.x = x
			self.y = y