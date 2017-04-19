class Robot:
    def __init__(self, c1,c2,c3):
		self.xpos = 0
		self.ypos = 0
		self.dir = 0
		self.c1 = c1
		self.c2 = c2
		self.c3 = c3
		self.inview = False
		self.HOST = ''

    def displayRobot(self):
        print
        "Robot #: ", self.ID, ", x: ", self.xpos, ", y: ", self.ypos, ", dir: ", self.dir

    def getX(self):
        return self.xpos

    def getY(self):
        return self.ypos

    def getDir(self):
        return self.dir

    def getPosXY(self):
        return (self.xpos, self.ypos)

    def setPos(self, xpos, ypos, dir):
        self.xpos = xpos
        self.ypos = ypos
        self.dir = dir
		