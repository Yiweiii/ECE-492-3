class Robot:
    def __init__(self, ID):
        self.xpos = 0
        self.ypos = 0
        self.dir = 0
        self.ID = ID

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