class Robot:
    def __init__(self, ID):
        self.xpos = 0
        self.ypos = 0
        self.dir = 0
        self.ID = ID
		self.X_estimate = np.asmatrix(np.zeros((1,4)))
		self.X_predict = np.asmatrix(np.zeros((1,4)))
		self.P_estimate = np.mat([[10000,5000,0,0],[5000,5000,0,0],[0,0,10000,5000],[0,0,5000,5000]])
		self.P_predict = np.asmatrix(np.zeros((4,4)))
		self.count = 0

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
		
	def setKal(count, X_estimate, X_predict, P_estimate, P_predict):
		self.X_estimate = X_estimate 
		self.X_predict = X_estimate 
		self.P_estimate = X_estimate 
		self.P_predict = P_predict
		self.count = count