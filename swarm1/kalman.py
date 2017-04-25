from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
import random
from camera_actions import *
from server import *
# X_estimate = np.asmatrix(np.zeros((1,4)))
# X_predict = np.asmatrix(np.zeros((1,4)))
# A = np.mat([[1,1,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]])
# H = np.mat([[1,0,0,0],[0,0,1,0]])
# I = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
# R = np.mat([[10000,0],[0,10000]])
# P_estimate = np.mat([[10000,5000,0,0],[5000,5000,0,0],[0,0,10000,5000],[0,0,5000,5000]])
# P_predict = np.asmatrix(np.zeros((4,4)))
# new_sample = np.mat([[0],[0]])
# kg = np.asmatrix(np.zeros((4,2)))
# x = []
# y = []
# x_cor = []
# y_cor = []
# count = 0

# #X_predict = np.mat([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])
# #P_estimate = np.mat([[1,1,0,0],[1,1,0,0],[1,1,0,0],[1,1,0,0]])
# #X_estimate[k-2,:] = X_est;
# #u[k-2] = u1;
# k = 2
# i = 0
# coeff = 0.8
delta_t = 0.04

x_1 = -velocity*delta_t*sin(Robotdir)
x_2 = velocity*delta_t*cos(Robotdir)
v_1 = delta_t*cos(Robotdir)
v_2 = delta_t*sin(Robotdir)



A = np.mat([[1,0,x_1],[0,1,x_2],[0,0,1]])
B = np.mat([[v_1,0],[v_2,0],[0,1]])
Q = np.mat([[10,0,0],[0,10,0],[0,0,10]])
R = np.mat([[1,0],[0,1]])
U = np.mat([[v],[ang_v]])
I = np.mat([[1,0,0],[0,1,0],[0,0,1]])

def kal_predict():
    #u(k-1) = u_trans
	#global A,X_estimate,X_predict,P_estimate,P_predict
	X_predict = (A*X_estimate) + (B*U)
	P_predict = A*P_estimate*(A.transpose()) + Q
	return (X_predict, P_predict)
	
def kal_update(new_sample):
	global A,H,I
	Kg = P_predict*(H.transpose())*(np.linalg.inv(H*P_predict*(H.transpose()) + R))
	X_estimate = X_predict + Kg*(new_sample - H*X_predict)
	P_estimate = (I - Kg*H)*P_predict
	return (X_estimate, P_estimate)
	#new_message[:,k] = new_sample - np.dot(H,X_predict[k,:].transpose())
	#new_deviation = np.dot(H,np.dot(P_predict,H.transpose())) + R
	#delta(k) = np.dot(np.dot(new_message[:,k].transpose(),np.linalg.inv(new_deviation)),new_message[:,k])
	#u(k) = coeff * u(k-1) + delta(k)
	#u_trans = u(k)
	

#while (i <= 80):
	#x.append(i + random.uniform(1,5))
	#y.append(4*i + random.uniform(1,5))
	#i = i + 1
	


#plt.figure(1)
#plt.plot(x,y)
#plt.plot(x_cor,y_cor)
#plt.show()

