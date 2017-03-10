from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
import random

X_estimate = np.asmatrix(np.zeros((1,4)))
X_predict = np.asmatrix(np.zeros((1,4)))
A = np.mat([[1,1,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]])
H = np.mat([[1,0,0,0],[0,0,1,0]])
I = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
R = np.mat([[10000,0],[0,10000]])
P_estimate = np.mat([[10000,5000,0,0],[5000,5000,0,0],[0,0,10000,5000],[0,0,5000,5000]])
P_predict = np.asmatrix(np.zeros((4,4)))
new_sample = np.mat([[0],[0]])
kg = np.asmatrix(np.zeros((4,2)))
x = []
y = []
x_cor = []
y_cor = []

#X_predict = np.mat([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])
#P_estimate = np.mat([[1,1,0,0],[1,1,0,0],[1,1,0,0],[1,1,0,0]])
#X_estimate[k-2,:] = X_est;
#u[k-2] = u1;
k = 2
i = 0
coeff = 0.8
	

def kal_predict():
    #u(k-1) = u_trans
	global A,X_estimate,X_predict,P_estimate,P_predict
	X_predict = (A*X_estimate.transpose()).transpose()
	P_predict = A*P_estimate*(A.transpose())
	
def kal_update():
	global A,H,I,Kg,new_sample,X_estimate,X_predict,P_estimate,P_predict
	Kg = P_predict*(H.transpose())*(np.linalg.inv(H*P_predict*(H.transpose()) + R))
	X_estimate = (X_predict.transpose() + Kg*(new_sample - H*(X_predict.transpose()))).transpose()
	P_estimate = (I - Kg*H)*P_predict
	#new_message[:,k] = new_sample - np.dot(H,X_predict[k,:].transpose())
	#new_deviation = np.dot(H,np.dot(P_predict,H.transpose())) + R
	#delta(k) = np.dot(np.dot(new_message[:,k].transpose(),np.linalg.inv(new_deviation)),new_message[:,k])
	#u(k) = coeff * u(k-1) + delta(k)
	#u_trans = u(k)
	

while (i <= 80):
	x.append(i + random.uniform(1,5))
	y.append(4*i + random.uniform(1,5))
	i = i + 1
	
X_estimate[0,0] = x[1]
X_estimate[0,1] = (x[1] - x[0])/2
X_estimate[0,2] = y[1]
X_estimate[0,3] = (y[1] - y[0])/2
x_cor.append(X_estimate[0,0])
y_cor.append(X_estimate[0,2])


while (k <= 80):
	new_sample[0,:] = x[k]
	new_sample[1,:] = y[k]
	kal_predict()
	kal_update()
	x_cor.append(X_estimate[0,0])
	y_cor.append(X_estimate[0,2])
	k = k + 1

plt.figure(1)
plt.plot(x_cor,y_cor)
plt.show()

#plt.figure(2)
#plt.plot(x_cor,y_cor)
#plt.show()

