# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 19:48:26 2016

@author: Pero_2912
"""
import matplotlib.pyplot as plt
import numpy as np

def runningMeanFast(x, N):
    return np.convolve(x, np.ones((N,))/N, mode='valid')

fajl = open('D:\Elektronika\Reflow Oven\Oven Test\My PID\PID_Output_100C.txt','r')
a = fajl.read()

a = a.split('\n')
a.remove('')
a.pop(0)

x = [row.split()[0] for row in a]
y = [row.split()[1] for row in a]

y_new = []
for num in y:
    y_new.append(float(num))

y_avg = runningMeanFast(y_new, 5)


fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("PID Controller")    
ax1.set_xlabel('Time, s')
ax1.set_ylabel('Temp, degC')

ax1.plot(x[0:len(y_avg)], y_avg, c='r')
ax1.grid()

leg = ax1.legend()

plt.show()