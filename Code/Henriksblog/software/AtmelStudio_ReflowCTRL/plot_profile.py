# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 19:48:26 2016

@author: Pero_2912
"""
import matplotlib.pyplot as plt

fajl = open('ProfileTest_FirstTry.txt','r')
a = fajl.read()

a = a.split('\n')
a.remove('')
a.pop(0)

x = [row.split()[0] for row in a]
y = [row.split()[1] for row in a]

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Profile")    
ax1.set_xlabel('Time, s')
ax1.set_ylabel('Temp, degC')

ax1.plot(x,y, c='r', label='the profile')
ax1.grid()

leg = ax1.legend()

plt.show()