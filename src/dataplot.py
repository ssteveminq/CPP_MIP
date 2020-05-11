#!/usr/bin/env python3

from numpy import genfromtxt
import numpy as np
from math import *
import argparse
from obstacle import Obstacle
import csv
import matplotlib.pyplot as plt
import pandas as pd
import os
import re
#import numpy as np
f_max=0.35
v_max=0.45

# timeindex = "04021858"
timeindex = "04302233"
# Open the desired file for reading
dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path=dir_path[:-4]
file_name =dir_path + "/results/data/robot_" +timeindex+"_.csv"
wayfile_name =dir_path + "/results/data/waypoints_" +timeindex+"_.csv"
obsfile_name =dir_path + "/results/data/obstacles_"+timeindex+"_.csv"

df = pd.read_csv(file_name, delimiter=',', names = ['index', 'x', 'y', 'vx', 'vy', 'fx', 'fy'])
waydf = pd.read_csv(wayfile_name, delimiter=',', names = ['index', 'time', 'coords'])
obsdf = pd.read_csv(obsfile_name , delimiter=',', names = ['obstacle'])

waytime= np.asarray(waydf['time'][1:])
waytimes= waytime.astype(np.float)

# idx_array = np.arange(len(list(df['index'])))   #time index
pos_x = np.asarray(df['x'][1:])             #robot pos_x
pos_y = np.asarray(df['y'][1:])             #robot pos_y

#convert from string to float 
pos_x = pos_x.astype(np.float)
pos_y = pos_y.astype(np.float)

vel_x = np.asarray(df['vx'][1:])             #robot pos_x
vel_y = np.asarray(df['vy'][1:])             #robot pos_y

#convert from string to float 
vel_x = vel_x.astype(np.float)
vel_y = vel_y.astype(np.float)

force_x = np.asarray(df['fx'][1:])             #robot pos_x
force_y = np.asarray(df['fy'][1:])             #robot pos_y

#convert from string to float 
force_x = force_x.astype(np.float)
force_y = force_y.astype(np.float)

#waypoint
way_x=[]
way_y=[]
regex = re.compile('[-+]?\d*\.\d+|[-+]?\d+')                #set pattern in order to find integer in string
way_coords = np.asarray(waydf['coords'][1:])
for i in range(len(way_coords)):
    nums = [float(k) for k in regex.findall(way_coords[i])] #find integer value in string format '[ int, int ]'
    way_x.append(nums[0])
    way_y.append(nums[1])


#obstacles
floatregex =re.compile('[-+]?\d*\.\d+|[-+]?\d+') 
obstacles = []                                  # list which will contain all obstacles
obstacle_coords = np.asarray(obsdf['obstacle'][0:])
for i in range(len(obstacle_coords)):
    nums = [float(k) for k in floatregex.findall(obstacle_coords[i])] #find integer value in string format '[ int, int ]'
    tmp = Obstacle(nums[0], nums[1], nums[2], nums[3])          #xmin,ymin, 
    tmp.draw()
    obstacles.append(tmp)                                   # attach obstacle to obstacle list


#plot figures 
plt.scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial point
plt.scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
plt.plot(pos_x, pos_y, 'o', markersize = 25, fillstyle='none',color='black')             #trajectory point
plt.plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='green')             #trajectory point
for i in range(len(waytimes)):
    plt.text(way_x[i], way_y[i]-1,str(waytimes[i]), color='g')


area_size=13
locs, labels = plt.xticks()
locs, labels = plt.yticks()
plt.xticks(np.arange(-area_size,area_size,1.0))
plt.yticks(np.arange(-area_size,area_size,1.0))
ax = plt.axes()

plt.xlabel('x')
plt.ylabel('y')
plt.xlim([-area_size, area_size])   # limit the plot space
plt.ylim([-area_size, area_size])   # limit the plot space
plt.grid(True)
plt.tight_layout()



plt.figure()
#figure 2: Plot velocity and force
fig = plt.subplot(2,1,1)
plt.xlabel('Time steps [-]')
plt.ylabel("Velocity [m/s]")
plt.title("Velocity per time step")
v_mag= []
for i in range(len(vel_x)):
    v_mag.append(sqrt(vel_x[i] ** 2 + vel_y[i] ** 2))   # velocity magnitude

plt.plot(range(len(v_mag)), [v_max] * len(v_mag), color='red', label="Maximum velocity"+str(f_max) + ' [N]', linestyle = '--')
plt.plot(range(len(v_mag)), v_mag, color='black')


#figure 2: Plot the force
fig2 = plt.subplot(2, 1, 2)
plt.xlabel('Time steps [-]')
plt.ylabel("Force [N]")
plt.title("Force input per time step")
f_mag = []
for i in range(len(force_x)):
    f_mag.append(sqrt(force_x[i] ** 2 + force_y[i] ** 2))   # force magnitude

plt.plot(range(len(f_mag)), f_mag, color='black')

# Plot the maximum force
fig2.plot(range(len(f_mag)), [f_max] * len(f_mag), color='red', label="Maximum force  "+str(f_max) + ' [N]', linestyle = '--')
plt.legend()
plt.grid(True)
plt.tight_layout()                                   # Make sure the titles and labels are visible
plt.show()



