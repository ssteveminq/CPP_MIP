#!/usr/bin/env python3

from numpy import genfromtxt
import numpy as np
from math import *
import argparse
import csv
import matplotlib.pyplot as plt
import pandas as pd
import os
import re
import time
#import numpy as np
f_max=0.3
v_max=0.4

class Params:
    def __init__(self):
        self.numiters = 1000
        self.dt = 0.2
        self.goal_tol = 0.25
        self.max_vel = 0.2 # m/s
        self.min_vel = 0.0 # m/s
        self.sensor_range_m = 0.25 # m
        # self.time_to_switch_goal = 5.0 # sec #inactive for now
        # self.sweep_resolution = 0.4 # m



def motion(state, goal, params):
    # state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]
    goal_yaw = math.atan2(dy, dx)
    K_theta = 2.0
    state[4] = K_theta*math.sin(goal_yaw - state[2]) # omega(rad/s)
    state[2] += params.dt*state[4] # yaw(rad)

    dist_to_goal = np.linalg.norm(goal - state[:2])
    K_v = 0.1
    state[3] += K_v*dist_to_goal
    if state[3] >= params.max_vel: state[3] = params.max_vel
    if state[3] <= params.min_vel: state[3] = params.min_vel

    dv = params.dt*state[3]
    state[0] += dv*np.cos(state[2]) # x(m)
    state[1] += dv*np.sin(state[2]) # y(m)

    return state


params = Params()

# Open the desired file for reading
dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path=dir_path[:-4]
file_name =dir_path + "/results/data/robot_03290755_.csv"
wayfile_name =dir_path + "/results/data/waypoints_03290755_.csv"

df = pd.read_csv(file_name, delimiter=',', names = ['index', 'x', 'y', 'vx', 'vy', 'fx', 'fy'])
waydf = pd.read_csv(wayfile_name, delimiter=',', names = ['index', 'time', 'coords'])

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
regex = re.compile(r'[\+\-]?[0-9]+')                #set pattern in order to find integer in string
way_coords = np.asarray(waydf['coords'][1:])
for i in range(len(way_coords)):
    nums = [int(k) for k in regex.findall(way_coords[i])] #find integer value in string format '[ int, int ]'
    way_x.append(nums[0])
    way_y.append(nums[1])

#plot figures 
fig,axes=plt.figure(figsize=(10,20))
plt.scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial point
plt.scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
plt.plot(pos_x, pos_y, 'o', markersize = 20, fillstyle='none',color='black')             #trajectory point
plt.plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='green')             #trajectory point
# for i in range(len(waytimes)):
    # plt.text(way_x[i], way_y[i]-1,str(waytimes[i]), color='g')

area_size=5
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

ntimestep = len(pos_x)
goal_tol=0.1

goali = 0                           #define goal from waypoints set
goal = [goal_x[goali], goal_y[goali]]
	

# initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
state = np.array([pos_x[0],pos_y[0], 0.0, np.pi/2, 0.0, 0.0])
traj = state[:2]

for i in range(ntimestep):
    state = motion(state, goal, params)
    goal_dist = np.linalg.norm(goal - state[:2])
    t_current = time.time()
    if goal_dist < goal_tol: # goal is reached
        print('Switching to the next goal.')
        print('Time from the previous reached goal:', t_current - t_prev_goal)
        if goali < len(goal_x) - 1:
            goali += 1
        else:
            break
        t_prev_goal = time.time()
        goal = [goal_x[goali], goal_y[goali]]


    traj = np.vstack([traj, state[:2]])
    axes[1].plot(goal[0], goal[1], 'ro', markersize=12, label='Goal position', zorder=20)
    visualize(traj, state, params)
     
     






