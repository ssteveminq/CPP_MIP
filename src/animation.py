# !/usr/bin/env python3


from numpy import genfromtxt
import numpy as np
from math import *
import math
import argparse
import csv
import matplotlib.pyplot as plt
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config
import pandas as pd
import os
import re
import time
from matplotlib.patches import Polygon, Rectangle, Circle
import matplotlib as mpl
#import numpy as np
f_max=0.3
v_max=0.4
#probability
p_occ_given_zocc=0.95
p_occ_given_zfree= 0.01
p_free_given_zocc=0.05
p_free_given_zfree= 0.99

#pp control 
k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
Kv = 0.05  # speed propotional gain
ktheta = 0.5
dt = 0.2  # [s]
L = 1.0  # [m] wheel base of vehicle

class map_params:
    def __init__(self):
        self.xyreso = 0.25  # x-y grid resolution [m]
        self.yawreso = math.radians(6)  # yaw angle resolution [rad]
        self.xmin_global=-15
        self.xmax_global=15
        self.ymin_global=-15
        self.ymax_global=15
        self.xw = int(round((self.xmax_global - self.xmin_global) / self.xyreso))
        self.yw = int(round((self.ymax_global - self.ymin_global) / self.xyreso))


class Params:
    def __init__(self):
        self.numiters = 2000
        self.dt = 0.2
        self.goal_tol = 0.25
        self.max_vel = 0.25 # m/s
        self.min_vel = 0.0 # m/s
        self.sensor_range_m = 0.5 # m
        self.animate = 1
        self.area_size=5
        # self.time_to_switch_goal = 5.0 # sec #inactive for now
        # self.sweep_resolution = 0.4 # m

# class State:
    # def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        # self.x = x
        # self.y = y
        # self.yaw = yaw
        # self.v = v

def draw_occmap(data, minx, maxx, miny, maxy, xyreso,agent_x, agent_y, ax):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    ax.pcolor(x+agent_x, y+agent_y, data, vmax=1.0, cmap=plt.cm.Blues)
    ax.set_xlim([agent_x-2*params.area_size, agent_x+2*params.area_size])   # limit the plot space
    ax.set_ylim([agent_y-2*params.area_size, agent_y+2*params.area_size])   # limit the plot space
    # plt.axis("equal")

def draw_occmap_global(data, minx, maxx, miny, maxy, xyreso, ax):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    ax.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    ax.set_xlim([1.1*minx, 1.1*maxx])   # limit the plot space
    ax.set_ylim([1.1*miny, 1.1**maxy])   # limit the plot space
    # plt.axis("equal")

def get_preocc(previous_map, grid,map_params):
    #Find grid value from the previouwmap
    # 1) find the global position in the map
    # 2) find the index based on the global position
    px = grid.px
    py = grid.py

    #map index
    ix = math.floor((px-map_params.xmin_global)/map_params.xyreso)
    iy = math.floor((py-map_params.ymin_global)/map_params.xyreso)

    return previous_map[ix][iy]
 

def update_occ_grid_map(state,observed_grids, previous_map, mapparams):

    ##for observed cell --> update

    print("length of updated grids = ", len(observed_grids))
    for grid in observed_grids:
        prior= get_preocc(previous_map, grid, mapparams)
        if grid.value==1: #the object detection case
            # posterior = (p_occ_given_zocc*prior)/(p_occ_given_zocc*prior +p_free_given_zocc *(1-prior))
            posterior =1 
        elif grid.value==0: # if the detection is free
            # posterior = (p_free_given_zfree*prior)/(p_free_given_zfree*prior +p_occ_given_zfree *(1-prior))
            posterior = 0
        elif grid.value==0.5:
            posterior = 0.5
            # print("here")
            # print
        
        px = grid.px
        py = grid.py

        ix = math.floor((px-mapparams.xmin_global)/mapparams.xyreso)
        iy = math.floor((py-mapparams.ymin_global)/mapparams.xyreso)
        # print("px,py - ix,iy",px,py,", ", ix,iy, ": posterior -->", posterior)
        pmap_global[ix][iy]=posterior

    return pmap_global

def initialize_global_occ_grid_map(params_map):

    pmap_global = [[0.0 for i in range(params_map.yw)] for i in range(params_map.xw)]
    return pmap_global


def plot_robot(pose, params):
    # print("robot")
    r = params.sensor_range_m
    # plt.axis("equal")
    # ax = plt.gca()
    axes[0].plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
                [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='b')
    axes[0].plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
                [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='b')

    # axes[0.plot(pose[0], pose[1], 'ro', markersize=5)
    # circle= Circle((pose[0], pose[1]),r,linewidth=1,edgecolor='k',facecolor='k',alpha=0.3 )
    # ax.add_patch(circle)
    # axes[0.plot(pose[0], pose[1], 'ro', markersize=40, alpha=0.1)
    # print("plot_circle")
    # ax.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
    # head_length=0.1, head_width=0.1)
    axes[0].arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
                head_length=0.1, head_width=0.1)

    FOV_ANGLE=math.pi/4
    LENGTH = 0.8  # [m]
    WIDTH = 0.5  # [m]
    HALF_LENGTH = LENGTH/2.0  # [m]
    SENSOR_LENGTH = 1.5  # [m]
    WHEEL_LEN = 0.2  # [m]
    WHEEL_WIDTH = 0.2  # [m]

    sensor_outline = np.matrix([[0.0, SENSOR_LENGTH , SENSOR_LENGTH, 0.0],                          #sensor center
                         [0.0,SENSOR_LENGTH*math.tan(FOV_ANGLE),  -SENSOR_LENGTH*math.tan(FOV_ANGLE), 0.0]])

    outline = np.matrix([[-HALF_LENGTH, HALF_LENGTH, HALF_LENGTH, -HALF_LENGTH, -HALF_LENGTH],
                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
    yaw = pose[2]

    Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                      [-math.sin(yaw), math.cos(yaw)]])

    outline = (outline.T * Rot1).T
    outline[0, :] += pose[0]
    outline[1, :] += pose[1]

    sensor_outline = (sensor_outline.T * Rot1).T
    sensor_outline[0, :] += pose[0]
    sensor_outline[1, :] += pose[1]
 
    #DRAW an agent_body
    axes[0].plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(),'b')

    #DRAW SENSOR FOV
    # axes[0].plot(np.array(sensor_outline[0, :]).flatten(),
             # np.array(sensor_outline[1, :]).flatten(),'y')
    # axes[0].fill(np.array(sensor_outline[0, :]).flatten(),
             # np.array(sensor_outline[1, :]).flatten(),'y', alpha=0.25)


def plot_map(pos_x,pos_y,way_x, way_y, waytimes):
    axes[0].scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial point
    axes[0].scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
    # axes[0].plot(pos_x, pos_y, 'o', markersize = 20, fillstyle='none',color='black')             #trajectory point
    axes[0].plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='red')             #trajectory point
    axes[0].set_xlabel("x[m]")
    axes[0].set_ylabel("y[m]")
    axes[0].grid(True)
    for i in range(len(waytimes)):
        axes[0].text(way_x[i], way_y[i]-1,str(waytimes[i]), color='r')


#obstacles
def plot_obstacles(obstacles):
    for obs in obstacles:
        obs.draw(axes[0])


def visualize(traj, pose, obstacles,params):
    # ax = plt.gca()
    # plt.plot(traj[:,0], traj[:,1], 'g')
    plot_robot(pose, params)
    plot_obstacles(obstacles)

    axes[0].set_xlim([-params.area_size, params.area_size])   # limit the plot space
    axes[0].set_ylim([-params.area_size, params.area_size])   # limit the plot space
    axes[0].plot(traj[:,0], traj[:,1], 'k')
    # plt.legend()


#dyanmics
def simple_motion(state, goal, params):
    # state = [x(m), y(m), yaw(rad) ,velocity(m/s)]
    # input = [a(m/s**2), steering(rad) ]
    a =Update_a(state,goal)
    delta = Update_phi(state,goal)
    print("a:", a, ", delta: ", delta)
    print("pre-state[2]:", state[2])

    state[0] +=  state[3] * math.cos(state[2]) * dt
    state[1] +=  state[3] * math.sin(state[2]) * dt
    # state[2] +=  state[3] / L * math.tan(delta) * dt
    state[2] +=  0.75*math.sin(delta) * dt
    state[3] +=  a * dt

    print("pre-state[2]:", state[2])

    if state[3] >= params.max_vel: state[3] = params.max_vel
    if state[3] <= params.min_vel: state[3] = params.min_vel

    return state

def Update_a(state, goal):
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]
    dist_to_goal = sqrt(dx ** 2 + dy ** 2)
    print("dist_to_goal", dist_to_goal )
    input_a = Kv * dist_to_goal
    print("input_a", input_a )
    return input_a

def Update_phi(state, goal):
    des_phi = math.atan2(goal[1] - state[1], goal[0] - state[0])
    cur_yaw = state[2]
    err_phi = des_phi-cur_yaw
    # err_phi = math.atan2(goal[1] - state[1], goal[0] - state[0]) - state[2]
    # print("des_phi: ",des_phi)
    # print("cur_phi: ",cur_yaw)
    # print("err_phi: ",err_phi)
    # if phi>0.0:
        # phi=0.9
    # else:
        # phi=-0.9
    # if err_phi > math.pi:
       # err_phi = err_phi-2*math.pi 
    # elif err_phi <- math.pi:
        # err_phi=err_phi+2*math.pi

    return err_phi

#dyanmics
def motion(state, goal, params):
    # state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]
    goal_yaw = atan2(dy, dx)
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

#Define two windows: 
# axes[0] : robot, obstacle, waypoints, trajectory
# axes[1] : sensor_map,occ_grid
fig,axes=plt.subplots(nrows=3,ncols=1,figsize=(10,30))

params = Params()
params_map =  map_params()

timeindex = "04171450"
# timeindex = "04021858"
# Open the desired file for reading
dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path=dir_path[:-4]
file_name =dir_path + "/results/data/robot_" +timeindex+"_.csv"
wayfile_name =dir_path + "/results/data/waypoints_" +timeindex+"_.csv"
obsfile_name =dir_path + "/results/data/obstacles_"+timeindex+"_.csv"

# Open the desired file for reading
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

trajectories = [pos_x, pos_y, vel_x, vel_y, ]

#waypoint
way_x=[]
way_y=[]

regex = re.compile('[-+]?\d*\.\d+|[-+]?\d+')                #set pattern in order to find integer in string
way_coords = np.asarray(waydf['coords'][1:])
for i in range(len(way_coords)):
    nums = [float(k) for k in regex.findall(way_coords[i])] #find integer value in string format '[ int, int ]'
    way_x.append(nums[0])
    way_y.append(nums[1])
    # print("waypoints : (x,y ) = (", way_x,", ", way_y,")")

# print("waypoints :way_x)
floatregex =re.compile('[-+]?\d*\.\d+|[-+]?\d+') 
obstacles = []                                  # list which will contain all obstacles
obstacle_coords = np.asarray(obsdf['obstacle'][0:])
# print("obstacle coords")
# print(obstacle_coords)
for i in range(len(obstacle_coords)):
    if i<1:
        nums = [float(k) for k in floatregex.findall(obstacle_coords[i])] #find integer value in string format '[ int, int ]'
        # print(nums)
        obs = Obstacle(nums[0]-1, nums[1]-1, nums[2], nums[3])          #xmin,ymin, 
        # obs.draw()
        obstacles.append(obs)                                   # attach obstacle to obstacle list
# print("num ofobstacles:", len(obstacles))



#plot figures 
# fig,axes=plt.figure(figsize=(10,20))
axes[0].scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial point
axes[0].scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
axes[0].plot(pos_x, pos_y, 'o', markersize = 20, fillstyle='none',color='black')             #trajectory point
axes[0].plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='green')             #trajectory point
for i in range(len(waytimes)):
    axes[0].text(way_x[i], way_y[i]-1,str(waytimes[i]), color='g')

area_size=5
locs, labels = plt.xticks()
# locs, labels = plt.yticks()
#FixMe!
# axes[0].xticks(np.arange(-area_size,area_size,1.0))
# axes[0].yticks(np.arange(-area_size,area_size,1.0))
# ax = plt.axes()

axes[0].set_xlabel('x')
axes[0].set_ylabel('y')
axes[0].set_xlim([-area_size, area_size])   # limit the plot space
axes[0].set_ylim([-area_size, area_size])   # limit the plot space
axes[0].grid(True)
# axes[0].tight_layout()

#simulation settings
ntimestep = len(pos_x)
goal_tol=0.2

goali = 0                           #define goal from waypoints set
goal = [way_x[goali], way_y[goali]]
	

# initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
# state = np.array([pos_x[0],pos_y[0], 0.0, np.pi/2, 0.0, 0.0])
state = np.array([pos_x[0],pos_y[0],np.pi/2, 0.0])
print("initial state: ",state)
traj = state[:2]
iter=0
simtime=0.0

#Checking initial and final goal
print("initial state: ",state)
print("goal : ",goal)

t_prev_goal = time.time()
pmap_global = initialize_global_occ_grid_map(params_map)

# for i in range(ntimestep):
for _ in range(params.numiters):
    state = simple_motion(state, goal, params)                        #dynamics
    goal_dist = sqrt((goal[0] - state[0])**2+(goal[1] - state[1])**2) #distance to gaol
    simtime = simtime + dt
    print("simtime" , simtime)
    t_current = time.time()
    if goal_dist < goal_tol:                                          # goal is reached
        print('Time from the previous reached goal:', t_current - t_prev_goal)
        # if goali < len(goal_x) - 1:
        if goali < len(way_x) - 1:
            goali += 1
        else:
            break
        t_prev_goal = time.time()
        goal = [way_x[goali], way_y[goali]]

    #plot
    if params.animate:
        #figure1
        axes[0].cla()
        # plt.plot(goal[0], goal[1])
        plot_map(pos_x,pos_y,way_x,way_y,waytimes)
        axes[0].plot(goal[0], goal[1])
        traj = np.vstack([traj, state[:2]])
        visualize(traj, state, obstacles, params)

        #figure2- local sensor window
        axes[1].cla()
        pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, minx, maxx, miny, maxy, xyreso = generate_ray_casting_grid_map(obstacles, params_map.xyreso, params_map.yawreso, state[0],state[1], state[2])
        draw_occmap(pmap_local, minx, maxx, miny, maxy, xyreso, state[0],state[1], axes[1])
        #draw sensor ray to obstacles
        for i in range(len(obstacles)):
            axes[0].plot([state[0], obstacles[i].vertices[obs_verticeid[i][0]][0]], [state[1], obstacles[i].vertices[obs_verticeid[i][0]][1]], color='orange')
            axes[0].plot([state[0], obstacles[i].vertices[obs_verticeid[i][1]][0]], [state[1], obstacles[i].vertices[obs_verticeid[i][1]][1]], color='orange')
            axes[0].plot([state[0], obstacles[i].vertices[closest_vertexid[i]][0]], [state[1], obstacles[i].vertices[closest_vertexid[i]][1]], color='orange')
        #test intersection
        for angle,inter_point in intersect_dic.items():
            axes[0].plot(inter_point[0], inter_point[1], '*', markersize= 5, fillstyle='none',color='green')


        # axes[1].plot(ox, oy, "xr")


        #figure3- global occupancy grid
        axes[2].cla()
        pmap_global = update_occ_grid_map(state, updated_grids,pmap_global,params_map)
        draw_occmap_global(pmap_global,params_map.xmin_global, params_map.xmax_global, params_map.ymin_global,
                params_map.ymax_global, params_map.xyreso, axes[2])

        plt.pause(0.001)
        # plt.show()

    iter=iter+1
    # if iter%50==1:
        # input()

plt.show()
     
     






