# !/usr/bin/env python3
from numpy import genfromtxt
import numpy as np
from numpy import linalg as LA
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
from grid_map import GridMap

#import numpy as np
f_max=0.3
v_max=0.4
#probability
l_occ=np.log(0.8/0.2)
l_free=np.log(0.2/0.8)

Krep = 10.0 # repulsive potential gain
Katt = 5.0 # attractive potential gain
POT_WIDTH = 5.0 # potential area width [m]
SAFETY_DIST = 1.5

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
        self.xmin=-15
        self.xmax=15
        self.ymin=-15
        self.ymax=15
        self.xw = int(round((self.xmax - self.xmin) / self.xyreso))
        self.yw = int(round((self.ymax - self.ymin) / self.xyreso))
        self.sensor_range=5

class human_map_params: # This is where to change the human FOV size, unsure how to change shape...
    def __init__(self):
        self.xyreso = 0.25  # x-y grid resolution [m]
        self.yawreso = math.radians(6)  # yaw angle resolution [rad]
        self.xmin=-15
        self.xmax=15
        self.ymin=-15
        self.ymax=15
        self.xw = int(round((self.xmax - self.xmin) / self.xyreso))
        self.yw = int(round((self.ymax - self.ymin) / self.xyreso))
        self.sensor_range=2


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

class humanParams:
    def __init__(self):
        self.numiters = 2000
        self.dt = 0.2
        self.goal_tol = 0.25
        self.max_vel = 0.5 # m/s
        self.min_vel = 0.0 # m/s
        self.sensor_range_m = 1.0 # m
        self.animate = 1
        self.area_size = 5
        # self.time_to_switch_goal = 5.0 # sec #inactive for now
        # self.sweep_resolution = 0.4 # m

# class State:
    # def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        # self.x = x
        # self.y = y
        # self.yaw = yaw
        # self.v = v

def draw_occmap(data, params_map,agent_x, agent_y, ax):

    minx=params_map.xmin
    miny=params_map.ymin
    maxx=params_map.xmax
    maxy=params_map.ymax
    xyreso=params_map.xyreso

    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    ax.pcolor(x+agent_x, y+agent_y, data, vmax=1.0, cmap=plt.cm.Blues)
    ax.set_xlim([agent_x-2*params.area_size, agent_x+2*params.area_size])   # limit the plot space
    ax.set_ylim([agent_y-2*params.area_size, agent_y+2*params.area_size])   # limit the plot space

def draw_occmap_global(data,parmas_globalmap, ax):

    minx=params_globalmap.xmin
    miny=params_globalmap.ymin
    maxx=params_globalmap.xmax
    maxy=params_globalmap.ymax
    xyreso=params_globalmap.xyreso
    data = 1-1./(1.0+np.exp(data))

    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    ax.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    ax.set_xlim([1.1*minx, 1.1*maxx])   # limit the plot space
    ax.set_ylim([1.1*miny, 1.1*maxy])   # limit the plot space

def get_map_entropy(pmap_global,params_map):
    entropy_sum=0
    pmap= 1-1./(1.0+np.exp(pmap_global))
    for ix in range(params_map.xw-1):
        for iy in range(params_map.yw-1):
            p =pmap[ix][iy]
            # print("p: ", p)
            if p>0.0 and p<1.0:
                entropy_sum+=(p*math.log(p)+(1-p)*math.log(1-p))
            # entropy_sum+=p*math.log(p)

    return -entropy_sum




def update_occ_grid_map(state, local_map, params_local, global_map, params_global):
    ##for observed cell in local window--> update
    # print("local grids, xw, yw : ", params_local.xw, params_local.yw)
    # print("global grids, xw, yw : ", params_global.xw, params_global.yw)
    updated_list =[]

    for ix_local in range(params_local.xw-1):
        for iy_local in range(params_local.yw-1):
            px = params_local.xmin+ix_local*params_local.xyreso
            py = params_local.ymin+iy_local*params_local.xyreso

            ix_global= math.floor((px-params_global.xmin)/params_global.xyreso)
            iy_global= math.floor((py-params_global.ymin)/params_global.xyreso)
            # print("(ix_global, iy_global): ",ix_global, " , ", iy_global)
            meas = local_map[ix_local][iy_local]
            global_map[ix_global][iy_global] +=meas

    return global_map

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


def plot_human(pose, params):
    r = params.sensor_range_m

    axes[0].plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
                [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='r')
    axes[0].plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
                [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='r')

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
    LENGTH = 0.4  # [m]
    WIDTH = 0.25  # [m]
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
             np.array(outline[1, :]).flatten(),'r')


def plot_map(pos_x,pos_y,way_x, way_y, waytimes):
    axes[0].scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial robot point
    axes[0].scatter(-3.0, 0.0, facecolor='red',edgecolor='red')      #initial human point
    axes[0].scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
    
    # Plot the vertices of the human path
    axes[0].scatter(-2.0, 2.0, facecolor='green',edgecolor='green')
    axes[0].scatter(-2.0, -2.0, facecolor='green',edgecolor='green')
    axes[0].scatter(-4.0, 2.0, facecolor='green',edgecolor='green')
    axes[0].scatter(-4.0, -2.0, facecolor='green',edgecolor='green')

    # Plot the vertices of the pedestrian path
    axes[0].scatter(-2.0, -3.5, facecolor='black',edgecolor='black')
    axes[0].scatter(2.0, -3.5, facecolor='black',edgecolor='black')

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

def visualize_human(traj, pose, obstacles,params):
    # ax = plt.gca()
    # plt.plot(traj[:,0], traj[:,1], 'g')
    plot_human(pose, params)
    plot_obstacles(obstacles)

    axes[0].set_xlim([-params.area_size, params.area_size])   # limit the plot space
    axes[0].set_ylim([-params.area_size, params.area_size])   # limit the plot space
    axes[0].plot(traj[:,0], traj[:,1], 'c', ':')
    # plt.legend()

#dyanmics
def simple_motion(state, goal, params):
    # state = [x(m), y(m), yaw(rad) ,velocity(m/s)]
    # input = [a(m/s**2), steering angle(rad) ]
    a =Update_a(state,goal)
    delta = Update_phi(state,goal)
    # print("a:", a, ", delta: ", delta)
    # print("pre-state[2]:", state[2])

    state[0] +=  state[3] * math.cos(state[2]) * dt
    state[1] +=  state[3] * math.sin(state[2]) * dt
    # state[2] +=  state[3] / L * math.tan(delta) * dt
    state[2] +=  0.75*math.sin(delta) * dt
    # state[2] +=  (delta) * dt
    state[3] +=  a * dt

    # print("pre-state[2]:", state[2])

    if state[3] >= params.max_vel: state[3] = params.max_vel
    if state[3] <= params.min_vel: state[3] = params.min_vel

    return state

def Update_a(state, goal):
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]
    dist_to_goal = sqrt(dx ** 2 + dy ** 2)
    # print("dist_to_goal", dist_to_goal )
    input_a = Kv * dist_to_goal
    # print("input_a", input_a )
    return input_a

def Update_phi(state, goal):
    des_phi = math.atan2(goal[1] - state[1], goal[0] - state[0])
    cur_yaw = state[2]
    err_phi = des_phi-cur_yaw

    return err_phi

#human model of motion
def human_motion(motion_state, goal, human_params):
    # target_state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    dx = goal[0] - motion_state[0]
    dy = goal[1] - motion_state[1]
    goal_yaw = atan2(dy, dx)
    K_theta = 2.0
    # target_state[4] = K_theta*math.sin(goal_yaw - target_state[2]) # omega(rad/s)
    gamma = Update_phi(motion_state, goal)
    motion_state[2] +=  0.75*math.sin(gamma) * dt # yaw(rad)

    dist_to_goal = np.linalg.norm(goal - motion_state[:2])
    K_v = 0.1
    motion_state[3] += K_v*dist_to_goal
    if motion_state[3] >= human_params.max_vel: motion_state[3] = human_params.max_vel
    if motion_state[3] <= human_params.min_vel: motion_state[3] = human_params.min_vel

    dv = human_params.dt*motion_state[3]
    motion_state[0] += dv*np.cos(motion_state[2]) # x(m)
    motion_state[1] += dv*np.sin(motion_state[2]) # y(m)

    return target_state

class Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0


def check_robot_in_FOV(target_state, state, human_params_localmap, gridmap):
    # find area covered by the human FOV
    # print("target_state [m]: ", state[:2])

    top_right = np.array([target_state[0] + human_params_localmap.sensor_range, target_state[1] + human_params_localmap.sensor_range, -np.pi/2, 0.0])
    # print(" [m]: ", top_right[:2])
    
    lower_right = np.array([target_state[0] + human_params_localmap.sensor_range, target_state[1] - human_params_localmap.sensor_range, -np.pi/2, 0.0])
    # print("lower_right [m]: ", lower_right[:2])
    
    top_left = np.array([target_state[0] - human_params_localmap.sensor_range, target_state[1] + human_params_localmap.sensor_range, -np.pi/2, 0.0])
    # print("top_left [m]: ", top_left[:2])
    
    lower_left = np.array([target_state[0] - human_params_localmap.sensor_range, target_state[1] - human_params_localmap.sensor_range, -np.pi/2, 0.0])
    # print("lower_left [m]: ", lower_left[:2]) 
    
    # print("-------------------------------------------------")
    # convert the vertices to grid units
    pose_human = gridmap.meters2grid(target_state[:2])
    # print("pose_human check robot in FOV: ", pose_human)
    pose_tr = gridmap.meters2grid(top_right[:2])
    # print("top right corner of FOV check robot in FOV: ", pose_tr)
    pose_lr = gridmap.meters2grid(lower_right[:2])
    # print("lower right corner of FOV check robot in FOV: ", pose_lr)
    pose_tl = gridmap.meters2grid(top_left[:2])
    # print("top left corner of FOV check robot in FOV: ", pose_tl)
    pose_ll = gridmap.meters2grid(lower_left[:2])
    # print("lower left corner of FOV check robot in FOV: ", pose_ll)

    # find the robot pose in grid
    pose_robot = gridmap.meters2grid(state[:2])
    # print("pose_robot check robot in FOV: ", pose_robot)


    if pose_robot[0] <= pose_tr[0] and pose_robot[0] >= pose_tl[0]:
        if pose_robot[1] <= pose_tr[1] and pose_robot[1] >= pose_lr[1]:
            robot_in_FOV = True
        else: robot_in_FOV = False
    else: robot_in_FOV = False
    # see if robot is in this
    # return true/false

    # print("robot_in_FOV: ", robot_in_FOV)
    return robot_in_FOV

def check_wall_in_FOV(target_state, human_params_localmap, gridmap):
    wall_near_point_list = []
    # first we find out if wall in FOV
    if abs(target_state[0]) > 3.0 or abs(target_state[1]) > 3.0:
        wall_in_FOV = True
        # Then we find out the nearest point on the wall
        if target_state[0] > 3.0:
            wall_nearest_point = np.array([5.0, target_state[1]])
            wall_near_point_list.append(wall_nearest_point)
        
        if target_state[1] > 3.0:
            wall_nearest_point = np.array([target_state[0], 5.0])
            wall_near_point_list.append(wall_nearest_point)
        
        if target_state[0] < -3.0:
            wall_nearest_point = np.array([-5.0, target_state[1]])
            wall_near_point_list.append(wall_nearest_point)
        
        if target_state[1] < -3.0:
            wall_nearest_point = np.array([target_state[0], -5.0])
            wall_near_point_list.append(wall_nearest_point)
    else:
        wall_in_FOV = False  
    # print("wall_near_point_list: ", wall_near_point_list)

    return wall_in_FOV, wall_near_point_list

def check_obstacle_in_FOV(target_state, human_params_localmap, gridmap, obstacles_array):

    # init empty array 
    distance_vec = np.empty([len(obstacles_array[0])])
    # get the distance b/w human and each vertex and fill array
    for i in range(len(obstacles_array[0])):
        temp_dist = sqrt((obstacles_array[0][i][0] - target_state[0])**2+(obstacles_array[0][i][1] - target_state[1])**2)
        # print("temp_dist = ",temp_dist)
        distance_vec[i] = temp_dist
    # find the nearest vertex 
    A = np.partition(distance_vec, 1)[:1]
    # now find the original index of the closest vertex
    for j in range(len(distance_vec)):
        if A == distance_vec[j]:
            index1 = j
    # grab the two nearest vertices
    vertex_1 = np.array([obstacles_array[0][index1][0], obstacles_array[0][index1][1]])

    # print("obstacles_array[0]: ", obstacles_array[0])
    # print("vertex_1: ", vertex_1)
    # print("vertex_1[0]: ", vertex_1[0])
    # print("vertex_1[1]: ", vertex_1[1])

    # print("obstacles_array[0][0][1]: ", obstacles_array[0][0][1])
    # print("obstacles_array[0][1][1]: ", obstacles_array[0][3][1])
    # print("target_state = ", target_state)

    # if the robot is to the right or left of the obstacle
    if target_state[1] >= obstacles_array[0][0][1] and target_state [1] <= obstacles_array[0][3][1] and (target_state[0] < obstacles_array[0][0][0] or target_state[0] > obstacles_array[0][1][0]):
        obstacle_near_point = np.array([vertex_1[0],target_state[1]])
        # print("case 1")
    elif target_state[1] < obstacles_array[0][0][1] and (target_state[0] < obstacles_array[0][0][0] or target_state[0] > obstacles_array[0][1][0]):
        obstacle_near_point = np.array([vertex_1[0],vertex_1[1]])
        # print("case 2")
    elif target_state [1] > obstacles_array[0][3][1] and (target_state[0] < obstacles_array[0][0][0] or target_state[0] > obstacles_array[0][1][0]):
        obstacle_near_point = np.array([vertex_1[0],vertex_1[1]])
        # print("case 3")
    # if the robot is below / above 
    elif target_state[0] >= obstacles_array[0][0][0] and target_state[0] <= obstacles_array[0][1][0]:
        # print("case 4")
        obstacle_near_point = np.array([target_state[0], vertex_1[1]])
    else: 
        obstacle_near_point = np.array([0.0, 0.0])

    distance_to_nearest_vertex = sqrt((target_state[0] - obstacle_near_point[0])**2 + (target_state[1] - obstacle_near_point[1])**2)

    # is the object in FOV?
    if distance_to_nearest_vertex <= human_params_localmap.sensor_range:
        obstacle_in_FOV = True
    else:
        obstacle_in_FOV = False
    # obstacle_in_FOV = False
    # obstacle_near_point = np.array([0,0])
    return obstacle_in_FOV, obstacle_near_point

def potential_goal_update(target_goal, target_state, state, params, t_current, t_prev_goal, target_goali, robot_in_FOV, wall_in_FOV, wall_near_point_list, obstacle_in_FOV, obstacle_near_point):
    init = Point()
    final = Point()
    x_sum = 0
    y_sum = 0
    divisor = 0

    prev_target_goal = [target_goal[0], target_goal[1]]

    # if target_state[0] > -2.0:
    #     print("target_state[0]: ", target_state[0])

    # Initial human position 
    init.x = target_state[0]
    init.y = target_state[1]
    # vector for the wall potentials
    wall_pot_vectors = np.empty([len(wall_near_point_list), 2]) 


    if robot_in_FOV: ### THIS NEEDS TO BE DEBUGGED???? MAYBE? 
        print("robot_in_FOV = True")
        human_robot_dist = sqrt((state[0] - target_state[0])**2+(state[1] - target_state[1])**2) 
        robot_potential = (1.0/2.0) * Krep * ((1.0/human_robot_dist)-(1/SAFETY_DIST))**2

        # Get human robot relative info
        dx = target_state[0] - state[0]
        dy = target_state[1] - state[1]
        human_robot_pot_vec = robot_potential * np.array([dx, dy])
        x_sum += human_robot_pot_vec[0]
        y_sum += human_robot_pot_vec[1]
        divisor += 1
        # print("human_robot_pot_vec: ", human_robot_pot_vec)
    
    if wall_in_FOV: 
        for i in range(len(wall_near_point_list)):
            wall_dist = sqrt((wall_near_point_list[i][0] - target_state[0])**2+(wall_near_point_list[i][1] - target_state[1])**2) 
            if wall_dist < 0.3: # Only want to actually avoid though if within this dist 
                print("wall_in_FOV = True")
                wall_potential = (1.0/2.0) * Krep * ((1.0/wall_dist)-(1/SAFETY_DIST))**2
                dx = target_state[0] - wall_near_point_list[i][0]
                dy = target_state[1] - wall_near_point_list[i][1]
                tmp = wall_potential * np.array([dx, dy])
                # print("wall_pot_vector[i] = ", tmp)
                wall_pot_vectors[i] = tmp
            elif robot_in_FOV == False:
                if prev_target_goal == [-4,-2] or prev_target_goal == [-4,2] or prev_target_goal == [-2,2] or prev_target_goal == [-2,-2]:
                    target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
                else:
                    target_goali -= 1
                    if target_goali < 0:
                        target_goali = 3
                    target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
                return target_goal, target_goali
            if wall_dist < 0.3:
                for i in range(len(wall_pot_vectors)):
                    x_sum += wall_pot_vectors[i][0]
                    y_sum += wall_pot_vectors[i][1]
                    divisor += 1
    
    if obstacle_in_FOV: 
        obst_hum_dist = sqrt((target_state[0] - obstacle_near_point[0])**2 + (target_state[1] - obstacle_near_point[1])**2)
        if obst_hum_dist < 0.3:
            print("obstacle_in_FOV = True")
            obstacle_potential = (1.0/2.0) * Krep * ((1.0/obst_hum_dist)-(1/SAFETY_DIST))**2
            dx = target_state[0] - obstacle_near_point[0]
            dy = target_state[1] - obstacle_near_point[1]
            obst_pot_vec = obstacle_potential * np.array([dx, dy])
            x_sum += obst_pot_vec[0]
            y_sum += obst_pot_vec[1]
            divisor += 1
        elif robot_in_FOV == False:
            if prev_target_goal == [-4,-2] or prev_target_goal == [-4,2] or prev_target_goal == [-2,2] or prev_target_goal == [-2,-2]:
                target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
            else:
                target_goali -= 1
                if target_goali < 0:
                    target_goali = 3
                target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
            return target_goal, target_goali

    
    x_avg = x_sum / divisor
    y_avg = y_sum / divisor
    # print("x_avg = ", x_avg)
    # print("y_avg = ", y_avg)

    # Slope of the resultant potential force on the human
    if abs(x_avg) <= 0.00001:
        x_avg = 0.1
    slope = y_avg / x_avg
    # print("y_avg = ", y_avg)
    # print("x_avg = ", x_avg)
    # print("slope = ",slope)
    if slope == 0 and init.x < obstacles_array[0][0][0]:
        final.x = init.x - 0.1
        final.y = init.y
    elif slope == 0 and init.x > obstacles_array[0][1][0]:
        final.x = init.x + 0.1
        final.y = init.y
    elif slope > 0 and y_avg < 0:
        deltax = (0.5 / sqrt(1 + (slope*slope)))
        deltay = slope * deltax
        final.x = init.x - deltax
        final.y = init.y - deltay
    elif slope > 0 and y_avg > 0:
        deltax = (0.5 / sqrt(1 + (slope*slope)))
        deltay = slope * deltax
        final.x = init.x + deltax
        final.y = init.y + deltay
    elif slope < 0 and x_avg > 0:
        deltax = (0.5 / sqrt(1 + (slope*slope)))
        deltay = slope * deltax
        final.x = init.x + deltax
        final.y = init.y + deltay
    elif slope < 0 and x_avg < 0:
        deltax = (0.5 / sqrt(1 + (slope*slope)))
        deltay = slope * deltax
        final.x = init.x - deltax
        final.y = init.y - deltay

    if final.x < -5.0: 
        final.x = -4.5
    if final.x > 5.0: 
        final.x = 4.5
    if final.y < -5.0: 
        final.y = -4.5
    if final.y > 5.0: 
        final.y = 4.5
    target_goal = [final.x, final.y]
    print("human current state: ", target_state[:2])
    print("robot current state: ", state[:2])
    print("new human goal (Avoid trajectory): ", target_goal)
    return target_goal, target_goali

def target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali):
    
    target_goal_dist = sqrt((target_goal[0] - target_state[0])**2+(target_goal[1] - target_state[1])**2)

    if target_goal == [-4,-2] or target_goal == [-4,2] or target_goal == [-2,2] or target_goal == [-2,-2]:
        if target_goal_dist < goal_tol:
            #TODO FIX THE TIME 
            print('Time from the previous reached goal (human):', t_current - t_prev_goal)
            if target_goali == 0: 
                target_goali += 1
                target_goal = [-4, -2]
            elif target_goali == 1:
                target_goali += 1
                target_goal = [-4, 2]
            elif target_goali == 2:
                target_goali += 1
                target_goal = [-2, 2]
            else:
                target_goali = 0
                target_goal = [-2, -2]
            t_prev_goal = time.time()
            print("human new goal(loop trajectory): ", target_goal)
    else:
        if target_goali == 0: 
            # target_goali += 1
            target_goal = [-4, -2]
        elif target_goali == 1:
            # target_goali += 1
            target_goal = [-4, 2]
        elif target_goali == 2:
            # target_goali += 1
            target_goal = [-2, 2]
        else:
            # target_goali = 0
            target_goal = [-2, -2]
        t_prev_goal = time.time()
        print("human new goal(loop trajectory): ", target_goal)

    return target_goal, target_goali


def pedestrian_goal_update(state, goal, params, t_current, t_prev_goal, pedestrian_goal_bool):
    '''
    We simply need to check if the human is at the goal and if so then update the goal position
    to remain in the line trajectory.
    '''

    return pedestrian_goal, pedestrian_goal_bool

def obstacle_check(pose, gridmap, params):
    # print("obstacle_check pose", pose)
    gmap = gridmap
    # r = int(100*params.sensor_range_m)
    # r = int(params.sensor_range_m)
    r = int(1)
    # print("r", r)
    
    back = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
    front = [pose[0]+r*np.cos(pose[2]), pose[1]+r*np.sin(pose[2])]
    right = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
    left = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]

    pi = np.array(pose[:2], dtype=int)
    backi = np.array(back, dtype=int)
    fronti = np.array(front, dtype=int)
    lefti = np.array(left, dtype=int)
    righti = np.array(right, dtype=int)

    obstacle = {
        'front': 0,
        'back':  0,
        'right': 0,
        'left':  0,
                }

    # print("human_pose obstacle_check: ", pose)
    for i in np.arange(min(pi[0], fronti[0]), max(pi[0], fronti[0])+1):
        for j in np.arange(min(pi[1], fronti[1]), max(pi[1], fronti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            # print("front (m, n):", m, n)
            if gmap[m,n]:
                # print('FRONT collision')
                obstacle['front'] = 1

    for i in np.arange(min(pi[0], backi[0]), max(pi[0], backi[0])+1):
        for j in np.arange(min(pi[1], backi[1]), max(pi[1], backi[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            # print("back (m, n):", m, n)
            if gmap[m,n]:
                # print('BACK collision')
                obstacle['back'] = 1

    for i in np.arange(min(pi[0], lefti[0]), max(pi[0], lefti[0])+1):
        for j in np.arange(min(pi[1], lefti[1]), max(pi[1], lefti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            # print("left (m, n):", m, n)
            if gmap[m,n]:
                # print('LEFT collision')
                obstacle['left'] = 1

    for i in np.arange(min(pi[0], righti[0]), max(pi[0], righti[0])+1):
        for j in np.arange(min(pi[1], righti[1]), max(pi[1], righti[1])+1):
            m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
            # print("right (m, n):", m, n)
            if gmap[m,n]:
                # print('RIGHT collision')
                obstacle['right'] = 1

    return obstacle

def left_shift(pose, r):
    left = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
    return left
def right_shift(pose, r):
    right = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]
    return right
def back_shift(pose, r):
    back = pose
    back[:2] = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
    return back
def forward_shift(pose, r):
    forward = pose
    forward[:2] = [pose[0]+r*np.cos(pose[2]), pose[1]+r*np.sin(pose[2])]
    return forward
def turn_left(pose, yaw=np.pi/2*np.random.uniform(0.2, 0.6)):
    pose[2] -= yaw
    return pose
def turn_right(pose, yaw=np.pi/2*np.random.uniform(0.2, 0.6)):
    pose[2] += yaw
    return pose
def slow_down(state, params, dv=0.1):
    if state[3]>params.min_vel:
        state[3] -= dv
    return state


def collision_avoidance(target_state, gridmap, params):
    pose_grid = gridmap.meters2grid(target_state[:2])
    boundary = obstacle_check([pose_grid[0], pose_grid[1], target_state[2]], gridmap.gmap, params)

    if boundary['right'] or boundary['front']:
        # target_state = back_shift(target_state, 0.03)
        target_state = slow_down(target_state, params)
        target_state = turn_left(target_state, np.radians(60))
        print("Collision_avoidance front/right")
        # target_state = forward_shift(target_state, 0.02)
    elif boundary['left']:
        # target_state = back_shift(target_state, 0.03)
        target_state = slow_down(target_state, params)
        target_state = turn_right(target_state, np.radians(60))
        print("Collision_avoidance left")
        # target_state = forward_shift(target_state, 0.02)

    # Additionally need to avoid leaving the area
    if abs(target_state[0]) > 4.7 or abs(target_state[1]) > 4.7:
        target_state = slow_down(target_state, params)
        target_state = turn_left(target_state, np.radians(60))
        target_state = forward_shift(target_state, 0.2)
    
    return target_state


#Define two windows: 
# axes[0] : robot, obstacle, waypoints, trajectory
# axes[1] : sensor_map,occ_grid
fig,axes=plt.subplots(nrows=3,ncols=1,figsize=(10,40))

params = Params()
human_params = humanParams()
params_globalmap =  map_params()
params_localmap =  map_params()
human_params_localmap =  human_map_params()

timeindex = "04171450"

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
axes[0].scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial robot point
axes[0].scatter(-3.0, 0.0, facecolor='red',edgecolor='red')      #initial human point
axes[0].scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
# Plot the vertices of the target path
axes[0].scatter(-2.0, 2.0, facecolor='green',edgecolor='green')
axes[0].scatter(-2.0, -2.0, facecolor='green',edgecolor='green')
axes[0].scatter(-4.0, 2.0, facecolor='green',edgecolor='green')
axes[0].scatter(-4.0, -2.0, facecolor='green',edgecolor='green')
axes[0].plot(pos_x, pos_y, 'o', markersize = 20, fillstyle='none',color='black')             #trajectory point
axes[0].plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='green')             #trajectory point
# Plot the vertices of the pedestrian path
axes[0].scatter(-2.0, -3.0, facecolor='black',edgecolor='black')
axes[0].scatter(2.0, -3.0, facecolor='black',edgecolor='black')
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
target_goal = [-2.0, -2.0]           # pre defined target goal
pedestrian_goal = [2.0, -3.0]           # pre defined target goal

target_goali = 0                       # used to determine next vertex of square trajectory
pedestrian_goal_bool = True
	

# initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
# state = np.array([pos_x[0],pos_y[0], 0.0, np.pi/2, 0.0, 0.0])
state = np.array([pos_x[0],pos_y[0],np.pi/2, 0.0])
target_state = np.array([-3.0, 0.0, -np.pi/2, 0.0])
pedestrian_state = np.array([0.0, 1.0, -np.pi/2, 0.0]) 

traj = state[:2]
target_traj = target_state[:2]
pedestrian_traj = pedestrian_state[:2]

iter=0
simtime=0.0

# Checking initial position and first goal
print("robot initial state: ", state)
print("robot initial goal : ", goal)

# Checking the target intial position and first goal 
print("target initial state: ", target_state)
print("target initial goal : ", target_goal)

# Checking the target intial position and first goal 
print("pedestrian initial state: ", pedestrian_state)
print("pedestrian initial goal : ", pedestrian_goal)

t_prev_goal = time.time()
pmap_global = initialize_global_occ_grid_map(params_globalmap)
initial_entropy = get_map_entropy(pmap_global,params_globalmap)
print("initial entropy: ", initial_entropy )

flight_area_vertices = [ [-5.0, 5.0],
                         [5.0, 5.0],
                         [5.0, -5.0],
                         [-5.0, -5.0] ]

gridmap = GridMap(flight_area_vertices, state[:2])

<<<<<<< HEAD
Region_Boundary =12.5
walls=[]
obs = Obstacle(-Region_Boundary, -Region_Boundary, -Region_Boundary, Region_Boundary,True)          
walls.append(obs)                                   # attach obstacle to obstacle list
obs = Obstacle(-Region_Boundary, Region_Boundary, -Region_Boundary, -Region_Boundary,True)         
walls.append(obs)                                   # attach obstacle to obstacle list
obs = Obstacle(-Region_Boundary, Region_Boundary, Region_Boundary, Region_Boundary,True)          
walls.append(obs)                                   # attach obstacle to obstacle list
obs = Obstacle(Region_Boundary, Region_Boundary, -Region_Boundary, Region_Boundary,True)          
walls.append(obs)                                   # attach obstacle to obstacle list
obstacles_array = []
for i in range(len(obstacles)):
    
    tmp = np.array([ obstacles[i].vertices[0], 
                     obstacles[i].vertices[1], 
                     obstacles[i].vertices[2], 
                     obstacles[i].vertices[3] ])
    obstacles_array.append(tmp)
                   
gridmap.add_obstacles_to_grid_map(obstacles_array)

wall_near_point_list = []

#main simulation
# for i in range(ntimestep):
for _ in range(params.numiters):
    state = simple_motion(state, goal, params)                        # robot dynamics
    
    target_state = human_motion(target_state, target_goal, human_params) # human motion model
    target_state = collision_avoidance(target_state, gridmap, human_params)
    
    pedestrian_state = human_motion(target_state, target_goal, human_params) # human motion model
    pedestrian_state = collision_avoidance(target_state, gridmap, human_params)
    
    goal_dist = sqrt((goal[0] - state[0])**2+(goal[1] - state[1])**2) # robot distance to goal
    target_goal_dist = sqrt((target_goal[0] - target_state[0])**2+(target_goal[1] - target_state[1])**2) # target distance to goal
    pedestrian_goal_dist = sqrt((pedestrian_goal[0] - pedestrian_state[0])**2+(pedestrian_goal[1] - pedestrian_state[1])**2) # pedestrian distance to goal

    human_robot_dist = sqrt((state[0] - target_state[0])**2+(state[1] - target_state[1])**2)

    simtime = simtime + dt
    # print("simtime" , simtime)
    t_current = time.time()

    # robot goal is reached
    if goal_dist < goal_tol:                                          
        print('Time from the previous reached goal (robot):', t_current - t_prev_goal)
        # if goali < len(goal_x) - 1:
        if goali < len(way_x) - 1:
            goali += 1
        else:
            break
        t_prev_goal = time.time()
        goal = [way_x[goali], way_y[goali]]

    robot_in_FOV = check_robot_in_FOV(target_state, state, human_params_localmap, gridmap)
    wall_in_FOV, wall_near_point_list = check_wall_in_FOV(target_state, human_params_localmap, gridmap)
    obstacle_in_FOV, obstacle_near_point = check_obstacle_in_FOV(target_state, human_params_localmap, gridmap, obstacles_array)

    # New goal for the human: either continue on square or travel away from human
    if robot_in_FOV or wall_in_FOV or obstacle_in_FOV:
        target_goal, target_goali = potential_goal_update(target_goal, target_state, state, params, t_current, t_prev_goal, target_goali, robot_in_FOV, wall_in_FOV, wall_near_point_list, obstacle_in_FOV, obstacle_near_point)
        robot_in_FOV = False
        wall_in_FOV = False
        obstacle_in_FOV = False
    else:
        prev_target_goal = [target_goal[0], target_goal[1]]
        if prev_target_goal == [-4,-2] or prev_target_goal == [-4,2] or prev_target_goal == [-2,2] or prev_target_goal == [-2,-2]:
            target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
        else:
            target_goali -= 1
            if target_goali < 0:
                target_goali = 3
            target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
    #plot
    if params.animate:
        #figure1
        axes[0].cla() # cla() clears an axes
        # plt.plot(goal[0], goal[1])
        plot_map(pos_x,pos_y,way_x,way_y,waytimes) # plots the static points
        axes[0].plot(goal[0], goal[1])
        traj = np.vstack([traj, state[:2]])
        target_traj = np.vstack([target_traj, target_state[:2]])
        visualize(traj, state, obstacles, params)
        visualize_human(target_traj, target_state, obstacles, params)

        #figure2- robot local sensor window
        axes[1].cla()
        # pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, params_localmap.xmin, params_localmap.xmax, params_localmap.ymin, params_localmap.ymax, params_localmap.xyreso, params_localmap.xw, params_localmap.yw= generate_ray_casting_grid_map(obstacles, params_localmap, state[0],state[1], state[2])
        pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, params_localmap.xmin, params_localmap.xmax, params_localmap.ymin, params_localmap.ymax, params_localmap.xyreso, params_localmap.xw, params_localmap.yw= generate_ray_casting_grid_map(obstacles, walls, params_localmap, state[0],state[1], state[2])
        draw_occmap(pmap_local, params_localmap, state[0],state[1], axes[1])
        #draw sensor ray to obstacles
        for i in range(len(obstacles)):
            axes[0].plot([state[0], obstacles[i].vertices[obs_verticeid[i][0]][0]], [state[1], obstacles[i].vertices[obs_verticeid[i][0]][1]], color='orange')
            axes[0].plot([state[0], obstacles[i].vertices[obs_verticeid[i][1]][0]], [state[1], obstacles[i].vertices[obs_verticeid[i][1]][1]], color='orange')
            axes[0].plot([state[0], obstacles[i].vertices[closest_vertexid[i]][0]], [state[1], obstacles[i].vertices[closest_vertexid[i]][1]], color='orange')
        #test intersection
        for angle,inter_point in intersect_dic.items():
            axes[0].plot(inter_point[0], inter_point[1], '*', markersize= 5, fillstyle='none',color='green')

        ##################################################################
        # # figure 4- human local sensor window
        # axes[3].cla() #clear axes
        # human_pmap_local, human_updated_grids, human_intersect_dic, human_obs_verticeid, human_closest_vertexid, human_params_localmap.xmin, human_params_localmap.xmax, human_params_localmap.ymin, human_params_localmap.ymax, human_params_localmap.xyreso, human_params_localmap.xw, human_params_localmap.yw= generate_ray_casting_grid_map(obstacles, human_params_localmap, target_state[0],target_state[1], target_state[2])
        # draw_occmap(human_pmap_local, human_params_localmap, target_state[0],target_state[1], axes[3])
        # # draw sensor ray to obstacles
        # for i in range(len(obstacles)):
        #     axes[0].plot([target_state[0], obstacles[i].vertices[human_obs_verticeid[i][0]][0]], [target_state[1], obstacles[i].vertices[human_obs_verticeid[i][0]][1]], color='orange')
        #     axes[0].plot([target_state[0], obstacles[i].vertices[human_obs_verticeid[i][1]][0]], [target_state[1], obstacles[i].vertices[human_obs_verticeid[i][1]][1]], color='orange')
        #     axes[0].plot([target_state[0], obstacles[i].vertices[human_closest_vertexid[i]][0]], [target_state[1], obstacles[i].vertices[human_closest_vertexid[i]][1]], color='orange')
        # # TODO: Intersection is broken  
        # # for angle,inter_point in intersect_dic.items():
        # #     axes[0].plot(inter_point[0], inter_point[1], '*', markersize= 5, fillstyle='none',color='red')
        ##################################################################3

        # axes[1].plot(ox, oy, "xr")


        #figure3- global occupancy grid
        axes[2].cla()
        pmap_global = update_occ_grid_map(state, pmap_local,params_localmap, pmap_global,params_globalmap)
        draw_occmap_global(pmap_global,params_globalmap, axes[2])
        entropy = get_map_entropy(pmap_global, params_globalmap)
        # print("----entropy : ", entropy)

        plt.pause(0.001)
        # plt.show()

    iter=iter+1
    # if iter%50==1:
        # input()

    # if human_robot_dist < 0.25:
    #     print("human has been captured")
    #     break

plt.show()
     
     






