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
from humans_real_simulation import Update_phi, Update_a

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
        self.sensor_range=5


class humanParams:
    def __init__(self):
        self.numiters = 2000
        self.dt = 0.2
        self.goal_tol = 0.25
        self.max_vel = 0.6 # m/s
        self.min_vel = 0.0 # m/s
        self.sensor_range_m = 1.0 # m
        self.animate = 1
        self.area_size = 5
        # self.time_to_switch_goal = 5.0 # sec #inactive for now
        # self.sweep_resolution = 0.4 # m



def plot_target(pose, params, ax):
    r = params.sensor_range_m

    ax.plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
                [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='r')
    ax.plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
                [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='r')

    # Plot the vertices of the human path
    ax.scatter(-2.0, 2.0, facecolor='green',edgecolor='green')
    ax.scatter(-2.0, -2.0, facecolor='green',edgecolor='green')
    ax.scatter(-4.0, 2.0, facecolor='green',edgecolor='green')
    ax.scatter(-4.0, -2.0, facecolor='green',edgecolor='green')

    # axes[0.plot(pose[0], pose[1], 'ro', markersize=5)
    # circle= Circle((pose[0], pose[1]),r,linewidth=1,edgecolor='k',facecolor='k',alpha=0.3 )
    # ax.add_patch(circle)
    # axes[0.plot(pose[0], pose[1], 'ro', markersize=40, alpha=0.1)
    # print("plot_circle")
    # ax.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
    # head_length=0.1, head_width=0.1)
    ax.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
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
    ax.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(),'r')

def plot_pedestrian(pose, params, ax):
    r = params.sensor_range_m

    # Plot the vertices of the pedestrian path
    ax.scatter(-2.0, -3.5, facecolor='black',edgecolor='black')
    ax.scatter(2.0, -3.5, facecolor='black',edgecolor='black')

    ax.plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
                [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='k')
    ax.plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
                [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='k')

    # axes[0.plot(pose[0], pose[1], 'ro', markersize=5)
    # circle= Circle((pose[0], pose[1]),r,linewidth=1,edgecolor='k',facecolor='k',alpha=0.3 )
    # ax.add_patch(circle)
    # axes[0.plot(pose[0], pose[1], 'ro', markersize=40, alpha=0.1)
    # print("plot_circle")
    # ax.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
    # head_length=0.1, head_width=0.1)
    ax.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
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

    out_line = np.matrix([[-HALF_LENGTH, HALF_LENGTH, HALF_LENGTH, -HALF_LENGTH, -HALF_LENGTH],
                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
    yaw = pose[2]

    Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                      [-math.sin(yaw), math.cos(yaw)]])

    out_line = (out_line.T * Rot1).T
    out_line[0, :] += pose[0]
    out_line[1, :] += pose[1]

    sensor_outline = (sensor_outline.T * Rot1).T
    sensor_outline[0, :] += pose[0]
    sensor_outline[1, :] += pose[1]
 
    #DRAW an agent_body
    ax.plot(np.array(out_line[0, :]).flatten(),
             np.array(out_line[1, :]).flatten(),'k')



def visualize_humans(traj, pose, obstacles, params, pedestrian_traj, pedestrian_state, ax):
    # ax = plt.gca()
    # plt.plot(traj[:,0], traj[:,1], 'g')
    plot_target(pose, params, ax)
    plot_pedestrian(pedestrian_state, params, ax)
    # plot_obstacles(obstacles)

    ax.set_xlim([-params.area_size, params.area_size])   # limit the plot space
    ax.set_ylim([-params.area_size, params.area_size])   # limit the plot space
    ax.plot(traj[:,0], traj[:,1], 'c', ':')
    ax.plot(pedestrian_traj[:,0], pedestrian_traj[:,1], 'c', ':')
    # plt.legend()

#human model of motion
def human_motion(motion_state, goal, human_params, goal_tol, target_bool):
    # target_state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    dx = goal[0] - motion_state[0]
    dy = goal[1] - motion_state[1]
    goal_yaw = atan2(dy, dx)
    K_theta = 2.0

    # keep between +/- 2*pi
    if motion_state[2] > np.pi*2.0:
        motion_state[2] -= np.pi*2.0
    if motion_state[2] < -np.pi*2.0:
        motion_state[2] += np.pi*2.0
    # for comparison puposes
    if goal_yaw < 0:
        goal_yaw += 2.0*np.pi
    if motion_state[2] < 0:
        motion_state[2] += 2.0*np.pi
     
    yaw_diff = goal_yaw - motion_state[2]
    if target_bool:
        if goal == [-4,-2] or goal == [-4,2] or goal == [-2,2] or goal == [-2,-2]:
            if abs(yaw_diff) > np.pi/3.:                # if orientation is very far off: stop and turn
                motion_state[3] = 0.1
            else:                                       # else continue as normal
                dist_to_goal = np.linalg.norm(goal - motion_state[:2])
                K_v = 0.1
                motion_state[3] += K_v*dist_to_goal
    else:
        if abs(yaw_diff) > np.pi/3.:                # if orientation is very far off: stop and turn
            motion_state[3] = 0.1
        else:                                       # else continue as normal
            dist_to_goal = np.linalg.norm(goal - motion_state[:2])
            K_v = 0.1
            motion_state[3] += K_v*dist_to_goal
    # Now make the turn
    # target_state[4] = K_theta*math.sin(goal_yaw - target_state[2]) # omega(rad/s)
    gamma = Update_phi(motion_state, goal)
    motion_state[2] +=  1.5*math.sin(gamma) * dt # yaw(rad) 0.75
    # keep velocity limits
    if motion_state[3] >= human_params.max_vel: motion_state[3] = human_params.max_vel
    if motion_state[3] <= human_params.min_vel: motion_state[3] = human_params.min_vel

    dv = human_params.dt*motion_state[3]
    motion_state[0] += dv*np.cos(motion_state[2]) # x(m)
    motion_state[1] += dv*np.sin(motion_state[2]) # y(m)

    return motion_state

def get_human_FOV(human_state, human_params_localmap, gridmap):
    # Return Value in Map Frame 
    human_state[2] = orientation_processing(human_state[2])  
    if human_state[2] >= 3*np.pi/2:
        # get the vertices of the sensor field
        temp = 2*np.pi - human_state[2]
        ilx = human_params_localmap.sensor_range*np.cos(temp) + human_state[0]
        ily = human_params_localmap.sensor_range*np.sin(temp) + human_state[1]
        left = np.array([ilx, ily])
        front_left = np.array([ilx + human_params_localmap.sensor_range*np.cos(temp), ily - human_params_localmap.sensor_range*np.sin(temp)])
        irx = human_state[0] - human_params_localmap.sensor_range*np.cos(temp)
        iry = human_state[1] - human_params_localmap.sensor_range*np.sin(temp)
        right = np.array([irx, iry])
        front_right = np.array([irx + human_params_localmap.sensor_range*np.cos(temp), iry - human_params_localmap.sensor_range*np.sin(temp)])
        
        # convert to grid space (map frame)
        if human_state[2] <= 7.0*np.pi/4.0:
            pose_tr = gridmap.meters2grid(left)
            pose_lr = gridmap.meters2grid(front_left)
            pose_tl = gridmap.meters2grid(right)
            pose_ll = gridmap.meters2grid(front_right)
            return front_left, front_right, left, right, pose_tr, pose_lr, pose_tl, pose_ll
        else:
            pose_tr = gridmap.meters2grid(front_left)
            pose_lr = gridmap.meters2grid(front_right)
            pose_tl = gridmap.meters2grid(left)
            pose_ll = gridmap.meters2grid(right)
            return front_right, right, front_left, left, pose_tr, pose_lr, pose_tl, pose_ll

    elif np.pi <= human_state[2] < 3*np.pi/2:
        # get the vertices of the sensor field
        temp = human_state[2] - np.pi
        ilx = human_params_localmap.sensor_range*np.cos(temp) + human_state[0]
        ily = human_state[1] - human_params_localmap.sensor_range*np.sin(temp) 
        left = np.array([ilx, ily])
        front_left = np.array([ilx - human_params_localmap.sensor_range*np.cos(temp), ily - human_params_localmap.sensor_range*np.sin(temp)])
        irx = human_state[0] - human_params_localmap.sensor_range*np.cos(temp)
        iry = human_state[1] + human_params_localmap.sensor_range*np.sin(temp)
        right = np.array([irx, iry])
        front_right = np.array([irx - human_params_localmap.sensor_range*np.cos(temp), iry - human_params_localmap.sensor_range*np.sin(temp)])
        
        # convert to grid space (map frame)
        if human_state[2] >= 5.0*np.pi/4.0:
            pose_tr = gridmap.meters2grid(left)
            pose_lr = gridmap.meters2grid(front_left)
            pose_tl = gridmap.meters2grid(right)
            pose_ll = gridmap.meters2grid(front_right)
            return front_left, front_right, left, right, pose_tr, pose_lr, pose_tl, pose_ll
        else:
            pose_tr = gridmap.meters2grid(right)
            pose_lr = gridmap.meters2grid(left)
            pose_tl = gridmap.meters2grid(front_right)
            pose_ll = gridmap.meters2grid(front_left)
            return left, front_left, right, front_right, pose_tr, pose_lr, pose_tl, pose_ll

    elif np.pi/2 <= human_state[2] < np.pi:
        # get the vertices of the sensor field
        temp = np.pi - human_state[2]
        ilx = human_state[0] - human_params_localmap.sensor_range*np.cos(temp)
        ily = human_state[1] - human_params_localmap.sensor_range*np.sin(temp) 
        left = np.array([ilx, ily])
        front_left = np.array([ilx - human_params_localmap.sensor_range*np.cos(temp), ily + human_params_localmap.sensor_range*np.sin(temp)])
        irx = human_state[0] + human_params_localmap.sensor_range*np.cos(temp)
        iry = human_state[1] + human_params_localmap.sensor_range*np.sin(temp)
        right = np.array([irx, iry])
        front_right = np.array([irx - human_params_localmap.sensor_range*np.cos(temp), iry + human_params_localmap.sensor_range*np.sin(temp)])

        # convert to grid space (gridmap)
        if human_state[2] <= 3.0*np.pi/4.0:
            pose_tr = gridmap.meters2grid(front_right)
            pose_lr = gridmap.meters2grid(right)
            pose_tl = gridmap.meters2grid(front_left)
            pose_ll = gridmap.meters2grid(left)
            return right, left, front_right, front_left, pose_tr, pose_lr, pose_tl, pose_ll
        else:
            pose_tr = gridmap.meters2grid(right)
            pose_lr = gridmap.meters2grid(left)
            pose_tl = gridmap.meters2grid(front_right)
            pose_ll = gridmap.meters2grid(front_left)
            return left, front_left, right, front_right, pose_tr, pose_lr, pose_tl, pose_ll

    elif 0 <= human_state[2] < np.pi/2:
        # get the vertices of the sensor field
        ilx = human_state[0] - human_params_localmap.sensor_range*np.cos(human_state[2])
        ily = human_state[1] + human_params_localmap.sensor_range*np.sin(human_state[2]) 
        left = np.array([ilx, ily])
        front_left = np.array([ilx + human_params_localmap.sensor_range*np.cos(human_state[2]), ily + human_params_localmap.sensor_range*np.sin(human_state[2])])
        irx = human_state[0] + human_params_localmap.sensor_range*np.cos(human_state[2])
        iry = human_state[1] - human_params_localmap.sensor_range*np.sin(human_state[2])
        right = np.array([irx, iry])
        front_right = np.array([irx + human_params_localmap.sensor_range*np.cos(human_state[2]), iry + human_params_localmap.sensor_range*np.sin(human_state[2])])

        # convert to grid space (gridmap)
        if human_state[2] >= np.pi/4.0:
            pose_tr = gridmap.meters2grid(front_right)
            pose_lr = gridmap.meters2grid(right)
            pose_tl = gridmap.meters2grid(front_left)
            pose_ll = gridmap.meters2grid(left)
            return right, left, front_right, front_left, pose_tr, pose_lr, pose_tl, pose_ll
        else:
            pose_tr = gridmap.meters2grid(front_left)
            pose_lr = gridmap.meters2grid(front_right)
            pose_tl = gridmap.meters2grid(left)
            pose_ll = gridmap.meters2grid(right)
            return front_right, right, front_left, left, pose_tr, pose_lr, pose_tl, pose_ll



class Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0



def check_robot_in_FOV(target_state, state, human_params_localmap, gridmap):
    # find area covered by the human FOV

    target_state[2] = orientation_processing(target_state[2])

    lower_right, lower_left, front_right, front_left, pose_tr, pose_lr, pose_tl, pose_ll = get_human_FOV(target_state, human_params_localmap, gridmap)
    
    

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

def check_wall_in_FOV(target_state, human_params_localmap, gridmap, map_params):
    wall_near_point_list = []
    wall_in_FOV = False
    
    target_state[2] = orientation_processing(target_state[2])

    # first we find out if wall in FOV
    if abs(target_state[0]) > (map_params.xmax - human_params_localmap.sensor_range) or abs(target_state[1]) > (map_params.ymax - human_params_localmap.sensor_range):
        # Then we find out the nearest point on the wall
        if target_state[0] > (map_params.xmax - SAFETY_DIST) and 0 <= target_state[2] < np.pi/2.0 and 3.0*np.pi/2.0 < target_state[2] <= 2.0*np.pi:
            wall_in_FOV = True
            wall_nearest_point = np.array([map_params.xmax, target_state[1]])
            wall_near_point_list.append(wall_nearest_point)
        
        if target_state[1] > (map_params.ymax - SAFETY_DIST) and 0 < target_state[2] < np.pi:
            wall_in_FOV = True
            wall_nearest_point = np.array([target_state[0], map_params.ymax])
            wall_near_point_list.append(wall_nearest_point)
        
        if target_state[0] < (map_params.xmin + human_params_localmap.sensor_range) and np.pi/2.0 < target_state[2] < 3.0*np.pi/2.0:
            wall_in_FOV = True
            wall_nearest_point = np.array([map_params.xmin, target_state[1]])
            wall_near_point_list.append(wall_nearest_point)
        
        if target_state[1] < (map_params.ymin + human_params_localmap.sensor_range) and np.pi < target_state[2] < 2.0*np.pi:
            wall_in_FOV = True
            wall_nearest_point = np.array([target_state[0], map_params.ymin])
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

def potential_goal_update(target_goal, target_state, state, params, t_current, t_prev_goal, target_goali, robot_in_FOV, wall_in_FOV, wall_near_point_list, obstacle_in_FOV, obstacle_near_point, map_params, scaling_factor=1.5):
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
                # if prev_target_goal == [-4,-2] or prev_target_goal == [-4,2] or prev_target_goal == [-2,2] or prev_target_goal == [-2,-2]:
                #     target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
                # else:
                #     target_goali -= 1
                #     if target_goali < 0:
                #         target_goali = 3
                #     target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
                # return target_goal, target_goali
                pass
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
            # if prev_target_goal == [-4,-2] or prev_target_goal == [-4,2] or prev_target_goal == [-2,2] or prev_target_goal == [-2,-2]:
            #     target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
            # else:
            #     target_goali -= 1
            #     if target_goali < 0:
            #         target_goali = 3
            #     target_goal, target_goali = target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali)
            # return target_goal, target_goali
            pass

    if divisor <= 0.00001:
        divisor = 1 
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
        final.x = init.x - scaling_factor
        final.y = init.y
    elif slope == 0 and init.x > obstacles_array[0][1][0]:
        final.x = init.x + scaling_factor
        final.y = init.y
    elif slope > 0 and y_avg < 0:
        deltax = (scaling_factor / sqrt(1 + (slope*slope)))
        deltay = slope * deltax
        final.x = init.x - deltax
        final.y = init.y - deltay
    elif slope > 0 and y_avg > 0:
        deltax = (scaling_factor / sqrt(1 + (slope*slope)))
        deltay = slope * deltax
        final.x = init.x + deltax
        final.y = init.y + deltay
    elif slope < 0 and x_avg > 0:
        deltax = (scaling_factor / sqrt(1 + (slope*slope)))
        deltay = slope * deltax
        final.x = init.x + deltax
        final.y = init.y + deltay
    elif slope < 0 and x_avg < 0:
        deltax = (scaling_factor / sqrt(1 + (slope*slope)))
        deltay = slope * deltax
        final.x = init.x - deltax
        final.y = init.y - deltay

    if final.x < map_params.xmin: 
        final.x = map_params.xmin + 0.5
    if final.x > map_params.xmax: 
        final.x = map_params.xmax - 0.5
    if final.y < map_params.ymin: 
        final.y = map_params.ymin + 0.5
    if final.y > map_params.ymax: 
        final.y = map_params.ymax - 0.5
    target_goal = [final.x, final.y]
    # print("target current state: ", target_state[:2])
    # print("robot current state: ", state[:2])
    print("new target goal (Avoid trajectory): ", target_goal)
    return target_goal, target_goali

def target_goal_update(target_goal, target_state, params, t_current, t_prev_goal, target_goali):
    
    target_goal_dist = sqrt((target_goal[0] - target_state[0])**2+(target_goal[1] - target_state[1])**2)

    if target_goal == [-4,-2] or target_goal == [-4,2] or target_goal == [-2,2] or target_goal == [-2,-2]:
        if target_goal_dist < params.goal_tol:
            #TODO FIX THE TIME 
            print('Time from the previous reached goal (target):', t_current - t_prev_goal)
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
            print("[target] new goal(loop trajectory): ", target_goal)
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
        print("[target] new goal(loop trajectory): ", target_goal)

    return target_goal, target_goali


def pedestrian_goal_update(pedestrian_state, pedestrian_goal, params, t_current, t_prev_goal, pedestrian_goal_bool):
    '''
    We simply need to check if the human is at the goal and if so then update the goal position
    to remain in the line trajectory.
    '''
    if pedestrian_goal_bool == True:
        pedestrian_goal = [-2.0, -3.5]
        pedestrian_goal_bool = False
        print('Time from the previous reached goal (pedestrian):', t_current - t_prev_goal)
    else:
        pedestrian_goal = [2.0, -3.5]
        pedestrian_goal_bool = True
        print('Time from the previous reached goal (pedestrian):', t_current - t_prev_goal)

    print("[pedestrian] new goal: ", pedestrian_goal)
    
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