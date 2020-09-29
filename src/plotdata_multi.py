#!/usr/bin/env python3

from numpy import genfromtxt
import numpy as np
from math import *
import argparse
from obstacle import Obstacle
import csv
import matplotlib.pyplot as plt
from matplotlib import gridspec
import pandas as pd
import os
import re
import ast

#import numpy as np
def read_inputfile(FILE_NAME="input4.txt"):

    line_ctr = 0
    polygons=[]
    with open(FILE_NAME) as f:
        num_lines = sum(1 for l in f)
    with open(FILE_NAME) as f:
        for l in f:
            line_ctr += 1
            if line_ctr == 1:
                boundary = list(ast.literal_eval(l))
            elif line_ctr in range(2,num_lines):
                polygons.append(list(ast.literal_eval(l)))
            else:
                temp = list(ast.literal_eval(l))
                # start_state = [temp[0],temp[1], temp[2], temp[3]]
                # init_pos = [temp[0],temp[1]]
                start_states = [temp[0],temp[1]]
                # start_states = [temp[0],temp[1], temp[2], temp[3]]
                init_poses = [[temp[0][0],temp[0][1]],[temp[1][0],temp[1][1]]]
 
                # goal_state = temp[1]

    #Create wall objects
    walls=[]
    xmin=100
    ymin=100
    xmax=-100
    ymax=-100
    for point in boundary:
        if xmin>point[0]:
            xmin = point[0]
        if xmax<point[0]:
            xmax = point[0]
        if ymin>point[1]:
            ymin = point[1]
        if ymax<point[1]:
            ymax = point[1]

    print("xmin: " , xmin , ", xmax: ", xmax, ", ymin", ymin, ", ymax: ", ymax)
    wall = Obstacle(xmin, xmin, ymin, ymax,True)          
    walls.append(wall)
    wall = Obstacle(xmin, xmax, ymin, ymin,True)          
    walls.append(wall)
    wall = Obstacle(xmax, xmax, ymin, ymax,True)          
    walls.append(wall)
    wall = Obstacle(xmin, xmax, ymax, ymax,True)          
    walls.append(wall)


    #Create obstacle objects
    obstacles=[]
    for obs in polygons:
        xmin=100
        ymin=100
        xmax=-100
        ymax=-100
        for point in obs:
            if xmin>point[0]:
                xmin = point[0]
            if xmax<point[0]:
                xmax = point[0]
            if ymin>point[1]:
                ymin = point[1]
            if ymax<point[1]:
                ymax = point[1]

        tmp = Obstacle(xmin, xmax,ymin,ymax)
        obstacles.append(tmp)                       # attach obstacle to obstacle list
    # print(obstacles)

    return start_states, init_poses, obstacles, walls

def plot_obstacles(obstacles, walls):
    for obs in obstacles:
        obs.draw()
    for wall in walls:
        wall.draw()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-in",help="input file (default: input2.txt)",default="input4.txt")
    parser.add_argument("-load",help="load saved data? [y/n] (default: n)",default="n")
    parser.add_argument("-index",help="time index? [06111301] (default: 06111301)",default="07150017")
    args = vars(parser.parse_args())
    start_state, init_pos, obstacles, walls = read_inputfile(args['in'])
    timeindex = args['index']

    v_max=0.5
    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path=dir_path[:-4]
    # timeindex = "06111301"
    cm =plt.get_cmap('gist_rainbow')

    file_name =dir_path+"/results/entropy/entropy_"+timeindex+"_.csv"
    imgfile_name =dir_path+"/results/entropy/entropy_"+timeindex+".png"
    df = pd.read_csv(file_name, delimiter=',', names = ['time','pos_x', 'pos_y', 'yaw', 'velocity', 'pos_xx', 'pos_yy','entropy', 'goal_x', 'goal_y', 'goal_xx', 'goal_yy'])

    times = np.asarray(df['time'][1:])
    pos_x = np.asarray(df['pos_x'][1:])                  #robot pos_x
    pos_y = np.asarray(df['pos_y'][1:])                  #robot pos_y
    pos_x2 = np.asarray(df['pos_xx'][1:])                  #robot pos_x
    pos_y2 = np.asarray(df['pos_yy'][1:])                  #robot pos_y
    yaw   = np.asarray(df['yaw'][1:])                    #robot pos_y
    vel   = np.asarray(df['velocity'][1:])               #robot pos_y
    entropy= np.asarray(df['entropy'][1:])               #robot pos_y

    goal_x= np.asarray(df['goal_x'][1:])                  #robot pos_x
    goal_y= np.asarray(df['goal_y'][1:])                  #robot pos_y
    goal_x2= np.asarray(df['goal_xx'][1:])                  #robot pos_x
    goal_y2= np.asarray(df['goal_yy'][1:])                  #robot pos_y

    times= times.astype(np.float)
    # print(times)
    # input()
    pos_x = pos_x.astype(np.float)
    pos_y = pos_y.astype(np.float)
    pos_x2 = pos_x2.astype(np.float)
    pos_y2 = pos_y2.astype(np.float)
    entropy = entropy.astype(np.float)
    goal_x = goal_x.astype(np.float)
    goal_y = goal_y.astype(np.float)
    goal_x2 = goal_x2.astype(np.float)
    goal_y2 = goal_y2.astype(np.float)


    fig =plt.figure(figsize=(9,6))
    spec = gridspec.GridSpec(ncols=2, nrows=1, width_ratios=[2,1])
    ax0=fig.add_subplot(spec[0])
    # gs = gridspec.GridSpec(1,2, width_ratios=[2,1])
    # fig1 = plt.subplot(1,2,1)
    # ax0=plt.subplot(gs[0])
    # fig,axes=plt.subplots(nrows=1,ncols=2,figsize=(20,40))
    # ax = plt.axes()
    ax0.scatter(pos_x[0], pos_y[0],s=200, marker="s", facecolor='blue',edgecolor='blue')      #initial point
    ax0.scatter(pos_x2[0], pos_y2[0],s=200, marker="s", facecolor='red',edgecolor='red')      #initial point
    ax0.plot(pos_x, pos_y, 'o', markersize = 8, fillstyle='none',color='blue', alpha=0.5, label="robot trajectory")             #trajectory point
    ax0.plot(pos_x2, pos_y2, 'o', markersize = 8, fillstyle='none',color='red', alpha=0.5, label="robot trajectory")             #trajectory point
    ax0.plot(goal_x, goal_y, '*', markersize = 18, fillstyle='none',color='green', label="goal")             #trajectory point
    ax0.plot(goal_x2, goal_y2, '*', markersize = 18, fillstyle='none',color='yellow', label="goal")             #trajectory point
    plot_obstacles(obstacles,walls)

    area_size=13
    locs, labels = plt.xticks()
    locs, labels = plt.yticks()
    plt.xticks(np.arange(-area_size,area_size,1.0))
    plt.yticks(np.arange(-area_size,area_size,1.0))
    # ax = plt.axes()

    plt.xlabel('x')
    plt.ylabel('y')
    plt.xlim([-area_size, area_size])   # limit the plot space
    plt.ylim([-area_size, area_size])   # limit the plot space
    plt.grid(True)

    # fig2 = plt.subplot(1, 2, 2)
    # ax1=plt.subplot(gs[1])
    ax1 = fig.add_subplot(spec[1])

    ax1.plot(times,entropy, linewidth=3.0, color='k')
    ax1.text(times[-1], 500,str(times[-1]), color='b', fontsize = 12)
    plt.xlabel('time')
    plt.ylabel('entropy')
    plt.grid(True)


    plt.savefig(imgfile_name)                   # save the resulting plot





    plt.show()


#waypoint
# way_x=[]
# way_y=[]
# regex = re.compile('[-+]?\d*\.\d+|[-+]?\d+')                #set pattern in order to find integer in string
# way_coords = np.asarray(waydf['coords'][1:])
# for i in range(len(way_coords)):
        # nums = [float(k) for k in regex.findall(way_coords[i])] #find integer value in string format '[ int, int ]'
        # way_x.append(nums[0])
        # way_y.append(nums[1])


#obstacles
# floatregex =re.compile('[-+]?\d*\.\d+|[-+]?\d+') 
# obstacles = []                                  # list which will contain all obstacles
# obstacle_coords = np.asarray(obsdf['obstacle'][0:])
# for i in range(len(obstacle_coords)):
        # nums = [float(k) for k in floatregex.findall(obstacle_coords[i])] #find integer value in string format '[ int, int ]'
        # tmp = Obstacle(nums[0], nums[1], nums[2], nums[3])          #xmin,ymin, 
        # tmp.draw()
        # obstacles.append(tmp)                                   # attach obstacle to obstacle list


'''
#plot figures 
    ax = plt.axes()
    plt.scatter(pos_x[0], pos_y[0],s=200, marker="s", facecolor='green',edgecolor='green')      #initial point
    plt.plot(pos_x, pos_y, 'o', markersize = 8, fillstyle='none',color='blue', alpha=0.5, label="robot trajectory")             #trajectory point
    plt.plot(goal_x, goal_y, '*', markersize = 18, fillstyle='none',color='red', label="goal")             #trajectory point
    # for i in range(len(goal_x)):
        # plt.scatter(goal_x[i], goal_y[i], s=40, facecolor='red', edgecolor='red')             #trajectory point
    plot_obstacles(obstacles,walls)

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
    plt.savefig(imgfile_name)                   # save the resulting plot
# plt.tight_layout()
'''

#figure 2: Plot velocity and force
    # plt.figure()
    # plt.plot(times,entropy, linewidth=2.0, color='red')
    # plt.xlabel('time')
    # plt.ylabel('entropy')
    # plt.grid(True)



