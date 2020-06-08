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
import ast


#import numpy as np
def read_inputfile(FILE_NAME="input2.txt"):

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
                start_state = [temp[0],temp[1], temp[2], temp[3]]
                init_pos = [temp[0],temp[1]]
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

    return start_state, init_pos, obstacles, walls

def plot_obstacles(obstacles, walls):
    for obs in obstacles:
        obs.draw()
    for wall in walls:
        wall.draw()




if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-in",help="input file (default: input2.txt)",default="input2.txt")
    parser.add_argument("-load",help="load saved data? [y/n] (default: n)",default="n")
    args = vars(parser.parse_args())
    start_state, init_pos, obstacles, walls = read_inputfile(args['in'])

    v_max=0.5
    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path=dir_path[:-4]
    timeindex = "06051358"
    timesets= ["06051150", "06051331", "06050452", "06051202", "06051344","06051358"]
    num_colors = len(timesets)
    cm =plt.get_cmap('gist_rainbow')

    for i, timeindex in enumerate(timesets):
    
        file_name =dir_path+"/results/entropy/entropy_"+timeindex+"_.csv"
        imgfile_name =dir_path+"/results/entropy/entropy_"+timeindex+".png"

        df = pd.read_csv(file_name, delimiter=',', names = ['time','pos_x', 'pos_y', 'yaw', 'velocity', 'entropy', 'goal_x', 'goal_y'])

        # print(df)

# idx_array = np.arange(len(list(df['index'])))      #time index
        times = np.asarray(df['time'][1:])
        # input()
        pos_x = np.asarray(df['pos_x'][1:])                  #robot pos_x
        # input()
        pos_y = np.asarray(df['pos_y'][1:])                  #robot pos_y
        # input()
        yaw   = np.asarray(df['yaw'][1:])                    #robot pos_y
        # input()
        vel   = np.asarray(df['velocity'][1:])               #robot pos_y
        # input()
        entropy= np.asarray(df['entropy'][1:])               #robot pos_y

        goal_x= np.asarray(df['goal_x'][1:])                  #robot pos_x
        # input()
        goal_y= np.asarray(df['goal_y'][1:])                  #robot pos_y

#convert from string to float 
        times= times.astype(np.float)
        # print(times)
        # input()
        pos_x = pos_x.astype(np.float)
        pos_y = pos_y.astype(np.float)
        entropy = entropy.astype(np.float)
        goal_x = goal_x.astype(np.float)
        goal_y = goal_y.astype(np.float)
        # print(entropy, 'goal_x', 'goal_y')
        # input()

       # plt.figure()
        col = cm(1.*i/num_colors)
        plt.plot(times,entropy, linewidth=2.0, color=col)
        plt.xlabel('time')
        plt.ylabel('entropy')
        plt.grid(True)
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



