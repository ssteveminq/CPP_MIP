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
    # timesets= ["06051150", "06051331", "06050452", "06051202", "06051344","06051358"]
    timesets= ["06111301", "06111231", "06111239", "06111247" ]
    num_colors = len(timesets)
    cm =plt.get_cmap('gist_rainbow')

    for i, timeindex in enumerate(timesets):
    
        file_name =dir_path+"/results/entropy/entropy_"+timeindex+"_.csv"
        imgfile_name =dir_path+"/results/entropy/entropy_"+timeindex+".png"
        df = pd.read_csv(file_name, delimiter=',', names = ['time','pos_x', 'pos_y', 'yaw', 'velocity', 'entropy', 'goal_x', 'goal_y'])

        times = np.asarray(df['time'][1:])
        pos_x = np.asarray(df['pos_x'][1:])                  #robot pos_x
        pos_y = np.asarray(df['pos_y'][1:])                  #robot pos_y
        yaw   = np.asarray(df['yaw'][1:])                    #robot pos_y
        vel   = np.asarray(df['velocity'][1:])               #robot pos_y
        entropy= np.asarray(df['entropy'][1:])               #robot pos_y
        goal_x= np.asarray(df['goal_x'][1:])                  #robot pos_x
        goal_y= np.asarray(df['goal_y'][1:])                  #robot pos_y

        #convert from string to float 
        times= times.astype(np.float)
        pos_x = pos_x.astype(np.float)
        pos_y = pos_y.astype(np.float)
        entropy = entropy.astype(np.float)
        goal_x = goal_x.astype(np.float)
        goal_y = goal_y.astype(np.float)
        # input()

       # plt.figure()
        col = cm(1.*i/num_colors)
        plt.plot(times,entropy, linewidth=2.0, color=col, label=timeindex)
        plt.text(times[-1], entropy[-1]+500, str(times[-1]), color=col, fontsize = 18)
        plt.xlabel('time')
        plt.ylabel('entropy')
        # plt.legend()
        plt.grid(True)
    plt.show()

