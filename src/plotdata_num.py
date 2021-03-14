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

ColorSet=['blue','red','green','yellow','cyan','brown','navy','gray','orange','purple']

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
            elif line_ctr in range(2,num_lines-1):
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

    mapboundaries=[xmin,xmax,ymin,ymax]
    wall = Obstacle(xmin, xmin, ymin, ymax,True)          
    walls.append(wall)
    wall = Obstacle(xmin, xmax, ymin, ymin,True)          
    walls.append(wall)
    wall = Obstacle(xmax, xmax, ymin, ymax,True)          
    walls.append(wall)
    wall = Obstacle(xmin, xmax, ymax, ymax,True)          
    walls.append(wall)


    #Create obstacle objects
    # print("polygons", polygons)
    # input("check-polygon")
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

    return start_states, init_poses, obstacles, walls,mapboundaries 

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
    parser.add_argument("-num",help="number of agent? [1-10] (default: 2)",default="2")
    args = vars(parser.parse_args())
    start_state, init_pos, obstacles, walls, mapboundaries = read_inputfile(args['in'])
    timeindex = args['index']
    num_agent =int(args['num'])

    v_max=0.5
    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path=dir_path[:-4]
    # timeindex = "06111301"
    cm =plt.get_cmap('gist_rainbow')

    file_name =dir_path+"/results/entropy/entropy_"+timeindex+"__"+args['num']+".csv"
    imgfile_name =dir_path+"/results/entropy/entropy_"+timeindex+"_"+args['num']+".png"
    # df = pd.read_csv(file_name, delimiter=',', names = ['index','time','entropy', 'pos_x', 'pos_y', 'goal_x', 'goal_y']).T
    df = pd.read_csv(file_name, delimiter=',')
    # df = pd.read_csv(file_name, delimiter=',', names = ['time','pos_x', 'pos_y', 'yaw', 'velocity', 'pos_xx', 'pos_yy','entropy', 'goal_x', 'goal_y', 'goal_xx', 'goal_yy'])

    # print("df", df)
    timess = np.asarray(df.head(1))
    # times =df.head(1)
    entropy= np.asarray(df.loc[1])
    # pos_xs= np.asarray(df.loc[2])
    pos_xs= df.loc[2]
    pos_ys= np.asarray(df.loc[3])
    goal_xs= np.asarray(df.loc[4])
    goal_ys= np.asarray(df.loc[5])
    # print("times", times[0])
    # print("lentimes", len(times[0]))

    # print("times", timess[0][1])
    times= timess[0][1]
    times_str= re.split(",",times)
    for i, time_ele in enumerate(times_str):
        if '[' in time_ele:
            # print("time_ele", time_ele)
            times_str[i]=time_ele.replace('[','')
            # print("time_ele", time_ele)
        if ']' in time_ele:
            times_str[i]=time_ele.replace(']','')
            # print("time_ele", times_str[i])
        if ' ' in time_ele:
            times_str[i]=time_ele.replace(' ','')

    # times_str2= re.split("[",times_str)
    print("times", times_str)
    len_data = len(times_str)
    times_str[len_data-1]=times_str[len_data-1].replace(']','')
    entropys= entropy[1]
    # entropys= entropy[0]
    # len_data = len(times)-1
    # len_data2 = len(entropys)-1
    # print("len_data", len_data)
    # print("len_data2", len_data2)
    entropys_str= re.split(",",entropys)
    print("len_data2", len(entropys_str))
    # len_data = len(entropys_str)
    for i, ent_ele in enumerate(entropys_str):
        if '[' in ent_ele:
            # print("ent_ele", ent_ele)
            entropys_str[i]=ent_ele.replace('[','')
            # print("ent_ele", ent_ele)
        if ']' in ent_ele:
            entropys_str[i]=ent_ele.replace(']','')
            # print("ent_ele", ent_ele)
        if ' ' in ent_ele:
            entropys_str[i]=ent_ele.replace(' ','')

    len_data = len(entropys_str)
    entropys_str[len_data-1]=entropys_str[len_data-1].replace(']','')


    #set num_agent
    pos_xs=pos_xs[1]
    pos_ys=pos_ys[1]
    # print("pos_xs", pos_xs)
    # print("pos_xs[0]", pos_xs[0])
    # print("pos_xs[1]", pos_xs[1])

    pos_xs=re.findall(r"[-+]?\d*\.\d+|\d+", pos_xs)
    pos_ys=re.findall(r"[-+]?\d*\.\d+|\d+", pos_ys)
    # print("pos_xs2", pos_xs)
    # input("hi")
    # num_agent =  int(len(pos_xs[5])/11) 
    # num_agent = 2
    # print("Num", num_agent)
    index_set=[1,12]

    # print("pos_xs", pos_xs[1])
    # space_idx = pos_xs[1].find(' ')
    # print("space_idx", space_idx)
    # input("here")
    # print("pos_xs[1]", pos_xs[1])
    # temp = float(pos_xs[1][1:6])
    # print("temp", temp)
    # temp2 = float(pos_xs[1][7:12])
    # print("temp2", temp2)

    times_t=np.zeros(len_data-1)
    entropy_t=np.zeros(len_data-1)
    for i in range(len(entropys_str)):
        if i>0:
            # print("times[", i,"] ", times_str[i])
            # print("entropys[", i,"] ", entropys_str[i])
            times_t[i-1]=float(times_str[i])
            entropy_t[i-1]=float(entropys_str[i])
    # print("times_t", times_t) 
    # print("entropy_t", entropy_t) 
    # input("check-entropy")

    len_data=len_data-50
    complete_time= times_t[len_data]
    print("times", times_t[len_data])

    agent_poses_x=np.zeros((num_agent,len_data))
    agent_poses_y=np.zeros((num_agent,len_data))
    goal_poses_x=np.zeros((num_agent,len_data))
    goal_poses_y=np.zeros((num_agent,len_data))



    for i, pose in enumerate(pos_xs):
        for j in range(num_agent):
            idx=num_agent*(i)+j
            # print("j: ", j," i: ", i)
            # print("idx", idx)
            # input("debug--")
            # print("pos_xs[ " ,idx, " ] ", pos_xs[idx])
            if i<len_data:
                agent_poses_x[j,i]=float(pos_xs[idx])
                agent_poses_y[j,i]=float(pos_ys[idx])
            # agent_poses_y[j,i-1]=temp4
    ''' 
    for i, pose in enumerate(pos_xs):
        if i>0:
            for j in range(num_agent):
                temp3 = float()
                temp4 = float()
                if j==0:
                    idx = 1
                    idxy= 1
                    print("idx", idx)
                    print("idxy", idxy)

                else:
                    idx = pos_xs[i][2:].find(' ')+3
                    idxy = pos_ys[i][2:].find(' ')+3
                    print("idx", idx)
                    print("idxy", idxy)
                    
                # print("idx", idx)
                # print("idxy", idxy)
                
                input("here-stop")

                print("pos_xs[i]", pos_xs[i])
                print("pos_ys[i]", pos_ys[i])
                input("here")
                if i==1:
                    temp3= float(pos_xs[i][idx:idx+3])
                    temp4 = float(pos_ys[i][idxy:idxy+3])
                else:
                    temp3= float(pos_xs[i][idx:idx+6])
                    temp4 = float(pos_ys[i][idxy:idxy+6])

                # print("temp", temp)
                    print("j", j)
                    print("temp4", temp4)
                agent_poses_x[j,i-1]=temp3
                agent_poses_y[j,i-1]=temp4
    '''
    # print("agent_poses_x", agent_poses_x)
    # print("agent_poses_y", agent_poses_y)
    # input("_")
    # print("goal_xs[1]", goal_xs[1])
    # temp = float(goal_xs[1][1:3])
    # print("temp", temp)
    # temp2 = float(goal_xs[1][5:8])
    # print("temp2", temp2)
    # for i, pose in enumerate(goal_xs):
        # if i>0:
            # for j in range(num_agent):
                # idx = 5*(j-1)+1
                # temp = float(goal_xs[i][idx:idx+3])
                # temp2 = float(goal_ys[i][idx:idx+3])
                # goal_poses_x[j,i-1]=temp
                # goal_poses_y[j,i-1]=temp2
    # print("goal_poses_x", goal_poses_x)
    # print("goal_poses_y", goal_poses_y)
    # input("wow")
                
    fig =plt.figure(figsize=(9,6))
    spec = gridspec.GridSpec(ncols=2, nrows=1, width_ratios=[2,1])
    ax0=fig.add_subplot(spec[0])
    # gs = gridspec.GridSpec(1,2, width_ratios=[2,1])
    # fig1 = plt.subplot(1,2,1)
    # ax0=plt.subplot(gs[0])
    # fig,axes=plt.subplots(nrows=1,ncols=2,figsize=(20,40))
    # ax = plt.axes()
    # ax0.scatter(agent_poses_x[0][0], agent_poses_y[0][0],s=200, marker="s", facecolor='blue',edgecolor='blue')      #initial point
    # ax0.scatter(agent_poses_x[1][0], agent_poses_y[1][0],s=200, marker="s", facecolor='red',edgecolor='red')      #initial point

    ax0.plot(agent_poses_x[0], agent_poses_y[0], 'o', markersize = 8, fillstyle='none',color='blue', alpha=0.5, label="robot trajectory")             #trajectory point
    ax0.plot(agent_poses_x[1], agent_poses_y[1], 'o', markersize = 8, fillstyle='none',color='red', alpha=0.5, label="robot trajectory")             #trajectory point

    if num_agent==3:
        ax0.scatter(agent_poses_x[2][0], agent_poses_y[2][0],s=200, marker="s", facecolor='green',edgecolor='green')      #initial point
        ax0.plot(agent_poses_x[2], agent_poses_y[2], 'o', markersize = 8, fillstyle='none',color='green', alpha=0.5, label="robot trajectory")             #trajectory point

    elif num_agent==4:
        ax0.scatter(agent_poses_x[2][0], agent_poses_y[2][0],s=200, marker="s", facecolor='green',edgecolor='green')      #initial point
        # ax0.scatter(agent_poses_x[3][0], agent_poses_y[3][0],s=200, marker="s", facecolor='yellow',edgecolor='yellow')      #initial point
        ax0.plot(agent_poses_x[2], agent_poses_y[2], 'o', markersize = 8, fillstyle='none',color='green', alpha=0.5, label="robot trajectory")             #trajectory point
        ax0.plot(agent_poses_x[3], agent_poses_y[3], 'o', markersize = 8, fillstyle='none',color='yellow', alpha=0.5, label="robot trajectory")             #trajectory point
    elif num_agent==5:
        # ax0.scatter(agent_poses_x[2][0], agent_poses_y[2][0],s=200, marker="s", facecolor='green',edgecolor='green')      #initial point
        # ax0.scatter(agent_poses_x[3][0], agent_poses_y[3][0],s=200, marker="s", facecolor='yellow',edgecolor='yellow')      #initial point
        # ax0.scatter(agent_poses_x[4][0], agent_poses_y[4][0],s=200, marker="s", facecolor='cyan',edgecolor='cyan')      #initial point
        ax0.plot(agent_poses_x[2], agent_poses_y[2], 'o', markersize = 8, fillstyle='none',color='green', alpha=0.5, label="robot trajectory")             #trajectory point
        ax0.plot(agent_poses_x[3], agent_poses_y[3], 'o', markersize = 8, fillstyle='none',color='yellow', alpha=0.5, label="robot trajectory")             #trajectory point
        ax0.plot(agent_poses_x[4], agent_poses_y[4], 'o', markersize = 8, fillstyle='none',color='cyan', alpha=0.5, label="robot trajectory")             #trajectory point

    elif num_agent==6:
        # ax0.scatter(agent_poses_x[2][0], agent_poses_y[2][0],s=200, marker="s", facecolor='green',edgecolor='green')      #initial point
        # ax0.scatter(agent_poses_x[3][0], agent_poses_y[3][0],s=200, marker="s", facecolor='yellow',edgecolor='yellow')      #initial point
        # ax0.scatter(agent_poses_x[4][0], agent_poses_y[4][0],s=200, marker="s", facecolor='cyan',edgecolor='cyan')      #initial point
        ax0.plot(agent_poses_x[2], agent_poses_y[2], 'o', markersize = 8, fillstyle='none',color='green', alpha=0.5, label="robot trajectory")             #trajectory point
        ax0.plot(agent_poses_x[3], agent_poses_y[3], 'o', markersize = 8, fillstyle='none',color='yellow', alpha=0.5, label="robot trajectory")             #trajectory point
        ax0.plot(agent_poses_x[4], agent_poses_y[4], 'o', markersize = 8, fillstyle='none',color='cyan', alpha=0.5, label="robot trajectory")             #trajectory point
        ax0.plot(agent_poses_x[5], agent_poses_y[5], 'o', markersize = 8, fillstyle='none',color='brown', alpha=0.5, label="robot trajectory")             #trajectory point

    elif num_agent==10:
        # ax0.scatter(agent_poses_x[2][0], agent_poses_y[2][0],s=200, marker="s", facecolor='green',edgecolor='green')      #initial point
        # ax0.scatter(agent_poses_x[3][0], agent_poses_y[3][0],s=200, marker="s", facecolor='yellow',edgecolor='yellow')      #initial point
        # ax0.scatter(agent_poses_x[4][0], agent_poses_y[4][0],s=200, marker="s", facecolor='cyan',edgecolor='cyan')      #initial point
        for i in range(2, num_agent):
            ax0.plot(agent_poses_x[i], agent_poses_y[i], 'o', markersize = 8, fillstyle='none',color=ColorSet[i], alpha=0.5, label="robot trajectory")             #trajectory point
        # ax0.plot(agent_poses_x[4], agent_poses_y[4], 'o', markersize = 8, fillstyle='none',color='cyan', alpha=0.5, label="robot trajectory")             #trajectory point
        # ax0.plot(agent_poses_x[5], agent_poses_y[5], 'o', markersize = 8, fillstyle='none',color='brown', alpha=0.5, label="robot trajectory")             #trajectory point




        # ax0.scatter(agent_poses_x[1][0], agent_poses_y[1][0],s=200, marker="s", facecolor='red',edgecolor='red')      #initial point
    # ax0.plot(goal_x, goal_y, '*', markersize = 18, fillstyle='none',color='green', label="goal")             #trajectory point
    # ax0.plot(goal_x2, goal_y2, '*', markersize = 18, fillstyle='none',color='yellow', label="goal")             #trajectory point
    plot_obstacles(obstacles,walls)

    area_size=mapboundaries[1]
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

    ax1.plot(times_t,entropy_t, linewidth=3.0, color='k')
    ax1.text(complete_time, 500,str(complete_time), color='b', fontsize = 12)
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



