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
import yaml
from colour import Color

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
            elif line_ctr in range(2,num_lines-2):
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
    parser.add_argument("-cut",help="cut last data [1-300] (default: 100)",default="50")
    args = vars(parser.parse_args())
    start_state, init_pos, obstacles, walls, mapboundaries = read_inputfile(args['in'])
    timeindex = args['index']
    cut_length=int(args['cut'])

    with open("config/test.yaml", 'r') as stream:
        try:
            yamldata=yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    # for i in range(yamldata['num_agents']):
        # temp_params= robot_param(yamldata['speeds'][i], yamldata['sensor_range'][i])
        # robot_params.append(temp_params)
        # print("speed", temp_params.max_speed )
    num_agent =yamldata['num_agents']   #set num_agent from argparser


    # num_agent =int(args['num'])

    v_max=0.5
    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path=dir_path[:-4]
    cm =plt.get_cmap('gist_rainbow')

    file_name =dir_path+"/results/entropy/entropy_"+timeindex+"__"+str(num_agent)+".csv"
    imgfile_name =dir_path+"/results/entropy/entropy_"+timeindex+"_"+str(num_agent)+".png"
    # df = pd.read_csv(file_name, delimiter=',', names = ['index','time','entropy', 'pos_x', 'pos_y', 'goal_x', 'goal_y']).T
    df = pd.read_csv(file_name, delimiter=',')
    # df = pd.read_csv(file_name, delimiter=',', names = ['time','pos_x', 'pos_y', 'yaw', 'velocity', 'pos_xx', 'pos_yy','entropy', 'goal_x', 'goal_y', 'goal_xx', 'goal_yy'])

    # print("df", df)
    timess = np.asarray(df.head(1))
    entropy= np.asarray(df.loc[1])
    print("entropy", entropy)
    # pos_xs= np.asarray(df.loc[2])
    pos_xs= np.asarray(df.loc[2])
    pos_ys= np.asarray(df.loc[3])
    goal_xs= np.asarray(df.loc[4])
    goal_ys= np.asarray(df.loc[5])
    print("pos_xs", pos_xs)
    # print("times", times[0])
    # print("lentimes", len(times[0]))

    # times_str= times
    times_str= timess[0]
    entropys_str=entropy
    # print("times", times)
    # if len(timess[0][1])>1:
        # times= timess[0][1]
    # else:
        # times=timess[0]

    # print("times", times)
    '''
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
    '''

    # times_str2= re.split("[",times_str)
    # print("times", times_str)
    len_data = len(times_str)
    # times_str[len_data-1]=times_str[len_data-1].replace(']','')
    # entropys= entropy[0]
    # print("entropys", entropys)
    # entropys_str=entropys
    # entropys_str= re.split(",",entropys)
    print("len_data2", len(entropys_str))
    # len_data = len(entropys_str)
    '''
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

    '''

    len_data = len(entropys_str)
    print("len_data", len_data)
    # entropys_str[len_data-1]=entropys_str[len_data-1].replace(']','')


    #set num_agent
    # pos_xs=pos_xs[1]
    # pos_ys=pos_ys[1]
    # goal_xs=goal_xs[1]
    # goal_ys=goal_ys[1]
    # print("pos_xs", pos_xs)
    # input("--")
    # print("pos_xs[0]", pos_xs[0])

    # print("len_pos_xs", len(pos_xs))
    # input("---")
    agent_poses_x=np.zeros((num_agent,len_data-1))
    agent_poses_y=np.zeros((num_agent,len_data-1))
    goal_poses_x=np.zeros((num_agent,len_data-1))
    goal_poses_y=np.zeros((num_agent,len_data-1))


    
    for k in range(1, len(pos_xs)-1):
        pos_xs_=re.findall(r"[-+]?\d*\.\d+|\d+", str(pos_xs[k]))
        pos_ys_=re.findall(r"[-+]?\d*\.\d+|\d+", str(pos_ys[k]))
        # goal_xs=re.findall(r"[-+]?\d*\.\d+|\d+", str(goal_xs[k]))
        # goal_ys=re.findall(r"[-+]?\d*\.\d+|\d+", str(goal_ys[k]))
        for pos_x in pos_xs_:
            if pos_x=='01':
                pos_xs_.remove(pos_x)
            elif pos_x=='00':
                pos_xs_.remove(pos_x)
            elif pos_x=='02':
                pos_xs_.remove(pos_x)
        for pos_y in pos_ys_:
            if pos_y=='01':
                pos_ys_.remove(pos_y)
            elif pos_y=='00':
                pos_ys_.remove(pos_y)
            elif pos_y=='02':
                pos_ys_.remove(pos_y)

        for j in range(num_agent):
            # print("pos_xs[ " ,idx, " ] ", pos_xs[idx])
                agent_poses_x[j,k]=float(pos_xs_[j])
                agent_poses_y[j,k]=float(pos_ys_[j])
                # goal_poses_x[k,i]=float(goal_xs[idx])
                # goal_poses_y[k,i]=float(goal_ys[idx])
                # print("i", i)

  



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

    len_data=len_data
    times_t=np.zeros(len_data)
    entropy_t=np.zeros(len_data)
    for i in range(len(entropys_str)):
        if i>0:
            # print("times[", i,"] ", times_str[i])
            # print("entropys[", i,"] ", entropys_str[i])
            times_t[i-1]=float(times_str[i])
            entropy_t[i-1]=float(entropys_str[i])
            # if entropy_t[i-1]<4900:
                # len_data=i-1
                # break;

    # print("times_t", times_t) 
    # print("entropy_t", entropy_t) 
    # input("check-entropy")

    # len_data=len_data-cut_length
    # len_data=len_data
    # print("len_data", len_data)
    complete_time= times_t[len_data-1]
    # print("times", times_t[len_data-1])

        # print("goal_poses_x", goal_xs)
    # print("------------------------")
    # print("goal_poses_x", goal_poses_x)

# cmap = clr.LinearSegmentedColormap.from_list("", ["darkblue", "blue", "violet", "yellow", "orange", "red"])

    # print("goal_x", goal_poses_x)
    # print("goal_y", goal_poses_y)
    # print("len(goal_x)", goal_poses_y.shape[1])
    len_goal=goal_poses_y.shape[1]

    ####################Plot####################

    fig =plt.figure(figsize=(9,6))
    spec = gridspec.GridSpec(ncols=2, nrows=1, width_ratios=[2,1])
    ax0=fig.add_subplot(spec[0])
    # for k in range(len_goal):
        # if k%30==0:
            # ax0.scatter(goal_poses_x[0][k], goal_poses_y[0][k],s=200, marker="v", facecolor=(0, 0, (k)/(2.5*360)),edgecolor=(0, 0, (k)/(2.5*360)))      #initial point
            # ax0.scatter(goal_poses_x[1][k], goal_poses_y[1][k],s=200, marker="v", facecolor=((k)/(2.5*360),0,0),edgecolor=((k)/(2.5*360),0,0))      #initial point
    ax0.scatter(agent_poses_x[0][0], agent_poses_y[0][0],s=200, marker="v", facecolor='blue',edgecolor='blue')      #initial point
    ax0.scatter(agent_poses_x[1][0], agent_poses_y[1][0],s=200, marker="v", facecolor='red',edgecolor='red')      #initial point


    def uniqueish_color(n):
        return plt.cm.gist_ncar(np.linspace(0,1,len_data-1))


    ax0.plot(agent_poses_x[0], agent_poses_y[0], 'o', markersize = 8, fillstyle='none',color='blue', alpha=0.5, label="robot trajectory")             #trajectory point
    ax0.plot(agent_poses_x[1], agent_poses_y[1], 'o', markersize = 8, fillstyle='none',color='red', alpha=0.5, label="robot trajectory")             #trajectory point
    # ax0.set_prop_cycle('color', cm(np.linspace(0,1,len_data-1)))

    if num_agent==3:
        # ax0.scatter(agent_poses_x[2][0], agent_poses_y[2][0],s=200, marker="v", facecolor='green',edgecolor='green')      #initial point
        for k in range(len_goal):
            if k%30==0:
                ax0.scatter(goal_poses_x[2][k], goal_poses_y[2][k],s=200, marker="v", facecolor=ColorSet[2],edgecolor=ColorSet[2])      #initial point
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
    elif num_agent==8:
        # ax0.scatter(agent_poses_x[2][0], agent_poses_y[2][0],s=200, marker="s", facecolor='green',edgecolor='green')      #initial point
        # ax0.scatter(agent_poses_x[3][0], agent_poses_y[3][0],s=200, marker="s", facecolor='yellow',edgecolor='yellow')      #initial point
        # ax0.scatter(agent_poses_x[4][0], agent_poses_y[4][0],s=200, marker="s", facecolor='cyan',edgecolor='cyan')      #initial point
        for i in range(2, num_agent):
            ax0.plot(agent_poses_x[i], agent_poses_y[i], 'o', markersize = 8, fillstyle='none',color=ColorSet[i], alpha=0.5, label="robot trajectory")             #trajectory point
        # ax0.plot(agent_poses_x[4], agent_poses_y[4], 'o', markersize = 8, fillstyle='none',color='cyan', alpha=0.5, label="robot trajectory")             #trajectory point
        # ax0.plot(agent_poses_x[5], agent_poses_y[5], 'o', markersize = 8, fillstyle='none',color='brown', alpha=0.5, label="robot trajectory")             #trajectory point




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
    plt.xticks(np.arange(-area_size,area_size,5.0))
    plt.yticks(np.arange(-area_size,area_size,5.0))
    # ax = plt.axes()

    plt.xlabel('x')
    plt.ylabel('y')
    plt.xlim([-area_size, area_size])   # limit the plot space
    plt.ylim([-area_size, area_size])   # limit the plot space
    plt.grid(True)

    # fig2 = plt.subplot(1, 2, 2)
    # ax1=plt.subplot(gs[1])
    ax1 = fig.add_subplot(spec[1])

    ax1.plot(times_t[1:len_data],entropy_t[1:len_data], linewidth=3.0, color='k')
    ax1.text(complete_time, 500,str(complete_time), color='b', fontsize = 12)
    plt.xlabel('time')
    plt.ylabel('entropy')
    plt.grid(True)


    plt.savefig(imgfile_name)                   # save the resulting plot




    # plt.show()
    agent_vel_x =  np.zeros((num_agent,len_data-1))
    agent_vel_y =  np.zeros((num_agent,len_data-1))
    agent_vels =  np.zeros((num_agent,len_data-1))
    agent_dist=  np.zeros((num_agent,1))
    dt=0.2

    for i in range(len_data-1):
        if i>0:
            for j in range(num_agent):
                if i<len_data:
                    agent_dist[j]=agent_dist[j]+sqrt((agent_poses_x[j,i]-agent_poses_x[j,i-1])**2+
                                               (agent_poses_y[j,i]-agent_poses_y[j,i-1])**2)
                    agent_vel_x[j,i-1]=(agent_poses_x[j,i]-agent_poses_x[j,i-1])/dt
                    agent_vel_y[j,i-1]=(agent_poses_y[j,i]-agent_poses_y[j,i-1])/dt
                    agent_vels[j,i-1]=sqrt(agent_vel_x[j,i-1]**2+agent_vel_y[j,i-1]**2)
                    if abs(agent_vels[j,i-1]-agent_vels[j,i-2])>2:
                        agent_vels[j,i-1]=agent_vels[j,i-2]
                        agent_vel_x[j,i-1]= agent_vel_x[j,i-2]
                        agent_vel_y[j,i-1]= agent_vel_y[j,i-2]
                        # print("i", i)
                        # print("vel_x: ", agent_vel_x[j,i-1])
                        # print("vel_y: ", agent_vel_y[j,i-1])
                        # print("pos_x-2: ", agent_poses_x[j,i-3])
                        # print("pos_x-1: ", agent_poses_x[j,i-2])
                        # print("pos_x: ", agent_poses_x[j,i-1])
                        # print("pos_x2: ", agent_poses_x[j,i])
                        # print("pos_x3: ", agent_poses_x[j,i+1])
                        # print("pos_x4: ", agent_poses_x[j,i+2])
                        # print("pos_x4: ", agent_poses_x[j,i+2])
                        # print("pos_y-1: ", agent_poses_y[j,i-2])
                        # print("pos_y: ", agent_poses_y[j,i-1])
                        # print("pos_y2: ", agent_poses_y[j,i])
                        # print("pos_y3: ", agent_poses_y[j,i+1])
                        # input()

    print("dist",agent_dist )


#figure 2: Plot velocity and force
    # plt.figure()
    # for j in range(num_agent):
        # plt.plot(times_t[1:len_data-1],agent_vel_x[j][1:len_data-1], linewidth=2.0, color=ColorSet[j])
        # plt.plot(times_t[1:len_data-1],agent_vel_y[j][1:len_data-1], linewidth=2.0, color=ColorSet[j])
    # plt.xlabel('time')
    # plt.ylabel('velocities')
    # plt.grid(True)
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



