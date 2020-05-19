# !/usr/bin/env python3
import argparse
from numpy import genfromtxt
import numpy as np
from math import *
import random
import math
import argparse
import csv
import matplotlib.pyplot as plt
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config, atan_zero_to_twopi
from state_lattice_planner import uniform_terminal_state_sampling_test1
from utils.configuration_space import configuration_space
from cubic_spline_planner import Spline2D 
from  VCD import VerticalCellDecomposition
import pandas as pd
import os
import re
import time
from matplotlib.patches import Polygon, Rectangle, Circle
import matplotlib as mpl
#import numpy as np
f_max=0.3
v_max=0.4
#probability constant for log odd(occupancy grid )
l_occ=np.log(0.8/0.2)
l_free=np.log(0.2/0.8)

#pp control 
k = 0.1  # look forward gain
Kv = 0.15  # speed propotional gain
dt = 0.2  # [s]
L = 1.0  # [m] wheel base of vehicle
AlphabetSet=['a','b','c','d','e','f','g','h','i','j','k','l','m', 
                'n','o','p','q','r']

class map_params:
    def __init__(self):
        self.xyreso = 0.25              # x-y grid resolution [m]
        self.yawreso = math.radians(6)  # yaw angle resolution [rad]
        self.xmin=-12.5                 # search region (xmin)
        self.xmax=12.5                  # search region (xmax)
        self.ymin=-12.5                 # search region (ymin)
        self.ymax=12.5                  # search region (xmax)
        self.xw = int(round((self.xmax - self.xmin) / self.xyreso))
        self.yw = int(round((self.ymax - self.ymin) / self.xyreso))
        self.boundaries=[]
        self.boundaries.append((self.xmin,self.ymin))
        self.boundaries.append((self.xmax,self.ymin))
        self.boundaries.append((self.xmax,self.ymax))
        self.boundaries.append((self.xmin,self.ymax))
        self.sensor_range=5

class Params:
    def __init__(self):
        self.numiters = 4000
        self.dt = 0.2
        self.goal_tol = 0.25
        self.max_vel = 0.25 # m/s
        self.min_vel = 0.0 # m/s
        self.sensor_range_m = 0.5 # m
        self.animate = 1
        self.area_size=13

# class State:
    # def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        # self.x = x
        # self.y = y
        # self.yaw = yaw
        # self.v = v

def random_sampling(params, nums):
    #nums = the number of sample we need

    reso= (2*params.area_size)/(nums+1)
    x = np.arange(-0.75*params.area_size, 0.75*params.area_size, reso)
    # y = np.arange(params.area_size, 0.75*params.area_size, reso)
    sample_xy=[]
    for i in range(len(x)-1):
        temp_x= random.uniform(x[i],x[i+1])
        temp_y= random.uniform(-0.8*params.area_size,0.8*params.area_size)
        sample_xy.append([temp_x,temp_y])
        
    # print(sample_xy)
    return sample_xy


def generating_globaltrjs(cur_state, cspace, obstacles,goals, params_global):

    trjs=[]
    for goal_pos in goals:
        init_pos = [cur_state[0], cur_state[1]]
        cspace.reset_cspace(params_global.boundaries,init_pos,goal_pos, obstacles)
        planner.reset_cspace(cspace)
        path, path_idx = planner.search(False, goal_pos)
        if path!=None:
            xs=[]
            ys=[]
            for i in range(len(path)):
                xs.append(path[i][0])
                ys.append(path[i][1])
            # for x_c, y_c in path:
                # xs.append(x_c)
                # ys.append(y_c)
            sp=Spline2D(xs,ys)
            trjs.append(sp)

    print("num_trjs", len(trjs))

        #plot spline
        # ds = 0.2  # [m] distance of each intepolated points
        # s = np.arange(0, sp.s[-1], ds)
        # rx, ry, ryaw, rk = [], [], [], []
        # for i_s in s:
            # ix, iy = sp.calc_position(i_s)
            # rx.append(ix)
            # ry.append(iy)
        # plt.plot(rx, ry, "-r", label="spline")

    return trjs


def draw_occmap(data, params_map,agent_x, agent_y, ax):

    minx=params_map.xmin
    miny=params_map.ymin
    maxx=params_map.xmax
    maxy=params_map.ymax
    xyreso=params_map.xyreso
    # print("min x, min y:", minx, ", ", miny)
    # print("max x, max y:", maxx, ", ", maxy)

    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    # print("(x)", x)
    # print("(y)", y)
    ax.pcolor(x+agent_x, y+agent_y, data, vmax=1.0, cmap=plt.cm.Blues)
    # ax.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    # ax.set_xlim([-params_map.sensor_range+agent_x, agent_x+params_map.sensor_range])   # limit the plot space
    # ax.set_ylim([agent_y-params_map.sensor_range, agent_y+params_map.sensor_range])   # limit the plot space
    # ax.set_ylim([-params_map.sensor_range, params_map.sensor_range])   # limit the plot space
    # ax.set_xlim([agent_x-params_map.sensor_range, agent_x+params_map.sensor_range])   # limit the plot space
    # ax.set_ylim([agent_y-params_map.sensor_range, agent_y+params_map.sensor_range])   # limit the plot space

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
            if px > params_global.xmax or px < params_global.xmin:
                continue
            if py > params_global.ymax or py < params_global.ymin:
                continue

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
    r = params.sensor_range_m
    # ax = plt.gca()
    axes[0,0].plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
                [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='b')
    axes[0,0].plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
                [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='b')

    # axes[0.plot(pose[0], pose[1], 'ro', markersize=5)
    # circle= Circle((pose[0], pose[1]),r,linewidth=1,edgecolor='k',facecolor='k',alpha=0.3 )
    # ax.add_patch(circle)
    # axes[0.plot(pose[0], pose[1], 'ro', markersize=40, alpha=0.1)
    # print("plot_circle")
    # ax.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
    # head_length=0.1, head_width=0.1)
    axes[0,0].arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
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
    axes[0,0].plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(),'b')

    #DRAW SENSOR FOV
    # axes[0].plot(np.array(sensor_outline[0, :]).flatten(),
             # np.array(sensor_outline[1, :]).flatten(),'y')
    # axes[0].fill(np.array(sensor_outline[0, :]).flatten(),
             # np.array(sensor_outline[1, :]).flatten(),'y', alpha=0.25)


def plot_map(pos_x,pos_y,way_x, way_y, waytimes):
    axes[0,0].scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial point
    axes[0,0].scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
    # axes[0,0].plot(pos_x, pos_y, 'o', markersize = 20, fillstyle='none',color='black')             #trajectory point
    axes[0,0].plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='red')             #trajectory point
    axes[0,0].set_xlabel("x[m]")
    axes[0,0].set_ylabel("y[m]")
    axes[0,0].grid(True)
    # for i in range(len(waytimes)):
        # axes[0].text(way_x[i], way_y[i]-1,str(waytimes[i]), color='r')


#obstacles
def plot_obstacles(obstacles):
    for obs in obstacles:
        obs.draw(axes[0,0])


def visualize(traj, pose, obstacles,params):
    # ax = plt.gca()
    # plt.plot(traj[:,0], traj[:,1], 'g')
    plot_robot(pose, params)
    plot_obstacles(obstacles)

    axes[0,0].set_xlim([-params.area_size, params.area_size])   # limit the plot space
    axes[0,0].set_ylim([-params.area_size, params.area_size])   # limit the plot space
    axes[0,0].plot(traj[:,0], traj[:,1], 'k')
    # plt.legend()


#dyanmics
def simple_motion(state, goal, params):
    # state = [x(m), y(m), yaw(rad) ,velocity(m/s)]
    # input = [a(m/s**2), steering(rad) ]
    a =Update_a(state,goal)
    delta = Update_phi(state,goal)

    # print("delta:", delta)
    # print("pre-state[2]:", state[2])
    # print("goal:", goal)

    state[0] +=  state[3] * math.cos(state[2]) * dt
    state[1] +=  state[3] * math.sin(state[2]) * dt
    state[2] +=   delta * dt
    # state[2] +=  math.sin(delta) * dt
    state[3] +=  a * dt

    # print("post-state[2]:", state[2])

    if state[3] >= params.max_vel: state[3] = params.max_vel
    if state[3] <= params.min_vel: state[3] = params.min_vel

    if state[2] >= 2*math.pi: state[2] -= 2*math.pi
    if state[2] <= -2*math.pi: state[2] += 2*math.pi
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
    # des_phi = math.atan2(goal[1] - state[1], goal[0] - state[0])
    des_phi = atan_zero_to_twopi(goal[1] - state[1], goal[0] - state[0])
    cur_yaw = state[2]
    # print("des_phi:, ", des_phi, "cur_yaw: ", state[2] )
    err_phi = 0.5*(des_phi-cur_yaw)
    # err_phi = des_phi
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-in",help="input file (default: input.txt)",default="input2.txt")
    parser.add_argument("-load",help="load saved data? [y/n] (default: n)",default="n")
    args = vars(parser.parse_args())

    #Define four windows: 
    # axes[0,0] : robot, obstacle, waypoints, trajectory
    # axes[0,1] : local sensor map
    # axes[1,0] : Sensor Candidates, VCD regions
    # axes[1,1] : global sensor map
    fig,axes=plt.subplots(nrows=2,ncols=2,figsize=(30,30))

    params = Params()
    params_globalmap =  map_params()
    params_localmap =  map_params()

    if args['load']=='y':
        timeindex = "04171450"              #log time index for input 
        # Open the desired file for reading
        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path=dir_path[:-4]
        file_name =dir_path + "/results/data/robot_" +timeindex+"_.csv"
        wayfile_name =dir_path + "/results/data/waypoints_" +timeindex+"_.csv"
        obsfile_name =dir_path + "/results/data/obstacles2_"+timeindex+"_.csv"

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
            nums = [float(k) for k in floatregex.findall(obstacle_coords[i])] #find integer value in string format '[ int, int ]'
            # print(nums)
            obs = Obstacle(nums[0]-1, nums[1]-1, nums[2], nums[3])          #xmin,ymin, 
            obs.draw()
            obstacles.append(obs)                                   # attach obstacle to obstacle list
        # print("num ofobstacles:", len(obstacles))
        print("---load completed")

    #create cspace
    init_pos=[pos_x[0],pos_y[0]]
    # init_pos=[3, 4]
    goal_pos=[-5, 9]
    # cspace = configuration_space(args['in'])
    cspace=configuration_space()
    cspace.reset_cspace(params_globalmap.boundaries,init_pos,goal_pos, obstacles)
    # cspace.plot_config_space()
    planner = VerticalCellDecomposition(cspace)
    planner.construct_graph()
    # path, path_idx = planner.search(False)

    '''
    new_goal_pos=[8,7]
    cspace.reset_cspace(params_globalmap.boundaries,init_pos,new_goal_pos, obstacles)
    planner.reset_cspace(cspace)
    path, path_idx = planner.search(False, new_goal_pos)
    print("path", path)
    xs=[]
    ys=[]
    for x_c, y_c in path:
        xs.append(x_c)
        ys.append(y_c)
    sp=Spline2D(xs,ys)
    ds = 0.1  # [m] distance of each intepolated points
    s = np.arange(0, sp.s[-1], ds)
    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    axes[4].plot(rx, ry, "-r", label="spline")
    '''
    waypoint_vcd = planner.generate_waypoint(params_localmap)
    planner.plot_regions(axes[1,0])

    #waypoint from vcd
    way_x=[]
    way_y=[]

    for point in waypoint_vcd:
        # print("x: ", point[0])
        # print("y: ", point[1])
        way_x.append(point[0])
        way_y.append(point[1])
        # print("waypoints : (x,y ) = (", way_x,", ", way_y,")")


    #plot figures 
    # fig,axes=plt.figure(figsize=(10,20))
    axes[0,0].scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial point
    # axes[0].scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
    axes[0,0].plot(pos_x, pos_y, 'o', markersize = 20, fillstyle='none',color='black')             #trajectory point
    axes[1,0].plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='green')             #trajectory point
    # for i in range(len(waytimes)):
        # axes[0].text(way_x[i], way_y[i]-1,str(waytimes[i]), color='g')

    area_size=13
    locs, labels = plt.xticks()
    # locs, labels = plt.yticks()
    #FixMe!
    # axes[0].xticks(np.arange(-area_size,area_size,1.0))
    # axes[0].yticks(np.arange(-area_size,area_size,1.0))
    # ax = plt.axes()

    axes[0,0].set_xlabel('x')
    axes[0,0].set_ylabel('y')
    axes[0,0].set_xlim([-area_size, area_size])   # limit the plot space
    axes[0,0].set_ylim([-area_size, area_size])   # limit the plot space
    axes[0,0].grid(True)
    # axes[0,0.tight_layout()

    #simulation settings
    ntimestep = len(pos_x)
    goal_tol=0.2

    goali = 0                           #define goal from waypoints set
    goal = [way_x[goali], way_y[goali]]
        
    # plt.show()

    # initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    # state = np.array([pos_x[0],pos_y[0], 0.0, np.pi/2, 0.0, 0.0])
    # state = np.array([pos_x[0],pos_y[0],np.pi/2, 0.0])
    state = np.array([init_pos[0],init_pos[1],0.0, 0.0])
    print("initial state: ",state)
    traj = state[:2]
    iter=0
    simtime=0.0

    #Checking initial and final goal
    print("initial state: ",state)
    print("goal : ",goal)
    goal=[7,6]

    t_prev_goal = time.time()
    pmap_global = initialize_global_occ_grid_map(params_globalmap)
    initial_entropy = get_map_entropy(pmap_global,params_globalmap)
    print("initial entropy: ", initial_entropy )

    # for i in range(ntimestep):
    for _ in range(params.numiters):
        state = simple_motion(state, goal, params)                        #dynamics
        goal_dist = sqrt((goal[0] - state[0])**2+(goal[1] - state[1])**2) #distance to gaol
        simtime = simtime + dt
        # print("simtime" , simtime)
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
            axes[0,0].cla()
            # plt.plot(goal[0], goal[1])
            plot_map(pos_x,pos_y,way_x,way_y,waytimes)
            axes[0,0].plot(goal[0], goal[1])
            traj = np.vstack([traj, state[:2]])
            visualize(traj, state, obstacles, params)

            #figure2- local sensor window
            axes[0,1].cla()
            axes[0,1].set_title('local sesnor grid')
            pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, params_localmap.xmin, params_localmap.xmax, params_localmap.ymin, params_localmap.ymax, params_localmap.xyreso, params_localmap.xw, params_localmap.yw= generate_ray_casting_grid_map(obstacles, params_localmap, state[0],state[1], state[2])
            draw_occmap(pmap_local, params_localmap, state[0],state[1], axes[0,1])
            #draw sensor ray to obstacles
            # for i in range(len(obstacles)):
                # axes[0].plot([state[0], obstacles[i].vertices[obs_verticeid[i][0]][0]], [state[1], obstacles[i].vertices[obs_verticeid[i][0]][1]], color='orange')
                # axes[0].plot([state[0], obstacles[i].vertices[obs_verticeid[i][1]][0]], [state[1], obstacles[i].vertices[obs_verticeid[i][1]][1]], color='orange')
                # axes[0].plot([state[0], obstacles[i].vertices[closest_vertexid[i]][0]], [state[1], obstacles[i].vertices[closest_vertexid[i]][1]], color='orange')
            #test intersection
            # for angle,inter_point in intersect_dic.items():
                # axes[0].plot(inter_point[0], inter_point[1], '*', markersize= 5, fillstyle='none',color='green')

            # axes[0,1.plot(ox, oy, "xr")


            #figure3- global occupancy grid
            axes[1,1].cla()
            axes[1,1].set_title('global occupancy grid')
            pmap_global = update_occ_grid_map(state, pmap_local,params_localmap, pmap_global,params_globalmap)
            draw_occmap_global(pmap_global,params_globalmap, axes[1,1])
            entropy = get_map_entropy(pmap_global, params_globalmap)
            # print("----entropy : ", entropy)

            plt.pause(0.001)
            # plt.show()

        iter=iter+1

        #middle frequency
        if iter%20==1:
            
            if iter%40==1:
                axes[1,0].cla()
                axes[1,0].set_title('global & Local motion primitives')
                # uniform_terminal_state_sampling_test1(state,axes[4])
                sample_goals = random_sampling(params,9)
                trjs= generating_globaltrjs(state, cspace,obstacles,sample_goals,params_globalmap)
                num_colors = len(trjs)
                cm =plt.get_cmap('gist_rainbow')
                for i, sp in enumerate(trjs):
                    col = cm(1.*i/num_colors)
                    ds = 0.1  # [m] distance of each intepolated points
                    s = np.arange(0, sp.s[-1], ds)
                    rx, ry, ryaw, rk = [], [], [], []
                    for i_s in s:
                        ix, iy = sp.calc_position(i_s)
                        rx.append(ix)
                        ry.append(iy)
                    axes[1,0].plot(rx, ry, color='g', label="spline")
                planner.plot_regions(axes[1,0])

            # uniform_terminal_state_sampling_test1(state,axes[1,0])
            # axes[1,0].set_xlim([-1.2*params.area_size, 1.2*params.area_size])   # limit the plot space
            # axes[1,0].set_ylim([-1.2*params.area_size, 1.2*params.area_size])   # limit the plot space
            # planner.plot_regions(axes[1,0])


    plt.show()






