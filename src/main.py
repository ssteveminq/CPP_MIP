from gurobipy import *
from vehicle import Vehicle
import argparse
from numpy import genfromtxt
import numpy as np
from math import *
import math
import argparse
import csv
import matplotlib.pyplot as plt
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config
from utils.configuration_space import configuration_space
from  VCD import VerticalCellDecomposition
import pandas as pd
import os
import re
import time
from matplotlib.patches import Polygon, Rectangle, Circle
import matplotlib as mpl
#import numpy as np
# f_max=0.4
# v_max=0.4
#probability
l_occ=np.log(0.8/0.2)
l_free=np.log(0.2/0.8)

#pp control 
k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
Kv = 0.15  # speed propotional gain
ktheta = 0.5
dt = 0.2  # [s]
L = 1.0  # [m] wheel base of vehicle

class map_params:
    def __init__(self):
        self.xyreso = 0.25  # x-y grid resolution [m]
        self.yawreso = math.radians(6)  # yaw angle resolution [rad]
        self.xmin=-12.5
        self.xmax=12.5
        self.ymin=-12.5
        self.ymax=12.5
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
        self.numiters = 2000
        self.dt = 0.2
        self.goal_tol = 0.25
        self.max_vel = 0.25 # m/s
        self.min_vel = 0.0 # m/s
        self.sensor_range_m = 0.5 # m
        self.animate = 1
        self.area_size=10
        # self.time_to_switch_goal = 5.0 # sec #inactive for now
        # self.sweep_resolution = 0.4 # m

def draw_occmap(data, params_map,agent_x, agent_y, ax):

    minx=params_map.xmin
    miny=params_map.ymin
    maxx=params_map.xmax
    maxy=params_map.ymax
    xyreso=params_map.xyreso

    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    ax.pcolor(x+agent_x, y+agent_y, data, vmax=1.0, cmap=plt.cm.Blues)
    ax.set_xlim([agent_x-params.sesnor_range, agent_x+params.sesnor_range])   # limit the plot space
    ax.set_ylim([agent_y-params.sesnor_range, agent_y+params.sesnor_range])   # limit the plot space

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
    r = params.sensor_range_m
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
    # for i in range(len(waytimes)):
        # axes[0].text(way_x[i], way_y[i]-1,str(waytimes[i]), color='r')


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
    # print("a:", a, ", delta: ", delta)
    # print("pre-state[2]:", state[2])

    state[0] +=  state[3] * math.cos(state[2]) * dt
    state[1] +=  state[3] * math.sin(state[2]) * dt
    # state[2] +=  state[3] / L * math.tan(delta) * dt
    state[2] +=  0.75*math.sin(delta) * dt
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

# def load_files(timeindex)




if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-in",help="input file (default: input.txt)",default="input2.txt")
    parser.add_argument("-load",help="load saved data? [y/n] (default: n)",default="n")
    parser.add_argument("-opt",help="optimization? [y/n] (default: y)",default="y")
    args = vars(parser.parse_args())
    #Define two windows: 
    # axes[0] : robot, obstacle, waypoints, trajectory
    # axes[1] : sensor_map,occ_grid
    fig,axes=plt.subplots(nrows=3,ncols=1,figsize=(10,30))

    params = Params()
    params_globalmap =  map_params()
    params_localmap =  map_params()

    cspace = configuration_space(args['in'])
    init_pos=cspace.start_state 
    goal_pos=cspace.goal_state 
    planner = VerticalCellDecomposition(cspace)
    planner.construct_graph()
    waypoint_vcd = planner.generate_waypoint(params_localmap)

    if args['load']=='y':

        timeindex = "04171450"
        # timeindex = "04021858"
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
            # obs.draw()
            obstacles.append(obs)                                   # attach obstacle to obstacle list
        # print("num ofobstacles:", len(obstacles))
        print("---load completed")

    if args['opt']=='y':
        area_size = 12      # window size
        wp = True           # switch for use of waypoints. True: waypoints can be used. False: function deactivated
        vx_init = [0.0]   # initial x-component velocity
        vy_init = [0.0]    # initial y-component velocity
        T = 152             # maximum time of travel
        dt = 4.             # time step size
        d_obs = 1.0         # minimum distance required from obstacle
        M = 75              # number of constraints in order to approximate the force and velocity magnitudes
        
        obs_coords = cspace.get_obs()
        steps = int(T / dt)                             # number of steps
        obstacles=[]
        for ob in obs_coords:                           # for every obstacle
            tmp = Obstacle(ob[0], ob[1], ob[2], ob[3])  # local obstacle variable
            # tmp.draw()                                  # draw local obstacle
            obstacles.append(tmp)                       # attach obstacle to obstacle list


        wp_vcds = planner.generate_waypoint(params_localmap)
        print("wp_vcds", wp_vcds)
        # obs_coords = [[-0.5, 0.5, 0, 5]]     # array containing all obstacles in [x_min,x_max,y_min,y_max] format
        veh_coords = [[4, 8, -3.5, 6]]    # array containing all vehicles in [x_0,y_0,x_fin,y_fin] format
        # veh_coords = [[5, 5]]    # array containing all vehicles in [x_0,y_0] format
        # wp_coords = [[[0, -2],[3,3], [-3, 4]]]  # array containing all waypoint in [x_wp,y_wp] format
        name = 'waypoints_obs.png'        # name of the figure to be saved
        folder = 'results/waypoints_obs/'        # folder name

        constrain_multiple_vehicles = False   # True: add contraints related to multiple vehicle, False: do not add
        constrain_waypoints = True            # True: add contraints related to waypoints, False: do not add
        constrain_obstacles = True           # True: add contraints related to avoiding obstacles, False: do not add

        num_vehicles = len(veh_coords)
        x0 = []; y0 = []; th0 = []; v0=[];                                 # initial positions for all vehicles/
        x_fin = []; y_fin = []                           # final positions for all vehicles
        for i in range(num_vehicles):
            x0.append(veh_coords[i][0])
            y0.append(veh_coords[i][1])
            # th0.append(0.0)
            # v0.append(0.0)
            x_fin.append(veh_coords[i][2])
            y_fin.append(veh_coords[i][3])
        # print("x0", x0)
        # print("y0", y0)
        # print("th0", th0)
        # print("v0", v0)

    # Create location of all waypoints for all vehicles
    n_way_points = len(wp_vcds)
    x_wp = []; y_wp = []                             # position of all waypoints of all vehicles
    if wp:                                           # if wp is True, waypoints are used
        for i in range(num_vehicles):
            x_dummy = []; y_dummy = []               # position of all waypoints of one vehicle
            for j in range(n_way_points):
                x_dummy.append(wp_vcds[j][0])
                y_dummy.append(wp_vcds[j][1])

            x_wp.append(x_dummy)
            y_wp.append(y_dummy)

    print("x_wp", x_wp[0])
    print("y_wp", y_wp[0])
    print("area_size", area_size)

    ###### optimization #######
    # Initialize model
    m = Model("ppl")

    ###### Inputs to the generation of the vehicles ######
    vehicle_mass = 5        # mass of the vehicles
    # v_max = 0.225         # maximum velocity of the vehicle
    v_max = 0.75             # maximum velocity of the vehicle
    vx_init = [0]           # initial x-component velocity
    vy_init = [0]           # initial y-component velocity
    v_init=0.0
    f_max = [1.0]           # maximum force experienced by a vehicle
    obs_coords = [[5, 7, -2, 1], [-1.0, 1.0, 0.0, 2.0]]     # array containing all obstacles in [x_min,x_max,y_min,y_max] format
    performance_graphs = True  # include the velocity and acceleration performance of the vehicles
    obj_acceleration = False   # when True the acceleration is taken into consideration in the objective function

    if not obj_acceleration:  # if the acceleration is not included in the objective function, 'acc' is added to the file name
        extra = 'acc_'
    else:
        extra = ''

    # Create vehicles and add model main variables
    vehicles = []
    for i in range(num_vehicles):
        # print(f_max[min(i, len(f_max)-1)])
        if wp:
            vehicles.append(
                Vehicle(vehicle_mass, dt, T, x0[i], y0[i], i, m, M, v_max, f_max[min(i, len(f_max)-1)], area_size, x_fin[i], y_fin[i],
                        wp, x_wp[i], y_wp[i]))
        else:
            vehicles.append(
                Vehicle(vehicle_mass, dt, T, x0[i], y0[i],i, m, M, v_max, f_max[min(i, len(f_max)-1)], area_size, x_fin[i], y_fin[i],
                        wp))

    # Add constraints and add model secondary variables
    for i in range(num_vehicles):        
        vehicles[i].constrain_dynamics(vx_init[i], vy_init[i])
        vehicles[i].constrain_positions()

        if(constrain_obstacles):
            vehicles[i].constrain_obstacles(obstacles, d_obs)
        
        # if(constrain_multiple_vehicles):
            # vehicles[i].constrain_multiple_vehicles(vehicles, 0.6)
        
        if(constrain_waypoints):
            vehicles[i].constrain_waypoints()


    # Obtaining the objective function
    total = 0                                # total number of time steps between all the vehicles (minimize)
    epsilon = 0.1                          # effect of the force on the objective function
    for veh in range(len(vehicles)):
            if obj_acceleration:
                for i in range(steps):
                    total += vehicles[veh].fm[i]*epsilon  # Objective function with acceleration
                total += vehicles[veh].Tf
            else:
                # for i in range(steps):
                    # total += vehicles[veh].b[i] * i    # Objective function without acceleration
                total = vehicles[veh].Tf  # Objective function without acceleration


    m.setObjective(total, GRB.MINIMIZE)
    # Optimizing the model and obtaining the values of he parameters and the objective function
    m.optimize()
    m.getVars()
    # input()


    # Update filename with time
    datafolder = 'results/data/'       # folder name
    filename_data= 'robot_'
    filename_waypoint = 'waypoints_'
    filename_obstacle= 'obstacles_'
    t = time.localtime()
    timestamp =time.strftime('%m%d%H%M_', t)
    name=name+timestamp +".png"
    filename_data= datafolder+filename_data+timestamp+'.csv'
    filename_waypoint =datafolder+filename_waypoint+timestamp+'.csv'
    filename_obstacle =datafolder+filename_obstacle+timestamp+'.csv'

    # Plotting the results
    for i in range(num_vehicles):
        z = 0
        plt.scatter(vehicles[i].x[18].x, vehicles[i].y[18].x, facecolor = 'black', edgecolor = 'black')
        # Plot dashed lines connecting initial and final points for all vehicles
        plt.plot([veh_coords[i][0], veh_coords[i][2]], [veh_coords[i][1], veh_coords[i][3]], 'k--', alpha = 0.5)
        '''
        if args.obs == 1:
            # Plot a bold point at the 18th point as done in the paper
            plt.scatter(vehicles[i].x[18].x, vehicles[i].y[18].x, facecolor = 'black', edgecolor = 'black')
            # Plot dashed lines connecting initial and final points for all vehicles
            plt.plot([veh_coords[i][0], veh_coords[i][2]], [veh_coords[i][1], veh_coords[i][3]], 'k--', alpha = 0.5)
        elif args.obs == 2:
            # Plot dashed lines connecting initial and final points for all vehicles
            plt.plot([veh_coords[i][0], veh_coords[i][2]], [veh_coords[i][1], veh_coords[i][3]], 'k--', alpha=0.5)
        elif args.obs == 3 or args.obs == 4:
            # Plot arrow as shown in the paper
            plt.arrow(7.5, 5, -2, 0, length_includes_head=True, head_width=0.3)

        '''
        # for k in range(steps):                 # obtaining time step at which vehicle reaches the final point
            # Z = str(vehicles[i].b[k])
            # if Z[-5] == "1":
                # z = k
                # break
        # obtaining time step at which vehicle reaches which waypoint
        wp_times={}
        for k in range(len(vehicles[i].kset)):
            for j in range(steps):
                Z = str(vehicles[i].kset[k][j])
                if Z[-5] == "1":
                    wp_times[j]=k
                    # wp_times[j]=wp_coords[k]
        wp_times= dict(sorted(wp_times.items()))
        z=list(wp_times.keys())[-1]           #printing the final time
        print("final_time:", z)

     
        z=z+1
        coords = np.zeros([z,2])
        for j in range(z):                    # obtaining the coordinates to plot
            coords[j, :] = [vehicles[i].x[j].x,vehicles[i].y[j].x]
        if wp:                                # plotting the location of the waypoints
            for jj in range(len(x_wp[i])):
                plt.plot(x_wp[i][jj], y_wp[i][jj], '*', color='b')
                plt.text(x_wp[i][jj], y_wp[i][jj]-1,str(jj), color='b')
        
        # if args.obs == 0:
        labels = ['Turn rate 15 $\degree$/s', 'Turn rate 12 $\degree$/s']
        shape = ['o', '^']
        plt.plot(coords[:,0], coords[:,1], shape[i], fillstyle='none',color='black',label=labels[i])
        plt.legend()
        # else:
            # plt.scatter(coords[:,0], coords[:,1], facecolor = 'none', edgecolor = 'black')  # plot the trajectories of the vehicles


        #comment below 2 lines because of elimination fix final points
        # plt.plot(vehicles[i].x_fin, vehicles[i].y_fin, '*', color='k')    # plot the final points star
        # plt.scatter(vehicles[i].x_fin, vehicles[i].y_fin, facecolor = 'none', edgecolor = 'black')  # plot the final points circle

    plt.xlim([-area_size, area_size])   # limit the plot space
    plt.ylim([-area_size, area_size])   # limit the plot space
    # if args.obs == 0:
        # plt.xlim([-10, area_size])   # limit the plot space
        # plt.ylim([-10, area_size])   # limit the plot space
 

    # plt.savefig(folder + extra + name)                   # save the resulting plot
    if performance_graphs:  # Plot the velocity and acceleration of the vehicles of the different experiments
        line_styles = ["--", ":", "-.", '-', '-']
        marker_styles = ['None', 'None', 'None', 'x', 'None']

        # Plot the velocity
        fig = plt.subplot(2,1,1)
        plt.xlabel('Time steps [-]')
        plt.ylabel("Velocity [m/s]")
        plt.title("Velocity per time step")
        v_coords_x = []
        v_coords_y = []
        v_coords = []

        # gathering velocity and force data 
        v_data=np.zeros([z,2])
        f_data=np.zeros([z,2])
        for j in range(z):                    # obtaining the coordinates to plot
            v_data[j, :] = [vehicles[i].vx[j].x,vehicles[i].vy[j].x]
            f_data[j, :] = [vehicles[i].fx[j].x,vehicles[i].fy[j].x]
    
        # for multiple vehicles : v_coords_x, v_coords_y are temporary variables
        for j in range(len(vehicles)):                  # extract the velocity of each vehicle
            for i in range(len(vehicles[0].vx)):
                v_coords_x.append(vehicles[j].vx[i].x)  # velocity in the x-direction
                v_coords_y.append(vehicles[j].vy[i].x)  # velocity in the y-direction
                v_coords.append(sqrt(vehicles[j].vy[i].x ** 2 + vehicles[j].vx[i].x ** 2))  # velocity magnitude
            n_steps = len(v_coords)
            fig.plot(range(len(v_coords)), v_coords, color = 'black', label= "Vehicle " + str(j+1), linestyle = line_styles[j], marker = marker_styles[j])
            #need to comment in for multiple vehicles#############
            v_coords_x = []
            v_coords_y = []
            v_coords = []
            #################################################################

        # Plot the maximum velocity
        fig.plot(range(n_steps), [vehicles[0].v_max]*n_steps, color = 'black', label="Maximum velocity = " + str(vehicles[0].v_max) + ' [m/s]', linestyle = line_styles[4], marker = marker_styles[4])
        plt.legend()
        plt.grid(True)

        # Plot the force
        fig2 = plt.subplot(2, 1, 2)
        plt.xlabel('Time steps [-]')
        plt.ylabel("Force [N]")
        plt.title("Acceleration per time step")
        f_coords_x = []
        f_coords_y = []
        f_coords = []
        for j in range(len(vehicles)):                 # extract the forces applied to each vehicle
            for i in range(len(vehicles[0].vx)):
                f_coords_x.append(vehicles[j].fx[i].x)  # force applied in the x-direction
                f_coords_y.append(vehicles[j].fy[i].x)  # force applied  in the y-direction
                f_coords.append(sqrt(vehicles[j].fy[i].x ** 2 + vehicles[j].fx[i].x ** 2))   # force magnitude

            plt.plot(range(n_steps), f_coords, color='black', label= "Vehicle " + str(j+1), linestyle = line_styles[j], marker = marker_styles[j])
            #need to comment in for multiple vehicles#############
            f_coords_x = []
            f_coords_y = []
            f_coords = []
            #################################################################

        # Plot the maximum force
        fig2.plot(range(n_steps), [vehicles[0].f_max] * n_steps, color='black', label="Maximum force = " + str(vehicles[0].f_max) + ' [N]', linestyle = line_styles[4], marker = marker_styles[4])
        plt.legend()
        plt.grid(True)
        plt.tight_layout()                                   # Make sure the titles and labels are visible
        plt.savefig(folder + extra + "Performance_" + name)  # save the resulting plot

        #save data to csv file
        data= np.append(coords,v_data,1)
        data= np.append(data,f_data,1)
        pd.DataFrame(data, columns=["x" , "y", "vx" , "vy", "fx" , "fy"]).to_csv(filename_data,header=True)

        wpdata=[]
        iters=0
        for key,value in wp_times.items():
            # wpdata.append([key, wp_coords[0][iters]])
            wpdata.append([key, wp_vcds[int(value)]])

        (pd.DataFrame(data=wpdata, columns=["time", "(x,y)"]).to_csv(filename_waypoint, header=True))

        obs_column=[]
        for obs in obstacles:
            obs_str = '['+str(obs.x_min)+','+str(obs.x_max)+','+str(obs.y_min)+','+str(obs.y_max)+']'
            obs_column.append(obs_str)     # array containing all obstacles in [x_min,x_max,y_min,y_max] format

        (pd.DataFrame(data=obs_column, columns=["obstacle"]).to_csv(filename_obstacle, header=True))








    plt.show()



    #create cspace
    # init_pos=[pos_x[0],pos_y[0]]
    # init_pos=[5, 4]
    # goal_pos=[-5, -8]
    # cspace = configuration_space(args['in'])
    # cspace=configuration_space()
    # cspace.reset_cspace(params_globalmap.boundaries,init_pos,goal_pos, obstacles )
    # cspace.plot_config_space()
    # planner = VerticalCellDecomposition(cspace)
    # planner.construct_graph()
    # path, path_idx = planner.search(True)
    # waypoint_vcd = planner.generate_waypoint(params_localmap)

    #waypoint from vcd
    # way_x=[]
    # way_y=[]

    # for point in waypoint_vcd:
        # print("x: ", point[0])
        # print("y: ", point[1])
        # way_x.append(point[0])
        # way_y.append(point[1])
        # print("waypoints : (x,y ) = (", way_x,", ", way_y,")")


    '''
    #plot figures 
    # fig,axes=plt.figure(figsize=(10,20))
    axes[0].scatter(pos_x[0], pos_y[0], facecolor='blue',edgecolor='blue')      #initial point
    axes[0].scatter(pos_x[-1], pos_y[-1], facecolor='red',edgecolor='red')      #final point
    axes[0].plot(pos_x, pos_y, 'o', markersize = 20, fillstyle='none',color='black')             #trajectory point
    axes[2].plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='green')             #trajectory point
    # for i in range(len(waytimes)):
        # axes[0].text(way_x[i], way_y[i]-1,str(waytimes[i]), color='g')

    area_size=13
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
        
    plt.show()

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
            axes[0].cla()
            # plt.plot(goal[0], goal[1])
            plot_map(pos_x,pos_y,way_x,way_y,waytimes)
            axes[0].plot(goal[0], goal[1])
            traj = np.vstack([traj, state[:2]])
            visualize(traj, state, obstacles, params)

            #figure2- local sensor window
            axes[1].cla()
            pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, params_localmap.xmin, params_localmap.xmax, params_localmap.ymin, params_localmap.ymax, params_localmap.xyreso, params_localmap.xw, params_localmap.yw= generate_ray_casting_grid_map(obstacles, params_localmap, state[0],state[1], state[2])
            draw_occmap(pmap_local, params_localmap, state[0],state[1], axes[1])
            #draw sensor ray to obstacles
            # for i in range(len(obstacles)):
                # axes[0].plot([state[0], obstacles[i].vertices[obs_verticeid[i][0]][0]], [state[1], obstacles[i].vertices[obs_verticeid[i][0]][1]], color='orange')
                # axes[0].plot([state[0], obstacles[i].vertices[obs_verticeid[i][1]][0]], [state[1], obstacles[i].vertices[obs_verticeid[i][1]][1]], color='orange')
                # axes[0].plot([state[0], obstacles[i].vertices[closest_vertexid[i]][0]], [state[1], obstacles[i].vertices[closest_vertexid[i]][1]], color='orange')
            #test intersection
            # for angle,inter_point in intersect_dic.items():
                # axes[0].plot(inter_point[0], inter_point[1], '*', markersize= 5, fillstyle='none',color='green')

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

    plt.show()
    '''






