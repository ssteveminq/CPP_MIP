from gurobipy import *
from vehicle import Vehicle
from obstacle import Obstacle
import matplotlib.pyplot as plt
import numpy as np
from math import *
import time
import csv
import argparse
import pandas as pd

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Usage: main.py obs")
    parser.add_argument("obs", metavar='obs', type=int)

    args = parser.parse_args()
    if args.obs == None or args.obs==0:
        area_size = 10  # window size
        wp = True       # switch for use of waypoints. True: waypoints can be used. False: function deactivated
        vx_init = [0]   # initial x-component velocity
        vy_init = [0]   # initial y-component velocity
        # f_max = [0.2]    # maximum force experienced by a vehicle
        f_max = [0.3]    # maximum force experienced by a vehicle
        T = 100         # maximum time of travel
        dt = 5          # time step size
        d_obs = 0.1     # minimum distance required from obstacle
        M = 75          # number of constraints in order to approximate the force and velocity magnitudes
        
        obs_coords = []                  # array containing all obstacles in [x_min,x_max,y_min,y_max] format
        veh_coords = [[5, 5, 0, -2]]     # array containing all vehicles in [x_0,y_0,x_fin,y_fin] format
        wp_coords = [[[-0.7, 6], [3,3],[1,5],[-5, 4]]]  # array containing all waypoint in [x_wp,y_wp] format
        # wp_coords = [[[1, 3], [3,3],[-2,3], [1, 1], [3,1],[-2,1], [3,-1],[-2,-1],[1, -1], [1,-3],[3,-3], [-2, -3]]]  # array containing all waypoint in [x_wp,y_wp] format
        name = 'waypoints'              # name of the figure to be saved
        filename_data= 'robot_'
        filename_waypoint = 'waypoints_'
        folder = 'results/waypoints/'       # folder name
        datafolder = 'results/data/'       # folder name

        constrain_multiple_vehicles = False   # True: add contraints related to multiple vehicle, False: do not add
        constrain_waypoints = True            # True: add contraints related to waypoints, False: do not add
        constrain_obstacles = False           # True: add contraints related to avoiding obstacles, False: do not add

    if args.obs == 1:
        area_size = 10      # window size
        wp = True           # switch for use of waypoints. True: waypoints can be used. False: function deactivated
        vx_init = [-0.19]   # initial x-component velocity
        vy_init = [-0.1]    # initial y-component velocity
        f_max = [0.15]      # maximum force experienced by a vehicle
        T = 200             # maximum time of travel
        dt = 4.             # time step size
        d_obs = 0.1         # minimum distance required from obstacle
        M = 75              # number of constraints in order to approximate the force and velocity magnitudes
        
        obs_coords = [[0, 1.7, 0, 9]]     # array containing all obstacles in [x_min,x_max,y_min,y_max] format
        # veh_coords = [[5, 5, -0.7, 6]]    # array containing all vehicles in [x_0,y_0,x_fin,y_fin] format
        veh_coords = [[5, 5]]    # array containing all vehicles in [x_0,y_0] format
        wp_coords = [[[0, -2], [-5, 4]]]  # array containing all waypoint in [x_wp,y_wp] format
        name = 'waypoints_obs.png'        # name of the figure to be saved
        folder = 'results/waypoints_obs/'        # folder name

        constrain_multiple_vehicles = False   # True: add contraints related to multiple vehicle, False: do not add
        constrain_waypoints = True            # True: add contraints related to waypoints, False: do not add
        constrain_obstacles = True            # True: add contraints related to avoiding obstacles, False: do not add

    steps = int(T / dt)                             # number of steps
    obstacles = []                                  # list which will contain all obstacles
    for ob in obs_coords:                           # for every obstacle
        tmp = Obstacle(ob[0], ob[1], ob[2], ob[3])  # local obstacle variable
        tmp.draw()                                  # draw local obstacle
        obstacles.append(tmp)                       # attach obstacle to obstacle list


    # Create initial and final positions
    num_vehicles = len(veh_coords)
    x0 = []; y0 = []                                 # initial positions for all vehicles
    x_fin = []; y_fin = []                           # final positions for all vehicles
    for i in range(num_vehicles):
        x0.append(veh_coords[i][0])
        y0.append(veh_coords[i][1])
        x_fin.append(veh_coords[i][2])
        y_fin.append(veh_coords[i][3])

    # Create location of all waypoints for all vehicles
    n_way_points = len(wp_coords[0])
    x_wp = []; y_wp = []                             # position of all waypoints of all vehicles
    if wp:                                           # if wp is True, waypoints are used
        for i in range(num_vehicles):
            x_dummy = []; y_dummy = []               # position of all waypoints of one vehicle
            for j in range(n_way_points):
                x_dummy.append(wp_coords[i][j][0])
                y_dummy.append(wp_coords[i][j][1])

            x_wp.append(x_dummy)
            y_wp.append(y_dummy)

    # Initialize model
    m = Model("ppl")

    ###### Inputs to the generation of the vehicles ######
    vehicle_mass = 5           # mass of the vehicles
    # v_max = 0.225              # maximum velocity of the vehicle
    v_max = 0.4              # maximum velocity of the vehicle
    performance_graphs = True  # include the velocity and acceleration performance of the vehicles
    obj_acceleration = True    # when True the acceleration is taken into consideration in the objective function

    if not obj_acceleration:  # if the acceleration is not included in the objective function, 'acc' is added to the file name
        extra = 'acc_'
    else:
        extra = ''

    # Create vehicles and add model main variables
    vehicles = []
    for i in range(num_vehicles):
        print(f_max[min(i, len(f_max)-1)])
        if wp:
            vehicles.append(
                Vehicle(vehicle_mass, dt, T, x0[i], y0[i], i, m, M, v_max, f_max[min(i, len(f_max)-1)], area_size, x_fin[i], y_fin[i],
                        wp, x_wp[i], y_wp[i]))
        else:
            vehicles.append(
                Vehicle(vehicle_mass, dt, T, x0[i], y0[i], i, m, M, v_max, f_max[min(i, len(f_max)-1)], area_size, x_fin[i], y_fin[i],
                        wp))

                
    print("num_vehicles", num_vehicles)
    # Add constraints and add model secondary variables
    for i in range(num_vehicles):        
        vehicles[i].constrain_dynamics(vx_init[i], vy_init[i])
        vehicles[i].constrain_positions()

        if(constrain_obstacles):
            vehicles[i].constrain_obstacles(obstacles, d_obs)
        
        if(constrain_multiple_vehicles):
            vehicles[i].constrain_multiple_vehicles(vehicles, 0.6)
        
        if(constrain_waypoints):
            vehicles[i].constrain_waypoints()

    # Obtaining the objective function
    total = 0                                # total number of time steps between all the vehicles (minimize)
    epsilon = 0.001                          # effect of the force on the objective function
    for veh in range(len(vehicles)):
            if obj_acceleration:
                # total += vehicles[veh].b[i] * i + vehicles[veh].fm[i]*epsilon  # Objective function with acceleration
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

    # Update filename with time
    t = time.localtime()
    timestamp =time.strftime('%m%d%H%M_', t)
    name=name+timestamp +".png"
    filename_data= datafolder+filename_data+timestamp+'.csv'
    filename_waypoint =datafolder+filename_waypoint+timestamp+'.csv'

    # Plotting the results
    for i in range(num_vehicles):
        z = 0
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
        wp_times= dict(sorted(wp_times.items()))
        z=list(wp_times.keys())[-1]           #printing the final time
        print("final_time:", z)

     
        coords = np.zeros([z,2])
        for j in range(z):                    # obtaining the coordinates to plot
            coords[j, :] = [vehicles[i].x[j].x,vehicles[i].y[j].x]
        if wp:                                # plotting the location of the waypoints
            for jj in range(len(x_wp[i])):
                plt.plot(x_wp[i][jj], y_wp[i][jj], '*', color='b')
                plt.text(x_wp[i][jj], y_wp[i][jj]-1,str(jj), color='b')
        
        if args.obs == 0:
            labels = ['Turn rate 15 $\degree$/s', 'Turn rate 12 $\degree$/s']
            shape = ['o', '^']
            plt.plot(coords[:,0], coords[:,1], shape[i], fillstyle='none',color='black',label=labels[i])
            plt.legend()
        else:
            plt.scatter(coords[:,0], coords[:,1], facecolor = 'none', edgecolor = 'black')  # plot the trajectories of the vehicles


        #comment below 2 lines because of elimination fix final points
        # plt.plot(vehicles[i].x_fin, vehicles[i].y_fin, '*', color='k')    # plot the final points star
        # plt.scatter(vehicles[i].x_fin, vehicles[i].y_fin, facecolor = 'none', edgecolor = 'black')  # plot the final points circle

    plt.xlim([-area_size, area_size])   # limit the plot space
    plt.ylim([-area_size, area_size])   # limit the plot space
    if args.obs == 0:
        plt.xlim([-10, area_size])   # limit the plot space
        plt.ylim([-10, area_size])   # limit the plot space
    
    plt.savefig(folder + extra + name)                   # save the resulting plot

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
        v_data=np.zeros([z,2])
        f_data=np.zeros([z,2])
        for j in range(z):                    # obtaining the coordinates to plot
            v_data[j, :] = [vehicles[i].vx[j].x,vehicles[i].vy[j].x]
            f_data[j, :] = [vehicles[i].fx[j].x,vehicles[i].fy[j].x]
    
        for j in range(len(vehicles)):                  # extract the velocity of each vehicle
            for i in range(len(vehicles[0].vx)):
                v_coords_x.append(vehicles[j].vx[i].x)  # velocity in the x-direction
                v_coords_y.append(vehicles[j].vy[i].x)  # velocity in the y-direction
                v_coords.append(sqrt(vehicles[j].vy[i].x ** 2 + vehicles[j].vx[i].x ** 2))  # velocity magnitude
            n_steps = len(v_coords)
            fig.plot(range(len(v_coords)), v_coords, color = 'black', label= "Vehicle " + str(j+1), linestyle = line_styles[j], marker = marker_styles[j])
            #need to comment in for multiple vehicles#############
            # v_coords_x = []
            # v_coords_y = []
            # v_coords = []
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
            # f_coords_x = []
            # f_coords_y = []
            # f_coords = []
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
        (pd.DataFrame.from_dict(data=wp_times, orient='index').to_csv(filename_waypoint, header=False))



    # plt.show()
    exit()
