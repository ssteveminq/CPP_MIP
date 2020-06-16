# !/usr/bin/env python3
import numpy as np
from math import *
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config
from grid_map import GridMap
from humans_real_simulation import *
from tools import *
from humans import *


def get_agent_FOV(agent_state, agent_params_localmap, gridmap):
    # Return Value in Map Frame 
    agent_state[2] = orientation_processing(agent_state[2])  
    if agent_state[2] >= 3*np.pi/2:
        # get the vertices of the sensor field
        temp = 2*np.pi - agent_state[2]
        ilx = agent_params_localmap.sensor_range*np.cos(temp) + agent_state[0]
        ily = agent_params_localmap.sensor_range*np.sin(temp) + agent_state[1]
        left = np.array([ilx, ily])
        front_left = np.array([ilx + agent_params_localmap.sensor_range*np.cos(temp), ily - agent_params_localmap.sensor_range*np.sin(temp)])
        irx = agent_state[0] - agent_params_localmap.sensor_range*np.cos(temp)
        iry = agent_state[1] - agent_params_localmap.sensor_range*np.sin(temp)
        right = np.array([irx, iry])
        front_right = np.array([irx + agent_params_localmap.sensor_range*np.cos(temp), iry - agent_params_localmap.sensor_range*np.sin(temp)])
        
        # convert to grid space (map frame)
        if agent_state[2] <= 7.0*np.pi/4.0:
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

    elif np.pi <= agent_state[2] < 3*np.pi/2:
        # get the vertices of the sensor field
        temp = agent_state[2] - np.pi
        ilx = agent_params_localmap.sensor_range*np.cos(temp) + agent_state[0]
        ily = agent_state[1] - agent_params_localmap.sensor_range*np.sin(temp) 
        left = np.array([ilx, ily])
        front_left = np.array([ilx - agent_params_localmap.sensor_range*np.cos(temp), ily - agent_params_localmap.sensor_range*np.sin(temp)])
        irx = agent_state[0] - agent_params_localmap.sensor_range*np.cos(temp)
        iry = agent_state[1] + agent_params_localmap.sensor_range*np.sin(temp)
        right = np.array([irx, iry])
        front_right = np.array([irx - agent_params_localmap.sensor_range*np.cos(temp), iry - agent_params_localmap.sensor_range*np.sin(temp)])
        
        # convert to grid space (map frame)
        if agent_state[2] >= 5.0*np.pi/4.0:
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

    elif np.pi/2 <= agent_state[2] < np.pi:
        # get the vertices of the sensor field
        temp = np.pi - agent_state[2]
        ilx = agent_state[0] - agent_params_localmap.sensor_range*np.cos(temp)
        ily = agent_state[1] - agent_params_localmap.sensor_range*np.sin(temp) 
        left = np.array([ilx, ily])
        front_left = np.array([ilx - agent_params_localmap.sensor_range*np.cos(temp), ily + agent_params_localmap.sensor_range*np.sin(temp)])
        irx = agent_state[0] + agent_params_localmap.sensor_range*np.cos(temp)
        iry = agent_state[1] + agent_params_localmap.sensor_range*np.sin(temp)
        right = np.array([irx, iry])
        front_right = np.array([irx - agent_params_localmap.sensor_range*np.cos(temp), iry + agent_params_localmap.sensor_range*np.sin(temp)])

        # convert to grid space (gridmap)
        if agent_state[2] <= 3.0*np.pi/4.0:
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

    elif 0 <= agent_state[2] < np.pi/2:
        # get the vertices of the sensor field
        ilx = agent_state[0] - agent_params_localmap.sensor_range*np.cos(agent_state[2])
        ily = agent_state[1] + agent_params_localmap.sensor_range*np.sin(agent_state[2]) 
        left = np.array([ilx, ily])
        front_left = np.array([ilx + agent_params_localmap.sensor_range*np.cos(agent_state[2]), ily + agent_params_localmap.sensor_range*np.sin(agent_state[2])])
        irx = agent_state[0] + agent_params_localmap.sensor_range*np.cos(agent_state[2])
        iry = agent_state[1] - agent_params_localmap.sensor_range*np.sin(agent_state[2])
        right = np.array([irx, iry])
        front_right = np.array([irx + agent_params_localmap.sensor_range*np.cos(agent_state[2]), iry + agent_params_localmap.sensor_range*np.sin(agent_state[2])])

        # convert to grid space (gridmap)
        if agent_state[2] >= np.pi/4.0:
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





class Observation:
    def __init__(self):
        self.human_awareness_array = [] # an array for whether each robot is aware of us
        self.human_FOV = [] # an array of arrays i.e numrows = numhumans, numcolumns = 4
        self.distance_array = [] # array of distances to humans
        self.count = 0 # integer counter of # humans in FOV of Robot

        
    def get_awareness(self, target_in_FOV, pedestrian_in_FOV, gridmap, state, target_state, pedestrian_state, human_params_localmap):
        pose_robot = gridmap.meters2grid(state[:2])

        # get the FOV for target
        target_lower_right, target_lower_left, target_top_right, target_top_left, target_pose_tr, target_pose_lr, target_pose_tl, target_pose_ll = get_agent_FOV(target_state, human_params_localmap, gridmap)
        target_FOV = np.array([target_top_left, target_top_right, target_lower_right, target_lower_left])
        # get the FOV for pedestrian
        ped_lower_right, ped_lower_left, ped_top_right, ped_top_left, ped_pose_tr, ped_pose_lr, ped_pose_tl, ped_pose_ll = get_agent_FOV(pedestrian_state, human_params_localmap, gridmap)
        pedestrian_FOV = np.array([ped_top_left, ped_top_right, ped_lower_right, ped_lower_left])
        # get the max and min x and y of the ped FOV in grid terms
        p_high_pt = max(ped_pose_tr[1], ped_pose_tl[1])
        p_low_pt = min(ped_pose_ll[1], ped_pose_lr[1])
        p_left_pt = min(ped_pose_tl[0], ped_pose_ll[0])
        p_right_pt = max(ped_pose_tr[0], ped_pose_lr[0])
        # get the max and min x and y of the target FOV in grid terms
        t_high_pt = max(target_pose_tr[1], target_pose_tl[1])
        t_low_pt = min(target_pose_ll[1], target_pose_lr[1])
        t_left_pt = min(target_pose_tl[0], target_pose_ll[0])
        t_right_pt = max(target_pose_tr[0], target_pose_lr[0])
        if self.count > 1:
            if self.ped_dist <= self.target_dist: # Allows us to order our member variables based on distance
                self.human_FOV = np.vstack((pedestrian_FOV, target_FOV))
                # determine if we are in pedestrian FOV
                if p_left_pt <= pose_robot[0] <= p_right_pt and p_low_pt <= pose_robot[1] <= p_high_pt:
                    self.human_awareness_array.append(True)
                else:
                    self.human_awareness_array.append(False)
                # determine if we are in target FOV
                if t_left_pt <= pose_robot[0] <= t_right_pt and t_low_pt <= pose_robot[1] <= t_high_pt:
                    self.human_awareness_array.append(True)
                else:
                    self.human_awareness_array.append(False)
            else: # if the target is closer
                self.human_FOV = np.vstack((target_FOV, pedestrian_FOV))
                # determine if we are in target FOV
                if t_left_pt <= pose_robot[0] <= t_right_pt and t_low_pt <= pose_robot[1] <= t_high_pt:
                    self.human_awareness_array.append(True)
                else:
                    self.human_awareness_array.append(False)
                # determine if we are in pedestrian FOV
                if p_left_pt <= pose_robot[0] <= p_right_pt and p_low_pt <= pose_robot[1] <= p_high_pt:
                    self.human_awareness_array.append(True)
                else:
                    self.human_awareness_array.append(False)
        else: # if only one human in FOV
            if pedestrian_in_FOV:
                self.human_FOV = np.vstack((pedestrian_FOV))
                if p_left_pt <= pose_robot[0] <= p_right_pt and p_low_pt <= pose_robot[1] <= p_high_pt:
                    self.human_awareness_array.append(True)
                else:
                    self.human_awareness_array.append(False)
            elif target_in_FOV:
                self.human_FOV = np.vstack((target_FOV))
                if t_left_pt <= pose_robot[0] <= t_right_pt and t_low_pt <= pose_robot[1] <= t_high_pt:
                    self.human_awareness_array.append(True)
                else:
                    self.human_awareness_array.append(False)
        # print("human_FOV: ", human_FOV)



    def get_obs(self, state, target_state, pedestrian_state, params, target_goal, pedestrian_goal, params_localmap, human_params_localmap, gridmap):
        # Maybe give robot access to their goal point which simulates us having a "policy"
        # for the human which allows us to better determine hostility
        # Additionally, having human_params could be our assumption about human params

         
        human_in_FOV = False # originally assume human in FOV = False
        pedestrian_in_FOV = False
        target_in_FOV = False

        self.count = 0 # keeps track of number of humans in our FOV
        self.human_awareness_array.clear()

        # determine if robot has human in its FOV
        lower_right, lower_left, top_right, top_left, pose_tr, pose_lr, pose_tl, pose_ll = get_agent_FOV(state, params_localmap, gridmap)

        pose_pedestrian = gridmap.meters2grid(pedestrian_state[:2])
        pose_target = gridmap.meters2grid(target_state[:2])
        pose_robot = gridmap.meters2grid(state[:2])

        high_pt = max(pose_tr[1], pose_tl[1])
        low_pt = min(pose_ll[1], pose_lr[1])
        left_pt = min(pose_tl[0], pose_ll[0])
        right_pt = max(pose_tr[0], pose_lr[0])

        # ped_dist = distance(state, pedestrian_state)
        # target_dist = distance(state, target_state)

        if left_pt <= pose_pedestrian[0] <= right_pt and low_pt <= pose_pedestrian[1] <= high_pt:
            # print("pedestrian in robot FOV")
            human_in_FOV = True
            pedestrian_in_FOV = True
            self.count += 1
            self.ped_dist = distance(state, pedestrian_state)
        if left_pt <= pose_target[0] <= right_pt and low_pt <= pose_target[1] <= high_pt:
            # print("target in robot FOV")
            human_in_FOV = True
            target_in_FOV = True
            self.count += 1
            self.target_dist = distance(state, target_state)

        if human_in_FOV:                # Can only determine human awareness if we can see them
            self.get_awareness(target_in_FOV, pedestrian_in_FOV, gridmap, state, target_state, pedestrian_state, human_params_localmap)
            # determine if I am in their FOV (probably easiest to use gridmap)
            # if true, append to awareness_array


                # determine_hostility()
