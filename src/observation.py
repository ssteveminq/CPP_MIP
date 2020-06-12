# !/usr/bin/env python3
import numpy as np
from math import *
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config
from grid_map import GridMap
from humans_real_simulation import *
from tools import orientation_processing
from humans import get_human_FOV

class Observation:
    def __init__(self):
        self.awareness_array = []

        # will use the right, left, front left, front right 
        # in order to determine if we are in human awareness zone

        # still need to resolve if right left front right front left ever switch at pi/4

    def get_obs(state, target_state, pedestrian_state, params, target_goal, pedestrian_goal, human_params, gridmap):
        # Maybe give robot access to their goal point which simulates us having a "policy"
        # for the human which allows us to better determine hostility
        # Additionally, having human_params could be our assumption about human params

        # convert each agents yaw to [0,2pi)
        state[2] = orientation_processing(state[2])
        target_state[2] = orientation_processing(target_state[2])
        pedestrian_state[2] = orientation_processing(pedestrian_state[2])

        # determine if robot has human in its FOV
        

        # if a human or more is in FOV:
            # get the FOV for each human
            # target_right, target_left, target_front_rigth, target_front_left, target_pose_tr, target_pose_lr, target_pose_tl, target_pose_ll = get_human_FOV(target_state, human_params_localmap, gridmap)
            # pedestrian_right, pedestrian_left, pedestrian_front_rigth, pedestrian_front_left, pedestrian_pose_tr, pedestrian_pose_lr, pedestrian_pose_tl, pedestrian_pose_ll = get_human_FOV(pedestrian_state, human_params_localmap, gridmap)

            # determine if I am in their FOV
            # if true, append to awareness_array


            # determine_hostility()