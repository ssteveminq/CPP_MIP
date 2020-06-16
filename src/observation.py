# !/usr/bin/env python3
import numpy as np
from math import *
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config
from grid_map import GridMap
from humans_real_simulation import *
from tools import *
from humans import *




class Observation:
    def __init__(self):
        self.awareness_array = []
        self.human_FOV = [] 

        # TODO: still need to resolve if right left front right front left ever switch at pi/4

    def get_obs(self, state, target_state, pedestrian_state, params, target_goal, pedestrian_goal, human_params_localmap, gridmap):
        # Maybe give robot access to their goal point which simulates us having a "policy"
        # for the human which allows us to better determine hostility
        # Additionally, having human_params could be our assumption about human params

        # determine if robot has human in its FOV
        

        # if a human or more is in FOV:
            # get the FOV for each human
        lower_right, lower_left, top_right, top_left, pose_tr, pose_lr, pose_tl, pose_ll = get_human_FOV(target_state, human_params_localmap, gridmap)
        lower_right, lower_left, top_right, top_left, pose_tr, pose_lr, pose_tl, pose_ll = get_human_FOV(pedestrian_state, human_params_localmap, gridmap)
            # determine if I am in their FOV (probably easiest to use gridmap)
            # if true, append to awareness_array


                # determine_hostility()