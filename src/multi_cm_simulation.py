# !/usr/bin/env python3
import argparse
import pandas as pd
from numpy import genfromtxt
import numpy as np
from math import *
import random
import math
import argparse
import csv
import matplotlib.pyplot as plt
from obstacle import Obstacle
from raycasting_grid_map import generate_ray_casting_grid_map, calc_grid_map_config, atan_zero_to_twopi, precasting
from state_lattice_planner import uniform_terminal_state_sampling_test1, lane_state_sampling_test1
from utils.configuration_space import configuration_space
from utils.cubic_spline_planner import Spline2D, calc_spline_course_trj
from utils.stanley_controller import *
from utils.graph_utils import *
from utils.dynamic_window_approach import *
from  VCD import VerticalCellDecomposition
import pandas as pd
import os
import re
import time
from matplotlib.patches import Polygon, Rectangle, Circle
import matplotlib
import matplotlib.animation as animation
import matplotlib as mpl
from matplotlib.pyplot import cm
import ast
from pynput import keyboard 
#probability
l_occ=np.log(0.95/0.05)
l_free=np.log(0.05/0.95)
l_same = np.log(0.9/0.1)
l_diff= np.log(0.01/0.99)
horizon=20
shorthorizon=10
boolsaved = False
weight_entropy = 0.15
weight_travel = 1.5

#pp control 
#test
k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
Kv = 0.1  # speed propotional gain
ktheta = 0.9
dt = 0.2  # [s]
L = 1.0  # [m] wheel base of vehicle
AlphabetSet=['a','b','c','d','e','f','g','h','i','j','k','l','m', 
				'n','o','p','q','r']
ColorSet=['green', 'red', 'blue','yellow', 'black', 'cyan' ]
Region_Boundary =12.5
t = time.localtime()
dir_path = os.path.dirname(os.path.realpath(__file__))
dir_path=dir_path[:-4]
timestamp =time.strftime('%m%d%H%M_', t)
file_name =dir_path+"/results/entropy/entropy_" +timestamp+".csv"



class map_info:
	sensor_range=5
	xmin=-5
	xmax=5
	ymin=-5
	ymax=5
	xyreso = 0.25
	width = 10.0/0.25
	legth = 10.0/0.25
		
	def __init__(self, x, y):
		self.xmin = x-sensor_range
		self.xmax = x+sensor_range
		self.ymin = y-sensor_range
		self.ymax = y+sensor_range


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
		self.numiters = 4000
		self.dt = 0.2
		self.goal_tol = 1.0
		self.weight_entropy = 0.02
		self.weight_travel =1.6
		self.max_vel = 1.0 # m/s
		self.min_vel = 0.0 # m/s
		self.sensor_range_m = 0.5 # m
		# self.animate = 1
		self.area_size=13
		#dwa parameters
		self.max_yawrate = 45.0 * math.pi / 180.0  # [rad/s]
		self.max_accel = 0.3  # [m/ss]
		self.max_dyawrate = 45.0 * math.pi / 180.0  # [rad/ss]
		self.v_reso = 0.02  # [m/s]
		self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
		self.max_speed = 0.8  # [m/s]
		self.min_speed = 0.0  # [m/s]
		self.predict_time = 2.0  # [s]
		self.to_goal_cost_gain = 1.0
		self.speed_cost_gain = 1.0
		self.robot_radius = 1.0  # [m]

		# self.time_to_switch_goal = 5.0 # sec #inactive for now

def random_sampling(params, nums):
	#nums = the number of sample we need

	reso= (2*params.area_size)/(nums+1)
	x = np.arange(-0.9*params.area_size, 0.9*params.area_size, reso)
	sample_xy=[]
	for i in range(len(x)-1):
		# print("i", i)
		temp_x= random.uniform(x[i],x[i+1])
		temp_y= random.uniform(-0.9*params.area_size,0.9*params.area_size)
		sample_xy.append([temp_x,temp_y])
	return sample_xy

def goal_sampling_VCD(waypoints, agent_x, agent_y, params_global):
	#nums = the number of sample we need
	dist_threshold = 2.0
	agent_pos = [agent_x, agent_y]
	goals = []
	
	for waypoint in waypoints:
		temppos=[0,0]
		if distance(waypoint, agent_pos)> dist_threshold:
			temppos[0]=waypoint[0]+random.uniform(-1.0,1.0)
			temppos[1]=waypoint[1]+random.uniform(-1.0,1.0)
			if temppos[0] > params_global.xmax or temppos[0] < params_global.xmin:
				temppos[0]=waypoint[0]
			if temppos[1] > params_global.ymax or temppos[1] < params_global.ymin:
				temppos[1]=waypoint[1]

			goals.append(temppos)

	return goals

def choose_goal_from_trj(best_trj, params_globalmap, horizon=20):
	goal =[0,0]

	if len(best_trj[0])>horizon and horizon>0:
		goal = [best_trj[0][horizon-1], best_trj[1][horizon-1]]
	else:
		goal = [best_trj[0][-1], best_trj[1][-1]]

	if goal[0]> params_globalmap.xmax:
		goal[0]=params_globalmap.xmax-0.5
	elif goal[0]< params_globalmap.xmin:
		goal[0]=params_globalmap.xmin+0.5
	if goal[1]> params_globalmap.ymax:
		goal[1]=params_globalmap.ymax-0.5
	elif goal[1]< params_globalmap.ymin:
		goal[1]=params_globalmap.ymin+0.5

	return goal


def calc_IG_trjs(trj_candidates, entropy_map, params_global, horizon=25):
	# print("TODO")
	# Calculating Information gain on trajectories
	# 1) calculating sampling point for time horizon 
	# 2) calculating FOV region over sampling points 
	# 3) Collecting IG gain for overlapped region
	# Suppose we have maximum velocity/ arc length
	# sampling points w.r.t distance 
	# print("trj_candidates", trj_candidates)
	# print("len trj_candidates", len(trj_candidates))
	weight_travel=0.5

	if len(trj_candidates)>0:
		w_t=1.0
		igs=[]
		travels=[]
		tpoints=[]
		for j, trj in enumerate(trj_candidates):
			ig=0
			travel=0
			#Set ref_x and ref_y as the first point of trajectory
			ref_x = trj[0][0]
			ref_y = trj[1][0]
			for i in range(len(trj[0])):
			  #sparse sampling (add sampoing point for the last point of trajectory)
				if i%10==0 or i==len(trj[0])-1:
					if horizon>0:
						if i<horizon:
						# print("i: ", i, "x: ", trj[0][i], ", y: ", trj[1][i], ", ig:", ig)
							y=trj[1][i]
							x=trj[0][i]
								# print("x", x)
								# print("y", y)
							ig+= get_expected_entropy_infov([x, y], entropy_map, params_global)
							dx=trj[0][i]-ref_x
							dy=trj[1][i]-ref_y
							travel+=(dx**2+dy**2)**0.5
							ref_x=trj[0][i]
							ref_y=trj[1][i]
							weight_travel=1.0
					else:                     #the infinite horizon case
						ig+= self.get_expected_entropy_infov([trj[0][i],trj[1][i]],entropy_map, params_global)
						dx=trj[0][i]-ref_x
						dy=trj[1][i]-ref_y
						travel+=(dx**2+dy**2)**0.5
						ref_x=trj[0][i]
						ref_y=trj[1][i]
						weight_travel=1.0

			# print("travel:" , travel)
			# print("ig", ig, ", travel: ",travel)
			cost = weight_entropy*ig-weight_travel*travel
			igs.append(cost)
			travels.append(travel)
			tpoints.append([trj[0][-1], trj[1][-1]])

		print("igs", igs)
		print("travels", travels)
		print("tpoints", tpoints)
		sorted_ig = sorted(((v,i) for i, v in enumerate(igs)),reverse=True)
		max_idx = sorted_ig[0][1]
		print("best trj idx : ", max_idx, "total num of candidates: ", len(igs))
		print("best_ig[] : ", igs[max_idx])
		# print("best information gain : ", sorted_ig[0][0])
		return trj_candidates[max_idx]
	else:
		print("no trajectory candidates")

def calc_IG_trjs_with_leadtrj( trj_candidates, entropy_map, lead_trj, params_global, horizon=20):
	# w_t=1.0
	if len(trj_candidates)>0:
		igs=[]
		for j, trj in enumerate(trj_candidates):
			ig=0
			travel=0
			ref_x = trj[0][0]
			ref_y = trj[1][0]
			for i in range(len(trj[0])):

				if i%10==0 or i==len(trj[0])-1:
					if horizon>0:
						if i<horizon:
							ig+= get_expected_entropy_infov_trj([trj[0][i],trj[1][i]],entropy_map,lead_trj, params_global)
							dx=trj[0][i]-ref_x
							dy=trj[1][i]-ref_y
							travel+=(dx**2+dy**2)**0.5
							ref_x=trj[0][i]
							ref_y=trj[1][i]
					else:                     #the infinite horizon case
						ig+= get_expected_entropy_infov_trj([trj[0][i],trj[1][i]],entropy_map, lead_trj, params_global)
						dx=trj[0][i]-ref_x
						dy=trj[1][i]-ref_y
						travel+=(dx**2+dy**2)**0.5
						ref_x=trj[0][i]
						ref_y=trj[1][i]
						weight_travel=0.2

			cost = weight_entropy*ig-weight_travel*travel
			igs.append(cost)

		sorted_ig = sorted(((v,i) for i, v in enumerate(igs)),reverse=True)
		max_idx = sorted_ig[0][1]
		return trj_candidates[max_idx]
	else:
		print("no trajectory candidates")
		return None

def calc_IG_trjs_hierarchy( trj_candidates, entropy_map, params_global, trjs, agentnum, horizon=20):
	# w_t=1.0

	if len(trj_candidates)>0:
		igs=[]
		considered_trjs=[]

		print("trjs", trjs)
		# input("enter here")
		for i, trj in enumerate(trjs):
			if i<=agentnum:
				considered_trjs.append(trj)

		print("agent_num", agentnum)        
		print("considered_trjs", len(considered_trjs))

		for j, trj in enumerate(trj_candidates):
			ig=0
			travel=0
			ref_x = trj[0][0]
			ref_y = trj[1][0]
			for i in range(len(trj[0])):

				if i%10==0 or i==len(trj[0])-1:
					if horizon>0:
						if i<horizon:
							ig+= get_expected_entropy_infov_trjs([trj[0][i],trj[1][i]],entropy_map,considered_trjs, params_global)
							dx = trj[0][i]-ref_x
							dy = trj[1][i]-ref_y
							travel+=(dx**2+dy**2)**0.5
							ref_x=trj[0][i]
							ref_y=trj[1][i]
						weight_travel=0.5
					else:                     #the infinite horizon case
						ig+= get_expected_entropy_infov_trjs([trj[0][i],trj[1][i]],entropy_map, considered_trjs, params_global)
						dx=trj[0][i]-ref_x
						dy=trj[1][i]-ref_y
						travel+=(dx**2+dy**2)**0.5
						ref_x=trj[0][i]
						ref_y=trj[1][i]
						weight_travel=0.2

			cost = weight_entropy*ig-weight_travel*travel
			igs.append(cost)

		sorted_ig = sorted(((v,i) for i, v in enumerate(igs)),reverse=True)
		max_idx = sorted_ig[0][1]
		return trj_candidates[max_idx]
	else:
		print("no trajectory candidates")
		return None





def calc_IG_trjs_follower(trj_candidates, best_trj, emap, params_local, params_local2, params_global,params,  horizon=15):
	# print("TODO")
	#Calculating Information gain on trajectories
	#1) calculating sampling point for time horizon 
	#2) calculating FOV region over sampling points 
	#3) Collecting IG gain for overlapped region
	#Suppose we have maximum velocity/ arc length
	#sampling points w.r.t distance 
	igs=[]
	pairs =[]
	# calculate each information gain from trajectory candidates
	# print("calc_ig_multi-before")
	for trj in trj_candidates:
		ig=0
		travel=0
		for i in range(len(trj[0])):
			if horizon>0 and i<horizon:             #The finite horizon case
				w_t=1.0         #weight 
				x=trj[0][i]
				y=trj[1][i]
				ig+= get_entropy_infov_multi([x,y],emap,params_local, params_global, best_trj)
				if i>0:
					dx = trj[0][i]-trj[0][i-1]
					dy = trj[1][i]-trj[1][i-1]
					travel+=(dx**2+dy**2)**0.5

		# print("ig: ", ig)
		cost = params. weight_entropy*ig-w_t*travel
		igs.append(cost)

	# print("calc_ig_multi")
	sorted_ig = sorted(((v,i) for i, v in enumerate(igs)),reverse=True)
	max_idx = sorted_ig[0][1]
	# print("best information gain : ", sorted_ig[0][0])
	return trj_candidates[max_idx] 

def calc_min_distance_from_trj(px,py, trj):
	min_dist = 100
	for i in range(len(trj[0])):
		tempdist = sqrt((px - trj[0][i])**2+(py-trj[1][i])**2) #distance to gaol
		if tempdist < min_dist:
			min_dist = tempdist
	return min_dist

def calc_min_distance_from_trjs(px,py, trjs):
	min_dist = 100
	for trj in trjs:
		for i in range(len(trj[0])):
			tempdist = sqrt((px - trj[0][i])**2+(py-trj[1][i])**2) #distance to gaol
			if tempdist < min_dist:
				min_dist = tempdist
	return min_dist

def Coord2CellIdx(x, y, map_info):
	temp_x = x-map_info.origin.position.x
	temp_y = y-map_info.origin.position.y
	coord_x= math.floor(temp_x/map_info.resolution)
	coord_y= math.floor(temp_y/map_info.resolution)
	# print("coord_x:" ,coord_x, "coord_y:" ,coord_y )
	idx= (int)(coord_x+map_info.width*coord_y)

	return idx

def Coord2CellIdx(x, y, params_searchmap):
	temp_x = x-map_info.origin.position.x
	temp_y = y-map_info.origin.position.y
	coord_x= math.floor(temp_x/map_info.resolution)
	coord_y= math.floor(temp_y/map_info.resolution)
	# print("coord_x:" ,coord_x, "coord_y:" ,coord_y )
	idx= (int)(coord_x+map_info.width*coord_y)

	return idx

def Coord2CellIdx_global(x, y, params_searchmap):
	#change coordinates w.r.t global frame
	temp_x = x-params_searchmap.xmin
	temp_y = y-params_searchmap.ymin
	coord_x= math.floor(temp_x/params_searchmap.xyreso)
	coord_y= math.floor(temp_y/params_searchmap.xyreso)
	# print("coord_x:" ,coord_x, "coord_y:" ,coord_y )
	idx= (int)(coord_x+params_searchmap.xw*coord_y)

	return idx

def Idx2Coord(self, idx, map_info):
	res = (int) (idx/map_info.width);
	div = (int) (idx%map_info.width);
	coord_x=(res+0.5)*map_info.resolution+map_info.origin.position.x;
	coord_y=(div+0.5)*map_info.resolution+map_info.origin.position.y;

	return coord_x, coord_y


def get_impossible_indices(pos, global_map, params_searchmap):
	center_x=pos[0]
	center_y=pos[1]
	minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(params_searchmap.xyreso
										, center_x,center_y, params_searchmap.sensor_range)

	precast = precasting(minx,miny, xw, yw, params_searchmap.xyreso,
						params_searchmap.yawreso, center_x,center_y)

	#generate min_angle_map[angleid]=distance
	min_angle_map={}
	for ix_local in range(xw-1):
		for iy_local in range(yw-1):
			px = minx+ix_local*params_searchmap.xyreso
			py = miny+iy_local*params_searchmap.xyreso

			if px >= params_searchmap.xmax or px <= params_searchmap.xmin:
				continue
			if py >= params_searchmap.ymax or py <= params_searchmap.ymin:
				continue

			#check if there exists static obstacle

			ix_global= math.floor((px-params_searchmap.xmin)/params_searchmap.xyreso)
			iy_global= math.floor((py-params_searchmap.ymin)/params_searchmap.xyreso)
			# searchmap_info= map_info(center_x,center_y)
			# gmap_idx = Coord2CellIdx_global(px,py, params_searchmap)
			if global_map[ix_global][iy_global]>0:
				obs_angle = atan_zero_to_twopi(py-center_y,px-center_x)
				angleid =  int(math.floor(obs_angle / params_searchmap.yawreso))
				d= math.sqrt((px-center_x)**2 + (py-center_y)**2)
				if angleid in min_angle_map:
					if d<min_angle_map[angleid]:
						min_angle_map[angleid]=d
				else:
					min_angle_map[angleid]=d

	#check feasible index (output: set of infeasible map_indices )
	impossible_idx=[]
	for angle_id, min_dist in min_angle_map.items():
		gridlist = precast[angleid]
		for grid in gridlist:
			if grid.d > min_dist:
				search_idx = Coord2CellIdx_global(grid.px, grid.py,params_searchmap)
				impossible_idx.append(search_idx)

	return impossible_idx





def get_entropy_infov_multi(state,entropy_map,params_local,params_global, trj, dist_th=5.0):

	minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(params_local.xyreso, state[0], state[1], params_local.sensor_range)
	entropy_sum=0.0
	for ix_local in range(xw-1):
		for iy_local in range(yw-1):
			px = minx+ix_local*params_local.xyreso
			py = miny+iy_local*params_local.xyreso
			# print("px: ", px, "py: ", py)
			if px >= params_global.xmax or px <= params_global.xmin:
				continue
			if py >= params_global.ymax or py <= params_global.ymin:
				continue

			ix_global= math.floor((px-params_global.xmin)/params_global.xyreso)
			iy_global= math.floor((py-params_global.ymin)/params_global.xyreso)
			min_dist = calc_min_distance_from_trj(px,py,trj)
			# print("(ix_global, iy_global): ",ix_global, " , ", iy_global)
			# entropy_sum+= entropy_map[ix_local][iy_local]
			if min_dist>dist_th:
				entropy_sum+= entropy_map[ix_global][iy_global]

	# print("entropy_sum", entropy_sum)
	return entropy_sum


def get_expected_entropy_infov(pos, entropymap, params_searchmap):

		center_x=pos[0]
		center_y=pos[1]
		minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(params_searchmap.xyreso
											, center_x,center_y, params_searchmap.sensor_range)

		# impossible_idx= get_impossible_indices(pos, entropymap, params_searchmap)
		impossible_idx= []
		# print("impossible_idx", impossible_idx)
		#iteration for calculating entropy_map
		entropy_sum=0.0
		cell_count=0
		for ix_local in range(xw-1):
			for iy_local in range(yw-1):
				px = minx+ix_local*params_searchmap.xyreso
				py = miny+iy_local*params_searchmap.xyreso
				if px >= params_searchmap.xmax or px <= params_searchmap.xmin:
					continue
				if py >= params_searchmap.ymax or py <= params_searchmap.ymin:
					continue

				# search_idx = Coord2CellIdx(px,py,entropymap)
				ix_global= math.floor((px-params_searchmap.xmin)/params_searchmap.xyreso)
				iy_global= math.floor((py-params_searchmap.ymin)/params_searchmap.xyreso)
				# print("entropy_map", entropymap)
				# input("stop")
				if entropymap[ix_global][iy_global]>0:
					cell_count=cell_count+1
					# entropy_sum+= entropy_map[ix_global][iy_global]
 
				#convert log-occ to probability
				# if search_idx not in impossible_idx:
					#count unknown cells 
					# if search_region.data[search_idx]==0:
					#if self.search_region.data[search_idx]!=int(l_occ) and self.search_region.data[search_idx]!=int(l_free):
						# cell_count=cell_count+1
					#p= 1-1./(1.0+np.exp(self.search_region.data[search_idx]))
					#jf p>0.0 and p<1.0:                 
					#    entropy_sum+=-(p*math.log(p)+(1-p)*math.log(1-p))
		print("cell counts: ", cell_count)
		entropy_sum=cell_count*0.693147

		return entropy_sum

   

def get_expected_entropy_infov_trj(pos, entropy_map, leader_trj, params_searchmap, dist_th=5.0):

		center_x=pos[0]
		center_y=pos[1]
		minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(params_searchmap.xyreso
											, center_x,center_y, params_searchmap.sensor_range)

		impossible_idx= self.get_impossible_indices(pos,entropy_map, params_searchmap)
		#iteration for calculating entropy_map
		entropy_sum=0.0
		cell_count=0
		for ix_local in range(xw-1):
			for iy_local in range(yw-1):
				px = minx+ix_local*params_searchmap.xyreso
				py = miny+iy_local*params_searchmap.xyreso
				if px >= params_searchmap.xmax or px <= params_searchmap.xmin:
					continue
				if py >= params_searchmap.ymax or py <= params_searchmap.ymin:
					continue

				search_idx = self.Coord2CellIdx(px,py, self.search_region.info)
				min_dist = self.calc_min_distance_from_trj(px,py,leader_trj)
				#convert log-occ to probability
				if search_idx not in impossible_idx and (min_dist>dist_th):
					#count unknown cells 
					if self.search_region.data[search_idx]==0:
						cell_count=cell_count+1

		# print("cell counts: ", cell_count)
		entropy_sum=cell_count*0.693147

		return entropy_sum


def get_expected_entropy_infov_trjs(pos, entropy_map, other_trjs, params_searchmap, dist_th=5.0):

		center_x=pos[0]
		center_y=pos[1]
		minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(params_searchmap.xyreso
											, center_x,center_y, params_searchmap.sensor_range)

		impossible_idx= get_impossible_indices(pos,entropy_map, params_searchmap)
		#iteration for calculating entropy_map

		# 
		entropy_sum=0.0
		cell_count=0
		for ix_local in range(xw-1):
			for iy_local in range(yw-1):
				px = minx+ix_local*params_searchmap.xyreso
				py = miny+iy_local*params_searchmap.xyreso
				if px >= params_searchmap.xmax or px <= params_searchmap.xmin:
					continue
				if py >= params_searchmap.ymax or py <= params_searchmap.ymin:
					continue

				search_idx = Coord2CellIdx_global(px,py, params_searchmap)
				min_dist = calc_min_distance_from_trjs(px,py,other_trjs)
				ix_global= math.floor((px-params_searchmap.xmin)/params_searchmap.xyreso)
				iy_global= math.floor((py-params_searchmap.ymin)/params_searchmap.xyreso)


				#convert log-occ to probability
				if search_idx not in impossible_idx and (min_dist>dist_th):
					#count unknown cells 
					if entropy_map[ix_global][iy_global]==0:
						cell_count=cell_count+1

		# print("cell counts: ", cell_count)
		entropy_sum=cell_count*0.693147

		return entropy_sum




def get_entropy_infov(state,entropy_map,params_local,params_global):

	minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(params_local.xyreso, state[0], state[1], params_local.sensor_range)
	entropy_sum=0.0
	for ix_local in range(xw-1):
		for iy_local in range(yw-1):
			px = minx+ix_local*params_local.xyreso
			py = miny+iy_local*params_local.xyreso
			# print("px: ", px, "py: ", py)
			if px >= params_global.xmax or px <= params_global.xmin:
				continue
			if py >= params_global.ymax or py <= params_global.ymin:
				continue

			ix_global= math.floor((px-params_global.xmin)/params_global.xyreso)
			iy_global= math.floor((py-params_global.ymin)/params_global.xyreso)
			# print("(ix_global, iy_global): ",ix_global, " , ", iy_global)
			# entropy_sum+= entropy_map[ix_local][iy_local]
			entropy_sum+= entropy_map[ix_global][iy_global]

	# print("entropy_sum", entropy_sum)
	return entropy_sum

def generating_globaltrjs(cur_state, cspace, planner, obstacles,goals, params_global):

	trjs=[]
	for goal_pos in goals:
		init_pos = [cur_state[0], cur_state[1]]
		cspace.reset_environment(params_global.boundaries,init_pos,goal_pos, obstacles)
		planner.reset_cspace(cspace)
		path, path_idx = planner.search(False, goal_pos)
		if path!=None:
			xs=[]
			ys=[]
			for i in range(len(path)):
				xs.append(path[i][0])
				ys.append(path[i][1])
			sp=Spline2D(xs,ys)
			trjs.append(sp)

	return trjs


def trjs_to_sample(trjs, num_points=60, showplot=True):
	spline_trjs=[]
	for i, sp in enumerate(trjs):
		ds = 0.2                            # [m] distance of each intepolated points
		s = np.arange(0, sp.s[-1], ds)
		rx, ry, ryaw, rk = [], [], [], []
		s_iter=0
		for i_s in s:
			ix, iy = sp.calc_position(i_s)
			rx.append(ix)
			ry.append(iy)

		spline_trjs.append([rx, ry])

	return spline_trjs


def plot_local_trjs(trjs, ax=None):
	for trj in trjs:
		if ax==None:
			plt.plot(trj[0], trj[1], "-r")
		else:
			ax.plot(trj[0], trj[1], "-r")


def plot_global_trjs(trjs, ax=None):
	# num_colors = len(gtrjs)
	# cm =plt.get_cmap('gist_rainbow')
	# for i, sp in enumerate(gtrjs):
		# col = cm(1.*i/num_colors)
		# ds = 0.2                    # [m] distance of each intepolated points
		# s = np.arange(0, sp.s[-1], ds)
		# rx, ry, ryaw, rk = [], [], [], []
		# s_iter=0
		# for i_s in s:
			# ix, iy = sp.calc_position(i_s)
			# rx.append(ix)
			# ry.append(iy)
	for trj in trjs:
		if ax==None:
			# plt.plot(trj[0][1:70], trj[1][1:70], color='g', label="spline")
			plt.plot(trj[0], trj[1], color='g', label="spline")
		else:
			# ax.plot(trj[0][1:70], trj[1][1:70], color='g', label="spline")
			ax.plot(trj[0], trj[1], color='g', label="spline")
			# ax.plot(rx[1:80], ry[1:80], color='g', label="spline")

def plot_best_trj(trj, horizon,ax=None):
	if ax==None:
		if horizon>0:
			plt.plot(trj[0][1:horizon], trj[1][1:horizon], color='b',linewidth=2.5,  label="best-spline")
		else:
			plt.plot(trj[0], trj[1], color='b',linewidth=2.5,  label="best-spline")
	else:
		if horizon>0:
			ax.plot(trj[0][1:horizon], trj[1][1:horizon], color='b',linewidth=2.5,  label="best-spline")
		else:
			ax.plot(trj[0], trj[1], color='b',linewidth=2.5,  label="best-spline")



def draw_occmap(data, params_map,params_global, agent_x, agent_y, ax):

	minx=params_map.xmin
	miny=params_map.ymin
	maxx=params_map.xmax
	maxy=params_map.ymax
	xyreso=params_map.xyreso
	# print("min x, min y:", minx, ", ", miny)
	# print("max x, max y:", maxx, ", ", maxy)

	x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
					slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
	# if px >= params_global.xmax or px <= params_global.xmin:
		# continue
	# if py >= params_global.ymax or py <= params_global.ymin:
		# continue

	# ax.pcolor(x+agent_x, y+agent_y, data, vmax=1.0, cmap=plt.cm.Blues)
	# print("data", data)
	ax.pcolormesh(x[:40,:40], y[:40,:40], data, vmax=1.0, cmap=plt.cm.Blues)
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

def draw_entropymap_global(data,parmas_globalmap, ax):

	minx=params_globalmap.xmin
	miny=params_globalmap.ymin
	maxx=params_globalmap.xmax
	maxy=params_globalmap.ymax
	xyreso=params_globalmap.xyreso

	x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
					slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
	# ax.pcolor(x, y, data, vmax=0.69315, cmap=plt.cm.Reds)
	ax.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Reds)
	ax.set_xlim([1.1*minx, 1.1*maxx])   # limit the plot space
	ax.set_ylim([1.1*miny, 1.1*maxy])   # limit the plot space



def get_map_entropy(pmap_global,params_map):
	entropy_sum=0
	# pmap= 1-1./(1.0+np.exp(pmap_global))
	for ix in range(params_map.xw-1):
		for iy in range(params_map.yw-1):
			p=1-1./(1.0+np.exp(pmap_global[ix][iy]))
			# p=1-1./(1.0+pow(2,pmap_global[ix][iy]))
			# p =pmap[ix][iy]
			# print("p: ", p)
			if p>0.0 and p<1.0:
				entropy_sum+=(p*math.log(p)+(1-p)*math.log(1-p))
			# entropy_sum+=p*math.log(p)

	return -entropy_sum

def get_map_entropy2(pmap_global,params_map):
	entropy_sum=0
	# pmap= 1-1./(1.0+np.exp(pmap_global))
	# print("xw", params_map.xw)
	# print("yw", params_map.yw)
	for ix in range(params_map.xw):
		for iy in range(params_map.yw):
			if pmap_global[ix][iy]!=l_free and pmap_global[ix][iy]!=l_occ:
				entropy_sum=entropy_sum+1
			# p=1-1./(1.0+np.exp(pmap_global[ix][iy]))
			# p=1-1./(1.0+pow(2,pmap_global[ix][iy]))
			# p =pmap[ix][iy]
			# print("p: ", p)
			# if p>0.0 and p<1.0:
				# entropy_sum+=(p*math.log(p)+(1-p)*math.log(1-p))
			# entropy_sum+=p*math.log(p)
	# entropy_sum= 

	return entropy_sum

def get_global_entropymap(pmap_global,params_map):
	entropy_sum=0
	#log odd to proabability
	pmap= 1-1./(1.0+np.exp(pmap_global))
	# pmap= 1-1./(1.0+pow(2,pmap_global))

	for ix in range(params_map.xw-1):
		for iy in range(params_map.yw-1):
			p =pmap[ix][iy]
			if p>0.0 and p<1.0:
				pmap[ix][iy]=-(p*math.log2(p)+(1-p)*math.log2(1-p))
			elif p==0.0 or p==1.0:
				 pmap[ix][iy]=0.0
			else:
				print("wrong probability value")

	return pmap

def get_local_entropymap(pmap_local,params_map):
	entropy_sum=0
	pmap= 1-1./(1.0+np.exp(pmap_global))
	# pmap= 1-1./(1.0+pow(2,pmap_global))

	for ix in range(params_map.xw-1):
		for iy in range(params_map.yw-1):
			p =pmap[ix][iy]
			if p>0.0 and p<1.0:
				pmap[ix][iy]=-(p*math.log(p)+(1-p)*math.log(1-p))
			elif p==0.0 or p==1.0:
				 pmap[ix][iy]=0.0
			else:
				print("wrong probability value")

	return pmap




def update_occ_grid_map(state, local_map, params_local, global_map, params_global):
	##for observed cell in local window--> update
	# print("local grids, xw, yw : ", params_local.xw, params_local.yw)
	# print("global grids, xw, yw : ", params_global.xw, params_global.yw)
	updated_list =[]

	#update log-odds
	for ix_local in range(params_local.xw-1):
		for iy_local in range(params_local.yw-1):
			px = params_local.xmin+ix_local*params_local.xyreso
			py = params_local.ymin+iy_local*params_local.xyreso
			if px >= params_global.xmax or px <= params_global.xmin:
				continue
			if py >= params_global.ymax or py <= params_global.ymin:
				continue

			ix_global= math.floor((px-params_global.xmin)/params_global.xyreso)
			iy_global= math.floor((py-params_global.ymin)/params_global.xyreso)
			# print("(ix_global, iy_global): ",ix_global, " , ", iy_global)
			meas = local_map[ix_local][iy_local]
			global_map[ix_global][iy_global] =meas
			updated_list.append([ix_global, iy_global])


	#convert log odds to probability 
	'''
	pmap= 1-1./(1.0+np.exp(global_map))
	for ix in range(params_global.xw-1):
		for iy in range(params_global.yw-1):
			if [ix,iy] in updated_list:
				# print("ix: ", ix, ", iy:",  iy)
				continue
			else:
				# prior=1-1./(1.0+np.exp(pmap_global[ix][iy]))
				# print("no-----list- ix: ", ix, ", iy:",  iy)
				# global_map[ix][iy]=global_map[ix][iy]*1.03
				prior = pmap[ix][iy]
				posterior = prior*0.999999+(1-prior)*0.000001
				# if posterior<0.5:
				global_map[ix][iy]= np.log(posterior/(1-posterior))
				# if prior!=0.5:
					# print("prior: ", prior, ", Posterior:", posterior)
					# print("log-odd: ", np.log(posterior/(1-posterior)))
					# print("-----------------")
				# global_map[ix][iy]+=l_same
	'''
	return global_map


def initialize_global_occ_grid_map(params_map):

	pmap_global = [[0.0 for i in range(params_map.yw)] for i in range(params_map.xw)]
	# pmap_global = [[l_free for i in range(params_map.yw)] for i in range(params_map.xw)]
	return pmap_global

def initialize_global_occ_grid_map_odds(params_map):

	pmap_global = [[0.0 for i in range(params_map.yw)] for i in range(params_map.xw)]
	# pmap_global = [[l_free for i in range(params_map.yw)] for i in range(params_map.xw)]
	return pmap_global

def plot_robot(poses, params):
	num_poses = len(poses)
	cm =plt.get_cmap('gist_rainbow')
	
	for i, pose in enumerate(poses):
		col = cm(1.*i/num_poses )
		r = params.sensor_range_m
		# ax = plt.gca()
		axes[0,0].plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
					[pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color=col)
		axes[0,0].plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
					[pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color=col)

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
				 np.array(outline[1, :]).flatten(),color=col)

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
def plot_obstacles(obstacles, walls):
	for obs in obstacles:
		obs.draw(axes[0,0])
	for wall in walls:
		wall.draw(axes[0,0])


def visualize(trajs, pose, obstacles, walls, params):
	# ax = plt.gca()
	# plt.plot(traj[:,0], traj[:,1], 'g')
	plot_robot(pose, params)
	plot_obstacles(obstacles,walls)

	axes[0,0].set_xlim([-(params.area_size+1), (params.area_size+1)])   # limit the plot space
	axes[0,0].set_ylim([-(params.area_size+1), (params.area_size+1)])   # limit the plot space
	axes[0,0].plot(trajs[0][:,0], trajs[0][:,1], 'r')
	axes[0,0].plot(trajs[1][:,0], trajs[1][:,1], 'g')
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
	err_phi = 0.7*math.sin(des_phi-cur_yaw)
	# err_phi = des_phi

	return err_phi

#dyanmics
def motion_dwa(state, inputs, goal, obpoints, walls, params):
	# state = [x(m), y(m), yaw(rad) ,velocity(m/s), angular velocity ]
	# input = [a(m/s**2), steering(rad) ]
	# a =Update_a(state,goal)
	# delta = Update_phi(state,goal)
	# print("before-state", state)
	# print("before-input", inputs)

	u, ltraj = dwa_control(state, inputs, goal, obpoints, params)

	state[0] += u[0] * math.cos(state[2]) * params.dt
	state[1] += u[0] * math.sin(state[2]) * params.dt
	state[2] += u[1] * params.dt
	state[3] = u[0]   #velocity
	state[4] = u[1]   #angular velocity

	# print("delta:", delta)
	# print("pre-state[2]:", state[2])
	# print("goal:", goal)
	# state[2] +=  math.sin(delta) * dt

	# print("post-state[2]:", state[2])

	if state[3] >= params.max_speed: state[3] = params.max_speed
	if state[3] <= params.min_speed: state[3] = params.min_speed

	if state[2] >= 2*math.pi: state[2] -= 2*math.pi
	if state[2] <= -2*math.pi: state[2] += 2*math.pi

	# print("after-state", state)
	# print("after-input", u)
	return state, u


def motion_sc(state, cx,cy,cyaw, target_idx,  params):
	acc = pid_control(params.max_vel, state[3])
	max_steer = np.radians(30.0)                # [rad] max steering angle

	delta, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
	delta = np.clip(delta, -max_steer, max_steer)

	state[0] += state[3]* np.cos(state[2]) * dt
	state[1] += state[3]* np.sin(state[2]) * dt
	# state[2] += state[3]/ np.tan(delta) * dt
	state[2] +=   delta * dt
	state[3] += acc* dt


	if state[3] >= params.max_speed: state[3] = params.max_speed
	if state[3] <= params.min_speed: state[3] = params.min_speed

	if state[2] >= 2*math.pi: state[2] -= 2*math.pi
	if state[2] <= -2*math.pi: state[2] += 2*math.pi


	return state



def read_inputfile(num_agent=2, FILE_NAME="input4.txt"):
	# input file format #
	#---bounary--------
	#--obstacles--
	#--obstacles--
	#--initial postion of agents--
	#--initial goal position of agents--

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
			elif line_ctr ==num_lines-1:
				temp = list(ast.literal_eval(l))
				# num_agent = len(temp)
				# print(temp)
				# print("gent num:", len(temp))
				start_states =[]
				init_poses=[]
				for i in range(num_agent):
					init_poses.append([temp[i][0], temp[i][1]])
					start_states.append(np.array([init_poses[i][0],init_poses[i][1],0.0, 0.0,0.0]))
				# start_states = [temp[0],temp[1], temp[2], temp[3]]
				# init_poses = [[temp[0][0],temp[0][1]],[temp[1][0],temp[1][1]]]
				# print(init_poses )
				# goal_state = temp[1]

				print("init_poses", init_poses)
			else:
				temp = list(ast.literal_eval(l))
				num_agent = len(temp)
				goal_poses=[]
				for i in range(num_agent):
					goal_poses.append([temp[i][0], temp[i][1]])
				print("goal_poses", goal_poses)

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

	return start_states, init_poses, goal_poses, obstacles, walls


if __name__ == "__main__":

	# dir_path = os.path.dirname(os.path.realpath(__file__))
	parser = argparse.ArgumentParser()
	parser.add_argument("-in",help="input file (default: input2.txt)",default="input4.txt")
	parser.add_argument("-load",help="load saved data? [y/n] (default: n)",default="n")
	parser.add_argument("-animation",help="show animation? [y/n] (default: y)",default="y")
	parser.add_argument("-num",help="agnet number ? [2/3] (default: 2)",default="2")
	args = vars(parser.parse_args())

	#set the num_agent
	num_agent =int(args['num'])
	#load starting pose and environment from text file
	states, init_poses, goal_poses,obstacles, walls = read_inputfile(num_agent, args['in'])

	if args['animation']=="y":
		show_animation = True
	else:
		show_animation = False

	print("Simulation Setting")
	print("input file: ", args['in'])
	print("num of agents: ", num_agent)
	print("show animation: ",args['animation'] )

	params = Params()
	params_globalmap =  map_params()
	params_local_map=map_params()
	##
	localgrids=[]
	for i in range(num_agent):
		params_local_map=map_params()
		localgrids.append(params_local_map)

	params_localmap =  map_params()
	# params_localmap2 =  map_params()

	#simulation settings
	# initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
	# state = np.array([init_pos[0],init_pos[1],0.0, 0.0,0.0])

	# states=[]
	# states.append(np.array([init_poses[0][0],init_poses[0][1],0.0, 0.0,0.0]))
	# states.append(np.array([init_poses[1][0],init_poses[1][1],0.0, 0.0,0.0]))
	#temporary goal state
	# goal_poses=[[2.5, -5.0],[5.5, 8.0], [-4, -4]]                 #temporary goal pose
	goal =goal_poses[0]                        #define goal from goal_pos(initial direction)
	goal2 =goal_poses[1]                        #define goal from goal_pos(initial direction)
	goal3 =goal_poses[2]                        #define goal from goal_pos(initial direction)
	# state = np.array(start_state)
	traj = states[0][:2]
	traj2 = states[1][:2]
	iter=0
	simtime=0.0
	spline_explored = False

	# create obpoints
	obpoints=[]
	xyressol = 0.5
	for obs in obstacles:
		num_x = int(round((obs.x_max-obs.x_min)/(2*params_globalmap.xyreso)))+1
		num_y = int(round((obs.y_max-obs.y_min)/(2*params_globalmap.xyreso)))+1
		for i in range(num_x):
			for j in range(num_y):
				obspoint_x = obs.x_min+i*2*params_globalmap.xyreso
				obspoint_y = obs.y_min+j*2*params_globalmap.xyreso
				obpoints.append([obspoint_x, obspoint_y])
	# print("obspoints:", obpoints)
	#create cspace
	# print("init_poses", init_poses)
	# goal_poses=[[2.5, -5.0],[5.5, 8.0]]                 #temporary goal pose
	cspace=configuration_space()
	cspace.reset_environment(params_globalmap.boundaries,init_poses[0],goal_poses[0], obstacles)
	planner = VerticalCellDecomposition(cspace)
	planner.reset_cspace(cspace)
	planner.vertical_lines()
	planner.region_disection(goal_poses[0])

	cspace2=configuration_space()
	cspace2.reset_environment(params_globalmap.boundaries,init_poses[1],goal_poses[1], obstacles)
	planner2 = VerticalCellDecomposition(cspace2)
	planner2.reset_cspace(cspace2)
	planner2.vertical_lines()
	planner2.region_disection(goal_poses[1])



	# planner.generate_waypoint(params_localmap)
	# planner.plot_regions()
	# cspace.plot_config_space()
	# planner.construct_graph()
	# path, path_idx = planner.search(True, goal_pos)

	#Define four windows: 
	# axes[0,0] : robot, obstacle, goal_points, 
	# axes[1,0] : trajectories, region
	# axes[0,1] : local sensor_map
	# axes[1,1] : entropy_map
	# fig,axes=plt.subplots(nrows=2,ncols=2,figsize=(40,40))
	waypoint_vcd = planner.generate_waypoint(params_localmap)
	# waypoint_vcd2 = planner2.generate_waypoint(params_localmap2)
	

	#waypoint from vcd
	way_x=[]
	way_y=[]
	for point in waypoint_vcd:
		way_x.append(point[0])
		way_y.append(point[1])

	#plot figures 
	if show_animation:
		fig,axes=plt.subplots(nrows=2,ncols=2,figsize=(40,40))
		planner.plot_regions(axes[1,0])

		for init_pos in init_poses:
			axes[0,0].scatter(init_pos[0], init_pos[1], facecolor='blue',edgecolor='blue')      #initial point
			axes[0,0].plot(init_pos[0], init_pos[1], 'o', markersize = 20, fillstyle='none',color='black')             #trajectory point
		axes[1,0].plot(way_x, way_y, '*', markersize= 10, fillstyle='none',color='green')             #trajectory point
		# for i in range(len(waytimes)):
			# axes[0].text(way_x[i], way_y[i]-1,str(waytimes[i]), color='g')

		area_size=params.area_size+1
		axes[0,0].set_xlabel('x')
		axes[0,0].set_ylabel('y')
		axes[0,0].set_xlim([-area_size, area_size])   # limit the plot space
		axes[0,0].set_ylim([-area_size, area_size])   # limit the plot space
		axes[0,0].grid(True)

	#simulation settings
	'''
	# initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
	# state = np.array([init_pos[0],init_pos[1],0.0, 0.0,0.0])
	states=[]
	states.append(np.array([init_poses[0][0],init_poses[0][1],0.0, 0.0,0.0]))
	states.append(np.array([init_poses[1][0],init_poses[1][1],0.0, 0.0,0.0]))
	#temporary goal state
	goal =goal_poses[0]                        #define goal from goal_pos(initial direction)
	goal2 =goal_poses[1]                        #define goal from goal_pos(initial direction)
	traj = states[0][:2]
	traj2 = states[1][:2]
	iter=0
	simtime=0.0
	spline_explored = False
	'''

	#parameters for logging
	times =[]
	entropys=[]
	#pose_set environment

	poses_xset=[]
	poses_yset=[]
	goal_xset=[]
	goal_yset=[]
	yawset=[]
	velocities=[]
	inputsets=[]
	goals=[]
	
	
	#inputs= np.zeros((num_agent,2))
	inputs= []
	poses_xs=np.zeros(num_agent, dtype=float)
	poses_ys=np.zeros(num_agent, dtype=float)
	yaws=np.zeros(num_agent, dtype=float)
	goal_xs=np.zeros(num_agent, dtype=float)
	goal_ys=np.zeros(num_agent, dtype=float)
	velocity=np.zeros(num_agent, dtype=float)
	#velocity=np.zeros(num_agent)
	for i in range(num_agent):
		inputs.append([0.0, 0.0])


	#inputs= np.array([0.0, 0.0])
	#inputs2= np.array([0.0, 0.0])
	#inputs3= np.array([0.0, 0.0])

	#Checking initial and final goal
	print("initial state: ",states)
	# print("goal : ",goal)
	t_prev_goal = time.time()
	pmap_global = initialize_global_occ_grid_map(params_globalmap)
	# pmap_global_test = initialize_global_occ_grid_map(params_globalmap)
	# initial_entropy = get_map_entropy(pmap_global,params_globalmap)
	initial_entropy = get_map_entropy2(pmap_global,params_globalmap)
	print("initial entropy: ", initial_entropy )
	# input("stop here")
	#########################################
	#test stanley_controller
	# sample_goals = goal_sampling_VCD(waypoint_vcd, state[0],state[1], params_globalmap)
	# gtrjs= generating_globaltrjs(state, cspace,obstacles,sample_goals,params_globalmap)
	# sp_gtrjs = trjs_to_sample(gtrjs)            #sampled_trajectories

	# pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, params_localmap.xmin, params_localmap.xmax, params_localmap.ymin, params_localmap.ymax, params_localmap.xyreso, params_localmap.xw, params_localmap.yw
				# = generate_ray_casting_grid_map(obstacles, walls, params_localmap, states[0][0],states[0][1], states[0][2])

	# pmap_local2, updated_grids2, intersect_dic, obs_verticeid, closest_vertexid, params_localmap.xmin, params_localmap.xmax, params_localmap.ymin, params_localmap.ymax, params_localmap.xyreso, params_localmap.xw, params_localmap.yw
				# = generate_ray_casting_grid_map(obstacles, walls, params_localmap, states[0][0],states[0][1], states[0][2])
	# pmap_global = update_occ_grid_map(states[0], pmap_local,params_localmap, pmap_global,params_globalmap)
	# pmap_global = update_occ_grid_map(states[1], pmap_local2,params_localmap, pmap_global,params_globalmap)
	# entropymap = get_global_entropymap(pmap_global,params_globalmap)
	# local_trjs = lane_state_sampling_test1(states[0],obstacles, params_globalmap)
	# local_trjs2 = lane_state_sampling_test1(states[1],obstacles, params_globalmap)
	# trjs_candidate =[]
	# trjs_candidate2 =[]
	# for ltrj in local_trjs:
		# trjs_candidate.append(ltrj)

	# for ltrj in local_trjs2:
		# trjs_candidate2.append(ltrj)

	# best_trj = calc_IG_trjs(trjs_candidate, entropymap , params_localmap, params_globalmap, params)
	# best_trj2 = calc_IG_trjs(trjs_candidate2, entropymap , params_localmap, params_globalmap, params)
	# cx, cy, cyaw, ck, s = calc_spline_course_trj(best_trj, ds=0.1)
	# cx2, cy2, cyaw2, ck2, s2 = calc_spline_course_trj(best_trj2, ds=0.1)
	# trj_last_idx = len(cx) - 1
	# trj_last_idx2 = len(cx2) - 1
	# trj_target_idx, _ = calc_target_index(states[0], cx, cy)
	# trj_target_idx2, _ = calc_target_index(states[1], cx2, cy2)
	#########################################
	goal_dist = np.zeros(num_agent)


	for _ in range(params.numiters):

		poses_xs=np.zeros(num_agent, dtype=float)
		poses_ys=np.zeros(num_agent, dtype=float)
		for i in range(num_agent):
			states[i], inputs[i] = motion_dwa(states[i], inputs[i], goal_poses[i], obpoints, walls, params)
			goal_dist[i]=distance(goal,states[i]) 
			print("states[", i,"]", states[i])  

			# poses_xset[i].append(states[i][0])
			# poses_yset[i].append(states[i][1])
			poses_xs[i]=states[i][0]
			poses_ys[i]=states[i][1]
			yaws[i]=states[i][2]
			velocity[i]=states[i][3]
			# print("states", states[i])

		#states[1], inputs2 = motion_dwa(states[1], inputs2, goal2, obpoints, walls, params)
		# state = simple_motion(state, goal, params)                        #dynamics
		# state = motion_sc(state, cx,cy,cyaw, trj_target_idx,  params)
		#logs
		simtime = simtime + dt
		times.append(simtime)
		t_current = time.time()
		#poses_xset
		poses_xset.append(poses_xs)
		poses_yset.append(poses_ys)
		yawset.append(yaws)
		velocities.append(velocity)
		#pos_xs.append(states[0][0])
		#pos_ys.append(states[0][1])
		#pos_xs2.append(states[1][0])
		#pos_ys2.append(states[1][1])
		# pos_xs.append(state[0])
		# pos_ys.append(state[1])
		# yaws.append(states[0][2])
		# velocities.append(states[0][3])

		if iter>0:
			for i in range(num_agent):
				goal_xs[i]=goal_poses[i][0]
				goal_ys[i]=goal_poses[i][1]

			goal_xset.append(goal_xs)
			goal_yset.append(goal_ys)
 

		#check if one of the agent reaches to to it's goal
		goal_reached = False
		for i in range(num_agent):
			if goal_dist[i]< params.goal_tol:
				goal_reached =True

		# if goal is reached, start cacluating new positions
		sample_goalset=[];
		trjs_candidateset=[];
		best_trjs=[]
		goals=[]

		if goal_reached :                                          # goal is reached
			print('Time from the previous reached goal:', t_current - t_prev_goal)
			t_prev_goal = time.time()

			#setting paths--> start finding best trajectories
			if show_animation:
				axes[1,0].cla()
				axes[1,0].set_title('global & Local motion primitives')
				planner.plot_regions(axes[1,0])
			# sample_goals = random_sampling(params,8)
			# generate goal points from waypoints vcd
			# sample_goals = random_sampling(params,5)
			best_trjs=[]
			for i in range(num_agent):
				sample_goalset.append(goal_sampling_VCD(waypoint_vcd, states[i][0],states[i][1], params_globalmap))
			# sample_goals2 = goal_sampling_VCD(waypoint_vcd, states[1][0],states[1][1], params_globalmap)
			# sample_goals_total=[sample_goals,sample_goals2]
			# generate global trjs to each sample goal
			#agent1
			# input("stop")
		
				gtrjs= generating_globaltrjs(states[i], cspace,planner, obstacles,sample_goalset[i],params_globalmap)
				sp_gtrjs = trjs_to_sample(gtrjs)
				local_trjs = lane_state_sampling_test1(states[i],obstacles, params_globalmap)
				trjs_candidate =[]
				for gtrj in sp_gtrjs:
					trjs_candidate.append(gtrj)
				for ltrj in local_trjs:
					trjs_candidate.append(ltrj)

				trjs_candidateset.append(trjs_candidate)

				# if i==1:
				best_trj = calc_IG_trjs_hierarchy(trjs_candidateset[i], pmap_global,  params_globalmap, best_trjs, i, horizon )
				best_trjs.append(best_trj)
				# else:
					# best_trj = calc_IG_trjs(trjs_candidateset[i], entropymap,  params_globalmap, horizon )


			#agent2
			# gtrjs2= generating_globaltrjs(states[1], cspace2,planner2, obstacles,sample_goals2,params_globalmap)
			# sp_gtrjs2 = trjs_to_sample(gtrjs2)
			# local_trjs2 = lane_state_sampling_test1(states[1],obstacles, params_globalmap)
			# trjs_candidate2 =[]
			# for gtrj in sp_gtrjs2:
				# trjs_candidate2.append(gtrj)
			# for ltrj in local_trjs2:
				# trjs_candidate2.append(ltrj)

			#Obtain best_trjaectory
			# best_trj = calc_IG_trjs(trjs_candidate, entropymap,  params_globalmap, horizon )
			# best_trj=self.calc_IG_trjs(trjs_candidate, self.params_searchmap, self.horizon)
			# best_trj = calc_IG_trjs(trjs_candidate, best_trj2, entropymap, params_localmap, params_globalmap, params,horizon )
			# best_trj2 = calc_IG_trjs(trjs_candidate2, entropymap , params_localmap2, params_globalmap, params,horizon )
			# best_trj2 = calc_IG_trjs_with_leadtrj(trjs_candidate2, best_trj, entropymap, params_globalmap, params)
			# best_trj2 = calc_IG_trjs_wfollower(trjs_candidate2, best_trj, entropymap, params_localmap, params_localmap2, params_globalmap, params)

			# cx, cy, cyaw, ck, s = calc_spline_course_trj(best_trj, ds=0.2)
			# last_idx = len(cx) - 1
			# trj_target_idx, _ = calc_target_index(state, cx, cy)

			#Choose next goal point from trajectory
				goal = choose_goal_from_trj(best_trj, params_globalmap, horizon)

				goal_xs[i]=goal[0]
				goal_ys[i]=goal[1]
				goal_poses[i]=[goal[0],goal[1]]

				if show_animation:
					plot_local_trjs(local_trjs, axes[1,0])
					plot_global_trjs(sp_gtrjs, axes[1,0])
					plot_best_trj(best_trj, horizon,axes[1,0])

			goal_xset.append(goal_xs)
			goal_yset.append(goal_ys)
			# goal2 = choose_goal_from_trj(best_trj2, params_globalmap, horizon)

			
				# plot_local_trjs(local_trjs2, axes[1,0])
				# plot_global_trjs(sp_gtrjs2, axes[1,0])
				# plot_best_trj(best_trj2, horizon,axes[1,0])


			# pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, params_localmap.xmin, params_localmap.xmax, params_localmap.ymin, params_localmap.ymax, params_localmap.xyreso, params_localmap.xw, params_localmap.yw= generate_ray_casting_grid_map(obstacles, walls, params_localmap, states[0][0],states[0][1], states[0][2])
			# pmap_local2, updated_grids2, intersect_dic2, obs_verticeid2, closest_vertexid, params_localmap2.xmin, params_localmap2.xmax, params_localmap2.ymin, params_localmap2.ymax, params_localmap2.xyreso, params_localmap2.xw, params_localmap2.yw= generate_ray_casting_grid_map(obstacles, walls, params_localmap, states[1][0],states[1][1], states[1][2])
			# pmap_global = update_occ_grid_map(states[0], pmap_local,params_localmap, pmap_global,params_globalmap)
			# pmap_global = update_occ_grid_map(states[1], pmap_local2,params_localmap2, pmap_global,params_globalmap)
			# entropymap = get_global_entropymap(pmap_global,params_globalmap)
			# curentropy = get_map_entropy(pmap_global, params_globalmap)

		# print("states", states)
		# print("states[0][0]", states[0][0])
		# print("states[0][2]", states[0][2])
		#sensing part
		for i in range(num_agent):
			pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, localmap= generate_ray_casting_grid_map(obstacles, walls, params_localmap, states[i][0],states[i][1], states[i][2])
			pmap_global = update_occ_grid_map(states[i], pmap_local, localmap, pmap_global,params_globalmap)
			# pmap_local2, updated_grids2, intersect_dic2, obs_verticeid2, closest_vertexid, localmap2= generate_ray_casting_grid_map(obstacles, walls, params_localmap, states[1][0],states[1][1], states[1][2])
			# pmap_local2, updated_grids2, intersect_dic2, obs_verticeid2, closest_vertexid, params_localmap2.xmin, params_localmap2.xmax, params_localmap2.ymin, params_localmap2.ymax, params_localmap2.xyreso, params_localmap2.xw, params_localmap2.yw= generate_ray_casting_grid_map(obstacles, walls, params_localmap, states[1][0],states[1][1], states[1][2])

		# pmap_global = update_occ_grid_map(states[1], pmap_local2,localmap2, pmap_global,params_globalmap)

		# entropymap = get_global_entropymap(pmap_global,params_globalmap)
		curentropy = get_map_entropy2(pmap_global, params_globalmap)
		entropys.append(curentropy)

		#plot
		if show_animation:
			axes[0,0].cla()
			axes[0,0].scatter(goal[0], goal[1], facecolor='red',edgecolor='red')
			axes[0,0].scatter(goal2[0], goal2[1], facecolor='green',edgecolor='green')
			traj = np.vstack([traj, states[0][:2]])

			traj2 = np.vstack([traj2, states[1][:2]])
			visualize([traj, traj2], states, obstacles, walls, params)

			#figure2- local sensor window
			axes[0,1].cla()
			axes[0,1].set_title('local sesnor grid')
			# pmap_local, updated_grids, intersect_dic, obs_verticeid, closest_vertexid, params_localmap.xmin, params_localmap.xmax, params_localmap.ymin, params_localmap.ymax, params_localmap.xyreso, params_localmap.xw, params_localmap.yw= generate_ray_casting_grid_map(obstacles, walls, params_localmap, state[0],state[1], state[2])
			draw_occmap(pmap_local, params_localmap, params_globalmap, states[0][0],states[0][1], axes[0,1])
			# draw_occmap(pmap_local2, params_localmap, params_globalmap, states[1][0],states[1][1], axes[0,1])

			# axes[0,1].cla()
			# axes[0,1].set_title('local sesnor grid')
			# draw_occmap(pmap_local2, params_localmap, params_globalmap, states[1][0],states[1][1], axes[0,1])
			
			#figure3- global occupancy grid
			axes[1,1].cla()
			axes[1,1].set_title('global occupancy grid')
			# pmap_global = update_occ_grid_map(state, pmap_local,params_localmap, pmap_global,params_globalmap)
			entropymap = get_global_entropymap(pmap_global,params_globalmap)
			draw_entropymap_global(entropymap,params_globalmap, axes[1,1])
			# curentropy = get_map_entropy(pmap_global, params_globalmap)
			# entropys.append(curentropy)
			# entropy_log[simtime]=curentropy
			# print(entropy_log)
			# print("----entropy : ", entropy)
			plt.pause(0.001)
			# plt.show()

		iter+=1

		#middle frequency
		# if iter%20==1:
			
		if iter==1: #FixMe: it was 1

			if show_animation:
				axes[1,0].cla()
				axes[1,0].set_title('global & Local motion primitives')
				planner.plot_regions(axes[1,0])


			best_trjs=[]        
			for i in range(num_agent):
				sample_goalset.append(goal_sampling_VCD(waypoint_vcd, states[i][0],states[i][1], params_globalmap))
				# sample_goalset[i]= goal_sampling_VCD(waypoint_vcd, states[i][0],states[i][1], params_globalmap)

				gtrjs= generating_globaltrjs(states[i], cspace,planner, obstacles,sample_goalset[i],params_globalmap)
				sp_gtrjs = trjs_to_sample(gtrjs)
				local_trjs = lane_state_sampling_test1(states[i],obstacles, params_globalmap)
				trjs_candidate =[]
				for gtrj in sp_gtrjs:
					trjs_candidate.append(gtrj)
				for ltrj in local_trjs:
					trjs_candidate.append(ltrj)

				trjs_candidateset.append(trjs_candidate)

				best_trj = calc_IG_trjs_hierarchy(trjs_candidateset[i], pmap_global,  params_globalmap, best_trjs, i, horizon )
				goal = choose_goal_from_trj(best_trj, params_globalmap, horizon)
				# goals.append(goal)
				goal_xs[i]=goal[0]
				goal_ys[i]=goal[1]
				goal_poses[i]=[goal[0],goal[1]]


			# sample_goals = goal_sampling_VCD(waypoint_vcd, states[0][0],states[0][1], params_globalmap)
			# sample_goals2 = goal_sampling_VCD(waypoint_vcd, states[1][0],states[1][1], params_globalmap)

			#agent1
			# gtrjs= generating_globaltrjs(states[0], cspace,planner, obstacles,sample_goals,params_globalmap)
			# sp_gtrjs = trjs_to_sample(gtrjs)            #sampled_trajectories
			# local_trjs = lane_state_sampling_test1(states[0],obstacles, params_globalmap)
			# trjs_candidate =[]
			# for gtrj in sp_gtrjs:
				# trjs_candidate.append(gtrj)
			# for ltrj in local_trjs:
				# trjs_candidate.append(ltrj)

			# best_trj = calc_IG_trjs(trjs_candidate, entropymap , params_globalmap)

			#agent2
			# gtrjs2= generating_globaltrjs(states[1], cspace2,planner2, obstacles,sample_goals2,params_globalmap)
			# sp_gtrjs2 = trjs_to_sample(gtrjs2)            #sampled_trajectories
			# local_trjs2 = lane_state_sampling_test1(states[1],obstacles, params_globalmap)
			# trjs_candidate2 =[]
			# for gtrj in sp_gtrjs2:
				# trjs_candidate2.append(gtrj)
			# for ltrj in local_trjs2:
				# trjs_candidate2.append(ltrj)

			# best_trj2 = calc_IG_trjs_with_leadtrj(trjs_candidate2, entropymap, best_trj, params_globalmap)
			# goal = choose_goal_from_trj(best_trj, params_globalmap, horizon)
			# goal2 = choose_goal_from_trj(best_trj2, params_globalmap, horizon)

			# goal_xs.append(goal[0])
			# goal_ys.append(goal[1])
			# goal_xs2.append(goal2[0])
			# goal_ys2.append(goal2[1])

				if show_animation:
					plot_local_trjs(local_trjs, axes[1,0])
					plot_global_trjs(sp_gtrjs, axes[1,0])
					plot_best_trj(best_trj, horizon, axes[1,0])
					axes[0,0].scatter(goal[0],goal[1], facecolor='red',edgecolor='red')
					# plot_local_trjs(local_trjs2, axes[1,0])
					# plot_global_trjs(sp_gtrjs2, axes[1,0])
					# plot_best_trj(best_trj2, horizon, axes[1,0])
					# axes[0,0].scatter(goal2[0],goal2[1], facecolor='green',edgecolor='green')
			
			goal_xset.append(goal_xs)
			goal_yset.append(goal_ys)

		if curentropy < 0.4*initial_entropy:
			horizon = 30
			params.weight_entropy=0.05

		if curentropy < 0.35*initial_entropy and boolsaved==False:
			# data=[times, pos_xs,pos_ys,yaws,velocities, pos_xs2, pos_ys2, entropys, goal_xs, goal_ys, goal_xs2, goal_ys2]
			# data=[times, entropys]
			# for i in range(num_agent):
				# data.append(poses_xset[i])
				# data.append(poses_yset[i])
				# data.append(goal_xset[i])
				# data.append(goal_yset[i])
			# print("data", data) 
			data=np.array([times, entropys, poses_xset, poses_yset, goal_xset, goal_yset])
			# data = np.transpose(data)
			# print("data", data)
			columns=['time', 'entropy', 'pos_x', 'pos_y', 'goal_x', 'goal_y']
			pd.DataFrame(data, columns).to_csv(file_name,float_format='%2f', header=True)
			# pd.DataFrame(data, columns=['time', 'pos_x', 'pos_y', 'yaw', 'velocity', 'pos_xx', 'pos_yy', 'entropy', 'goal_x', 'goal_y', 'goal_xx', 'goal_yy']).to_csv(file_name,header=True)
			print("entropy file saved")
			boolsaved =True
			input("done")
		if params.numiters>2 and params.numiters%300==1:
			data=[times, pos_xs,pos_ys,yaws,velocities, pos_xs2, pos_ys2, entropys, goal_xs, goal_ys, goal_xs2, goal_ys2]
			data = np.transpose(data)
			pd.DataFrame(data, columns=['time', 'pos_x', 'pos_y', 'yaw', 'velocity', 'pos_xx', 'pos_yy', 'entropy', 'goal_x', 'goal_y', 'goal_xx', 'goal_yy']).to_csv(file_name,header=True)
			print("entropy file temporary saved")
			boolsaved =True
			input("done")

		print("cur entropy: ", curentropy)
		print("exploration rate: ", float(curentropy/initial_entropy))

	plt.show()
	# plt.show(aspect='auto')






