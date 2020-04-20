"""
Ray casting 2D grid map example
author: Atsushi Sakai (@Atsushi_twi)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

EXTEND_AREA = 15.0
show_animation = True

def polygon_contains_point(point, obstacle):

    point = Point(point[0], point[1])
    obstacle_pts=[]
    #Here obstacle has four points
    obstacle_pts.append([obstacle.x_min,obstacle.y_min])
    obstacle_pts.append([obstacle.x_min,obstacle.y_max])
    obstacle_pts.append([obstacle.x_max,obstacle.y_min])
    obstacle_pts.append([obstacle.x_max,obstacle.y_max])

    polygon = Polygon(obstacle_pts)
    return polygon.contains(point)

def finding_intersects(agent_x,agent_y, obstacle,yawreso):
    #TODo: points from polygon
    print("hi")




def check_polygon_visible(min_angle,max_angle,fov_anglelist):
    #min_angle, max_angle, fov_anglelist are in range between (0,2pi)
    if max_angle>min_angle:
        if min_angle>fov_anglelist[0] and max_angle<fov_anglelist[1]:
            print("case-complete visible")
        elif min_angle>fov_anglelist[0] and min_angle>fov_anglelist[1]:
            print("case-3-un-visible")
        elif min_angle<fov_anglelist[0] and max_angle<fov_anglelist[0]:
            print("case-4-un-visible")
        elif min_angle>fov_anglelist[1] and max_angle>fov_anglelist[1]:
            print("case-5-un-visible")
        elif min_angle>fov_anglelist[0] and min_angle<fov_anglelist[1] and max_angle>fov_anglelist[1]:
            print("case-16-partially-visible")
            input()
        elif max_angle>fov_anglelist[0] and max_angle>fov_anglelist[1]:
            print("case-6-un-visible")
        elif min_angle<fov_anglelist[0] and max_angle<fov_anglelist[1]:
            print("case-2-partially visible")
            input()
        elif min_angle>fov_anglelist[0] and max_angle>fov_anglelist[1]:
            print("case-1-partially visible")
            input()
        else:
            print("case-10-un-visible")
    else:
        #max angle is smaller than min angle 
        if max_angle<fov_anglelist[1] and min_angle>fov_anglelist[0] and fov_anglelist[0]>fov_anglelist[1]:
            print("case-10-complete visible")
            input()
        elif max_angle<fov_anglelist[1] and min_angle<fov_anglelist[0] and fov_anglelist[0]>fov_anglelist[1]:
            print("case-11-partially visible")
            input()
        elif max_angle<fov_anglelist[0] and max_angle<fov_anglelist[1]:
            print("case-12-un-visible")
        elif min_angle>fov_anglelist[0] and min_angle>fov_anglelist[1] and fov_anglelist[0]>math.pi:
            print("case-13-un-visible")
        elif max_angle<fov_anglelist[1] and min_angle<fov_anglelist[0]:
            print("case-14-partially visible")
            input()
        elif max_angle>fov_anglelist[1] and min_angle>fov_anglelist[0]:
            print("case-15-partially visible")
            input()
        else:
            print("case-16-un-visible")
    # input()






def calc_grid_map_config(xyreso, agent_x,agent_y):
    sensor_range=10
    minx = agent_x - sensor_range
    maxx = agent_x + sensor_range
    miny = agent_y - sensor_range
    maxy = agent_y + sensor_range
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))

    return minx, miny, maxx, maxy, xw, yw

class precastDB:

    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.ix = 0
        self.iy = 0
        self.value=0.0

    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle)


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0

    return angle

def get_raycast_to_line(origin,obstacle, obs_points,yawreso):
    #obs points = 2 or 3 points (min_angle, max_angle, nn_anlge)
    #ouput = dictonary ={anlge: intersecton pointt}
    #check if nn_angle is same as the first or second elment 
    #if they are same, we can chek only one points

    if obs_points[0] == obs_points[2]:
        point3_x = None
        point3_y = None

    elif obs_points[1] == obs_points[2]:

        point3_x = None
        point3_y = None
    else:

        point3_x = obstacle.vertices[obs_points[2]][0]
        point3_y = obstacle.vertices[obs_points[2]][1]

    origin_x = origin[0]
    origin_y = origin[1]

    point1_x = obstacle.vertices[obs_points[0]][0]
    point1_y = obstacle.vertices[obs_points[0]][1]
    point2_x = obstacle.vertices[obs_points[1]][0]
    point2_y = obstacle.vertices[obs_points[1]][1]


    angle_pt = atan_zero_to_twopi(point1_y-origin_y,point1_x-origin_x)
    angle_pt2 = atan_zero_to_twopi(point2_y-origin_y,point2_x-origin_x)

    if angle_pt2>angle_pt:
        min_angle = angle_pt
        max_angle = angle_pt2

    else:
        min_angle = angle_pt2
        max_algle = angle_pt

        point1_x = obstacle.vertices[obs_points[1]][0]
        point1_y = obstacle.vertices[obs_points[1]][1]
        point2_x = obstacle.vertices[obs_points[0]][0]
        point2_y = obstacle.vertices[obs_points[0]][1]


    angle_intersect = dict()

    if point3_x == None:
        num_angle =math.floor((max_angle-min_angle)/yawreso)
        for i in range(num_angle):
            cur_angle = min_angle + i*yawreso
            slope = math.tan(cur_angle)
            if abs(slope)==0:
                print("slope is zero")
                return []

            intersects=[]
            #intersects
            if point1_x == point2_x: #vertical
                temp_y =slope*(point1_x-origin_x)+origin_y
                intersects =[point1_x,temp_y]
            elif point1_y == point2_y:#horizontal
                temp_x = 1/slope*(point1_y+slope*origin_x+origin_y)
                intersects =[temp_x,point1_y]
            else:
                print("obs is not rectangle")

            angle_intersect[cur_angle]=intersects
    else:
        # print("point3", point3_x , " , ", point3_y)
        
        angle_nn = atan_zero_to_twopi(point3_y-origin_y,point3_x-origin_x)

        num_angles=[]
        num_angle1 =math.floor((angle_nn-min_angle)/yawreso)
        num_angle2 =math.floor((max_angle-angle_nn)/yawreso)
        num_angles.append(num_angle1)
        num_angles.append(num_angle2)
    
        #change nextpoint
        previous_x  = point1_x
        previous_y  = point1_y
        next_x = obstacle.vertices[obs_points[2]][0]
        next_y = obstacle.vertices[obs_points[2]][1]

        small_anlge = min_angle 
        intersects=[]
        for num_angle in num_angles:
            print("======num_angles", num_angle)
            for i in range(num_angle):
                print("iiii-----", i)
                cur_angle = small_anlge + i*yawreso
                slope = math.tan(cur_angle)
                if abs(slope)==0:
                    print("slope is zero")
                    return []
            
                #intersects
                if previous_x == next_x: #vertical
                    print("vertical")
                    temp_y =slope*(previous_x-origin_x)+origin_y
                    intersects =[previous_x,temp_y]
                elif previous_y == next_y:#horizontal
                    print("horizontal")
                    temp_x = 1/slope*(previous_y+slope*origin_x+origin_y)
                    intersects =[temp_x,previous_y]
                else:
                    print("obs is not rectangle")
                    # print("previous_x:" , previous_x)
                    # print("previous_y:" , previous_y)
                    # print("next_x:" , next_x)
                    # print("next_y:" , next_y)
                if i ==num_angle-1:
                    small_angle = angle_nn 
                    previous_x = obstacle.vertices[obs_points[2]][0]
                    previous_y = obstacle.vertices[obs_points[2]][1]
                    next_x = point2_x
                    next_y = point2_y

                angle_intersect[cur_angle]=intersects

    print("angle_dictionaries")
    print(angle_intersect)

    return angle_intersect



def precasting(minx, miny, xw, yw, xyreso, yawreso,agent_x,agent_y):

    precast = [[] for i in range(int(round((math.pi * 2.0) / yawreso)) + 1)]
    num_angleid = int(round((math.pi * 2.0) / yawreso))
    # print("num_angleid"+str(num_angleid))
    # print(precast)

    for ix in range(xw):
        for iy in range(yw):
            #get x,y coordinates
            px = ix * xyreso + minx
            py = iy * xyreso + miny

            #distance from robot(origin)
            d = math.sqrt((px-agent_x)**2 + (py-agent_y)**2)
            angle = atan_zero_to_twopi((py-agent_y), (px-agent_x))
            angleid = int(math.floor(angle / yawreso))

            pc = precastDB()

            pc.px = px
            pc.py = py
            pc.d = d
            pc.ix = ix
            pc.iy = iy
            pc.angle = angle
            pc.occ =0.0

            precast[angleid].append(pc)

    return precast

def generate_ray_casting_grid_map(obstacles,xyreso, yawreso, agent_x=0.0, agent_y=0.0,  agent_yaw=0.0):


    updated_gridlist=[]
    minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(xyreso, agent_x, agent_y )
    #make grid map with xw, yw (initialize)
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]
    
    precast = precasting(minx, miny, xw, yw, xyreso, yawreso, agent_x, agent_y)

    print("agent_yaw: ", agent_yaw)
    fov_type, fov_anglelist = generate_fov_angle(agent_yaw, 2*math.pi)
    # fov_type, fov_anglelist = generate_fov_angle(agent_yaw, math.pi/3)
    print(fov_anglelist)
    print("----------------")
    #fov_anglelist[0] = min_angle 315
    #fov_anglelist[1] = max_angle 45

    '''
    # generate fov grid
    num_angleid = int(round((math.pi * 2.0) / yawreso))
    for angleid in range(num_angleid):
        angle = angleid*yawreso
        #fov_type means the angle range 
        if fov_type==0:
            if angle <= fov_anglelist[1] and angle >=fov_anglelist[0]:
                # print(str(angleid)+" , "+str(angle))
                angleidx = int(math.floor(angle / yawreso))
                gridlist = precast[angleid]
                #case for in fov
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 0.0
            elif angle >=fov_anglelist[0] and fov_anglelist[0]>math.pi and fov_anglelist[1] <math.pi:
                #This case is for when min angle is negative(positive over pi) and max is small positive: <
                angleidx = int(math.floor(angle / yawreso))
                gridlist = precast[angleid]
                #case for in fov
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 0.0
            elif angle <= math.pi and angle <=fov_anglelist[1] and fov_anglelist[0]>math.pi:
                angleidx = int(math.floor(angle / yawreso))
                gridlist = precast[angleid]
                # case for in fov
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 0.0

            else:
                #case for out of fov
                # print("angle-inelse", angle)
                gridlist = precast[angleid]
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 1.0
        else:
            if angle <= fov_anglelist[0] or angle >=fov_anglelist[1]:
                # print(str(angleid)+" , "+str(angle))
                angleidx = int(math.floor(angle / yawreso))
                gridlist = precast[angleid]
                #case for in fov
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 0.0
            else:
                #case for out of fov
                gridlist = precast[angleid]
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 1.0

            '''

   #managing obstacles_mk
    sensor_pose=[agent_x,agent_y]
    for obs in obstacles:
        if polygon_contains_point(sensor_pose, obs)==False:  #check if sensor is in side obstacles
            max_angle = -100
            min_angle = +100
            for i in range(len(obs.vertices)):
                obs_angle = math.atan2(obs.vertices[i][1]-sensor_pose[1],obs.vertices[i][0]-sensor_pose[0])
                # obs_angle-=agent_yaw
                if obs_angle>max_angle:
                    max_angle = obs_angle
                if obs_angle<min_angle:
                    min_angle = obs_angle

            if min_angle <0 and max_angle <0:
                min_angle+=2*math.pi
                max_angle+=2*math.pi

            if min_angle<-math.pi/2 and max_angle >math.pi/2:
                min_angle+=2*math.pi
                tmp = min_angle
                min_angle = max_angle
                max_angle = tmp
            if max_angle>0 and max_angle<math.pi/2 and min_angle<0:
                #min angle will be larger than max angle for this case
                min_angle+=2*math.pi
            
            # if obs_angle<0:
                # obs_angle+=2*math.pi
        else:
            print("Error! - agent is in an obstacle region")
            break

        ##print for debug ##
        # print("----obstacle angle check ---")
        # print("min angle: ", min_angle)
        # print("max angle: ", max_angle)
        # print("fov_min: ", fov_anglelist[0])
        # print("fov_max: ", fov_anglelist[1])
        # print("agent_yaw: ", agent_yaw)
        ##print for debug ##
        #check if obs in fov i
        # check_polygon_visible(min_angle,max_angle,fov_anglelist)


    #obtain min/max vertices of obstacles/ 
    obs_angles=[]
    obs_vertices=[] 
    closest_vertices=[]
    iterator =0

    for obs in obstacles:
        if polygon_contains_point(sensor_pose, obs)==False:  #check if sensor is in side obstacles
            min_dist = 100
            max_angle = -100
            min_angle = +100
            max_id=0
            min_id=0
            nn_id=0
            for i in range(len(obs.vertices)):
                obs_angle = atan_zero_to_twopi(obs.vertices[i][1]-sensor_pose[1],obs.vertices[i][0]-sensor_pose[0])
                dist_to_agent = math.sqrt((obs.vertices[i][0]-sensor_pose[0]) **2 + (obs.vertices[i][1]-sensor_pose[1]) **2 )

                if obs_angle>max_angle:
                    max_angle = obs_angle
                    max_id=i
                if obs_angle<min_angle:
                    min_angle = obs_angle
                    min_id=i
                if dist_to_agent<min_dist:
                    min_dist = dist_to_agent
                    nn_id = i
                    nn_angle = obs_angle

            obs_vertices.append([min_id, max_id, nn_id])
            obs_angles.append([min_angle, max_angle, nn_angle])
            closest_vertices.append(nn_id)

            #get anlge and intersects from obstacles
            obsdict = get_raycast_to_line([agent_x,agent_y],obs, obs_vertices[iterator],yawreso)
    
            for angle,pt in obsdict.items():
                angleid =  int(math.floor(angle/ yawreso))
                gridlist = precast[angleid]
                ix = int((pt[0]- minx) / xyreso)
                iy = int((pt[1] - miny) / xyreso)

                d= math.sqrt((sensor_pose[0]-pt[0])**2 + (sensor_pose[1]-pt[1])**2)
                min_d = 100
                for grid in gridlist:
                    if grid.d > d:
                        pmap[grid.ix][grid.iy] = 0.5
                        grid.value=0.5
                        updated_gridlist.append(grid)
                        if min_d>grid.d:
                            min_d = grid.d
                            min_grid = grid
                pmap[min_grid.ix][min_grid.iy]=1.0
                # print("obstacle boundary:(x,y) : ",  min_grid.px, ", ", min_grid.py)
                min_grid.value=1.0
                updated_gridlist.append(grid)

            iterator+=1

    return pmap, updated_gridlist, obs_vertices, closest_vertices, minx, maxx, miny, maxy, xyreso

#shoulde be written in terms of radian
def generate_fov_angle(agent_yaw, fov_range):
    fov_type=0
    # fov_type=1
    min_angle = agent_yaw-fov_range/2.0
    max_angle = agent_yaw+fov_range/2.0
    fov_angles=[]

    if max_angle >2*math.pi:
        fov_type=1
        max_angle-=2*math.pi
        temp = min_angle
        min_angle = max_angle
        max_angle - min_angle
    if min_angle<0:
        min_angle+=2*math.pi
    
    if max_angle<0:
        max_angle+=2*math.pi
    #set value in a range (0, 2pi)
        
    fov_angles.append(min_angle)
    fov_angles.append(max_angle)

    return fov_type, fov_angles
        

def draw_heatmap(data, minx, maxx, miny, maxy, xyreso):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    plt.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def main():
    print(__file__ + " start!!")

    obstacles=[]
    xyreso = 0.25  # x-y grid resolution [m]
    yawreso = math.radians(5)  # yaw angle resolution [rad]
    agent_yaw= math.pi/2
    num_obs=3

    # for i in range(num_obs):
        # ox = [-2.02]
        # oy = [0.023]
        # ox = (np.random.rand(num_obs) - 0.5) * 10.0
        # oy = (np.random.rand(num_obs) - 0.5) * 10.0
        # print("ox")
        # print(ox)
        # print("oy")
        # print(oy)
    pmap, minx, maxx, miny, maxy, xyreso = generate_ray_casting_grid_map(obstacles,
             xyreso, yawreso, 0.0,0.0,agent_yaw)
    if show_animation:
        plt.cla()
        draw_heatmap(pmap, minx, maxx, miny, maxy, xyreso)
        plt.plot(ox, oy, "xr")
        plt.plot(0.0, 0.0, "ob")
        plt.pause(1.0)
    input("enter to continue")


if __name__ == '__main__':
    main()
