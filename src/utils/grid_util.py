"""
Grid util

Author: Minkyu Kim

"""
import math
import numpy as np
import bisect
import random
from math import *

l_occ=np.log(0.95/0.05)
l_free=np.log(0.05/0.95)
l_unknown=0.0

def Coord2CellIdx_global(x, y, params_searchmap):
    #change coordinates w.r.t global frame
    temp_x = x-params_searchmap.xmin
    temp_y = y-params_searchmap.ymin
    coord_x= math.floor(temp_x/params_searchmap.xyreso)
    coord_y= math.floor(temp_y/params_searchmap.xyreso)
    idx= (coord_x+params_searchmap.xw*coord_y)
    # print("xw: ", params_searchmap.xw,",  coord_x:" ,coord_x, "coord_y:" ,coord_y, ", ---idx: ", idx)

    return idx



def distance_point2D(pt1,pt2):
    '''Returns distance between two points.'''
    return ((pt1.x-pt2.x)**2+(pt1.y-pt2.y)**2)**0.5



class Point2D:
    def __init__(self):
        self.x=0.0
        self.y=0.0

class Frontier:
    def __init__(self):
        self.size = 1
        self.legth = 10.0/0.25
        self.min_distance=100.0
        self.cost=0.0
        self.points=[]
        self.centroid_x=0.0
        self.centroid_y=0.0
                
def Idx2Pose(idx, params_map):
    res = (int) (idx%params_map.xw);
    div = (int) (idx/params_map.xw);
    coord_x=(res+0.5)*params_map.xyreso+params_map.xmin;
    coord_y=(div+0.5)*params_map.xyreso+params_map.ymin;

    return coord_x, coord_y

def Idx2cellIds(idx, params_map):
    res = (int) (idx%params_map.xw);
    div = (int) (idx/params_map.yw);

    return res, div



def pose2Idx(x, y, params_searchmap):
    #change coordinates w.r.t global frame
    temp_x = x-params_searchmap.xmin
    temp_y = y-params_searchmap.ymin
    coord_x= math.floor(temp_x/params_searchmap.xyreso)
    coord_y= math.floor(temp_y/params_searchmap.xyreso)
    # print("coord_x:" ,coord_x, "coord_y:" ,coord_y )
    idx= (int)(coord_x+params_searchmap.xw*coord_y)

    return idx


def nearestCellIdx(start, val, gridmap, map_info):

    size_x_=map_info.xw
    size_y_=map_info.yw
    if start>(size_x_*size_y_ -1):
        print("Evaluating nhood for offmap point")
        return -1
    map_size=size_x_*size_y_ 
    visited = [False] * (map_size)
   # Create a queue for BFS
    queue = []

    queue.append(start)
    visited[start] = True

    while queue:

        # Dequeue a vertex from 
        # queue and print it
        s = queue.pop(0)
        # print (s, end = " ")

        # Get all adjacent vertices of the
        # dequeued vertex s. If a adjacent
        # has not been visited, then mark it
        # visited and enqueue it
        ix = (int) (s % size_x_)
        iy = (int) (s / size_y_)
        # print("ix: ", ix, "iy: ", iy)

        if gridmap[ix][iy]==val:
            result = s
            return s

        for n_idx in nhood4(s,gridmap,map_info):
            if visited[n_idx] == False:
                queue.append(n_idx)
                visited[n_idx] = True

    return False


def nearestCell(pos_x, pos_y, val, gridmap, map_info):

    size_x_=map_info.xw
    size_y_=map_info.yw
    if idx >(size_x_*size_y_ -1):
        print("Evaluating nhood for offmap point")
        return -1

    visited = [False] * (map_size)
   # Create a queue for BFS
    queue = []

    queue.append(start)
    visited[start] = True

    while queue:
        # Dequeue a vertex from 
        # queue and print it
        s = queue.pop(0)

        # Get all adjacent vertices of the
        # dequeued vertex s. If a adjacent
        # has not been visited, then mark it
        # visited and enqueue it
        ix = s % size_x_
        iy = s / size_y_

        if gridmap[ix][iy]==val:
            result = s
            return s

        for n_idx in nhood4(s,gridmap,map_info):
            if visited[n_idx] == False:
                queue.append(n_idx)
                visited[n_idx] = True

    return False




def nhood4(idx, gridmap, map_info):

    points=[]
    size_x_=map_info.xw
    size_y_=map_info.yw

    if idx >(size_x_*size_y_ -1):
        print("Evaluating nhood for offmap point")
        return -1

    if idx % size_x_ > 0:
        points.append(idx-1)

    if idx % size_x_ < (size_x_-1):
        points.append(idx+1)
    if idx >= size_x_:
        points.append(idx-size_x_)
    if idx < size_x_*(size_y_-1):
        points.append(idx+size_x_)

    return points

def nhood8(idx, gridmap, map_info):

    size_x_=map_info.xw
    size_y_=map_info.yw

    points=nhood4(idx,gridmap,map_info)

    if idx >(size_x_*size_y_ -1):
        print("Evaluating nhood for offmap point")
        return -1

    if idx % size_x_ > 0 and idx>=size_x_:
        points.append(idx-1-size_x_)
    if idx % size_x_ > 0 and idx<size_x_*(size_y_-1):
        points.append(idx-1+size_x_)

    if idx % size_x_ < (size_x_-1) and idx >=size_x_:
        points.append(idx+1-size_x_)
    if idx % size_x_ < (size_x_-1) and idx <size_x_*(size_y_-1):
        points.append(idx+1+size_x_)

    return points


# def get_frontier_map(px,py, pmap,  param_map):
    # frontiermap = [False] * (map_size)
    # for i in range(param_map.xw):
        # for j in range(param_map.yw):
            # print("i,j: ", i, j)
            # idx=j*param_map.xw+i
            # if pmap[i][j]!=0.0:
                # fron

def frontier_search(px,py, pmap,  param_map):
    print("before-frontier search with px, py", px, ", " , py)
    # input("check frontier")

    min_frontier_size_=1
    #find the closest free cell to start search
    map_size=param_map.xw * param_map.yw
    visited = [False] * (map_size)
    frontiermap = [False] * (map_size)
    frontier_list = []
    idx_sets=[]
    # Create a queue for BFS
    queue = []

    pos_idx =Coord2CellIdx_global(px,py, param_map)
    cell_idx= nearestCellIdx(pos_idx, l_free, pmap, param_map) #looking for unknown cell
    # cellx,celly=Idx2Pose(cell_idx, param_map)
    # print("px: ", px, ", py: ", py)
    # print("pos_idx", pos_idx)
    # print("cell_idx", cell_idx)
    # print("cellx: ", cellx, ", celly: ", celly)
    # print("------------")
    # input("check-pos-cell-idx")
    if cell_idx==False:
        print("Could not find the nearest start idx")
        return False

    # Mark the source node as 
    # visited and enqueue it
    queue.append(cell_idx)
    visited[cell_idx] = True

    while queue:
        s = queue.pop(0)
        # print (s, end = " ")
        # has not been visited, then mark it
        # visited and enqueue it

        sx,sy=Idx2Pose(s, param_map)
        # print("s_idx", s)
        # print("sx: ", sx, "sy: ", sy)
        for n_idx in nhood4(s, pmap,  param_map):
            nbrx,nbry=Idx2Pose(n_idx, param_map)
            # print("n_idx", n_idx)
            # print("nbrx: ", nbrx, "nbry: ", nbry)
            if visited[n_idx] == False:
                queue.append(n_idx)
                visited[n_idx] = True
            elif isNewFrontier(n_idx,pmap, frontiermap, param_map):

                fx,fy=Idx2Pose(n_idx, param_map)
                # print("--------------------", n_idx)
                # print("n_idx", n_idx)
                # print("fx: ", fx, "fy: ", fy)
                # print("--------------------", n_idx)
                # input("found frontier")
                frontiermap[n_idx] = True
                new_frontier = buildnewfrontier(n_idx, pmap, frontiermap, param_map, idx_sets);

                if new_frontier.size > min_frontier_size_:
                    frontier_list.append(new_frontier)
                    print("size is satisfied")
                else:
                    print("size is not satisfied")
        
    print("frontier_list", frontier_list)

    return frontier_list

def isNewFrontier(idx, pmap, frontier_map, param_map):
    #check that cell is unknown and not already marked as frontier

    ix = (int) (idx % param_map.xw)
    iy = (int) (idx / param_map.xw)
    # print("idx: ", idx, ", ix: ", ix, ", iy:", iy)

    if pmap[ix][iy] != 0.0 or frontier_map[idx]==True:
        return False
    
    #frontier cells should have at least one cell in 4-connected neighbourhood that is free

    for n_idx in nhood4(idx, pmap,  param_map):
        ix = math.floor(n_idx % param_map.xw)
        iy = math.floor(n_idx / param_map.xw)
    
        if pmap[ix][iy]==l_free:
            print("!!--frontier--!!")
            px, py = Idx2Pose(n_idx, param_map)
            print("n_idx: ", n_idx, "px: ",px, "py: ", py, ", pmap[ix][iy]: ",pmap[ix][iy])
            return True
        else:
            print("!!--frontier--!!")
            print("n_idx: ", n_idx, "ix: ",ix, "iy: ", iy, ", pmap[ix][iy]: ",pmap[ix][iy])

    # input("newfrontier completed")

    return False

def Unknown_search(px,py, pmap,  param_map, visited ):
    # print("before-unknown search with px, py", px, ", " , py)
    # input("check frontier")
    min_frontier_size_=25
    #find the closest free cell to start search
    map_size=param_map.xw * param_map.yw
    # visited = [False] * (map_size)
    # unknownmap = [False] * (map_size)
    frontier_list = []
    # Create a queue for BFS
    queue = []
    idx_sets=[]

    pos_idx =Coord2CellIdx_global(px,py, param_map)
    cell_idx= nearestCellIdx(pos_idx, 0.0, pmap, param_map) #looking for unknown cell
    # cellx,celly=Idx2Pose(cell_idx, param_map)
    cellx,celly=Idx2cellIds(cell_idx, param_map)
    # print("px: ", px, ", py: ", py)
    # print("pos_idx", pos_idx)
    # print("cell_idx", cell_idx)
    # print("cellx: ", cellx, ", celly: ", celly)
    if cell_idx==False:
        print("Could not find the nearest start idx")
        return False

    # visited and enqueue it
    queue.append(cell_idx)
    # visited[cell_idx] = True
    visited[cellx][celly] = True

    while queue:
        s = queue.pop(0)
        # has not been visited, then mark it
        sx,sy=Idx2cellIds(s, param_map)
        # cx,cy=Idx2Pose(cell_idx, param_map)
        # print("s_idx", s)
        # print("sx: ", sx, "sy: ", sy)
        # input('check s')
        for n_idx in nhood4(s, pmap,  param_map):
            nbrx,nbry=Idx2Pose(n_idx, param_map)
            nbx,nby=Idx2cellIds(n_idx, param_map)
            # print("n_idx", n_idx)
            # print("nbrx: ", nbrx, "nbry: ", nbry)
            # print("visited[n_idx] ", visited[n_idx])
            # if isNewUnknown(n_idx,pmap, unknownmap, param_map) and visited[n_idx]==False:
            if isNewUnknown(n_idx,pmap, param_map) and visited[nbx][nby]==False:
                fx,fy=Idx2Pose(n_idx, param_map)
                # print("--------------------", n_idx)
                # print("n_idx", n_idx)
                # print("fx: ", fx, "fy: ", fy)
                # print("--------------------", n_idx)
                # input("found frontier")
                # unknownmap[n_idx] = True
                new_frontier, visited, last_idx= buildnewUnknownfrontier(n_idx, pmap, visited, param_map)
                queue.append(last_idx)

                if new_frontier.size > min_frontier_size_:
                    frontier_list.append(new_frontier)
                    print("size is satisfied")
                # else:
                    # print("size is not satisfied")
            if visited[nbx][nby]==False:
                queue.append(n_idx)
                # visited[n_idx] = True
                visited[nbx][nby] = True
                # visited[n_idx] = True

        
    # print("frontier_list", frontier_list)

    return frontier_list, visited

def count_unvisted(visited, param_map):
    vcount=0
    for i in range(param_map.xw):
        for j in range(param_map.yw):
            if visited[i][j]==False:
                vcount=vcount+1

    return vcount


# 
def Unknown_decomposition(pmap,  param_map, visited ):
    # print("before-unknown search with px, py", px, ", " , py)
    min_frontier_size_=250
    #find the closest free cell to start search
    # visited = [False] * (map_size)
    size_x_=param_map.xw
    size_y_=param_map.yw
    map_size=size_x_*size_y_ 

    frontier_list = []
    while count_unvisted(visited, param_map)>0.1*map_size:

        px= random.uniform(0.95*param_map.xmin,0.95*param_map.xmax)
        py= random.uniform(0.85*param_map.ymin,0.85*param_map.ymax)
        # Create a queue for BFS
        queue = []
        pos_idx =Coord2CellIdx_global(px,py, param_map)
        cell_idx= nearestCellIdx(pos_idx, 0.0, pmap, param_map) #looking for unknown cell
        # cellx,celly=Idx2Pose(cell_idx, param_map)
        cellx,celly=Idx2cellIds(cell_idx, param_map)
        visited[cellx][celly] = True
        if cell_idx==False:
            print("Could not find the nearest start idx")
            return False

        # visited and enqueue it
        queue.append(cell_idx)
        # visited[cell_idx] = True

        while queue:
            s = queue.pop(0)
            # has not been visited, then mark it
            sx,sy=Idx2cellIds(s, param_map)
            # print("sx: ", sx, "sy: ", sy)
            # input('check s')
            for n_idx in nhood4(s, pmap,  param_map):
                nbrx,nbry=Idx2Pose(n_idx, param_map)
                nbx,nby=Idx2cellIds(n_idx, param_map)
                # print("n_idx", n_idx)
                # print("nbrx: ", nbrx, "nbry: ", nbry)
                # print("visited[n_idx] ", visited[n_idx])
                if isNewUnknown(n_idx,pmap, param_map) and visited[nbx][nby]==False:
                    fx,fy=Idx2Pose(n_idx, param_map)
                    new_frontier, visited, last_idx= buildnewUnknownfrontier(n_idx, pmap, visited, param_map)
                    queue.append(last_idx)

                    if new_frontier.size > min_frontier_size_:
                        frontier_list.append(new_frontier)
                        print("size is satisfied")
                    # else:
                        # print("size is not satisfied")
                if visited[nbx][nby]==False:
                    queue.append(n_idx)
                    # visited[n_idx] = True
                    visited[nbx][nby] = True
                    # visited[n_idx] = True

            
        # print("frontier_list", frontier_list)

    return frontier_list, visited






def isNewUnknown(idx, pmap, param_map):
    #check that cell is unknown and not already marked as frontier
    ix = (int) (idx % param_map.xw)
    iy = (int) (idx / param_map.yw)
    # print("idx: ", idx, ", ix: ", ix, ", iy:", iy)

    # if pmap[ix][iy] != 0.0 or frontier_map[idx]==True:
    if pmap[ix][iy] != 0.0:
        return False
    
    #Unknown cells should have at all cell in 4-connected neighbourhood that is unknown
    for n_idx in nhood4(idx, pmap,  param_map):
        ix = math.floor(n_idx % param_map.xw)
        iy = math.floor(n_idx / param_map.yw)
        # check all nhood is unknown
    
        if pmap[ix][iy]!=l_unknown:
            # print("!!--frontier--!!")
            px, py = Idx2Pose(n_idx, param_map)
            # print("n_idx: ", n_idx, "px: ",px, "py: ", py, ", pmap[ix][iy]: ",pmap[ix][iy])
            return False
        # else:
            # print("!!--frontier--!!")
            # print("n_idx: ", n_idx, "ix: ",ix, "iy: ", iy, ", pmap[ix][iy]: ",pmap[ix][iy])

    return True

    # input("newfrontier completed")

    # return False




def buildnewfrontier(start_cell, pmap, frontiermap, param_map):

    frt = Frontier()
    size_x_=param_map.xw
    size_y_=param_map.yw
    if start_cell>(size_x_*size_y_ -1):
        print("Evaluating nhood for offmap point")
        return -1
    map_size=size_x_*size_y_ 

    visited = [False] * (map_size)
    # frontiermap = [False] * (map_size)
    queue = []

    # Mark the source node as 
    # visited and enqueue it
    queue.append(start_cell)
    visited[start_cell] = True
    agent_pt=Point2D()
    agent_pt.x, agent_pt.y = Idx2Pose(start_cell, param_map)

    while queue:

        s = queue.pop(0)
        # visited and enqueue it
        # nsets = nhood8(s, pmap,  param_map)
        # print("nbrs", nsets)
        # input("hmm")
        for n_idx in nhood8(s, pmap,  param_map):
            if isNewFrontier(n_idx,pmap, frontiermap, param_map):
                # print("n_idx", n_idx, "--newfrontier")
                frontiermap[n_idx] = True;

                pt = Point2D()
                pt.x, pt.y = Idx2Pose(n_idx, param_map)
                frt.points.append(pt)
                frt.size=frt.size+1
                frt.centroid_x = frt.centroid_x+pt.x
                frt.centroid_y = frt.centroid_y+pt.y

                dist=distance_point2D(agent_pt,pt)
                if dist < frt.min_distance: 
                    frt.min_distance=dist

                queue.append(n_idx)


    print("found frontiers: ", frt.size)
    frt.centroid_x /= frt.size;
    frt.centroid_y /= frt.size;

    return frt

def buildnewUnknownfrontier(start_cell, pmap, visited, param_map ):

    # print("----------new_frontier----------")
    frt = Frontier()
    size_x_=param_map.xw
    size_y_=param_map.yw
    if start_cell>(size_x_*size_y_ -1):
        print("Evaluating nhood for offmap point")
        return -1
    map_size=size_x_*size_y_ 

    # visited = [False] * (map_size)
    # frontiermap = [False] * (map_size)
    queue = []
    # Mark the source node as 
    # visited and enqueue it
    queue.append(start_cell)
    # visited[start_cell] = True
    cellx,celly=Idx2cellIds(start_cell, param_map)
    visited[cellx][celly]= True
    # visited[start_cell] = True
    agent_pt=Point2D()
    agent_pt.x, agent_pt.y = Idx2Pose(start_cell, param_map)

    while queue:

        s = queue.pop(0)
        # visited and enqueue it
        # nsets = nhood8(s, pmap,  param_map)
        # print("nbrs", nsets)
        # input("hmm")
        for n_idx in nhood8(s, pmap,  param_map):

            cx,cy=Idx2cellIds(n_idx, param_map)

            if frt.size>1000:
                # print("max size reached")
                frt.centroid_x /= frt.size;
                frt.centroid_y /= frt.size;
                print("frt.avx: ", frt.centroid_x , ", frt.avy: ", frt.centroid_y)
                return frt, visited, n_idx

            # elif isNewUnknown(n_idx,pmap, frontiermap, param_map) and visited[n_idx]==False:
            elif isNewUnknown(n_idx,pmap, param_map) and visited[cx][cy]==False:
                # print("n_idx", n_idx, "--newfrontier")
                # frontiermap[n_idx] = True;
                # visited[n_idx]=True
                visited[cx][cy]= True

                pt = Point2D()
                pt.x, pt.y = Idx2Pose(n_idx, param_map)
                # print("pt.x: ", pt.x , ", pt.y: ", pt.y)
                frt.points.append(pt)
                frt.size=frt.size+1
                frt.centroid_x = frt.centroid_x+pt.x
                frt.centroid_y = frt.centroid_y+pt.y
                # idx_sets.append(n_idx)


                dist=distance_point2D(agent_pt,pt)

                if dist < frt.min_distance: 
                    frt.min_distance=dist

                queue.append(n_idx)
            if visited[cx][cy]==False:
               visited[cx][cy]==True
            # if visited[n_idx]==False:
                # visited[n_idx]=True

    print("found frontiers, size: ", frt.size)
    frt.centroid_x /= frt.size;
    frt.centroid_y /= frt.size;

    return frt, visited, n_idx 




