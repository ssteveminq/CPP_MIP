import ast, math, operator
import matplotlib.pyplot as plt
import numpy as np

from utils.configuration_space import Roadmap
from collections import defaultdict
from utils.uniform_cost_search import Search
from utils.graph_utils import *

AlphabetSet=['a','b','c','d','e','f','g','h','i','j','k','l','m', 
                'n','o','p','q','r']

class VerticalCellDecomposition:
    def __init__(self,cspace, goalpose=None):
        self.cspace = cspace
        self.agent_pose= cspace.start_state
        if goalpose ==None:
            self.goal_pose = cspace.goal_state
        else:
            self.goal_pose=goalpose


        self.polygon_vertices = [item for sublist in cspace.polygons for item in sublist]
        # print("self.polygon_vertices")
        # print(self.polygon_vertices)

        self.polygon_edges = []

        for polygon in cspace.polygons:
            for i in range(len(polygon)):
                self.polygon_edges.append([polygon[i%len(polygon)],polygon[(i+1)%len(polygon)]])
        # print("polygon-edge")
        # print(self.polygon_edges)

        self.decomposition_lines = []
        self.decomposition_lines_midpts = []
        self.decomposition_lines_map = defaultdict(list)
        self.regions = []
        self.roadmap = Roadmap()

    def reset_cspace(self,cspace):
        self.cspace = cspace
        self.goal_pose = cspace.goal_state
        self.polygon_vertices = [item for sublist in cspace.polygons for item in sublist]

        self.polygon_edges = []

        for polygon in cspace.polygons:
            for i in range(len(polygon)):
                self.polygon_edges.append([polygon[i%len(polygon)],polygon[(i+1)%len(polygon)]])

        self.decomposition_lines = []
        self.decomposition_lines_midpts = []
        self.decomposition_lines_map = defaultdict(list)
        self.regions = []
        self.roadmap = Roadmap()




    def construct_decomposition_lines(self,vertex,verticalLine,pointList,top):
        # print("top", top)
        # print("vertex: ", vertex)
        # print("vertical line: ", verticalLine)
        for edge in self.polygon_edges:
            if (vertex not in edge):  # exclude edges containing the self vertex
                point = line_intersection(verticalLine, edge)
                # print("edge:", edge, ", point: ",point )
                if point is not None: # If they intersect...
                    pointList.append(point)
            # else: #if an edge containg the self vertex and edge is vertical
                # if top and edge[0][1] == edge[1][1]:
                    # point =[vertex[0],-100]
                    # pointList.append(point)
        # print("pointList", pointList)

        # Find nearest point of intersection
        nearest_point = pointList[0]
        for i in range(1, len(pointList)):
            if top and pointList[i][1] < nearest_point[1]: #find the small y
                nearest_point = pointList[i]
            elif pointList[i][1] > nearest_point[1]: #find the big y
                nearest_point = pointList[i]

        # print("nearest point", nearest_point)

        # Check if the nearest point falls on same polygon. If so, ignore the vertical line
        for polygon in self.cspace.polygons:
            if vertex in polygon:
                if not on_polygon(nearest_point[0], nearest_point[1], polygon):
                    # print("not on polygon")
                    self.decomposition_lines.append([vertex, nearest_point])

                    # create vertice_decompLines_map {vertex : pointList}
                    self.decomposition_lines_map[vertex].append([vertex,nearest_point])
                # else:
                    # print("on polygon--nearest point", nearest_point[0], nearest_point[1],", polygon:", polygon )

        # print("decompsotion_lines_map", self.decomposition_lines_map)

    # Iterate over all edges for each vertex
    def vertical_lines(self):
        for vertex in self.polygon_vertices:
            # TopLine
            top = True
            pointList = [(vertex[0], self.cspace.boundary[2][1])]
            # verticalLine = [vertex[y], (vertex[0], self.cspace.boundary[2][1])]
            # verticalLine = [(vertex[0], self.cspace.boundary[2][1]), (vertex[0], self.cspace.boundary[2][1])]
            verticalLine = [vertex, (vertex[0], self.cspace.boundary[2][1])]
            self.construct_decomposition_lines(vertex,verticalLine,pointList,top)

            # BottomLine
            top = False
            pointList = [(vertex[0], self.cspace.boundary[0][1])]  # considering the intersection point on border
            verticalLine = [vertex, (vertex[0], self.cspace.boundary[0][1])]
            self.construct_decomposition_lines(vertex, verticalLine, pointList,top)

        for line in self.decomposition_lines:
            self.decomposition_lines_midpts.append([line[0][0],(line[0][1]+line[1][1])/2.0])

    # up = True for up, False for down
    def find_region_line(self, line_type, i, vertex, current_vertex):
        maxY = self.cspace.boundary[2][1]
        minY = self.cspace.boundary[0][1]

        if line_type is "middle":
            for next_vertex in self.polygon_vertices[i:]:  # next_vertexertex
                for k in range(len(self.decomposition_lines_map[next_vertex])):
                    temp_vertex = self.decomposition_lines_map[next_vertex][k][1]
                    if (temp_vertex[1] not in (maxY, minY)):  # middle line, not intersecting with top/bottom border
                        if ((vertex[1] <= next_vertex[1] <= current_vertex[1]) or
                                (vertex[1] <= temp_vertex[1] <= current_vertex[1]) or
                                (next_vertex[1] <= vertex[1] <= temp_vertex[1]) or
                                (next_vertex[1] <= current_vertex[1] <= temp_vertex[1]) or
                                (vertex[1] >= next_vertex[1] >= current_vertex[1]) or
                                (vertex[1] >= temp_vertex[1] >= current_vertex[1]) or
                                (next_vertex[1] >= vertex[1] >= temp_vertex[1]) or
                                (next_vertex[1] >= current_vertex[1] >= temp_vertex[1])):
                            return [next_vertex, temp_vertex]


        # if Up, check with 'up Yval', else down
        elif line_type is "up":
            yVal = maxY

        elif line_type is "down":
            yVal = minY

        if line_type is "up" or line_type is "down":
            for next_vertex in self.polygon_vertices[i:]:
                # iterate over its decomposition lines
                for j in range(len(self.decomposition_lines_map[next_vertex])):
                    current_vertex = self.decomposition_lines_map[next_vertex][j][1]
                    if (current_vertex[1] == yVal):
                        return [next_vertex, current_vertex]

        return []

    def print_region(self):
        for i,region in enumerate(self.regions):
            print(str(i), " -th region : ")
            for point in region:
                print(point)
    def average(self,lst):
        return sum(lst)/len(lst)

    def get_area_polygon(self, region):
        x_sets =[]
        y_sets =[]
        for point in region:
            x_sets.append(point[0])
            y_sets.append(point[1])
        x_mean =self.average(x_sets)
        y_mean =self.average(y_sets)

        if(len(y_sets)>2):
            tmp=y_sets[3]
            y_sets[3]=y_sets[2]
            y_sets[2]=tmp
        for i in range(len(x_sets)):
            x_sets[i] = x_sets[i] - x_mean 
            y_sets[i] = y_sets[i] - y_mean 
        # everything else is the same as maxb's code
        correction = x_sets[-1] * y_sets[0] - y_sets[-1]* x_sets[0]
        main_area = np.dot(x_sets[:-1], y_sets[1:]) - np.dot(y_sets[:-1], x_sets[1:])
        return 0.5*np.abs(main_area + correction)


    def region_disection(self, goalpos=None):
        maxY = self.cspace.boundary[2][1]
        minY = self.cspace.boundary[0][1]
        # print("minY", minY, ", maxY:", maxY)

        self.original_vertices = self.polygon_vertices
        self.polygon_vertices = sorted(self.polygon_vertices, key=lambda x: x[0])
        if self.polygon_vertices[0][0]==self.polygon_vertices[1][0]:
            first = (self.decomposition_lines_map[self.polygon_vertices[0]])
            first2= (self.decomposition_lines_map[self.polygon_vertices[1]])
            firstCell= [self.cspace.boundary[0],self.cspace.boundary[3],first[0][1],first2[0][1]]
        else:
            first = (self.decomposition_lines_map[self.polygon_vertices[0]])
            firstCell = [self.cspace.boundary[0],self.cspace.boundary[3],first[0][0],first[0][1]]


        if self.polygon_vertices[-1][0]==self.polygon_vertices[-2][0]:
            second= (self.decomposition_lines_map[self.polygon_vertices[-1]])
            second2= (self.decomposition_lines_map[self.polygon_vertices[-2]])
            lastCell =  [self.cspace.boundary[2],self.cspace.boundary[1],second[0][1],second2[0][1]]
        else:
            second= (self.decomposition_lines_map[self.polygon_vertices[-1]])
            lastCell =  [self.cspace.boundary[2],self.cspace.boundary[1],second[0][0],second[0][1]]

        # firstCell = [self.cspace.boundary[0],self.cspace.boundary[3],self.decomposition_lines_map[self.polygon_vertices[0]][0][1],\
             # self.decomposition_lines_map[self.polygon_vertices[0]][1][1]]
        # lastCell =  [self.cspace.boundary[2],self.cspace.boundary[1],self.decomposition_lines_map[self.polygon_vertices[-1]][0][1],\
             # self.decomposition_lines_map[self.polygon_vertices[-1]][1][1]]

        self.regions.append(firstCell)
        self.regions.append(lastCell)

        # Iterate over vertices
        for i,vertex in enumerate(self.polygon_vertices[:-2]): #MK
            # iterate over its decomposition lines
            for j in range(len(self.decomposition_lines_map[vertex])):
                # check if both vertices are having both up and down
                current_vertex = self.decomposition_lines_map[vertex][j][1]
                next_vertex = self.polygon_vertices[i+1]

                if(len(self.decomposition_lines_map[vertex])==2) and (len(self.decomposition_lines_map[next_vertex])==2):
                    ov = [self.decomposition_lines_map[vertex][0][1],self.decomposition_lines_map[vertex][1][1],\
                        self.decomposition_lines_map[next_vertex][0][1],self.decomposition_lines_map[next_vertex][1][1]]
                    # print("ov", ov)

                    if((ov[0][1] in (maxY, minY)) and (ov[1][1] in (maxY, minY)) and (ov[2][1] in (maxY, minY)) and \
                                    (ov[3][1] in (maxY, minY))):
                        self.regions.append(ov)
                        continue

                if(current_vertex[1] == maxY): # upper line
                    # print("upper")
                    # print(self.decomposition_lines_map[vertex][j]+self.find_region_line("up", i+1, vertex, current_vertex))
                    self.regions.append(self.decomposition_lines_map[vertex][j]+self.find_region_line("up", i+1, vertex, current_vertex))

                elif(current_vertex[1] == minY): # lower line
                    # print("lower")
                    # print(self.decomposition_lines_map[vertex][j]+self.find_region_line("down", i+1, vertex, current_vertex))
                    self.regions.append(self.decomposition_lines_map[vertex][j]+self.find_region_line("down", i+1, vertex, current_vertex))

                else: # middle
                    # print("middle")
                    lst = self.find_region_line("middle", i + 1, vertex, current_vertex)
                    if(len(lst) > 0):
                        self.regions.append(self.decomposition_lines_map[vertex][j]+lst)
                        # TODO, do not remove the comment below
                        # print(self.decomposition_lines_map[vertex][j]+lst)

        # input()
        # print(self.polygon_edges)

        merged_regions=[]
        #fix bugs ==> merge two cells if there doesn't exist obstacles between them
        for i,region_i in enumerate(self.regions):
            for j,region_j in enumerate(self.regions):
                if i==j or i>j:
                    continue
                else:
                    merged_region=[]
                    #check x coordinates
                    count=0
                    for k in range(4):
                        if region_i[k][0] ==region_j[k][0]:
                            count+=1
                
                    if count==4:
                        if self.check_polygon_between_regions(region_i, region_j)==False:
                            ymin =100
                            ymax =-100
                            for k in range(4):
                                if region_i[k][1]<ymin:
                                    ymin=region_i[k][1]
                                if region_j[k][1]<ymin:
                                    ymin=region_j[k][1]
                                if region_i[k][1]>ymax:
                                    ymax=region_i[k][1]
                                if region_j[k][1]>ymax:
                                    ymax=region_j[k][1]
                            # print("ymin: ", ymin, ", ymax:", ymax)
                            # merged_region.append((region_i[0][0], ymin))
                            # merged_region.append((region_i[0][0], ymax))
                            # merged_region.append((region_i[2][0], ymin))
                            # merged_region.append((region_i[2][0], ymax))
                            # print("merged_region",merged_region)
                            self.regions[i][0]=(region_i[0][0], ymin)
                            self.regions[i][1]=(region_i[0][0], ymax)
                            self.regions[i][2]=(region_i[2][0], ymin)
                            self.regions[i][3]=(region_i[2][0], ymax)
                            # self.regions.append(merged_region)
                            self.regions.pop(j)
                            # self.regions.remove(self.regions[i])
                            
                    # merged_regions.append(merged_region)

        # print("start_state: ", self.cspace.start_state)
        self.roadmap.vertices_dict[0] = list(self.cspace.start_state)
        if goalpos!=None:
            self.roadmap.vertices_dict[1] = list(goalpos)
        else:
            self.roadmap.vertices_dict[1] = list(self.cspace.goal_state)


        self.roadmap.vertices_dict_noedge[0] = list(self.cspace.start_state)

        for i,region in enumerate(self.regions):
            c_x = 0
            c_y = 0

            for point in region:
                c_x += point[0]/float(len(region))
                c_y += point[1]/float(len(region))

            self.roadmap.vertices_dict[i+2] = [c_x,c_y]      
            self.roadmap.vertices_dict_noedge[i+2] = [c_x,c_y]


    def check_polygon_between_regions(self,region1,region2):
        for edge in self.polygon_edges:
            if edge[0][0]==region1[0][0] and edge[1][0]==region1[2][0]:
                # print(edge)
                return True
        return False
            



    def construct_graph(self):
        self.vertical_lines()
        self.region_disection()
        self.construct_graph_main()

    def construct_graph_main(self):
        max_key = list(self.roadmap.vertices_dict.keys())[-1]
        # print("dict", self.roadmap.vertices_dict)
        # print("dict_noedge", self.roadmap.vertices_dict_noedge)

        # print("max_key", max_key)
        # input()


        for i,point in enumerate([self.cspace.start_state, self.goal_pose]):
            for j,centroid in enumerate(list(self.roadmap.vertices_dict.values())[2:]):
                skip = False
                graph_line = [centroid, point]
                # print("graph_line", graph_line, ", point: ",point)
                # check intersection with any of (edges, decomposition lines)

                # num_intersect=0
                for line in self.polygon_edges+self.decomposition_lines:
                    if(line_intersection(graph_line, line) is not None):
                        # print("intersect")
                        # print("graph_line", graph_line, ", point: ",point)
                        # print("line", line)
                        # int_x , int_y = line_intersection(graph_line, line)
                        # print("int_x : ", int_x, "int_y: ", int_y)
                        skip = True
                        break

                if skip:
                    continue

                self.roadmap.adjacency_dict[i].append(j+2)
                self.roadmap.adjacency_dict[j+2].append(i)
                self.roadmap.edge_weights[i].append(distance(point,centroid))
                self.roadmap.edge_weights[j+2].append(distance(point,centroid))

                break

        #mk - remove midpoints from vertices
        for i in range(len(self.decomposition_lines_midpts)):
            self.roadmap.vertices_dict[max_key+i+1] = self.decomposition_lines_midpts[i]

        for i,centroid in enumerate(list(self.roadmap.vertices_dict.values())[2:max_key+1]):
            for j,point in enumerate(list(self.roadmap.vertices_dict.values())[max_key+1:]):
                skip = False
                graph_line = [centroid, point]

                for edge in self.polygon_edges:
                    if line_intersection(graph_line,edge) is not None:
                        skip = True
                        break

                for decomposition_line in self.decomposition_lines:
                    insersection_point = line_intersection(graph_line,decomposition_line)
                    if insersection_point is not None and distance(insersection_point,point) != 0:
                        skip = True
                        break

                if skip:
                    continue

                self.roadmap.adjacency_dict[i+2].append(max_key+1+j)
                self.roadmap.edge_weights[i+2].append(distance(point,centroid))
                self.roadmap.adjacency_dict[max_key+j+1].append(i+2)
                self.roadmap.edge_weights[max_key+j+1].append(distance(point,centroid))

    def generate_waypoint(self, params_local):
        
        local_width = params_local.sensor_range*2
        local_height= params_local.sensor_range*2
        # for i 
        waypoints  =[]
        for i,region in enumerate(self.regions):
            # print("region", region)
            area = self.get_area_polygon(region)
            if area > local_width*local_height:
                xmin=100
                ymin=100
                xmax=-100
                ymax=-100
                for point in region:
                    if xmin>point[0]:
                        xmin = point[0]
                    if xmax<point[0]:
                        xmax = point[0]
                    if ymin>point[1]:
                        ymin = point[1]
                    if ymax<point[1]:
                        ymax = point[1]

                num_cell_x = math.ceil((xmax-xmin)/local_width)
                num_cell_y = math.ceil((ymax-ymin)/local_height)
                for u in range(num_cell_x):
                    for v in range(num_cell_y):
                        way_x = xmin+(2*u+0.5)*local_width/2
                        way_y = ymin+(2*v+0.5)*local_height/2
                        if way_x > xmax:
                            way_x = xmax-3.0
                        if way_y > ymax:
                            way_y = ymax-3.0
                        waypoints.append([way_x,way_y])
            else:
                x_sets=[]
                y_sets=[]
                for point in region:
                    x_sets.append(point[0])
                    y_sets.append(point[1])
                x_mean =self.average(x_sets)
                y_mean =self.average(y_sets)
                waypoints.append([x_mean,y_mean])
            self.waypoints = waypoints

        return waypoints



    def get_vcd_vertices(self):
        # points = # return self.roadmap.vertices_dict.values()
        return self.roadmap.vertices_dict_noedge.values()


    def plot_vcd(self):
        self.cspace.plot_config_space(showPlot=False)

        for point in self.roadmap.vertices_dict.values():
            plt.plot(point[0],point[1],marker='o',color='black')

        for key in self.roadmap.adjacency_dict.keys():
            for value in self.roadmap.adjacency_dict[key]:
                plt.plot([self.roadmap.vertices_dict[key][0],self.roadmap.vertices_dict[value][0]],\
                    [self.roadmap.vertices_dict[key][1],self.roadmap.vertices_dict[value][1]],color='y')

        # plot decomposition lines
        for line in self.decomposition_lines:
            x = [line[0][0],line[1][0]]
            y = [line[0][1],line[1][1]]
            plt.plot(x,y,'b')

    def plot_regions(self, ax=None):

        num_colors = len(self.regions)
        if ax==None:
            ax = plt.gca()

        cm =plt.get_cmap('gist_rainbow')
        for i,region in enumerate(self.regions):
            x_sets =[]
            y_sets =[]
            for point in region:
                x_sets.append(point[0])
                y_sets.append(point[1])
            col = cm(1.*i/num_colors)
            # area = planner.get_area_polygon(region)
            if len(y_sets)>2:
                tmp=y_sets[3]
                y_sets[3]=y_sets[2]
                y_sets[2]=tmp

            ax.fill(x_sets,y_sets, color=col,alpha=0.2)

        ax.set_xlim([-12.5, 12.5])   # limit the plot space
        ax.set_ylim([-12.5, 12.5])   # limit the plot space

        # ax.scatter(self.agent_pose[0], self.agent_pose[1], facecolor='black',edgecolor='black')      #initial point
        way_x=[]
        way_y=[]

        for point in self.waypoints:
            # print("x: ", point[0])
            # print("y: ", point[1])
            way_x.append(point[0])
            way_y.append(point[1])
        
        ax.plot(way_x, way_y, '*', markersize= 5, fillstyle='none',color='green')             #trajectory point
        # for i in range(len(way_x)):
            # ax.text(way_x[i]+0.5, way_y[i]-0.5,AlphabetSet[i], color='g')

    def smoothing_path(self,final_path, final_path_idx):
        # connect directly at the starting and the last point
        for i, path_vertex in enumerate(final_path):
            if i==0 or i==(len(final_path)-3):
                dist_to_next2 =  distance(final_path[i],final_path[i+1])+distance(final_path[i+1],final_path[i+2])
                dist_direct = distance(final_path[i],final_path[i+2])
                if 1.2*dist_direct < dist_to_next2:
                    final_path.pop(i+1)

        return final_path, final_path_idx



    def search(self,showPlot=False, goalpos=None):
        if goalpos !=None:
            #should reconstruct the graph
            # self.decomposition_lines = []
            # self.decomposition_lines_midpts = []
            # self.decomposition_lines_map = defaultdict(list)
            # self.regions = []
            # self.roadmap = Roadmap()
            self.vertical_lines()
            self.region_disection(goalpos)
            self.construct_graph_main()


        ucs = Search(self.roadmap)
        searchResult = ucs.perform_search(goalpos)

        if searchResult is None:
            print("Path could not be found!")
            return None,None
            sys.exit()

        final_path, final_path_idx, path_cost = searchResult

        final_path, final_path_idx  = self.smoothing_path(final_path, final_path_idx)

        for i in range(1,len(final_path)):
            plt.plot([elem[0] for elem in final_path[i-1:i+1]],[elem[1] for elem in final_path[i-1:i+1]],color='brown')

        if showPlot:

            self.plot_vcd()
            for i in range(1,len(final_path)):
                plt.plot([elem[0] for elem in final_path[i-1:i+1]],[elem[1] for elem in final_path[i-1:i+1]],color='red')
            plt.show()

        return final_path, final_path_idx
