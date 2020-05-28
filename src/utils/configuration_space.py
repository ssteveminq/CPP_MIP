import ast
import matplotlib.pyplot as plt
from collections import defaultdict

class configuration_space:
    def __init__(self,FILE_NAME=None):
        self.polygons = []
        line_ctr = 0
        if FILE_NAME==None:
            return
        with open(FILE_NAME) as f:
            num_lines = sum(1 for l in f)
        with open(FILE_NAME) as f:
            for l in f:
                line_ctr += 1
                if line_ctr == 1:
                    self.boundary = list(ast.literal_eval(l))
                elif line_ctr in range(2,num_lines):
                    self.polygons.append(list(ast.literal_eval(l)))
                else:
                    temp = list(ast.literal_eval(l))
                    self.start_state = [temp[0], temp[1]]
                    #Temprorary set the goal 
                    self.goal_state = [temp[0]-2, temp[1]+4]

    def plot_polygon(self,coords,axes=None):
        if axes==None:
            for i in range(len(coords)):
                plt.plot(coords[i][0],coords[i][1],marker='o',color='black',markersize=0.5)
                if i <len(coords)-1:
                    plt.plot([coords[i][0], coords[i+1][0]], [coords[i][1], coords[i+1][1]], color='black')
                else:
                    plt.plot([coords[i][0], coords[0][0]], [coords[i][1], coords[0][1]], color='black')

        else:
            for i in range(len(coords)):
                ax.plot(coords[i][0],coords[i][1],marker='o',color='black',markersize=0.5)
                if i <len(coords)-1:
                    ax.plot([coords[i][0], coords[i+1][0]], [coords[i][1], coords[i+1][1]], color='black')
                else:
                    ax.plot([coords[i][0], coords[0][0]], [coords[i][1], coords[0][1]], color='black')


    def plot_config_space(self,showPlot=True):
        axes = plt.gca()
        axes.set_xlim([self.boundary[0][0],self.boundary[1][0]])
        axes.set_ylim([self.boundary[0][1],self.boundary[2][1]])
        plt.plot(self.start_state[0],self.start_state[1],marker='o',color='red')
        for i in range(len(self.polygons)):
            self.plot_polygon(self.polygons[i])
        if showPlot:
            plt.show()
    def reset_environment(self, boundaries, init_pos,goal_pos, obstacles):
        # c-clockwise 
        self.boundary =boundaries
        self.polygons=[]
        for obs in obstacles:
            if obs.isWall==None:
                self.polygons.append(obs.vertices)

        self.start_state = init_pos
        self.goal_state = goal_pos
    def get_obs(self):
        obstacles=[]
        for obs in self.polygons:
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
            obstacles.append([xmin,xmax,ymin,ymax])

        return obstacles




            
class Roadmap:
    def __init__(self):
        self.vertices_dict = {}
        self.adjacency_dict = defaultdict(list)
        self.edge_weights = defaultdict(list)
        self.vertices_dict_noedge = {}

if __name__ == "__main__":
	cspace = configuration_space("input.txt")
	cspace.plot_config_space()
