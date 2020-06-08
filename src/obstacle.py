import matplotlib.pyplot as plt
import math

class Obstacle:
    def __init__(self,x_min,x_max,y_min,y_max,wall=None):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.vertices=[]
        self.generate_vertices()
        self.isWall=wall
        self.isVertical = False
        if wall:
            if x_min == x_max:
                self.isVertical = True
            elif y_min == y_max:
                self.isVertical = False
            else:
                print("wall is neither horizontal nor vertical")
                self.isVertical = False

    def check_point_obstaclefree(self, point_x, point_y):
        margin=1.0
        # return True if obstacle is free
        # print("point_x:", point_x, "point_y: ", point_y)
        # point_x = point2d[0]
        # point_y = point2d[1]
        if (self.x_min-margin< point_x) and (self.x_max+margin > point_x):
            if (self.y_min-margin< point_y) and (self.y_max+margin) > point_y:
                return False
        return True



    def generate_vertices(self):
        self.vertices =[]
        self.vertices.append((self.x_min, self.y_min))
        self.vertices.append((self.x_max, self.y_min))
        self.vertices.append((self.x_max, self.y_max))
        self.vertices.append((self.x_min, self.y_max))

    def get_visible_vertices(self, agent_x, agent_y):
        visible_vertices={}
        if agent_x < self.x_min: 
            if agent_y> self.y_max:
                visible_vertices[0]=self.vertices[0]
                visible_vertices[2]=self.vertices[2]
                visible_vertices[3]=self.vertices[3]
            elif agent_y>self.y_min:
                visible_vertices[0]=self.vertices[0]
                visible_vertices[3]=self.vertices[3]
            else:
                visible_vertices[0]=self.vertices[0]
                visible_vertices[1]=self.vertices[1]
                visible_vertices[3]=self.vertices[3]
        elif agent_x <self.x_max:
            if agent_y> self.y_max:
                visible_vertices[2]=self.vertices[2]
                visible_vertices[3]=self.vertices[3]
            elif agent_y< self.y_min:
                visible_vertices[0]=self.vertices[0]
                visible_vertices[1]=self.vertices[1]
            else:
                visible_vertices=[]
                print("inside polygon")
        elif agent_x>self.x_max:
            if agent_y> self.y_max:
                visible_vertices[1]=self.vertices[1]
                visible_vertices[2]=self.vertices[2]
                visible_vertices[3]=self.vertices[3]
            elif agent_y>self.y_min:
                visible_vertices[1]=self.vertices[1]
                visible_vertices[2]=self.vertices[2]
            else:
                visible_vertices[0]=self.vertices[0]
                visible_vertices[1]=self.vertices[1]
                visible_vertices[2]=self.vertices[2]

        return visible_vertices

    def get_visible_vertices_wall(self, agent_x, agent_y,params):
        #horizontal wall [ymin = ymax ]
        #vertical wall [xmin = xmax ]
        #determine wall horizontal or vertical
        # print("agent_x: ", agent_x, ", agent_y: ", agent_y)
        sensor_range = params.sensor_range
        fov_minx = agent_x - params.sensor_range
        fov_maxx = agent_x + params.sensor_range
        fov_miny = agent_y - params.sensor_range
        fov_maxy = agent_y + params.sensor_range
        # print("fov_maxx: ", fov_maxx)
        # print("fov_maxy: ", fov_maxy)
        # print("wall minx: ", self.x_min)
        # print("wall maxx: ", self.x_max)
        # print("wall miny: ", self.y_min)
        # print("wall maxy: ", self.y_max)

        xyreso = params.xyreso
        Vertices=[]
        if self.isVertical:
            wall_x = self.x_min
            if abs(agent_x-wall_x)<params.sensor_range:
                if self.y_min>fov_miny:
                    n=math.floor((fov_maxy-self.y_min)/xyreso)
                    for i in range(n):
                        y_coord = self.y_min+xyreso*i
                        Vertices.append([wall_x,y_coord])
                elif self.y_max < fov_maxy:
                    n=math.floor((self.y_max - fov_miny)/xyreso)
                    print(n)
                    for i in range(n):
                        y_coord = fov_miny+xyreso*i
                        Vertices.append([wall_x,y_coord])
                else:
                    n=math.floor((fov_maxy- fov_miny)/xyreso)
                    for i in range(n):
                        y_coord = fov_miny+xyreso*i
                        Vertices.append([wall_x,y_coord])
                #wall is within sensor_range
                return Vertices
            else:
                return None
            #case: vertical

        else:           # wall is horizontal
            wall_y = self.y_min
            if abs(agent_y-wall_y)<sensor_range:
                if self.x_min>fov_minx:
                    n=math.floor((fov_maxx-self.x_min)/xyreso)
                    for i in range(n):
                        x_coord = self.x_min+xyreso*i
                        Vertices.append([x_coord, wall_y])
                elif self.x_max<fov_maxx:
                    n=math.floor((self.x_max - fov_minx)/xyreso)
                    for i in range(n):
                        x_coord = fov_minx+xyreso*i
                        Vertices.append([x_coord, wall_y])
                else:
                    n=math.floor((fov_maxx- fov_minx)/xyreso)
                    for i in range(n):
                        x_coord = fov_minx+xyreso*i
                        Vertices.append([x_coord, wall_y])

                return Vertices
            else:
                return None



    def draw(self,axes=None):
        if axes==None:
            plt.plot([self.x_min, self.x_max], [self.y_max, self.y_max], color='black')
            plt.plot([self.x_min, self.x_max], [self.y_min, self.y_min], color='black')
            plt.plot([self.x_min, self.x_min], [self.y_min, self.y_max], color='black')
            plt.plot([self.x_max, self.x_max], [self.y_max, self.y_min], color='black')
        else:
            axes.plot([self.x_min, self.x_max], [self.y_max, self.y_max], color='black')
            axes.plot([self.x_min, self.x_max], [self.y_min, self.y_min], color='black')
            axes.plot([self.x_min, self.x_min], [self.y_min, self.y_max], color='black')
            axes.plot([self.x_max, self.x_max], [self.y_max, self.y_min], color='black')

    def generate_samples(self, reso=0.2):
        obstacles =[]
        width= self.x_max - self.x_min
        height= self.y_max - self.y_min
        num_x = int(width/reso)
        num_y = int(height/reso)

        for i in range(num_x):
            x_coord = self.x_min+i*reso
            y_coord = self.y_min
            obstacles.append([x_coord, y_coord])

        for i in range(num_x):
            x_coord = self.x_min+i*reso
            y_coord = self.y_max
            obstacles.append([x_coord, y_coord])
        
        for i in range(num_x):
            x_coord = self.x_min+i*reso
            y_coord = self.y_min
            obstacles.append([x_coord, y_coord])
        
        for i in range(num_x):
            x_coord = self.x_min+i*reso
            y_coord = self.y_min
            obstacles.append([x_coord, y_coord])




