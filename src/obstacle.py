import matplotlib.pyplot as plt

class Obstacle:
    def __init__(self,x_min,x_max,y_min,y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.vertices=[]
        self.generate_vertices()

    def generate_vertices(self):
        self.vertices =[]
        self.vertices.append((self.x_min, self.y_min))
        self.vertices.append((self.x_max, self.y_min))
        self.vertices.append((self.x_max, self.y_max))
        self.vertices.append((self.x_min, self.y_max))

    def get_visible_vertices(self, agent_x, agent_y):
        print("agent_x : ", agent_x)
        print("agent_y : ", agent_y)
        print("xmin, xmax: ", self.x_min, self.x_max)
        print("ymin, ymax: ", self.y_min, self.y_max)
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







    def draw(self,axes=None):
        if axes==None:
            plt.plot([self.x_min, self.x_max], [self.y_max, self.y_max], color='green')
            plt.plot([self.x_min, self.x_max], [self.y_min, self.y_min], color='green')
            plt.plot([self.x_min, self.x_min], [self.y_min, self.y_max], color='green')
            plt.plot([self.x_max, self.x_max], [self.y_max, self.y_min], color='green')
        else:
            axes.plot([self.x_min, self.x_max], [self.y_max, self.y_max], color='green')
            axes.plot([self.x_min, self.x_max], [self.y_min, self.y_min], color='green')
            axes.plot([self.x_min, self.x_min], [self.y_min, self.y_max], color='green')
            axes.plot([self.x_max, self.x_max], [self.y_max, self.y_min], color='green')

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




