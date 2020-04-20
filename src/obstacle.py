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
        self.vertices.append([self.x_min, self.y_min])
        self.vertices.append([self.x_min, self.y_max])
        self.vertices.append([self.x_max, self.y_min])
        self.vertices.append([self.x_max, self.y_max])
        print("generate_vertices")


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




