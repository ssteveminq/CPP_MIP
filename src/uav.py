from gurobipy import *
import gurobipy as grb
import numpy as np
from math import *

class UAV:
    def __init__(self, mass: float, dt: float, T: float, x0: float, y0: float, th0: float, v0: float, id: int, model, M, v_max, a_max, area_size, x_fin: float, y_fin: float, wp: bool, x_wp=None, y_wp=None):
        self.steps = int(T/dt)        # number of time steps
        self.id = id                  # vehicle id
        self.dt = dt                  # time step size
        self.x0 = x0                  # vehicle initial position x-coordinate
        self.y0 = y0                  # vehicle initial position y-coordinate
        self.th0 = th0                # vehicle initial orientation (radian)
        self.v0 = v0                # vehicle initial orientation (radian)
        self.x_fin = x_fin            # vehicle final position x-coordinate
        self.y_fin = y_fin            # vehicle final position y-coordinate
        self.m = mass                 # vehicle mass
        self.v_max = v_max            # maximum velocity
        self.wp = wp                  # switch of waypoints
        self.x_wp = x_wp              # x-coordinate of waypoints
        self.y_wp = y_wp              # y-coordinate of waypoints
        self.a_max = a_max            # max force magnitude to be applied
        self.v_max = v_max            # max allowed velocity
        self.model = model
        self.M = M
        self.FixFin = False

        # Add variable arrays, length = amount of steps
        self.x = self.model.addVars(self.steps, lb=-area_size, ub=area_size)     # x-ccordinate at each time step
        self.y = self.model.addVars(self.steps, lb=-area_size, ub=area_size)     # y-coordinate at each time step
        self.th = self.model.addVars(self.steps, lb=0, ub=2*math.pi)     # y-coordinate at each time step
        self.v = self.model.addVars(self.steps, lb=0.0, ub=v_max)   # velocity x-component at each time step
        self.a = self.model.addVars(self.steps, lb=-a_max, ub=a_max)   # velocity x-component at each time step
        self.phi = self.model.addVars(self.steps, lb=0.0, ub=2*math.pi)   # velocity x-component at each time step
        print("self.x0:", x0)
        print("self.y0:", y0)
        print("self.th0:", th0)
        print("self.v0:", v0)
        # print("self.a0:", a0)

    def approximate_cos(slef, x):
        x_aug = x / (2*math.pi)
        output =0
        if x_aug >=0 and x_aug <= 1/8:
            output=1
        elif x_aug<= 3/8:
            output = 2-8*x_aug
        elif x_aug<= 5/8:
            output = -1
        elif x_aug<= 7/8:
            output = -6+8*x_aug 
        elif x_aug<=1:
            output = 1
        return output

    def approximate_sin(slef, x):
        x_aug = x / (2*math.pi)
        output =0
        if x_aug >=0 and x_aug <= 1/8:
            output=8*x_aug
        elif x_aug<= 3/8:
            output = 1
        elif x_aug<= 5/8:
            output = +4-8*x_aug
        elif x_aug<= 7/8:
            output = -1
        elif x_aug<=1:
            output = -8+8*x_aug
        return output



    def constrain_dynamics(self, v_init):
        # Add constrain to each variable in the respective array
        # Step position and velocity constraints
        print("x_i: ", self.x[0])
        print("y_i: ", self.y[0])
        print("v_i: ", self.x[0])
        print("th_i: ", self.th[0])
        print("steps: ", self.steps)
        self.model.addConstrs((self.x[i+1] == (self.x[i] + self.dt*self.v[i]*1.0*(self.th[i])) for i in range(self.steps-1)))
        self.model.addConstrs((self.y[i+1] == (self.y[i] + self.dt*self.v[i]*1.0*(self.th[i])) for i in range(self.steps-1)))
        self.model.addConstrs((self.th[i+1] == (self.th[i] +(self.phi[i])*self.dt) for i in range(self.steps-1)))

        # self.model.addConstrs((self.x[i+1] == (self.x[i] + self.dt*self.v[i]*cos(self.th[i])) for i in range(self.steps-1)))
        # self.model.addConstrs((self.y[i+1] == (self.y[i] + self.dt*self.v[i]*sin(self.th[i])) for i in range(self.steps-1)))
        # self.model.addConstrs((self.th[i+1] == (self.th[i] + sin(self.phi[i])*self.dt) for i in range(self.steps-1)))
        self.model.addConstrs((self.v[i+1] == (self.v[i] + self.a[i]*self.dt) for i in range(self.steps-1)))

        # Initial velocity constraint
        self.model.addConstr(self.v[0] == v_init)
        self.model.addConstr(self.a[0] == 0.0)
        self.model.addConstr(self.phi[0] ==0.0)

        # Maximum velocity constrains making use of the sine and cosines
        # for m_small in range(1,self.M+1):
            # self.model.addConstrs((self.fx[i] * np.cos(2*np.pi * m_small / self.M) + self.fy[i] * np.sin(2*np.pi*m_small/self.M) <= self.fm[i] for i in range(self.steps)), name=("fm_cons_" + str(m_small)) )
        # for m_small in range(1,self.M+1):
            # self.model.addConstrs((self.vx[i] * np.cos(2*np.pi * m_small / self.M) + self.vy[i] * np.sin(2*np.pi*m_small/self.M) <= self.v_max for i in range(self.steps)), name=("vmax_cons_" + str(m_small)) )
        # for m_small in range(1,self.M+1):
            # self.model.addConstrs((self.fx[i] * np.cos(2*np.pi * m_small / self.M) + self.fy[i] * np.sin(2*np.pi*m_small/self.M) <= self.f_max for i in range(self.steps)), name=("fmax_cons_" + str(m_small)) )

    def constrain_positions(self):
        # Initial position constraint
        self.model.addConstr(self.x[0] == self.x0)
        self.model.addConstr(self.y[0] == self.y0)
        self.model.addConstr(self.th[0] == self.th0)
        self.model.addConstr(self.v[0] == self.v0)

        print("x0:", self.x[0])
        print("y0:", self.y[0])
        input()

        R = 100000 
        # Final position constraint (used in the objective function) --> Do we need this for coverage planning?
        if self.FixFin:
            self.b = self.model.addVars(self.steps, lb=0, vtype=GRB.BINARY)
            for t_step in range(self.steps):
                self.model.addConstr(self.x[t_step] - self.x_fin <= R*(1 - self.b[t_step]))
                self.model.addConstr(self.x[t_step] - self.x_fin >= - R * (1 - self.b[t_step]))
                self.model.addConstr(self.y[t_step] - self.y_fin <= R * (1 - self.b[t_step]))
                self.model.addConstr(self.y[t_step] - self.y_fin >= - R * (1 - self.b[t_step]))

            self.model.addConstr(self.b.sum() == 1)

    def constrain_obstacles(self, obstacles, d_obs):
        # Obstacle constraints
        R = 100000                    # high value factor
        for obs in obstacles:

            c = self.model.addVars(4, self.steps, lb=0, vtype=GRB.BINARY)

            self.model.addConstrs((self.x[i] - obs.x_max >= d_obs-R*c[0, i] for i in range(self.steps-1)))
            self.model.addConstrs((-self.x[i] + obs.x_min >= d_obs-R*c[1, i] for i in range(self.steps-1)))
            self.model.addConstrs((self.y[i] - obs.y_max >= d_obs-R*c[2, i] for i in range(self.steps-1)))
            self.model.addConstrs((-self.y[i] + obs.y_min >= d_obs-R*c[3, i] for i in range(self.steps-1)))

            self.model.addConstrs((c[0, i] + c[1, i] + c[2, i] + c[3, i] <= 3 for i in range(self.steps-1)))


    def constrain_multiple_vehicles(self, vehicles, d_veh):
        # UAV collision constraints
        new_vehicles = vehicles[0:self.id]+vehicles[self.id+1:len(vehicles)]  # list of vehicles excluding current object
        R = 100000
        for veh in new_vehicles:
            e = self.model.addVars(4, self.steps, lb=0, vtype=GRB.BINARY)

            self.model.addConstrs((self.x[i] - veh.x[i] >= d_veh - R * e[0, i] for i in range(self.steps - 1)))
            self.model.addConstrs((-self.x[i] + veh.x[i] >= d_veh - R * e[1, i] for i in range(self.steps - 1)))
            self.model.addConstrs((self.y[i] - veh.y[i] >= d_veh - R * e[2, i] for i in range(self.steps - 1)))
            self.model.addConstrs((-self.y[i] + veh.y[i] >= d_veh - R * e[3, i] for i in range(self.steps - 1)))

            self.model.addConstrs((e[0, i] + e[1, i] + e[2, i] + e[3, i] <= 3 for i in range(self.steps - 1)))


    def constrain_waypoints(self):
        # Waypoints constraints
        R = 100000
        self.kset=[]
        self.Tf = self.model.addVar(lb=0, vtype=GRB.INTEGER, name="tf")
        if self.wp:
            for i in range(len(self.x_wp)):

                k = self.model.addVars(self.steps, lb=0, vtype=GRB.BINARY)
                self.kset.append(k)

                #release this constraint when there is no fix final position constraint
                if self.FixFin:
                    for j in range(self.steps):
                        # Constraint that vehicle must pass through waypoint before reaching destination
                        self.MOdel.addConstr(self.kset[i][j] <= 1 - quicksum(self.b[i] for i in range(j+1)))
                for t_step in range(self.steps):
                    self.model.addConstr(self.x[t_step] - self.x_wp[i] <= R * (1 - self.kset[i][t_step]))
                    self.model.addConstr(self.x[t_step] - self.x_wp[i] >= - R * (1 - self.kset[i][t_step]))
                    self.model.addConstr(self.y[t_step] - self.y_wp[i] <= R * (1 - self.kset[i][t_step]))
                    self.model.addConstr(self.y[t_step] - self.y_wp[i] >= - R * (1 - self.kset[i][t_step]))

                self.model.addConstr(self.kset[i].sum() == 1)
                self.model.addConstr((quicksum(j*self.kset[i][j] for j in range(self.steps)) >= self.Tf))


            #for single agent
            # self.Tf = self.model.addVar(lb=0, vtype=GRB.INTEGER, name="tf")
        # z=list(wp_times.keys())[-1]           #printing the final time
     


