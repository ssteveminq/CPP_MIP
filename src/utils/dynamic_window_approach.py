"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt

show_animation = True

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 50.0 * math.pi / 180.0  # [rad/s]
        # self.max_accel = 0.2  # [m/ss]
        self.max_accel = 0.5  # [m/ss]
        self.max_dyawrate = 50.0 * math.pi / 180.0  # [rad/ss]
        # self.v_reso = 0.01  # [m/s]
        self.v_reso = 0.02  # [m/s]
        self.yawrate_reso = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.robot_radius = 1.0  # [m]


def motion(x, u, dt):
    # motion model

    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]   #velocity
    x[4] = u[1]   #angular velocity

    return x


def calc_dynamic_window(x, config):
    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]
    #  print(Vs, Vd)

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    # print(dw)
    # input("dw")

    return dw


def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    #  print(len(traj))
    return traj


def calc_final_input(x, u, dw, config, goal, ob):
    # print("x", x)
    # input("here)

    xinit = x[:]
    # print("xinit", xinit)
    min_cost = 10000.0
    min_u = u
    if x[0]<goal[0]:
        min_u[0] = 0.15
    else:
        min_u[0] = -0.15
    min_u[1] = 0.1
    best_traj = np.array([x])
    boolsolution=False


    # evaluate all trajectory with sampled input in dynamic window
    # print("dw[0], dw[1]", dw[0],", ", dw[1])
    # print("dw[2], dw[3]", dw[2],", ", dw[3])
    # input()
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            # print("v: , ", v, ", y", y)
            # input()
            traj = calc_trajectory(xinit, v, y, config)

            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost
            # print("final_cost: ", final_cost)

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj
                boolsolution=True
            # else:
                # print("no sol??")

    # print(min_u)
    if boolsolution==False:
        print("boolsolution faield")
        dx = goal[0] - x[0]
        dy = goal[1] - x[1]
        dist_to_goal = (dx ** 2 + dy ** 2)**0.5
        # print("dist_to_goal", dist_to_goal )
        input_v = 0.35 * dist_to_goal

        des_phi = math.atan2(goal[1] - x[1], goal[0] - x[0])
        # if des_phi<0.0:
        if des_phi<-math.pi:
            des_phi += math.pi*2
        cur_yaw = x[2]
        input_phi = 0.9*math.sin(des_phi-cur_yaw)
        min_u[0]=input_v
        min_u[1]=input_phi
 

    return min_u, best_traj


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")
    # print("length of ob", len(ob))

    for ii in range(0, len(traj[:, 1]), skip_n):
        # for i in range(len(ob[:, 0])):
        for i in range(len(ob)):
            # ox = ob[i, 0]
            # oy = ob[i, 1]
            ox = ob[i][0]
            oy = ob[i][1]
            # print("ox", "oy", ox, ", ", oy)
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy
            r = math.sqrt(dx**2 + dy**2)
            # print("traj[0]: ", traj[ii,0], ", dy: ", traj[ii,1])
            # print("dx: ", dx, ", dy: ", dy, "r = ", r)

            if r <= config.robot_radius:
                # print("collison!!")
                return float("Inf")  # collisiton

            if minr >= r:
                minr = r

    # print("minr", minr)

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    dy = goal[0] - traj[-1, 0]
    dx = goal[1] - traj[-1, 1]
    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = config.to_goal_cost_gain * goal_dis

    return cost


def dwa_control(x, u, goal, ob, config):
    # Dynamic Window control
    # Convert 
    # x = np.array([x[0], x[1], x[2], x[3], u[1]])
    dw = calc_dynamic_window(x, config)

    u, traj = calc_final_input(x, u, dw, config, goal, ob)

    return u, traj


def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def main():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([10, 10])
    # obstacles [x(m) y(m), ....]
    ob = np.matrix([[-1, -1],
                    [0, 2],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [7.0, 9.0],
                    [12.0, 12.0]
                    ])

    u = np.array([0.0, 0.0])
    config = Config()
    traj = np.array(x)

    for i in range(1000):
        u, ltraj = dwa_control(x, u, config, goal, ob)

        x = motion(x, u, config.dt)
        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.show()


if __name__ == '__main__':
    main()
