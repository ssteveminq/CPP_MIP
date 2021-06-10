"""

Object clustering with k-means algorithm

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import matplotlib.pyplot as plt
import random

# k means parameters
MAX_LOOP = 20
DCOST_TH = 0.05
show_animation = True


def kmeans_clustering(rx, ry, nc):
    clusters = Clusters(rx, ry, nc)
    clusters.calc_centroid()

    pre_cost = float("inf")
    for loop in range(MAX_LOOP):
        print("loop:", loop)
        cost = clusters.update_clusters()
        clusters.calc_centroid()

        d_cost = abs(cost - pre_cost)
        if d_cost < DCOST_TH:
            break
        pre_cost = cost

    return clusters

def Guided_kmeans_clustering(rx, ry, nc, centers, weights=None):

    clusters = Clusters(rx, ry, nc)
    clusters.set_centers(centers)
    if weights!=None:
        clusters.set_weightcenters(weights)
        print("weightedcenters")
        

    pre_cost = float("inf")
    for loop in range(MAX_LOOP):
        cost = clusters.update_weightedclusters()

        d_cost = abs(cost - pre_cost)
        if d_cost < DCOST_TH:
            break
        pre_cost = cost

    return clusters



class Clusters:

    def __init__(self, x, y, n_label):
        self.x = x
        self.y = y
        self.n_data = len(self.x)
        self.n_label = n_label
        self.labels = [random.randint(0, n_label - 1)
                       for _ in range(self.n_data)]
        self.center_x = [0.0 for _ in range(n_label)]
        self.center_y = [0.0 for _ in range(n_label)]
        self.center_weights=[1.0 for _ in range(n_label)]

    def plot_cluster(self, ax=None):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            if ax==None:
                plt.plot(x, y, ".")
            else:
                ax.plot(x,y,"o")

    def calc_num_labels(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            print(label , ": ", len(x))


    def get_centers(self):
        centers=[]
        for i in range(self.n_label):
            centers.append([self.center_x[i], self.center_y[i]])

        return centers

    def set_centers(self, centers):
        # centers=[]
        for i in range(self.n_label):
            self.center_x[i]=centers[i][0]
            self.center_y[i]=centers[i][1]

    def set_weightcenters(self, weights):
        # centers=[]
        if len(weights)==self.n_label:
            for i in range(self.n_label):
                self.center_weights[i]=(1.0/weights[i])

            print("set weights for center")
        else:
            print("input dimension is wrong")



    def calc_centroid(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            n_data = len(x)
            self.center_x[label] = sum(x) / n_data
            self.center_y[label] = sum(y) / n_data

    def update_weightedclusters(self):
        cost = 0.0

        for ip in range(self.n_data):
            px = self.x[ip]
            py = self.y[ip]

            dx = [icx - px for icx in self.center_x]
            dy = [icy - py for icy in self.center_y]

            # dx = [(icx - px) for icx, iw in zip(self.center_x, self.center_weights)]
            # dy = [(icy - py) for icy, iw in zip(self.center_y, self.center_weights)]

            dist_list = [math.hypot(idx, idy) for (idx, idy) in zip(dx, dy)]
            for dist, iw in zip(dist_list, self.center_weights):
                dist = dist*iw

            # print("dist_list", dist_list)
            # input("00--00")
            min_dist = min(dist_list)
            min_id = dist_list.index(min_dist)
            self.labels[ip] = min_id
            cost += min_dist

        return cost



    def update_clusters(self):
        cost = 0.0

        for ip in range(self.n_data):
            px = self.x[ip]
            py = self.y[ip]

            dx = [icx - px for icx in self.center_x]
            dy = [icy - py for icy in self.center_y]

            dist_list = [math.hypot(idx, idy) for (idx, idy) in zip(dx, dy)]
            min_dist = min(dist_list)
            min_id = dist_list.index(min_dist)
            self.labels[ip] = min_id
            cost += min_dist

        return cost

    def _get_labeled_x_y(self, target_label):
        x = [self.x[i] for i, label in enumerate(self.labels) if label == target_label]
        y = [self.y[i] for i, label in enumerate(self.labels) if label == target_label]
        return x, y


def calc_raw_data(cx, cy, n_points, rand_d):
    rx, ry = [], []

    for (icx, icy) in zip(cx, cy):
        for _ in range(n_points):
            rx.append(icx + rand_d * (random.random() - 0.5))
            ry.append(icy + rand_d * (random.random() - 0.5))

    return rx, ry


def update_positions(cx, cy):
    # object moving parameters
    DX1 = 0.4
    DY1 = 0.5
    DX2 = -0.3
    DY2 = -0.5

    cx[0] += DX1
    cy[0] += DY1
    cx[1] += DX2
    cy[1] += DY2

    return cx, cy


def main():
    print(__file__ + " start!!")

    cx = [0.0, 8.0]
    cy = [0.0, 8.0]
    n_points = 10
    rand_d = 3.0
    n_cluster = 2
    sim_time = 15.0
    dt = 1.0
    time = 0.0

    while time <= sim_time:
        print("Time:", time)
        time += dt

        # objects moving simulation
        cx, cy = update_positions(cx, cy)
        raw_x, raw_y = calc_raw_data(cx, cy, n_points, rand_d)

        clusters = kmeans_clustering(raw_x, raw_y, n_cluster)

        # for animation
        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            clusters.plot_cluster()
            plt.plot(cx, cy, "or")
            plt.xlim(-2.0, 10.0)
            plt.ylim(-2.0, 10.0)
            plt.pause(dt)

    print("Done")


if __name__ == '__main__':
    main()
