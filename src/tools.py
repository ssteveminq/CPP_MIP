import time
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

def polygon_contains_point(point, polygon_vertices):
        point = Point(point[0], point[1])
        polygon = Polygon(polygon_vertices)
        return polygon.contains(point)

def define_polygon(num_pts=4, ax=None):
        def tellme(s):
            print(s)
            plt.title(s, fontsize=16)
            plt.draw()

        if ax==None:
            ax = plt.gca()

        plt.setp(ax, autoscale_on=0)
        ax.set_xlim([-5.0, 5.0])
        ax.set_ylim([-5.0, 5.0])

        tellme('You will define a flight area with %d points.\nClick to begin.'%num_pts)

        plt.waitforbuttonpress()

        while True:
            pts = []
            while len(pts) < num_pts:
                tellme('Select %d corners with mouse'%num_pts)
                pts = np.asarray(plt.ginput(num_pts, timeout=-1))
                if len(pts) < num_pts:
                    tellme('Too few points, starting over')
                    time.sleep(1)  # Wait a second

            ph = plt.fill(pts[:, 0], pts[:, 1], 'y', lw=2)

            tellme('Happy? Key click for yes, mouse click for no')

            if plt.waitforbuttonpress():
                print("keyboard")
                break

            # Get rid of fill
            for p in ph:
                p.remove()

        return pts

# convert the angle into a positive value [0,2pi)
def orientation_processing(angle):
    if angle < -2.0*np.pi:
        angle += (2.0*np.pi)*2.0
        return angle
    if angle == -2.0*np.pi:
        angle = 0.0
        return angle 
    if -2.0*np.pi < angle < 0:
        angle += 2.0*np.pi
    if 0 <= angle < np.2.0*pi:
        return angle
    if 2.0*np.pi <= angle:
        angle -= 2.0*np.pi
        return angle
        
