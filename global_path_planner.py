import math
import numpy as np
import bisect
import sys
import pathlib
import matplotlib.pyplot as plt

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))  # root dir
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from RRTs import rrt_star_dubins
import cubic_spline_planner
from RRTs.rrt import RRT

def RRT_star_Dubins(obstacleList, start, goal):
    
    print("Start RRT star with Dubins planning")

    # ====Search Path with RRT====
    obstacleList = obstacleList
      # [x,y,size(radius)]

    show_animation = True

    rrtstar_dubins = rrt_star_dubins.RRTStarDubins(start, goal, rand_area=[-2.0, 15.0], obstacle_list=obstacleList)
    path = rrtstar_dubins.planning(animation=show_animation)

    path_arr = np.array(path)

    # Draw final path
    if show_animation:  # pragma: no cover
        rrtstar_dubins.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)

        # plt.show()
    
    return path_arr

def cubic_splines(path_arr, obstacleList):
    
    first = path_arr[np.shape(path_arr)[0]-1]
    last = path_arr[0]
    path_arr = path_arr[::40]
    path_arr = np.append(path_arr, [first], axis=0)
    # np.append(path_arr,last)
    # print(f'\nthe path: {path_arr}\n')
    
    
    print("\nRunning CubicSpline\n")
    x = path_arr[:,0]
    y = path_arr[:,1]
    
    ds = 0.1  # [m] distance of each interpolated points

    sp = cubic_spline_planner.CubicSpline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))
    
    ryaw = [yaw + np.pi for yaw in ryaw]
    
    plot_ryaw = ryaw[::10]
    plotx = rx[::10]
    ploty = ry[::10]
    
    idx_wrong_K = []    
    for idx, K in enumerate(rk):
        if K > 1:
            print(f'K at {len(rx)-idx} is larger than 1: {K}\n')
            idx_wrong_K = np.append(idx_wrong_K, idx)
    
    print(f'Number of points: {np.shape(rx)[0]}\n')
    
    print(f'Indexes of wrong Ks: {idx_wrong_K}\n')
        
    
    spline_points = np.zeros((len(rx),2))
    for i in range(len(rx)):
        spline_points[i] = (rx[i],ry[i])
        
    if cubic_spline_planner.check_collision(spline_points, obstacleList):
        print("Collision detected! Adjust spline generation.\n")
    else:
        print("No collision detected. Spline path is safe.\n")

    plt.subplots(1)
    plt.plot(x, y, "xb", label="Data points")
    plt.plot(rx, ry, "-r", label="Cubic spline path")
    plt.quiver(plotx, ploty, np.cos(plot_ryaw), np.sin(plot_ryaw), color='g', units='xy', scale=5, width=0.03, label='Yaw')
    for idx in idx_wrong_K:
        plt.scatter(rx[idx],ry[idx], "-r", label="Turn too sharp")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()
    
    for (ox, oy, size) in obstacleList:
            RRT.plot_circle(ox, oy, size)

    plt.show()
    
    return rx, ry, ryaw, rk


    
def main():
    
    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(90.0)]
    goal = [14.0, 4.0, np.deg2rad(90.0)]
    
    obstacleList = [(4,5,1),
                (4,1,1),
                (4,3,1), 
                (4,7,1) , 
                (4,-1,1),
                (4,-3,1),
                (0,14,1),
                (2,14,1),
                (4,14,1),
                (6,14,1),
                (8,14,1),
                (10,14,1),
                (12,14,1),
                (14,14,1),
                (16,14,1),
                (10,12,1),
                (10,10,1),
                (10,8,1),
                (10,6,1),
                (10,4,1),
                ]
    
    path = RRT_star_Dubins(obstacleList, start, goal)
    
    cx, cy, cyaw, ck = cubic_splines(path, obstacleList)
    
    
    return cx, cy, cyaw, ck

if __name__ == '__main__':
    main()