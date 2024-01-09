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
from three_environments import build_environment

def RRT_star_Dubins(obstacleList, start, goal):
    
    print("Start RRT star with Dubins planning")

    # ====Search Path with RRT====
    obstacleList = obstacleList
      # [x,y,size(radius)]

    # show the RRT search or not
    show_animation = True

    # calculate the path using RRT Start Dubins
    rrtstar_dubins = rrt_star_dubins.RRTStarDubins(start, goal, rand_area=[0.0, 30.0], obstacle_list=obstacleList)
    path = rrtstar_dubins.planning(animation=show_animation)

    # flip the path so it goes from start to finish
    path_arr = np.array(path)
    path_arr = np.flip(path_arr, axis=0)

    # Draw final path
    if show_animation:  # pragma: no cover
        rrtstar_dubins.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001)

        # plt.show()
    
    return path_arr

def cubic_splines(path_arr, obstacleList, env_id):
    
    finish = path_arr[np.shape(path_arr)[0]-1]
    
    if env_id == 0:
        path_arr = path_arr[::40]
    elif env_id == 1:
        path_arr = path_arr[::40]
    else:
        path_arr = path_arr[::40]
    
    path_arr = np.append(path_arr, [finish], axis=0)
    # print(f'The path: {path_arr}\n')
    
    
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
    
    # ryaw = [yaw + np.pi for yaw in ryaw]
    
    plot_ryaw = ryaw[::10]
    plotx = rx[::10]
    ploty = ry[::10]
    
    idx_wrong_K = []    
    for idx, K in enumerate(rk):
        if K > 1:
            print(f'K at {len(rx)-idx} is larger than 1: {K}\n')
            idx_wrong_K.append(idx)
            
    if np.shape(idx_wrong_K)[0] > 0:
        print(f'Indexes of wrong Ks: {idx_wrong_K}\n')
    else:
        print(f'All curvatures satisfy the curvature constraints!\n')
    
    print(f'Number of points: {np.shape(rx)[0]}\n')
    
    
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
        plt.scatter(rx[idx],ry[idx], c="black", label="Turn too sharp")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()
    
    for (ox, oy, size) in obstacleList:
            RRT.plot_circle(ox, oy, size)

    plt.show()
    
    return rx, ry, ryaw, rk, s


    
def global_path_planner_run():
    
    # select environment
    env_id = 0
    
    # set start and goal locationis
    if env_id == 0 or env_id == 1:
        start = [0.0, 0.0, np.deg2rad(90.0)]
        goal = [28.0, 28.0, np.deg2rad(90.0)]
    elif env_id == 2:
        start = [0.0, 0.0, np.deg2rad(90.0)]
        goal = [0.0, 26.0, np.deg2rad(180.0)]   
    
    
    obstacleList = build_environment(env_id) 
    
    path = RRT_star_Dubins(obstacleList, start, goal)
    
    cx, cy, cyaw, ck, s = cubic_splines(path, obstacleList, env_id)
    
    
    return cx, cy, cyaw, ck, s

if __name__ == '__main__':
    global_path_planner_run()