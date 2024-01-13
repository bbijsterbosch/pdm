import numpy as np
import matplotlib.pyplot as plt

from RRTs import rrt_star_dubins
import cubic_spline_planner
from RRTs.rrt import RRT
from three_environments import build_environment
from RRTs.RRT_dubins import RRTDubins
from RRTs.rrt_star import RRTStar
from datetime import datetime

def RRT_dubins_run(obstacles1, goal_pos, start_pos):

    print("Start " + __file__)
    # ====Search Path with RRT====
    obstacleList = obstacles1  # [x,y,size(radius)]
    
    show_animation = True

    rrt_dubins = RRTDubins(goal_pos, start_pos, obstacle_list=obstacleList, rand_area=[-15., 15.0])
    path = rrt_dubins.planning(animation=show_animation)
    
    # flip the path so it goes from start to finish
    path_arr = np.array(path)
    path_arr = np.flip(path_arr, axis=0)
    
    # Draw final path
    if show_animation:  # pragma: no cover
        rrt_dubins.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(False)
        plt.pause(0.001)
        plt.show()
        
    return path_arr

def RRT_star_Dubins(obstacleList, start, goal):
    
    print("Start RRT star with Dubins planning")

    # ====Search Path with RRT====
    obstacleList = obstacleList
      # [x,y,size(radius)]

    # show the RRT search or not
    show_animation = True

    # calculate the path using RRT Start Dubins
    rrtstar_dubins = rrt_star_dubins.RRTStarDubins(start, goal, rand_area=[-15., 15.], obstacle_list=obstacleList)
    path = rrtstar_dubins.planning(animation=show_animation)

    # flip the path so it goes from start to finish
    path_arr = np.array(path)
    path_arr = np.flip(path_arr, axis=0)

    # Draw final path
    if show_animation:  # pragma: no cover
        rrtstar_dubins.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(False)
        # plt.xlabel("x[m]")
        # plt.ylabel("y[m]")
        plt.pause(0.001)

        # plt.show()
    
    return path_arr

def rrt_star_run(obstacle1, goal_pos, start_pos):
    
    # ====Search Path with RRT====
    obstacle_list = obstacle1
    
    show_animation = False

    # Set Initial parameters
    rrt_star = RRTStar(
        start=start_pos,
        goal=goal_pos,
        rand_area=[0, 30],
        obstacle_list=obstacle_list,
        expand_dis=5,
        robot_radius=2.0)
    path = rrt_star.planning(animation=show_animation)
    
    # flip the path so it goes from start to finish
    path_arr = np.array(path)
    path_arr = np.flip(path_arr, axis=0)

    if path is None:
        print("Cannot find path!")
    else:
        print("found path!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r')
            plt.grid(False)
            # plt.show()
    return path_arr

def cubic_splines(path_arr, obstacleList, env_id):
    
    # finish = path_arr[np.shape(path_arr)[0]-1]
    path_rrt = path_arr
    rrt_x = path_rrt[:,0]
    rrt_y = path_rrt[:,1]
    

    path_arr = path_arr[::40]

    
    # path_arr = np.append(path_arr, [finish], axis=0)
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
        
    # Update the path based on limited curvature
    for i, curvature in enumerate(rk):
        if curvature > 1.0:
            print(f'CURVATURE {i} IS LARGER THAN 1!\n')
            rk[i] = 1.0  # Limit curvature to 1.0
            rx[i], ry[i] = sp.calc_position(s[i])  # Update path point based on limited curvature

    idx_wrong_K = []    
    for idx, K in enumerate(rk):
        if K > 1:
            print(f'K at {len(rx)-idx} is larger than 1: {K}\n')
            idx_wrong_K.append(idx)
            
    if np.shape(idx_wrong_K)[0] > 0:
        print(f'Indexes of wrong Ks: {idx_wrong_K}\n')
    else:
        print(f'All curvatures satisfy the curvature constraints!\n')
    
    print(f'Number of points:               {np.shape(rx)[0]} points\n')
    
    spline_points = np.zeros((len(rx),2))
    for i in range(len(rx)):
        spline_points[i] = (rx[i],ry[i])
        
    # calculate length of the path  
    length = 0.0
    for i in range(1, len(rx)):
        # Calculate distance between consecutive points and add to total length
        dx = rx[i] - rx[i - 1]
        dy = ry[i] - ry[i - 1]
        length += np.sqrt(dx ** 2 + dy ** 2)
    print(f"Total path length:              {length:.2f} meters\n")

        
    if cubic_spline_planner.check_collision(spline_points, obstacleList):
        print("Collision detected! Adjust spline generation.\n")
    else:
        print("No collision detected. Spline path is safe.\n")

    plt.subplots(1)
    plt.plot(rrt_x, rrt_y, "-b", label="RRT-star Dubins path")
    plt.plot(rx, ry, "-r", label="Cubic spline path")
    for idx in idx_wrong_K:
        plt.scatter(rx[idx],ry[idx], c="black")
    plt.grid(False)
    plt.title(f'Total path length: {length:.2f} meters\n')
    plt.axis([-17, 17, -17, 17])
    plt.legend()
    
    for (ox, oy, size) in obstacleList:
            RRT.plot_circle(ox, oy, size)

    plt.show()
    
    return rx, ry, ryaw, rk, s


    
def global_path_planner_run(env_id):
    
    # select environment. 0 = easy, 1 = medium, 2 = hard
    
    # set start and goal locationis
    if env_id == 0 or env_id == 1:
        start = [-13., -13., np.deg2rad(90.0)]
        goal = [13.0, 13.0, np.deg2rad(90.0)]
    elif env_id == 2:
        start = [0.0, 0.0, np.deg2rad(90.0)]
        goal = [0.0, 26.0, np.deg2rad(180.0)]   
    
    # retreive the obstacles of the environment
    obstacleList = build_environment(env_id) 
    
    # use RRT star Dubins for the path planning
    path = RRT_star_Dubins(obstacleList, start, goal)
    # path = RRT_dubins_run(obstacleList, start, goal)
    # path = rrt_star_run(obstacleList, goal, start)    
    
    # use cubic splines to smoothen the path
    cx, cy, cyaw, ck, s = cubic_splines(path, obstacleList, env_id)
    
    # calculate the difference in curvature
    k_diffs = []
    for i in range(len(ck)):
        diff_k = ck[i] - ck[i-1]
        k_diffs = np.append(k_diffs, diff_k)
        
    av_k_diff = np.average(k_diffs,axis=0)
    print(f'average curvature difference:   {av_k_diff}\n')
        
        

    
    return cx, cy, cyaw, ck, s


if __name__ == '__main__':
    
    # start timer to see how long the algorithm takes
    start_time = datetime.now()
    
    # select environment. 0 = easy, 1 = medium, 2 = hard
    global_path_planner_run(env_id = 1)
    
    # stop the timer
    end_time = datetime.now()
    elapsed_time = end_time - start_time
    print(f'time elapsed:                   {elapsed_time}')