import gymnasium as gym
import numpy as np
import sys
import pathlib
import os
import mpc_jules as mpc
sys.path.append(str(pathlib.Path(__file__).parent))

from urdfenvs.urdf_common.bicycle_model import BicycleModel
from real_enviroment.goal_jules_v2 import goal1
from real_enviroment import goal_jules_v2 
from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from urdfenvs.sensors.full_sensor import FullSensor
from real_enviroment.create_all_walls import sphere_list_export
from cubic_spline_planner import main_2d
from global_path_planner import global_path_planner_run
from reference_path import csteer_bas, cx_bas, cy_bas, cyaw_bas
#For the RRT's
from RRTs.RRT_dubins import RRT_dubins_run
from RRTs.rrt_star_dubins import rrt_star_dubins_run

from real_enviroment.goal_jules_v2 import goal_pos

#For the local path planner lqr
from lqr_speed_steer_control import lqr_run

def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
    robots = [
        BicycleModel(
            urdf='prius.urdf',
            mode="vel",
            scaling=0.3,
            wheel_radius = 0.31265,
            wheel_distance = 0.494,
            spawn_offset = np.array([-0.435, 0.0, 0.05]),
            actuated_wheels=['front_right_wheel_joint', 'front_left_wheel_joint', 'rear_right_wheel_joint', 'rear_left_wheel_joint'],
            steering_links=['front_right_steer_joint', 'front_left_steer_joint'],
        )
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    action = np.array([0, 0])
    pos0 = np.array([0.0, 0.0, 0.0])
    vel0 = np.array([0.0, 0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)
    
    # add walls
    for sphere_i in sphere_list_export:
        env.add_obstacle(sphere_i)

    # add goal
    env.add_goal(goal1)
    
    # add sensor
    sensor = FullSensor(['position'], ['position', 'size'], variance=0.0)
    env.add_sensor(sensor, [0])
    # Set spaces AFTER all components have been added.
    env.set_spaces()
    
    #env.set_spaces()

    print(f"Initial observation : {ob}")
    

    history = []
    # Perform action 
    action = np.zeros(env.n())
    ob, *_ = env.step(action)
    obst_dict = ob['robot_0']['FullSensor']['obstacles']
    
    obstacles = [obstacle for obstacle in obst_dict]
    obs_pos = []

    for i in obstacles:
        x = obst_dict[i]['position'][0]
        y = obst_dict[i]['position'][1]
        rad = obst_dict[i]['size'][0]
        obs_pos.append((x,y,rad))
    
    
    cx, cy, cyaw, ck, s = global_path_planner_run()
    
    goal = goal_jules_v2.goal1Dict["desired_position"]
    lqr_run(cx, cy, cyaw, ck, s, (goal[0],goal[1]))


    csteer = np.arctan(ck)
    # csteer = list(csteer)
    
    cx = cx_bas
    cy = cy_bas
    cyaw = cyaw_bas
    csteer = csteer_bas
    # print(f'cx: {cx}')
    # print(f' cy: {cy}')
    # print(f'cyaw: {cyaw}')
    # print(f'csteer: {csteer}')
    pind = 0
    n = 20

    for i in range(n_steps):
        ob, *_ = env.step(action)
        #ox, oy, o_yaw, o_steer, u_v, u_steer_vel, index_near = mpc.run_mpc(ob, cx, cy, cyaw, csteer, pind, n)
        #pind = index_near
        #action = np.array([u_v[0], o_yaw[0]])
        print(f'action: {action}')
        if ob['robot_0']['joint_state']['steering'] > 0.2:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
