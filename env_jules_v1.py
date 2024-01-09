import gymnasium as gym
import numpy as np
import sys
import pathlib
import os
import mpc_jules as mpc
sys.path.append(str(pathlib.Path(__file__).parent))

from urdfenvs.urdf_common.bicycle_model import BicycleModel
from real_enviroment.goal_jules_v2 import goal1
from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from urdfenvs.sensors.full_sensor import FullSensor
from real_enviroment.create_all_walls import sphere_list_export
from cubic_spline_planner import main_2d
from reference_path import csteer_bas, cx_bas, cy_bas, cyaw_bas
#For the RRT's
from RRTs.RRT_dubins import RRT_dubins_run
from RRTs.rrt_star_dubins import rrt_star_dubins_run
import model_predictive_speed_and_steer_control
from real_enviroment.goal_jules_v2 import goal_pos
dt = 0.1
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
    
    
    cx, cy, cyaw, ck = main_2d(obs_pos)
    
    # csteer = np.arctan(ck)
    # csteer = list(csteer)
    
    # cx = cx_bas[::-1]
    # cy = cy_bas[::-1]
    # cyaw = cyaw_bas[::-1]
    # csteer = csteer_bas[::-1]
    # ck = np.tan(csteer)

    goal = [cx[-1], cy[-1]]

    # print(f'cx: {cx}')
    # print(f' cy: {cy}')
    # print(f'cyaw: {cyaw}')
    # print(f'csteer: {csteer}')
    pind = 0
    n = 20
    state = model_predictive_speed_and_steer_control.State(ob)
    target_ind, _ = model_predictive_speed_and_steer_control.calc_nearest_index(state, cx, cy, cyaw, 0)
    sp = model_predictive_speed_and_steer_control.calc_speed_profile(cx, cy, cyaw, model_predictive_speed_and_steer_control.TARGET_SPEED)
    dl = 1.0

    for i in range(n_steps):
        ob, *_ = env.step(action)

        delta = ob['robot_0']['joint_state']['steering']
        v = ob['robot_0']['joint_state']['forward_velocity'][0]

        x0 = [state.x, state.y, state.v, state.yaw]
        

        cyaw = model_predictive_speed_and_steer_control.smooth_yaw(cyaw)

        xref, target_ind, dref = model_predictive_speed_and_steer_control.calc_ref_trajectory(
                state, cx, cy, cyaw, ck, sp, dl, target_ind)
        
        odelta, oa = None, None
        oa, odelta, ox, oy, oyaw, ov = model_predictive_speed_and_steer_control.iterative_linear_mpc_control(
                xref, ob, x0, dref, oa, odelta)

        di, ai = 0.0, 0.0
        if odelta is not None:
            di, ai = odelta[0], oa[0]
            state = model_predictive_speed_and_steer_control.State(ob)

        
        vi = v + ai*dt
        delta_dot = (di - delta) / dt
        action = np.array([vi, delta_dot])

        if model_predictive_speed_and_steer_control.check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            break

        print(f'action: {action}')
        # if ob['robot_0']['joint_state']['steering'] > 0.2:
        #     action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
