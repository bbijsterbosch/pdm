import gymnasium as gym
import numpy as np
import sys
import pathlib
import os
sys.path.append(str(pathlib.Path(__file__).parent))


from urdfenvs.urdf_common.bicycle_model import BicycleModel
from real_enviroment.goal_jules_v2 import goal1
from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from urdfenvs.sensors.full_sensor import FullSensor
from real_enviroment.create_all_walls import sphere_list_export
from cubic_spline_planner import main_2d
from reference_path import cx_bas, cy_bas, cyaw_bas, ck_bas
#For the RRT's
from RRTs.RRT_dubins import RRT_dubins_run
from RRTs.rrt_star_dubins import rrt_star_dubins_run
import mpc
from real_enviroment.goal_jules_v2 import goal_pos
from global_path_planner import global_path_planner_run
import three_environments
from dynamic_obstacle import dynamicSphereObst1, dynamicSphereObst2
from urdfenvs.urdf_common.urdf_env import UrdfEnv






dt = 0.1
def run_prius(n_steps=3000, render=False, goal=True, obstacles=True):
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
    env: UrdfEnv = gym.make(
        "urdf-env-v0",
        dt=0.1, robots=robots, render=render
    )
    action = np.array([1, 0])
    pos0 = np.array([-13.0, -13.0, np.pi*0.5])
    vel0 = np.array([0.0, 0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)
    
    # add walls
    environment = three_environments.build_environment(env_id = 1)  # Choose the environment (0 for easy, 1 for medium, 2 for hard)
    converted_spheres = three_environments.circles_to_spheres(environment)  
    
    for sphere in converted_spheres:
        env.add_obstacle(sphere)

    # add goal
    
    env.add_goal(goal1)
    env.add_obstacle(dynamicSphereObst1)
    env.add_obstacle(dynamicSphereObst2)
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
    
    
    # select environment. 0 = easy, 1 = medium, 2 = hard

    cx, cy, cyaw, ck, _ = global_path_planner_run(env_id=1)
    # cx, cy, cyaw, ck, = cx_bas, cy_bas, cyaw_bas, ck_bas

    # print(f"THIS IS THE CX {cx}", "\n \n")
    # print(f"THIS IS THE Cy {cy}", "\n \n")
    # print(f"THIS IS THE Cyaw {cyaw}", "\n \n")
    # print(f"THIS IS THE Ck {ck}", "\n \n")

    goal = [cx[-1], cy[-1]]

    
    #Adding path in the environment
    path_positions = []
    for i in range(0, len(cx), 10):

        xpath_i = cx[i]
        ypath_i = cy[i]
        # print(xpath_i, ypath_i)
        path_positions.append([xpath_i, ypath_i, 0])
        env.add_visualization(shape_type="cylinder", size=[0.10, 0.005], rgba_color=np.array([1, 0, 0, 1]))  # Small cylinder
    

  
    env.update_visualizations(positions=path_positions)
    




    ########################
        
    pind = 0
    n = 20
    state = mpc.State(ob)
    target_ind, _ = mpc.calc_nearest_index(state, cx, cy, cyaw, 0)
    sp = mpc.calc_speed_profile(cx, cy, cyaw, mpc.TARGET_SPEED)
    dl = 1.0

    for i in range(n_steps):
        ob, *_ = env.step(action)

        dynamic_obst = np.array([[ob['robot_0']['FullSensor']['obstacles'][49]['position'][0],
                                    ob['robot_0']['FullSensor']['obstacles'][49]['position'][1],
                                      ob['robot_0']['FullSensor']['obstacles'][49]['size'][0], -0.4],
                                      [ob['robot_0']['FullSensor']['obstacles'][50]['position'][0],
                                      ob['robot_0']['FullSensor']['obstacles'][50]['position'][1],
                                      ob['robot_0']['FullSensor']['obstacles'][50]['size'][0], 0.4]])
        
        print(dynamic_obst)
        delta = ob['robot_0']['joint_state']['steering']
        v = ob['robot_0']['joint_state']['forward_velocity'][0]

        x0 = [state.x, state.y, state.v, state.yaw]
        

        cyaw = mpc.smooth_yaw(cyaw)

        xref, target_ind, dref = mpc.calc_ref_trajectory(
                state, cx, cy, cyaw, ck, sp, dl, target_ind)
        
        odelta, oa = None, None
        oa, odelta, ox, oy, oyaw, ov = mpc.iterative_linear_mpc_control(
                xref, ob, x0, dref, oa, odelta, dynamic_obst)

        di, ai = 0.0, 0.0
        if odelta is not None:
            di, ai = odelta[0], oa[0]
            state = mpc.State(ob)

        vi = v + ai*dt
        delta_dot = (di - delta) / dt
        action = np.array([vi, delta_dot])
        
        
        

        if mpc.check_goal(state, goal, target_ind, len(cx)):
            print("Goal Reached!!")
            break

        print(f'action: {action}')
        # if ob['robot_0']['joint_state']['steering'] > 0.2:
        #     action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
