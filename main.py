import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from urdfenvs.sensors.full_sensor import FullSensor

from environment.goal import goal1
from local_path_planner import mpc
from global_path_planner.main_global_path_planner import global_path_planner_run
from environment import three_environments
from environment.dynamic_obstacle import dynamicSphereObst1, dynamicSphereObst2
from urdfenvs.urdf_common.urdf_env import UrdfEnv






dt = 0.1
def run_prius(n_steps=3000, render=False, goal=True, obstacles=True, dynamic_obstacle=False, gather_data=True):
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
    action = np.array([0, 0])
    pos0 = np.array([-13.0, -13.0, np.pi*0.5])
    vel0 = np.array([0.0, 0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)
    
    ob, *_ = env.step(action)
    # add walls
    environment = three_environments.build_environment(env_id = 1)  # Choose the environment (0 for easy, 1 for medium, 2 for hard)
    converted_spheres = three_environments.circles_to_spheres(environment)  
    
    for sphere in converted_spheres:
        env.add_obstacle(sphere)

    # add goal
    
    env.add_goal(goal1)
    if dynamic_obstacle:
        env.add_obstacle(dynamicSphereObst1)
    # add sensor
    sensor = FullSensor(['position'], ['position', 'size'], variance=0.0)
    env.add_sensor(sensor, [0])
    # Set spaces AFTER all components have been added.
    env.set_spaces()
    
    #env.set_spaces()



    print(f"Initial observation : {ob}")
    

    history = []
    
    # select environment. 0 = easy, 1 = medium, 2 = hard

    cx, cy, cyaw, ck, _ = global_path_planner_run(env_id=1, animation=False)
    # cx, cy, cyaw, ck, = cx_bas, cy_bas, cyaw_bas, ck_bas
    cyaw = mpc.smooth_yaw(cyaw)
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

    if gather_data:
        t = []
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        delta = [state.delta]
        


    for i in range(n_steps):
        ob, *_ = env.step(action)
        state = mpc.State(ob)
        if dynamic_obstacle:
            dynamic_obst = np.array([[ob['robot_0']['FullSensor']['obstacles'][54]['position'][0],
                                      ob['robot_0']['FullSensor']['obstacles'][54]['position'][1],
                                      ob['robot_0']['FullSensor']['obstacles'][54]['size'][0], 
                                      -0.4]
                                      ])
        else:
            dynamic_obst = None
        
        
        

        x0 = [state.x, state.y, state.v, state.yaw]
        

        

        xref, target_ind, dref = mpc.calc_ref_trajectory(
                state, cx, cy, cyaw, ck, sp, dl, target_ind)
        
        odelta, oa = None, None
        oa, odelta, ox, oy, oyaw, ov = mpc.iterative_linear_mpc_control(
                xref, ob, x0, dref, oa, odelta, dynamic_obstacle)

        di, ai = 0.0, 0.0
        if odelta is not None:
            di, ai = odelta[0], oa[0]
            state = mpc.State(ob)

        vi = state.v + ai*dt
        delta_dot = (di - state.delta) / dt
        action = np.array([vi, delta_dot])
        
        time += dt

        if gather_data:
            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            delta.append(state.delta)
            t.append(time)

        if mpc.check_goal(state, goal, target_ind, len(cx)):
            print("Goal Reached!!")
            break

        print(f'action: {action}')
        # if ob['robot_0']['joint_state']['steering'] > 0.2:
        #     action[1] = 0
        history.append(ob)
    env.close()

    if gather_data:
        
        plt.plot(ox, oy, "xr", label="MPC")
        plt.plot(cx, cy, "-r", label="course")
        plt.plot(x, y, "ob", label="trajectory")
        plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
        plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
        
        plt.axis("equal")
        plt.grid(True)
        plt.title("Time[s]:" + str(round(t, 2))
                + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
        plt.pause(0.0001)


    return history


if __name__ == "__main__":
    run_prius(render=False)
    