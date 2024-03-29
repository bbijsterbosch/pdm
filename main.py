import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import time
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from urdfenvs.sensors.full_sensor import FullSensor
from environment.goal import goal1
from local_path_planner import mpc
from global_path_planner.main_global_path_planner import global_path_planner_run
from environment import three_environments
from environment.dynamic_obstacle import dynamicSphereObst1, dynamicSphereObst2
from urdfenvs.urdf_common.urdf_env import UrdfEnv







def run_prius(n_steps=1000, render=True, goal=True, obstacles=True, dynamic_obstacle=True, gather_data=False, noise=False):
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
    dt = 0.1
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
    
    if dynamic_obstacle:
        env.add_obstacle(dynamicSphereObst2)
    # add sensor
    sensor = FullSensor(['position'], ['position', 'size'], variance=0.0)
    env.add_sensor(sensor, [0])
    # Set spaces AFTER all components have been added.
    env.set_spaces()
    
    #env.set_spaces()



    print(f"Initial observation : {ob}")
    

    history = []
    
    # select environment. 0 = easy, 1 = medium, 2 = hard and set animation=True to plot animation of Global Path Planner

    cx, cy, cyaw, ck, _ = global_path_planner_run(env_id=1, animation=True, n_it = 500)

    cyaw = mpc.smooth_yaw(cyaw)
   

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
    

    state = mpc.State(ob)
    target_ind, _ = mpc.calc_nearest_index(state, cx, cy, cyaw, 0)
    sp = mpc.calc_speed_profile(cx, cy, cyaw, mpc.TARGET_SPEED)
    dl = 1.0

    if gather_data:
        t = []
        timer = 0.0
        x = []
        y = []
        yaw = []
        v = []
        delta = []
        x_ref=[] 
        y_ref = []
        yaw_ref = []
        v_ref = []
        delta_ref = []
        v_input = []
        delta_input = []
        v_input_noise = []
        delta_input_noise = []
        computation_times = []
        

    for i in range(n_steps):
        
        ob, *_ = env.step(action)
        state = mpc.State(ob)
        if dynamic_obstacle:
            obs_pos = np.array([[ob['robot_0']['FullSensor']['obstacles'][53]['position'][0],
                                ob['robot_0']['FullSensor']['obstacles'][53]['position'][1],
                                ob['robot_0']['FullSensor']['obstacles'][53]['size'][0], 
                                -0.4]
                                      ]).T
        else:
            dynamic_obst = None
        
        
        

        x0 = [state.x, state.y, state.v, state.yaw]
        

        start_time = time.time()

        xref, target_ind, dref = mpc.calc_ref_trajectory(
                state, cx, cy, cyaw, ck, sp, dl, target_ind)
        
        odelta, oa = None, None
        oa, odelta, ox, oy, oyaw, ov = mpc.iterative_linear_mpc_control(
                xref, ob, x0, dref, oa, odelta, dynamic_obstacle, obs_pos)

        end_time = time.time()

        di, ai = 0.0, 0.0
        if odelta is not None:
            di, ai = odelta[0], oa[0]
            state = mpc.State(ob)

        vi = state.v + ai*dt
        delta_dot = (di - state.delta) / dt
        action = np.array([vi, delta_dot])
        
        computation_time = end_time - start_time 

        if noise:
            gaussian_noise = np.random.normal(0, 0.5)
            vi_noise = vi+gaussian_noise
            delta_dot_noise = delta_dot+gaussian_noise
            action = np.array([vi_noise, delta_dot_noise])
            delta_input_noise.append(delta_dot_noise)
            v_input_noise.append(vi_noise)
        

        if gather_data:
            timer += dt
            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            delta.append(state.delta)
            t.append(timer)
            x_ref.append(xref[0,0])
            y_ref.append(xref[1,0])
            v_ref.append(xref[2,0])
            yaw_ref.append(xref[3,0])
            delta_ref.append(dref)
            delta_input.append(delta_dot)
            v_input.append(vi)
            computation_times.append(computation_time)
            
        if mpc.check_goal(state, goal, target_ind, len(cx)):
            print("Goal Reached!!")
            break
        
        
        history.append(ob)
    env.close()

    return x, y, yaw, v, delta, t, delta, x_ref, y_ref, v_ref, yaw_ref, delta_ref, gather_data, v_input, v_input_noise, delta_input, delta_input_noise, computation_times


if __name__ == "__main__":

    x, y, yaw, v, delta, t,delta, x_ref, y_ref, v_ref, yaw_ref, delta_ref, gather_data, v_input, v_input_noise, delta_input, delta_input_noise, computation_times = run_prius()
    
    #Post Processing and Plotting
    eucledian_distances = []
    for i in range(len(x_ref)):
        eucledian_distance = np.sqrt((x_ref[i] - x[i])**2 + (y_ref[i] - y[i])**2)
        eucledian_distances.append(eucledian_distance)

    orientation_errors = []
    for i in range(len(yaw_ref)):
        orientation_error = min(np.abs(yaw[i] - yaw_ref[i]), (2*np.pi - np.abs(yaw[i]-yaw_ref[i])))
        orientation_errors.append(orientation_error)

    if gather_data:

        plt.subplots()
        plt.plot(x_ref, y_ref, "-r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.xlabel('x[m]')
        plt.ylabel('y[m]')
        plt.axis("equal")
        plt.legend()
        plt.grid(True)

        plt.subplots()
        plt.plot(t, yaw, "-r", label="orientation")
        plt.plot(t, yaw_ref, "-b", label="reference")
        plt.xlabel('t[s]')
        plt.ylabel('Orientation [rad]')
        plt.legend()
        plt.grid(True)

        plt.subplots()
        plt.plot(t, delta, "-r", label="Steering angle")
        plt.xlabel('t[s]')
        plt.ylabel('[$\delta$][rad]')
        plt.legend()
        plt.grid(True)
        plt.show()
        
        plt.clf()
        plt.subplot(3,1,1)
        plt.plot(t, computation_times, "-b", label="Computation time vs t positition")
        plt.ylabel('computation time - MPC[s]')
        plt.xlabel('t[s]')
        plt.axis("equal")
        plt.legend()
        plt.grid(True)

        plt.subplot(3,1,2)
        plt.plot(t, eucledian_distances, "-b", label="Euclidean distance")
        plt.xlabel('t[s]')
        plt.ylabel('Euclidean distance [m]')
        plt.axis("equal")
        plt.legend()
        plt.grid(True)

        plt.subplot(3,1,3)
        plt.plot(t, orientation_errors, "-b", label="Orientation Error")
        plt.xlabel('t[s]')
        plt.ylabel('Yaw Error[rad]')
        plt.axis("equal")
        plt.legend()
        plt.grid(True)

        plt.clf()
        plt.subplot(2,1,1)
        plt.plot(t,  v_input, label='velocity input')
        plt.xlabel('t[s]')
        plt.ylabel('velocity input [m/s]')
        
        plt.subplot(2,1,2)
        plt.plot(t, delta_input, label='Steering input')
        plt.xlabel('t[s]')
        plt.ylabel('steering input [rad/s]')
        
        plt.show()