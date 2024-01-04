import gymnasium as gym
import numpy as np

from urdfenvs.urdf_common.bicycle_model import BicycleModel

from obstacles_self import wall_obstacles_jules
from goal_jules import goal1
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from urdfenvs.sensors.full_sensor import FullSensor
from sphere_obstacles import sphere_list_export

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
    #sensor = ObstacleSensor()
    #env.add_sensor(sensor, [0])
    # add sensor
    sensor = FullSensor(['position'], ['position', 'size'], variance=0.0)
    env.add_sensor(sensor, [0])
    # Set spaces AFTER all components have been added.
    env.set_spaces()
    
    #env.set_spaces()



    print(f"Initial observation : {ob}")
    

    history = []
    for i in range(n_steps):
        ob, *_ = env.step(action)
        if i == 1:
            print(ob['robot_0']['FullSensor'])
        if ob['robot_0']['joint_state']['steering'] > 0.2:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
