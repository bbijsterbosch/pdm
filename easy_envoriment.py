import gymnasium as gym
import numpy as np

from urdfenvs.urdf_common.bicycle_model import BicycleModel
from urdfenvs.scene_examples.obstacles import (
    sphereObst1,
    urdfObst1,
    dynamicSphereObst3
)
from obstacles_self import wall_obstacles_jules
from goal_jules import goal1

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
    ob = env.reset(pos=pos0)

    print(f"Initial observation : {ob}")

    # add obstacles
    env.add_obstacle(sphereObst1)
    env.add_obstacle(urdfObst1)
    
    
    # add walls
    for wall_i in wall_obstacles_jules:
        env.add_obstacle(wall_i)

    # add goal
    env.add_goal(goal1)
    history = []
    for i in range(n_steps):
        ob, *_ = env.step(action)
        if ob['robot_0']['joint_state']['steering'] > 0.2:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
