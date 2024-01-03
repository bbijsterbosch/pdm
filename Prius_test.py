import gymnasium as gym
from urdfenvs.robots.prius import Prius
import numpy as np
from urdfenvs.scene_examples.goal import goal1
from urdfenvs.sensors.full_sensor import FullSensor

def run_prius(n_steps=3000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    sensor = FullSensor(['position'], ['position', 'size', 'type', 'orientation'], variance=0.0)
    

    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    env.add_goal(goal1)
    env.add_sensor(sensor, [0])
    env.set_spaces()
    pos0 = np.array([-1.0, -1.0, -1.0])
    vel0 = np.array([1.0, 0.0, 0.0])
    ob = env.reset(pos=pos0, vel=vel0)
    
    action = np.zeros(env.n())
    print(f"Initial observation : {ob}")
    print(f"this is the observation space: {env.observation_space}")
    history = []
    for i in range(n_steps):

        ob, *_ = env.step(action)
        prius_ob = ob['robot_0']
        position = prius_ob['joint_state']['position']
        steering = prius_ob['joint_state']['steering']
        velocity = prius_ob['joint_state']['velocity']
        forward_velocity = prius_ob['joint_state']['forward_velocity']
        
        goal_config = ob['robot_0']['FullSensor']['goals'][1]['position']
        
        p_ref = np.array([goal_config[0], goal_config[1]])
        p = np.array([position[0], position[1]])

        error = p_ref - p
        Kp = 0.5

        control_action = Kp * error
        
        action = control_action
        print(p_ref)
        print(position)

        if ob['robot_0']['joint_state']['steering'] > 0.2:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
