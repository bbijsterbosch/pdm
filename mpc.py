import numpy as np
import cvxpy as cp 


    
    
def get_state_space_model(theta, delta, WB,  dt):

    A = np.matrix(np.eye(4))
    B = np.array([[np.cos(theta)*dt, 0],
                  [np.sin(theta)*dt, 0],
                  [np.tan(delta)/WB, 0],
                  [0, 1]])
    
    return A, B

def get_state(ob):

    x = np.zeros((4,1))
    x[0] = ob['robot_0']['joint_space']['position'][0]      #get x_position
    x[1] = ob['robot_0']['joint_space']['position'][0]      #get y_position
    x[2] = ob['robot_0']['joint_space']['orientation']      #get theta
    x[3] = ob['robot_0']['joint_space']['steering']         #get delta

    return x

def next_state(A, B, x, u):

    return A.dot(x) + B.dot(u)

def mpc()