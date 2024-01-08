import numpy as np
import cvxpy as cp 
import math

dt = 0.2

def calc_B_matrix(yaw_angle, steer_angle, WB, dt):
    # WB => length of the vehicle
    B = np.array([[np.cos(yaw_angle)*dt, 0],
                  [np.sin(yaw_angle)*dt, 0],
                  [np.tan(steer_angle)/WB, 0],
                  [0, 1]])
    return B

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def get_State_space_matrix(yaw_angle, steer_angle, WB, dt):

    A = np.matrix(np.eye(4))
    B = calc_B_matrix(yaw_angle, steer_angle, WB, dt)
    
    return A, B


def get_next_state(A, B, x_current, u):
    x_new = A*x_current + B*u
    return x_new

def get_current_state(ob):

    x = np.zeros((4,1))
    x[0] = ob['robot_0']['joint_space']['position'][0]      #get x_position
    x[1] = ob['robot_0']['joint_space']['position'][1]      #get y_position
    x[2] = ob['robot_0']['joint_space']['position'][2]      #get yaw
    x[3] = ob['robot_0']['joint_space']['steering']         #get steering_angle

    return x

def next_state(A, B, x, u):

    return A.dot(x) + B.dot(u)


def calc_orientation_path(path):
    cx = [point[0] for point in path]
    cy = [point[1] for point in path]
    cyaw = []
    for i in range(1, len(cx)):
        dx = cx[i] - cx[i-1]
        dy = cy[i] - cy[i-1]
        yaw = np.arctan2(dy,dx)
        cyaw.append(yaw)
    
    cyaw.insert(0,cyaw[0])
    print(len(cyaw))
    print(len(cx))
    #cyaw.append(cyaw[-1])
    assert(len(cx)==len(cyaw))
    return None


def calc_nearest_index(xcurrent, cx, cy, cyaw, pind):
    """
    The calc_nearest_index function identifies the nearest point on a path to the current state of a vehicle. 
    It's typically used in path tracking algorithms 
    like MPC (Model Predictive Control) to find the segment of the path that the vehicle should aim for next.

    xcurrent => This is the curren state of the vehicle
    cx       => This is a list of x positions on the path
    cy       => This is a list of y positions on the path
    cyaw     => This is a list containing the orientations on the path
    pind     => This contains the previous nearest index of the closest point on the path

    """
    x_current = xcurrent[0]
    y_current = xcurrent[1]
    # Search index number
    N_IND_SEARCH = 10  # Search index number
    # Search at the indices from pind until pind + N_IND_SEARCH and use those indices to create a list of distances to the path.
    dx = [x_current - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [y_current - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    # Distance list.
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    # Get the square of the minimum distance that is present in the list
    mind = min(d)

    #Get the index in the list of distances where the minimum distance is located and add the previous index to it.
    ind = d.index(mind) + pind

    # Calculate the minimum distance
    mind = math.sqrt(mind)

    # Get the x position at the minimum distance and the y position at the minimum distance and calculate the x & y distance from that point on the path to the current state position
    dxl = cx[ind] - x_current
    dyl = cy[ind] - y_current


    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    # Return the index of the nearest point on the path and the distance
    return ind, mind


def calc_ref_trajectory(xcurrent, cx, cy, cyaw, ck, u_curent, dl, pind):
    # This trajectory is needed as reference trajectory for the mpc
    # xref => This will contain the reference trajectory
    # dref => This will contain the reference steering angles
    # dl   => Distance between points in the path (course tick).
    # U_current => the current input [velocity, steering speed]
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    v_current = u_curent[0]

    # The function calc_nearest_index finds the closest point on the course to the current state of the vehicle. 
    # This point serves as the starting point for constructing the reference trajectory.
    ind, _ = calc_nearest_index(xcurrent, cx, cy, cyaw, pind)

    #Check if the calculated index is greater than the previous one
    if pind >= ind:
        ind = pind

    
    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0


    travel = 0.0

    for i in range(T + 1):
        travel += abs(v_current) * dt
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            # x-pos
            xref[0, i] = cx[ind + dind]
            # y-pos
            xref[1, i] = cy[ind + dind]
            xref[2, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref
    


def mpc(NX, NU, T, traj_ref, Q, R, Rd, x_current, WB, dt):
    #NX   => number of state variables
    #NU   => number of input variables
    #T    => timer horizon
    #Trajectory => reference trajetory(t) [x_traj, y_traj, orientation_trajectory]
    #Q    => Weight matrix for penalizing inputs
    #R    => Weight matrix for penalizing error in reference states
    #x_current     => current state x_current => [xc_pos, yc_pos, yaw_current, steering_angle_current]
    
    MAX_STEER = math.radians(45.0)  # maximum steering angle [rad]
    MIN_STEER = math.radians(-45.0) # minimum steering angel [rad]
    MAX_DSTEER = math.radians(30.0)  # maximum steering speed [rad/s]
    MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]

    x = cp.Variable((NX, T + 1))
    u = cp.Variable((NU, T))
    
    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cp.quad_form(u[:, t], R)

        if t != 0:
            # Calculate the error in state 
            cost += cp.quad_form(traj_ref[:, t] - x_current[[0, 1, 2, 3], t], Q)


        # Linear model
        # Get current yaw angle and steering angle from current state
        yaw_c_angle = x_current[2]
        steer_c_angle = x_current[3]
        # Calculate the a matrix and B matric
        A, B = get_State_space_matrix(yaw_c_angle, steer_c_angle, WB, dt)
        # Calculate the new state and add it to the constraints
        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t]]

        # Can be used to smoothen input
        if t < (T - 1):
            cost += cp.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cp.abs(u[1, t + 1] - u[1, t])
                            <= MAX_DSTEER * dt]

    # Constraint that the first state is the current state
    constraints += [x[:, 0] == x_current]
    # Constraint that the steering angle must always be below the max steering angle
    constraints += [x[2, :] <= MAX_STEER ]
    # Constraint that the steering angle must alwasy be above the min steering angle
    constraints += [x[2, :] >= MIN_STEER ]
    # Constraint the input speed must alsways be below the maximum speed
    constraints += [cp.abs(u[0, :]) <= MAX_SPEED]
    # constraint that the input steering velocity must always be below the maximum steering angle
    constraints += [cp.abs(u[1, :]) <= MAX_DSTEER]


    prob = cp.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cp.ECOS, verbose=False)

    if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
        # Get all the predicted x position
        o_x = get_nparray_from_matrix(x.value[0, :])
        # Get all the predicted y position
        o_y = get_nparray_from_matrix(x.value[1, :])
        # Get the predicted yaw angles
        o_yaw = get_nparray_from_matrix(x.value[2, :])
        # Get the predicted steering angles
        o_steer = get_nparray_from_matrix(x.value[3, :])
        # Get the predicted velocity inputs
        u_v = get_nparray_from_matrix(u.value[0, :])
        # Get the predicted steering velocity
        u_steer_vel = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        o_x, o_y, o_yaw, o_steer, u_v, u_steer_vel = None, None, None, None, None, None

    return o_x, o_y, o_yaw, o_steer, u_v, u_steer_vel