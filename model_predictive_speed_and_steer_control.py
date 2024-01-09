"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
import sys
import pathlib
from angle import angle_mod
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
import cubic_spline_planner

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.2  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


def pi_2_pi(angle):
    return angle_mod(angle)


def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    print(phi)
    print(delta)
    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def update_state(state, a, delta):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC control with updating operational point iteratively
    """
    ox, oy, oyaw, ov = None, None, None, None

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False


def do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    odelta, oa = None, None

    cyaw = smooth_yaw(cyaw)

    while MAX_TIME >= time:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        di, ai = 0.0, 0.0
        if odelta is not None:
            di, ai = odelta[0], oa[0]
            state = update_state(state, ai, di)

        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)

        if check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            if ox is not None:
                plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw, steer=di)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            plt.pause(0.0001)

    return t, x, y, yaw, v, d, a


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course2(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course3(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl):
    
    path.reverse()

    ax = [points[0] for points in path]
    ay = [points[1] for points in path]

    cx = ax
    cy = ay


    return cx, cy


def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


def main():
    print(__file__ + " start!!")

    dl = 1.0  # course tick
    # cx, cy, cyaw, ck = get_straight_course(dl)
    # cx, cy, cyaw, ck = get_straight_course2(dl)
    # cx, cy, cyaw, ck = get_straight_course3(dl)\
    path = [[10.0, 10.0], [10.0, 10.0], [9.810158474706775, 10.0181852540961], [9.713088970992683, 10.042042766385702], [9.618886187440255, 10.075471870695015], [9.528491367123447, 10.118138554463977], [9.442807705206311, 10.169616506292098], [9.362691324515469, 10.22939137550106], [9.288942721443828, 10.296865911356669], [9.222298767655234, 10.371365930600776], [9.16342534750638, 10.452147053667524], [9.112910704750428, 10.538402142277917], [9.071259564999876, 10.629269364098707], [9.038888092675073, 10.723840803886166], [9.016119732826905, 10.821171535075102], [9.003181979380752, 10.920289061172735], [9.0002041020924, 11.020203032622197], [9.007215854927356, 11.119915142047933], [9.02414717876907, 11.218429099013003], [9.050828901426467, 11.314760584623794], [9.086994427946562, 11.40794708651904], [9.132282404343098, 11.497057515975166], [9.186240328126136, 11.581201511037039], [9.248329069557352, 11.659538332720306], [9.317928258456176, 11.73128526539742], [9.394342482733565, 11.795725437433452], [9.476808236719723, 11.852214983930507], [9.594319229533541, 11.924357324413094], [9.679540727926591, 11.976676500741123], [9.764762226319641, 12.02899567706915], [9.849983724712693, 12.081314853397178], [9.935205223105743, 12.133634029725206], [10.020426721498794, 12.185953206053233], [10.129098367064621, 12.243778370693107], [10.222021658379225, 12.280614920140767], [10.318158238877437, 12.307990590617655], [10.41654754362545, 12.325631853473556], [10.516206499213402, 12.333362443041443], [10.616139346299928, 12.331105117825745], [10.715347588910348, 12.318882432273845], [10.812839971078382, 12.296816511419495], [10.907642381148106, 12.26512783064985], [10.99880758477561, 12.224133012788272], [11.085424689381464, 12.174241664503752], [11.166628245488248, 12.115952283656458], [11.241606894005672, 12.049847278471912], [11.241606894005672, 12.049847278471912], [11.362393027999243, 11.907853408626796], [11.415618446324288, 11.82324416393053], [11.460131129278437, 11.733743937672871], [11.495486320847547, 11.640246986529746], [11.521330763644338, 11.543687501130993], [11.537406228539416, 11.445030271936394], [11.543552094801392, 11.345261049368318], [11.539706954966132, 11.245376694519344], [11.525909228399833, 11.146375218845746], [11.502296777425427, 11.049245812366916], [11.469105529847809, 10.954958960005678], [11.42666712164121, 10.864456744823737], [11.375405583352151, 10.778643435039136], [11.315833103326412, 10.698376448877132], [11.24854491009236, 10.62445778753072], [11.174213325034277, 10.557626021829812], [11.093581044779302, 10.498548912685646], [11.007453720418258, 10.447816739044493], [10.90032230449189, 10.39172675082352], [10.811730156184069, 10.345343227259601], [10.723138007876244, 10.298959703695681], [10.634545859568421, 10.252576180131763], [10.545953711260598, 10.206192656567845], [10.457361562952775, 10.159809133003927], [10.368769414644952, 10.113425609440007], [10.280177266337127, 10.06704208587609], [10.191585118029305, 10.020658562312171], [10.10299296972148, 9.974275038748253], [10.014400821413657, 9.927891515184335], [9.925808673105834, 9.881507991620417], [9.837216524798013, 9.835124468056497], [9.748624376490188, 9.788740944492579], [9.660032228182365, 9.742357420928661], [9.571440079874542, 9.695973897364741], [9.482847931566717, 9.649590373800823], [9.394255783258895, 9.603206850236905], [9.30566363495107, 9.556823326672987], [9.217071486643249, 9.510439803109069], [9.128479338335424, 9.46405627954515], [9.039887190027603, 9.417672755981233], [8.951295041719778, 9.371289232417313], [8.862702893411955, 9.324905708853395], [8.774110745104132, 9.278522185289477], [8.685518596796308, 9.232138661725557], [8.596926448488484, 9.185755138161639], [8.50833430018066, 9.139371614597719], [8.419742151872835, 9.092988091033801], [8.331150003565012, 9.046604567469883], [8.24255785525719, 9.000221043905965], [8.153965706949366, 8.953837520342045], [8.065373558641543, 8.907453996778127], [7.976781410333718, 8.861070473214209], [7.888189262025895, 8.81468694965029], [7.799597113718071, 8.768303426086371], [7.711004965410247, 8.721919902522453], [7.622412817102424, 8.675536378958533], [7.5338206687946006, 8.629152855394615], [7.4452285204867765, 8.582769331830697], [7.3566363721789525, 8.536385808266779], [7.268044223871129, 8.49000228470286], [7.179452075563306, 8.443618761138941], [7.090859927255481, 8.397235237575023], [7.002267778947658, 8.350851714011103], [6.913675630639835, 8.304468190447185], [6.825083482332011, 8.258084666883267], [6.736491334024187, 8.211701143319347], [6.647899185716364, 8.16531761975543], [6.55930703740854, 8.118934096191511], [6.470714889100717, 8.072550572627591], [6.382122740792893, 8.026167049063673], [6.2935305924850695, 7.979783525499754], [6.2049384441772455, 7.933400001935836], [6.116346295869422, 7.887016478371917], [6.027754147561598, 7.840632954807998], [5.939161999253775, 7.79424943124408], [5.850569850945951, 7.747865907680161], [5.761977702638128, 7.701482384116243], [5.673385554330304, 7.655098860552324], [5.584793406022481, 7.608715336988406], [5.496201257714657, 7.562331813424487], [5.32219864575359, 7.49117594839973], [5.225202388212251, 7.467022367569497], [5.126279373443701, 7.452672921829343], [5.026418007511905, 7.448270986097808], [4.926616072175477, 7.4538605430616345], [4.827870755382164, 7.469385743715303], [4.827870755382164, 7.469385743715303], [4.635084738305905, 7.490449635314649], [4.5351996683243705, 7.486623117480828], [4.436195621992868, 7.4728438484894015], [4.339061815015954, 7.449249506241311], [4.244768776284774, 7.416075837605047], [4.154258650673196, 7.373654302911866], [4.06843578543766, 7.322408764113726], [3.9256899862668106, 7.227185529249002], [3.8425009289482257, 7.171691558923882], [3.7593118716296416, 7.116197588598762], [3.6761228143110567, 7.060703618273643], [3.592933756992472, 7.005209647948523], [3.5097446996738872, 6.949715677623403], [3.4265556423553027, 6.894221707298284], [3.3433665850367182, 6.838727736973164], [3.2601775277181337, 6.783233766648044], [3.1769884703995492, 6.727739796322925], [3.0937994130809647, 6.672245825997805], [3.0106103557623802, 6.616751855672685], [2.9274212984437957, 6.561257885347565], [2.8442322411252112, 6.505763915022445], [2.7610431838066267, 6.450269944697325], [2.677854126488042, 6.394775974372206], [2.5946650691694577, 6.339282004047086], [2.5114760118508728, 6.283788033721967], [2.4282869545322883, 6.228294063396847], [2.3450978972137038, 6.172800093071727], [2.2619088398951193, 6.117306122746607], [2.178719782576535, 6.061812152421488], [2.0955307252579507, 6.006318182096368], [2.0123416679393658, 5.950824211771248], [1.9291526106207815, 5.895330241446128], [1.845963553302197, 5.839836271121009], [1.7627744959836125, 5.784342300795889], [1.679585438665028, 5.728848330470769], [1.5963963813464435, 5.67335436014565], [1.513207324027859, 5.61786038982053], [1.4300182667092745, 5.56236641949541], [1.2874528947080055, 5.447055688334899], [1.2193989008520714, 5.373841459088085], [1.1589941201544853, 5.294198933306785], [1.1068420972168123, 5.208923872782287], [1.0634639178130778, 5.118868317731163], [1.0292930023729647, 5.024932073491077], [1.0046707753881476, 4.928053719963281], [0.9898432540116331, 4.8292012336324985], [0.9849585899356881, 4.72936231586586], [0.9900655891091039, 4.6295345241273305], [1.0051132240842895, 4.530715304713281], [1.0299511438666598, 4.433892026598806], [1.0643311761720737, 4.340032115973304], [1.1079098070822635, 4.250073390037784], [1.1602516133223548, 4.1649146866453925], [1.2208336128662969, 4.085406883410703], [1.2890504904003934, 4.012344396021849], [1.3642206454338262, 3.946457240701572], [1.4455930026254498, 3.8884037401264364], [1.5323545162803311, 3.838763945684252], [1.6236382940335738, 3.7980338417923507], [1.7185322585522729, 3.766620390185238], [1.8160882607107314, 3.7448374636874657], [1.9153315531831088, 3.732902710100122], [2.015270529796492, 3.7309353775359457], [2.1149066333319944, 3.7389551229316047], [2.2132443327784053, 3.756881815642081], [2.309301070348978, 3.784536338079588], [2.4021170788740434, 3.8216423753973006], [2.490764971477312, 3.8678291763359645], [2.5743590077189897, 3.9226352576479275], [2.6520639436214646, 3.9855130150852025], [2.723103377151035, 4.0558341948800996], [2.7867675057703885, 4.1328961710491985], [2.8424202185509237, 4.215928965799922], [2.8895054519828784, 4.304102942894112], [2.8895054519828784, 4.304102942894112], [2.952779787119671, 4.478839342140029], [2.9730664412451313, 4.57671743523722], [2.9834802421507503, 4.676131831552072], [2.9839171385802237, 4.776089215298639], [2.9743727652088467, 4.875590845340042], [2.9549424862603964, 4.973642534279774], [2.9258204426593286, 5.069264582053528], [2.8872976122388376, 5.161501564768394], [2.8397589023865315, 5.24943188098258], [2.7836793041770442, 5.3321769600423705], [2.7196191464182355, 5.408910040469687], [2.648218497030959, 5.478864430689484], [2.570190767701975, 5.541341169558533], [2.4863155857102006, 5.595716010154117], [2.3974310041486486, 5.641445657042939], [2.3044251283749464, 5.678073194709497], [2.208227242356169, 5.70523265290488], [2.109798523570623, 5.722652663300524], [2.010122439240204, 5.730159170910846], [1.9101949198510106, 5.727677173193124], [1.8110144081451318, 5.715231469448115], [1.713571883010815, 5.692946413033636], [1.6188409579489857, 5.6610446688669205], [1.5277681530489815, 5.619844988630367], [1.441263437672696, 5.569759025910097], [1.3601911383415124, 5.511287223089392], [1.2853613026714084, 5.445013811093828], [1.2175216056449445, 5.371600971948989], [1.1573498790899872, 5.2917822224765505], [1.105447339008145, 5.2063550852366705], [1.062332578423217, 5.116173119946193], [1.0284363857711227, 5.022137394992107], [1.0040974406042502, 4.925187484254074], [0.9895589296173215, 4.826292079192811], [0.9849661168063251, 4.726439310005214], [0.9854603224584798, 4.60400720765928], [0.985863976088915, 4.504008022343866], [0.9862676297193501, 4.404008837028451], [0.9866712833497853, 4.304009651713036], [0.9870749369802205, 4.204010466397621], [0.9874785906106557, 4.104011281082206], [0.9878822442410908, 4.004012095766791], [0.988285897871526, 3.904012910451377], [0.9886895515019613, 3.8040137251359627], [0.9890932051323964, 3.7040145398205477], [0.9894968587628316, 3.604015354505133], [0.9899005123932667, 3.5040161691897183], [0.990304166023702, 3.4040169838743037], [0.9907078196541371, 3.3040177985588888], [0.9911114732845723, 3.2040186132434743], [0.9915151269150075, 3.1040194279280593], [0.9919187805454427, 3.0040202426126448], [0.9923224341758778, 2.9040210572972303], [0.992726087806313, 2.8040218719818153], [0.9931297414367483, 2.7040226866664008], [0.9935333950671834, 2.6040235013509863], [0.9939370486976186, 2.5040243160355713], [0.9943407023280537, 2.4040251307201568], [0.994744355958489, 2.3040259454047423], [0.9951480095889241, 2.2040267600893273], [0.9955516632193593, 2.1040275747739123], [0.9959553168497944, 2.004028389458498], [0.9963589704802297, 1.9040292041430833], [0.9967626241106649, 1.8040300188276688], [0.9971662777411, 1.7040308335122543], [0.9975699313715352, 1.6040316481968395], [0.9979735850019704, 1.5040324628814248], [0.9983772386324056, 1.4040332775660103], [0.9987808922628407, 1.3040340922505957], [0.9991845458932759, 1.204034906935181], [0.9995881995237111, 1.1040357216197663], [0.9999918531541463, 1.0040365363043517], [0.9854497299884603, 0.8300328570997592], [0.963558185417193, 0.7325011713754126], [0.9320390859672263, 0.6376422455233264], [0.8912073600614353, 0.5464038785744225], [0.8414709848078964, 0.45969769413186023], [0.7833269096274833, 0.3783900317293355], [0.7173560908995227, 0.3032932906528345], [0.644217687237691, 0.2351578127155115], [0.5646424733950354, 0.17466438509032167], [0.479425538604203, 0.12241743810962724], [0.3894183423086505, 0.0789390059971149], [0.2955202066613396, 0.04466351087439402], [0.19866933079506122, 0.019933422158758374], [0.09983341664682815, 0.0049958347219741794], [0.0, 0.0], [0.0, 0.0]]
    path.reverse()
    
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
    # cx, cy, cyaw, ck = get_switch_back_course(dl)
    ck = 0
    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()


def main2():
    print(__file__ + " start!!")

    dl = 1.0  # course tick
    cx, cy, cyaw, ck = get_straight_course3(dl)

    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()


if __name__ == '__main__':
    main()
    # main2()
