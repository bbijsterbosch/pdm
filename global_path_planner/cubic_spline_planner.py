"""
Cubic spline planner

"""
import math
import numpy as np
import bisect
import matplotlib.pyplot as plt

from global_path_planner.rrt import RRT


class CubicSpline1D:
    """
    1D Cubic Spline class

    Parameters
    ----------
    x : list
        x coordinates for data points. This x coordinates must be
        sorted
        in ascending order.
    y : list
        y coordinates for data points
    ----------
    """

    def __init__(self, x, y):

        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        """
        Calc `y` position for given `x`.

        if `x` is outside the data point's `x` range, return None.

        Returns
        -------
        y : float
            y position for given x.
        """
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return position

    def calc_first_derivative(self, x):
        """
        Calc first derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        dy : float
            first derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def calc_second_derivative(self, x):
        """
        Calc second derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        ddy : float
            second derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:
    """
    Cubic CubicSpline2D class

    Parameters
    ----------
    x : list
        x coordinates for data points.
    y : list
        y coordinates for data points.
    ----------
    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        x : float
            x position for given s.
        y : float
            y position for given s.
        """
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        k : float
            curvature for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        
        # Limit curvature to 1
        # k = min(1.0, k)
        
        return k

    def calc_yaw(self, s):
        """
        calc yaw

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        yaw : float
            yaw angle (tangent vector) for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = CubicSpline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s

def check_collision(spline_points, obstacles):
    for point in spline_points:
        for obstacle in obstacles:
            obstacle_x, obstacle_y, obstacle_radius = obstacle
            distance = np.sqrt((point[0] - obstacle_x) ** 2 + (point[1] - obstacle_y) ** 2)
            if distance + 1 <= obstacle_radius:
                print(f'point: {point} is causing a collision with distance {distance}')
                return True  # Collision detected
    return False  # No collisions detected


def main_2d(obstacleList):  # pragma: no cover
        
    """
    
    QUBIC SPLINE PLANNER:
    
    """
    
    first = path_arr[np.shape(path_arr)[0]-1]
    path_arr = path_arr[::40]
    path_arr = np.append(path_arr, [first], axis=0)
    
    
    print("\n! Running CubicSpline !\n")
    x = path_arr[:,0]
    y = path_arr[:,1]
    
    ds = 0.1  # [m] distance of each interpolated points

    sp = CubicSpline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))
    
    plot_ryaw = ryaw[::10]
    plotx = rx[::10]
    ploty = ry[::10]
    
    idx_wrong_K = []    
    for idx, K in enumerate(rk):
        if K > 1:
            print(f'K at {len(rx)-idx} is larger than 1: {K}\n')
            idx_wrong_K = np.append(idx_wrong_K, idx)
    
    print(f'Number of points: {np.shape(rx)[0]}\n')
    
    print(f'Indexes of wrong Ks: {idx_wrong_K}\n')
        
    
    spline_points = np.zeros((len(rx),2))
    for i in range(len(rx)):
        spline_points[i] = (rx[i],ry[i])
        
    if check_collision(spline_points, obstacleList):
        print("Collision detected! Adjust spline generation.\n")
    else:
        print("No collision detected. Spline path is safe.\n")

    plt.subplots(1)
    plt.plot(x, y, "xb", label="Data points")
    plt.plot(rx, ry, "-r", label="Cubic spline path")
    plt.quiver(plotx, ploty, np.cos(plot_ryaw), np.sin(plot_ryaw), color='g', units='xy', scale=5, width=0.03, label='Yaw')
    for idx in idx_wrong_K:
        plt.scatter(rx[idx],ry[idx], "-r")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots(1)
    plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    plt.subplots(1)
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")
    
    for (ox, oy, size) in obstacleList:
            RRT.plot_circle(ox, oy, size)

    plt.show()
    return rx, ry, ryaw, rk, s
    
    

if __name__ == '__main__':
    obstacleList = [(4,5,1),
                (4,1,1),
                (4,3,1), 
                (4,7,1) , 
                (4,-1,1),
                (4,-3,1),
                (0,14,1),
                (2,14,1),
                (4,14,1),
                (6,14,1),
                (8,14,1),
                (10,14,1),
                (12,14,1),
                (14,14,1),
                (16,14,1),
                (10,12,1),
                (10,10,1),
                (10,8,1),
                (10,6,1),
                (10,4,1),
                ]
    main_2d(obstacleList)
