import numpy as np
from numba import jit
from params import PATH_TIME_DURATION, PATH_TIME_DISCRETIZATION, PATH_INITIAL_SPEED, PATH_FINAL_SPEED
from sympy import symbols, Curve

@jit(nopython=True)
def fast_Euclidean_dist(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)


@jit(nopython=True)
def car_h(state):
    return state[0:-1]


def psi(y, ydot):
    return np.array([y[0], y[1], np.arctan2(ydot[1], ydot[0])])


@jit(nopython=True)
def lambdafunc(t):
    return np.array([t**3, t**2, t, 1]).reshape(-1, 1)


@jit(nopython=True)
def dlambdafunc(t):
    return np.array([3*t**2, 2*t, 1, 0]).reshape(-1, 1)


@jit(nopython=True)
def poly3_coeff(y0, dy0, yf, dyf, t1, t2):
    Y = np.hstack((y0, dy0, yf, dyf))
    L = np.hstack((lambdafunc(t1), dlambdafunc(t1), lambdafunc(t2), dlambdafunc(t2)))
    return Y @ np.linalg.inv(L)


def generate_trajectory_function(state0, statef):
    T = PATH_TIME_DURATION
    y0 = car_h(state0)
    yf = car_h(statef)
    dy0 = PATH_INITIAL_SPEED * np.array([np.cos(state0[2]), np.sin(state0[2])]).reshape(-1, 1)
    dyf = PATH_FINAL_SPEED * np.array([np.cos(statef[2]), np.sin(statef[2])]).reshape(-1, 1)

    A = poly3_coeff(y0, dy0, yf, dyf, 0.0, T)

    n_steps = PATH_TIME_DISCRETIZATION
    t1_vec = np.linspace(0, T, n_steps)
    path = np.zeros((3, n_steps))

    for t in range(n_steps):
        y = A @ lambdafunc(t1_vec[t])
        dy = A @ dlambdafunc(t1_vec[t])
        path[:, t] = psi(y, dy).flatten()

    # generate Curve object for this path
    t = symbols('t')
    collision_object = Curve((A[0, -1] + t*A[0, -2] + t**2*A[0, -3] + t**3*A[0, -4],
                              A[1, -1] + t*A[1, -2] + t**2*A[1, -3] + t**3*A[1, -4]), (t, 0, T))

    return path, collision_object
