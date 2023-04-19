import numpy as np
from numba import jit
from params import PATH_TIME_DURATION, PATH_TIME_DISCRETIZATION, PATH_INITIAL_SPEED, PATH_FINAL_SPEED, \
    DIST_BETWEEN_AXLES, MIN_STEER_ANGLE, MAX_STEER_ANGLE
from sympy import Segment


@jit(nopython=True)
def fast_Euclidean_dist(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)


@jit(nopython=True)
def car_h(state):
    return state[0:-1]


def psi(y, ydot):
    return np.array([y[0], y[1], np.arctan2(ydot[1], ydot[0])])


def alpha(y, ydot, yddot):
    u1 = np.linalg.norm(ydot)
    num = yddot[1, 0]*ydot[0, 0] - yddot[0, 0]*ydot[1, 0]
    u2 = np.arctan(DIST_BETWEEN_AXLES * num / (np.linalg.norm(ydot)**3))
    # simple controls check here
    if u2 < MIN_STEER_ANGLE or u2 > MAX_STEER_ANGLE:
        return None
    else:
        return np.array([u1, u2]).reshape(-1, 1)


@jit(nopython=True)
def lambdafunc(t):
    return np.array([t**3, t**2, t, 1]).reshape(-1, 1)


@jit(nopython=True)
def dlambdafunc(t):
    return np.array([3*t**2, 2*t, 1, 0]).reshape(-1, 1)


@jit(nopython=True)
def ddlambdafunc(t):
    return np.array([6*t, 2, 0, 0]).reshape(-1, 1)


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
    controls = np.zeros((2, n_steps))

    valid_path = True
    for t in range(n_steps):
        y = A @ lambdafunc(t1_vec[t])
        dy = A @ dlambdafunc(t1_vec[t])
        ddy = A @ ddlambdafunc(t1_vec[t])
        path[:, t] = psi(y, dy).flatten()
        new_controls = alpha(y, dy, ddy)
        if new_controls is None:
            valid_path = False
            break
        else:
            controls[:, t] = new_controls.flatten()

    # generate list of Line objects for this path
    collision_objects = [Segment(tuple(path[0:-1, t]), tuple(path[0:-1, t+1])) for t in range(n_steps - 1)]
    return path, collision_objects, controls, valid_path
