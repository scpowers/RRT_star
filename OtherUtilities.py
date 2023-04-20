import numpy as np
from numba import jit
from params import PATH_TIME_DURATION, PATH_TIME_DISCRETIZATION, PATH_INITIAL_SPEED, PATH_FINAL_SPEED, \
    DIST_BETWEEN_AXLES, MIN_STEER_ANGLE, MAX_STEER_ANGLE, HEADING_DIFF_THRESHOLD, GOAL_HEADING_DIFF_THRESHOLD, \
    MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, GOAL_DIST_THRESHOLD
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


def generate_trajectory(state0, statef):
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

    # check if final heading was approximately reached:
    if np.abs(path[-1, -1] - statef[-1, 0]) > HEADING_DIFF_THRESHOLD:
        valid_path = False

    # generate list of Line objects for this path
    if not valid_path:
        collision_objects = None
    else:
        collision_objects = [Segment(tuple(path[0:-1, t]), tuple(path[0:-1, t+1])) for t in range(n_steps - 1)]
    return path, collision_objects, controls, valid_path


def generate_straight_path(state0, statef, statef_is_goal):
    # first, compute heading that will be assigned to statef because of this line
    # (heading for a given state = heading along path going to that state from parent)
    new_heading = np.arctan2(statef[1, 0] - state0[1, 0], statef[0, 0] - state0[0, 0])

    # check if this new heading is too different from the heading going to state0 from its parent
    theta0 = state0[2, 0]
    thetaf = new_heading
    valid_path = angle_check(theta0, thetaf)
    if not valid_path:
        new_heading = theta0 + determine_delta_direction(theta0, thetaf) * HEADING_DIFF_THRESHOLD
        # need new statef corresponding to this heading from state0
        x0 = state0[0, 0]
        y0 = state0[1, 0]
        dist = fast_Euclidean_dist(x0, y0, statef[0, 0], statef[1, 0])
        new_x = x0 + dist*np.cos(new_heading)
        new_y = y0 + dist*np.sin(new_heading)
        if not bounds_check(new_x, new_y):
            return None, None, None, bounds_check
        statef[0, 0] = new_x
        statef[1, 0] = new_y
        valid_path = True
        if dist < GOAL_DIST_THRESHOLD:
            statef_is_goal = True

    # now, handle the case where the statef actually corresponds to the goal node (can't overwrite its heading value)
    if not statef_is_goal:
        # populate statef's heading slot
        statef[2, 0] = new_heading
    else:
        # see if the heading going to the goal is sufficiently close to the requested goal heading
        # note: statef[2, 0] should be the actual goal heading, new_heading is the heading coming from potential parent
        goal_heading_check = angle_check(statef[2, 0], new_heading, goal_check=True)
        if not goal_heading_check:
            return None, None, None, goal_heading_check

    # if you made it here, the path should be valid by all standards
    path = np.hstack((state0, statef))
    controls = None
    collision_objects = [Segment(tuple(path[0:-1, 0]), tuple(path[0:-1, 1]))]
    return path, collision_objects, controls, valid_path


@jit(nopython=True)
def angle_check(theta0, thetaf, goal_check=False):
    if goal_check:
        THRESHOLD = GOAL_HEADING_DIFF_THRESHOLD
    else:
        THRESHOLD = HEADING_DIFF_THRESHOLD
    theta0_abs = np.abs(theta0)
    thetaf_abs = np.abs(thetaf)

    valid_path = True
    # case 1: theta1 and theta2 are in diagonal quadrants from each other, not even close
    if np.abs(theta0_abs - thetaf_abs) > THRESHOLD:
        valid_path = False
    else:
        # case 2: abs delta is close, but the true delta can be larger
        if np.sign(theta0) != np.sign(thetaf):
            # if they're in the first and fourth quadrants
            if theta0_abs < np.pi/2:
                diff = theta0_abs + thetaf_abs
            # they're in the second and third quadrants
            else:
                diff = (np.pi - theta0_abs) + (np.pi - thetaf_abs)

            # now compare diff to threshold
            if diff > THRESHOLD:
                valid_path = False

    return valid_path


@jit(nopython=True)
# return sign of angle to add to theta0 to go towards thetaf (in the closest way)
# +1: add positive angle to theta0, -1: add negative angle to theta0
def determine_delta_direction(theta0, thetaf):
    if np.sign(theta0) == np.sign(thetaf):
        return np.sign(thetaf-theta0)
    else:
        if theta0 >= 0:
            if theta0 < np.pi/2:
                if np.pi - np.abs(thetaf) < theta0:
                    return 1
                else:
                    return -1
            else:
                if np.abs(thetaf) < np.pi - theta0:
                    return -1
                else:
                    return 1
        else:
            if theta0 > -np.pi/2:
                if np.pi - thetaf < np.abs(theta0):
                    return -1
                else:
                    return 1
            else:
                if thetaf < np.pi - np.abs(theta0):
                    return 1
                else:
                    return -1


@jit(nopython=True)
def bounds_check(x, y):
    return MAP_X_MIN <= x <= MAP_X_MAX and MAP_Y_MIN <= y <= MAP_Y_MAX
