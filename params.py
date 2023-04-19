import numpy as np

# global params
N_NODES = 50

# state plotting params
ARROW_LENGTH = 2

# parameters for sampling nodes
MAP_X_MIN = 0
MAP_X_MAX = 100
MAP_Y_MIN = 0
MAP_Y_MAX = 100
YAW_MIN = -np.pi
YAW_MAX = np.pi
GOAL_BIAS = 0.05

# steer function parameters
MIN_STEP = 2.0
MAX_STEP = 5.0
PATH_TIME_DURATION = 5
PATH_TIME_DISCRETIZATION = 10
PATH_INITIAL_SPEED = 1
PATH_FINAL_SPEED = 1

# goal node checking params
GOAL_DIST_THRESHOLD = 0.2

# RRT* params
GAMMA_STAR = 5
NEAR_DIST_THRESHOLD = MAX_STEP # can be at most MAX_STEP
