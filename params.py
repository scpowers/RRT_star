import numpy as np

# global params
N_NODES = 100

# state plotting params
ARROW_LENGTH = 0.2

# parameters for sampling nodes
MAP_X_MIN = 0
MAP_X_MAX = 10
MAP_Y_MIN = 0
MAP_Y_MAX = 10
YAW_MIN = 0
YAW_MAX = 2*np.pi
GOAL_BIAS = 0.05

# steer function parameters
MIN_STEP = 0.0
MAX_STEP = 0.5
YAW_DIFF_THRESHOLD = 15 * np.pi / 180
PATH_LINSPACE_N = 5

# path cost params
EUCLIDEAN_DIST_COEFF = 1
HEADING_DIFF_COEFF = 0.1

# goal node checking params
GOAL_DIST_THRESHOLD = 0.1

# RRT* params
GAMMA_STAR = 5
NEAR_DIST_THRESHOLD = 0.5  # can be at most MAX_STEP
