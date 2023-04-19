import numpy as np

# global params
N_SAMPLES = 5000

# state plotting params
ARROW_LENGTH = 2
ARROW_WIDTH = 0.5

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
MAX_STEP = 10.0
PATH_TIME_DURATION = 10
PATH_TIME_DISCRETIZATION = 10
PATH_INITIAL_SPEED = 1
PATH_FINAL_SPEED = 1
DIST_BETWEEN_AXLES = 2
MIN_STEER_ANGLE = -np.pi/4
MAX_STEER_ANGLE = np.pi/4
HEADING_DIFF_THRESHOLD = 10 * np.pi / 180

# goal node checking params
GOAL_DIST_THRESHOLD = 5.0

# RRT* params
GAMMA_STAR = 5
NEAR_DIST_THRESHOLD = MAX_STEP  # can be at most MAX_STEP
