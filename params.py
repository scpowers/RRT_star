import numpy as np

# parameters for sampling nodes
MAP_X_MIN = 0
MAP_X_MAX = 10
MAP_Y_MIN = 0
MAP_Y_MAX = 10
YAW_MIN = 0
YAW_MAX = 2*np.pi

# steer function parameters
MIN_STEP = 0.2
MAX_STEP = 3
YAW_DIFF_THRESHOLD = 15 * np.pi / 180
PATH_LINSPACE_N = 5

# path cost params
EUCLIDEAN_DIST_COEFF = 1
HEADING_DIFF_COEFF = 0.1
