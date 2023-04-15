import numpy as np

# parameters for sampling nodes
MAP_X_MIN = 0
MAP_X_MAX = 10
MAP_Y_MIN = 0
MAP_Y_MAX = 10
YAW_MIN = -np.pi
YAW_MAX = np.pi

# steer function parameters
MIN_STEP = 0.2
MAX_STEP = 3
YAW_DIFF_THRESHOLD = 15 * np.pi / 180
