import numpy as np
from numba import jit


@jit(nopython=True)
def fast_euclidean_dist(x1, y1, x2, y2):
    """Calculate Euclidean distance between two (x,y) coordinate pairs"""
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
