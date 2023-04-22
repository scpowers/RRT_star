import numpy as np
from numba import jit


@jit(nopython=True)
def fast_Euclidean_dist(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
