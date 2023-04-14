import numpy as np


class Node:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.yaw = theta
        self.parent = None
        self.cost_to_come = np.inf
        self.path_to_parent = None
