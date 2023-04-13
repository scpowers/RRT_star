import numpy as np


class Node:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.parent = None
        self.cost_to_come = np.inf
        self.path_to_parent = None
