import numpy as np


class Node:
    def __init__(self, state):
        self.state = state
        self.parent = None
        self.cost_to_come = np.inf
        self.path_to_parent = None


class XYThetaNode(Node):
    def __init__(self, state):
        super().__init__(state)
        self.x = state[0]
        self.y = state[1]
        self.yaw = state[2]