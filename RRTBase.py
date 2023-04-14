from Node import *
from Obstacle import *
from abc import ABC, abstractmethod
import numpy as np
from params import *


class RRTBase(ABC):
    def __init__(self, q_start, q_goal, obs):
        self.T = [XYThetaNode(q_start)]
        self.goal = q_goal
        self.obs = obs
        self.found_solution = False

    @abstractmethod
    def add_node(self):
        pass

    def sample_free(self):
        """Return a random Node outside any obstacles"""
        while True:
            node = XYThetaNode()
            # check if sampled node is inside an obstacle
            if self.is_free(node):
                return node

    def is_free(self, node: Node):
        return np.any([obstacle.is_within_obs(node) for obstacle in self.obs])

    def get_nearest_node_idx(self, node: Node):
        return np.argmin([node.dist_to_node(node2) for node2 in self.T])


