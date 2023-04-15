from Node import *
from Obstacle import *
from abc import ABC, abstractmethod
import numpy as np


class RRTBase(ABC):
    def __init__(self, q_start, q_goal, obs):
        root = XYThetaNode(q_start)
        root.cost_to_come = 0.0
        self.T = [root]
        self.goal = q_goal
        self.obs = obs
        self.found_solution = False

    @abstractmethod
    def add_node(self):
        """Add a new node to the tree"""
        pass

    def sample_free(self):
        """Return a random Node outside any obstacles"""
        while True:
            node = XYThetaNode()
            if self.is_node_free(node):
                return node

    def is_node_free(self, node: Node):
        """Return True if the node is not within any obstacles"""
        return np.any([obstacle.is_within_obs(node) for obstacle in self.obs])

    def is_path_free(self, node: Node):
        """Return True if the node is not within any obstacles"""
        return np.any([obstacle.is_within_obs(node) for obstacle in self.obs])

    def get_nearest_node_idx(self, node: Node):
        """Get index of the node closest to the given node within the tree"""
        return np.argmin([node.dist_to_node(node2) for node2 in self.T])



