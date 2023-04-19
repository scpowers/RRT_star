from Node import *
from Obstacle import *
from abc import ABC, abstractmethod
import numpy as np
from params import N_SAMPLES
from tqdm import tqdm


class RRTBase(ABC):
    def __init__(self, q_start, q_goal, obs):
        root = XYThetaNode(q_start[0], q_start[1], q_start[2])
        root.cost_to_come = 0.0
        self.T = [root]
        self.goal = XYThetaNode(q_goal[0], q_goal[1], q_goal[2])
        self.goal.is_goal = True
        self.obs = obs
        self.goal_solution = None

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
        return not np.any([obstacle.is_within_obs(node) for obstacle in self.obs])

    def is_path_free(self, node: Node):
        """Return True if the node's path to parent does not intersect any obstacles"""
        return not np.any([obstacle.line_collision_check(node) for obstacle in self.obs])

    def get_nearest_node_idx(self, node: Node):
        """Get index of the node closest to the given node within the tree"""
        return np.argmin([node.dist_to_node(node2) for node2 in self.T])

    def is_close_to_goal(self, node: Node):
        """Check if a given node is close enough to the goal node to call them equivalent"""
        return self.goal.dist_to_node(node) < GOAL_DIST_THRESHOLD

    def build_tree(self):
        for i in tqdm(range(N_SAMPLES)):
            self.add_node()
        if self.goal_solution is None:
            print('No path to goal found.')
        else:
            print('Found a path to the goal!')

    def visualize_tree(self):
        fig, ax = plt.subplots()
        for obs in self.obs:
            ax.add_patch(obs.plotting_object)
        for i, node in enumerate(self.T):
            node.plot_node()
            if i > 0:
                plt.plot(node.path_to_parent[0][0, :], node.path_to_parent[0][1, :], color='k')
        if self.goal_solution is None:
            self.goal.plot_node()
        else:
            node = self.goal_solution
            while node.path_to_parent is not None:
                plt.plot(node.path_to_parent[0][0, :], node.path_to_parent[0][1, :], color='m')
                node = node.parent
        ax.set_aspect('equal')
        ax.set_xlim(MAP_X_MIN, MAP_X_MAX)
        ax.set_ylim(MAP_Y_MIN, MAP_Y_MAX)
        plt.show()



