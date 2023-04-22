from Node import *
from Obstacle import *
from abc import ABC, abstractmethod
import numpy as np


class RRTBase(ABC):
    """
    Abstract base class for rapidly-exploring random trees.

    **Attribute**
        T: **list, Node**
            List of Node objects representing the tree
        root: **Node**
            Node object representing the root configuration
        goal: **Node**
            Node object representing the goal configuration
        obs: **list, Obstacle**
            List of Obstacle objects in the environment
        goal_solution: **Node**
            Node object representing the actual discovered goal node with a parent node and cost-to-come

    **Methods**
        * add_node: abstract method that derived classes must implement to add a node to the tree.
        * sample_free: return a random Node outside of any obstacles
        * is_node_free: return True if the node is not witin any obstacles
        * is_path_free: return True if the node's path to parent does not intersect any obstacles
        * get_nearest_node_idx: get index of the node closest to the given node within the tree
        * is_close_to_goal: return True if a given node is close enough to the goal to call them equivalent
        * build_tree: abstract method that derived classes must implement to build the tree
        * visualize_tree: plot the tree's nodes and edges in the environment with obstacles

    """
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

    @abstractmethod
    def build_tree(self):
        """Build the tree"""
        pass

    def visualize_tree(self):
        """Plot the tree's nodes and edges in the environment with obstacles"""
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



