import numpy as np
from params import *
from sympy import Point
import matplotlib.pyplot as plt


class Node:
    """
    Base class for nodes.

    ** Attributes **
        Point: Point object corresponding to position in state space
        parent: Node object that is this node's parent in the tree
        cost_to_come: cost to travel to this node via its parents from the root
        path_to_parent: array(s) of points describing the trajectory from the parent to this node in state space

    ** Methods **
        dist_to_node: get Euclidean distance to a given Node
    """
    def __init__(self, states):
        """
        Parameter constructor for general node.

        Args:
        """
        assert len(states) >= 2, "Must have at least two state variables"
        self.Point = Point(states)
        self.parent = None
        self.cost_to_come = np.inf
        self.path_to_parent = None
        self.plotting_object = np.array(states)

    def dist_to_node(self, node):
        return self.Point.distance(node.Point)

    def plot_node(self):
        pass


class XYThetaNode(Node):
    """
    class for x-y-theta nodes.

    ** Attributes **
        Point: Point object corresponding to x-y position
        yaw: heading in x-y plane
        parent: Node object that is this node's parent in the tree
        cost_to_come: cost to travel to this node via its parents from the root
        path_to_parent: array(s) of points describing the trajectory from the parent to this node in state space

    ** Methods **
        dist_to_node: get distance metric to a given Node
    """
    def __init__(self, x=None, y=None, theta=None):
        """
        Constructor for x-y-theta node.

        Args:
            x: **double or int**
                x-coordinate of vehicle
            y: **double or int**
                y-coordinate of vehicle
            theta: **double or int**
                heading of vehicle on x-y plane (in radians)
        """
        if x is None:
            x = np.random.uniform(MAP_X_MIN, MAP_X_MAX)
        if y is None:
            y = np.random.uniform(MAP_Y_MIN, MAP_Y_MAX)
        if theta is None:
            theta = np.random.uniform(YAW_MIN, YAW_MAX)
        super().__init__([x, y])
        self.yaw = theta

    def dist_to_node(self, node):
        """Get distance metric to a given node. The Euclidean distance was chosen for this node type."""
        return self.Point.distance(node.Point)

    def plot_node(self):
        plt.arrow(self.plotting_object[0], self.plotting_object[1],
                  ARROW_LENGTH * np.cos(self.yaw), ARROW_LENGTH * np.sin(self.yaw), width=0.1, facecolor='g')
