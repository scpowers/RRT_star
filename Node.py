import numpy as np
from params import *
from sympy import Point
import matplotlib.pyplot as plt
from utilities import fast_Euclidean_dist


class Node:
    """
    Base class for nodes.

    **Attributes**
        Point: **sympy Point**
            Point object corresponding to position in state space
        parent: **Node**
            Node object that is this node's parent in the tree
        cost_to_come: **float or int**
            cost to travel to this node via its parents from the root
        path_to_parent: **numpy array**
            array(s) of points describing the trajectory from the parent to this node in state space

    **Methods**
        dist_to_node: get Euclidean distance to a given Node
        plot_node: plot the node on the current Matplotlib figure
    """
    def __init__(self, states):
        """
        Constructor for general node.

        **Args**
            states: **list, float**
                state vector corresponding to this node
        """
        assert len(states) >= 2, "Must have at least two state variables"
        self.Point = Point(states)
        self.parent = None
        self.cost_to_come = np.inf
        self.path_to_parent = None
        self.plotting_object = np.array(states)

    def dist_to_node(self, node):
        """Compute Euclidean distance between this node and a given node"""
        (x1, y1) = map(float, self.Point.coordinates)
        (x2, y2) = map(float, node.Point.coordinates)
        return fast_Euclidean_dist(x1, y1, x2, y2)

    def plot_node(self):
        """Plot this node in the current Matplotlib figure"""
        pass


class XYThetaNode(Node):
    """
    Class for x-y-theta nodes.

    **Attributes**
        Point: **sympy Point**
            Point object corresponding to position in state space
        parent: **Node**
            Node object that is this node's parent in the tree
        cost_to_come: **float or int**
            cost to travel to this node via its parents from the root
        path_to_parent: **numpy array**
            array(s) of points describing the trajectory from the parent to this node in state space
        yaw: **float**
            Heading corresponding to this state vector
        is_goal: **bool**
            True if this node is a goal node

    **Methods**
        dist_to_node: get distance metric to a given Node
        plot_node: plot the node on the current Matplotlib figure
    """
    def __init__(self, x=None, y=None, theta=None):
        """
        Constructor for x-y-theta node.

        Args:
            x: **double or int, optional**
                x-coordinate of vehicle
            y: **double or int, optional**
                y-coordinate of vehicle
            theta: **double or int, optional**
                Heading of vehicle on x-y plane (in radians)
        """
        if x is None:
            x = np.random.uniform(MAP_X_MIN, MAP_X_MAX)
        if y is None:
            y = np.random.uniform(MAP_Y_MIN, MAP_Y_MAX)
        if theta is None:
            theta = np.random.uniform(YAW_MIN, YAW_MAX)
        super().__init__([x, y, theta])
        self.yaw = theta
        self.is_goal = False

    def dist_to_node(self, node):
        """Get distance metric to a given node. The Euclidean distance was chosen for this node type."""
        (x1, y1, t1) = map(float, self.Point.coordinates)
        (x2, y2, t1) = map(float, node.Point.coordinates)
        return fast_Euclidean_dist(x1, y1, x2, y2)

    def plot_node(self):
        """Plot this node in the current Matplotlib figure"""
        if self.is_goal:
            plt.arrow(self.plotting_object[0], self.plotting_object[1],
                      ARROW_LENGTH * np.cos(self.yaw), ARROW_LENGTH * np.sin(self.yaw), width=ARROW_WIDTH, facecolor='m')
        else:
            plt.arrow(self.plotting_object[0], self.plotting_object[1],
                      ARROW_LENGTH * np.cos(self.yaw), ARROW_LENGTH * np.sin(self.yaw), width=ARROW_WIDTH, facecolor='g')
