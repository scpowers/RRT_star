import numpy as np
from params import *
from sympy import Point
import matplotlib.pyplot as plt
from OtherUtilities import fast_Euclidean_dist


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
        (x1, y1) = map(float, self.Point.coordinates)
        (x2, y2) = map(float, node.Point.coordinates)
        return fast_Euclidean_dist(x1, y1, x2, y2)

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
        super().__init__([x, y, theta])
        self.is_goal = False

    def dist_to_node(self, node):
        """Get distance metric to a given node. The Euclidean distance was chosen for this node type."""
        (x1, y1, t1) = map(float, self.Point.coordinates)
        (x2, y2, t1) = map(float, node.Point.coordinates)
        return fast_Euclidean_dist(x1, y1, x2, y2)

    def plot_node(self):
        """
        if self.is_goal:
            plt.arrow(self.plotting_object[0], self.plotting_object[1],
                      ARROW_LENGTH * np.cos(self.yaw), ARROW_LENGTH * np.sin(self.yaw), width=0.1, facecolor='m')
        else:
            plt.arrow(self.plotting_object[0], self.plotting_object[1],
                      ARROW_LENGTH * np.cos(self.yaw), ARROW_LENGTH * np.sin(self.yaw), width=0.1, facecolor='g')
        """
        if self.is_goal:
            plt.plot(self.plotting_object[0], self.plotting_object[1], color='m', marker='o')
        else:
            #plt.plot(self.plotting_object[0], self.plotting_object[1], color='g', marker='.')
            pass
