import numpy as np
from params import *


class Node:
    """
    Base class for nodes.

    ** Attributes **
        x: x-coordinate of vehicle
        y: y-coordinate of vehicle
        parent: Node object that is this node's parent in the tree
        cost_to_come: cost to travel to this node via its parents from the root
        path_to_parent: array(s) of points describing the trajectory from the parent to this node in state space

    ** Methods **
        dist_to_node: get Euclidean distance to a given Node
    """
    def __init__(self, x=None, y=None):
        """
        Parameter constructor for general node.

        Args:
            x: **double or int**
                x-coordinate of vehicle
            y: **double or int**
                y-coordinate of vehicle
        """
        if x is None:
            x = np.random.uniform(MAP_X_MIN, MAP_X_MAX)
        if y is None:
            y = np.random.uniform(MAP_Y_MIN, MAP_Y_MAX)
        self.x = x
        self.y = y
        self.parent = None
        self.cost_to_come = np.inf
        self.path_to_parent = None

    def dist_to_node(self, node):
        return np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)


class XYThetaNode(Node):
    """
    class for x-y-theta nodes.

    ** Attributes **
        x: x-coordinate of vehicle
        y: y-coordinate of vehicle
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
        super().__init__(x, y)
        if theta is None:
            theta = np.random.uniform(YAW_MIN, YAW_MAX)
        self.yaw = theta

    def dist_to_node(self, node):
        """Get distance metric to a given node. The Euclidean distance was chosen for this node type."""
        super().dist_to_node(node)
