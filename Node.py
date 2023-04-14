import numpy as np
from abc import ABC


class Node(ABC):
    """
    Base class for nodes.

    ** Attributes **
        x: x-coordinate of vehicle
        y: y-coordinate of vehicle
        parent: Node object that is this node's parent in the tree
        cost_to_come: cost to travel to this node via its parents from the root
        path_to_parent: array(s) of points describing the trajectory from the parent to this node in state space
    """
    def __init__(self, x, y):
        """
        Constructor for general node.

        Args:
            x: **double or int**
                x-coordinate of vehicle
            y: **double or int**
                y-coordinate of vehicle
        """
        self.x = x
        self.y = y
        self.parent = None
        self.cost_to_come = np.inf
        self.path_to_parent = None


class XYThetaNode(Node):
    def __init__(self, x, y, theta):
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
        self.yaw = theta
