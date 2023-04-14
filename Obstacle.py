from abc import ABC, abstractmethod
from Node import Node
import numpy as np


class Obstacle(ABC):
    """
    Abstract base class for obstacles.

    ** Methods **
    is_within_obs: takes a Node object, and returns True if the Node is within the Obstacle
    """
    @abstractmethod
    def is_within_obs(self, node: Node) -> bool:
        """Return True if node is within this Obstacle"""
        return True


class CircularObstacle(Obstacle):
    """
    Class for 2D circular obstacles. This is a subclass of the Obstacle class.

    ** Attributes **
    x: x-coordinate of center
    y: y-coordinate of center
    r: radius of obstacle

    ** Methods **
    is_within_obs: takes a Node object, and returns True if the Node is within the Obstacle
    """
    def __init__(self, x, y, r):
        """
        Constructor for circular obstacle.

        Args:
            x: **double or int**
                x-coordinate of circle
            y: **double or int**
                y-coordinate of circle
            r: **double or int**
                radius of circle
        """
        self.x = x
        self.y = y
        self.r = r

    def is_within_obs(self, node: Node):
        """Takes a Node object and returns True if the node is within the circle"""
        assert hasattr(node, "x") and hasattr(node, "y"), "Node object must have x and y attributes"
        dist = np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)
        return dist < self.r


class AxisAlignedRectObstacle(Obstacle):
    def __init__(self, x_bl, y_bl, lx, ly):
        """
        Constructor for axis-aligned rectangular obstacle.

        Args:
            x_bl: **double or int**
                x-coordinate of bottom-left corner of rectangle
            y_bl: **double or int**
                y-coordinate of bottom-left corner of rectangle
            lx: **double or int**
                length of side of rectangle along x-axis
            ly: **double or int**
                length of side of rectangle along y-axis
        """
        self.x_bl = x_bl
        self.y_bl = y_bl
        self.lx = lx
        self.ly = ly

    def is_within_obs(self, node: Node):
        """Takes a Node object and returns True if the node is within the rectangle"""
        assert hasattr(node, "x") and hasattr(node, "y"), "Node object must have x and y attributes"
        return (self.x_bl < node.x < self.x_bl + self.lx) and (self.y_bl < node.y < self.y_bl + self.ly)

