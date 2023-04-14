from abc import ABC, abstractmethod
from Node import Node
import numpy as np


class Obstacle(ABC):
    @abstractmethod
    def is_within_obs(self, node: Node) -> bool:
        return True


class CircularObstacle(Obstacle):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def is_within_obs(self, node: Node):
        assert hasattr(node, "x") and hasattr(node, "y"), "Node object must have x and y attributes"
        dist = np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)
        return dist < self.r


class AxisAlignedRectObstacle(Obstacle):
    def __init__(self, x_bl, y_bl, lx, ly):
        self.x_bl = x_bl
        self.y_bl = y_bl
        self.lx = lx
        self.ly = ly

    def is_within_obs(self, node: Node):
        assert hasattr(node, "x") and hasattr(node, "y"), "Node object must have x and y attributes"
        return (self.x_bl < node.x < self.x_bl + self.lx) and (self.y_bl < node.y < self.y_bl + self.ly)

