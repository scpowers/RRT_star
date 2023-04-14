from abc import ABC, abstractmethod
from Node import Node, XYThetaNode
import numpy as np


class Obstacle(ABC):
    @abstractmethod
    def is_within_obs(self, node: Node):
        pass


class CircularObstacle(Obstacle):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def is_within_obs(self, node: Node):
        dist = np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)
        return dist < self.r

