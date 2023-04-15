from abc import ABC, abstractmethod
from Node import Node
from sympy import Circle, Point, Polygon
from matplotlib.patches import Circle as mCircle
from matplotlib.patches import Polygon as mPolygon


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

    @abstractmethod
    def line_collision_check(self, node: Node) -> bool:
        """Return True if node's path to parent intersects this Obstacle"""
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
        self.Circle = Circle(Point(x, y), r)
        self.plotting_object = mCircle((x, y), r)

    def is_within_obs(self, node: Node) -> bool:
        """Takes a Node object and returns True if the node is within the circle"""
        return self.Circle.center.distance(node.Point) < self.Circle.radius

    def line_collision_check(self, node: Node) -> bool:
        """Return True if node's path to parent intersects this Obstacle"""
        if node.parent is None:  # root node case, there is no path to parent
            return False
        intersection_pts = node.path_to_parent[1].intersection(self.Circle)
        return len(intersection_pts) > 0


class PolygonObstacle(Obstacle):
    def __init__(self, *points):
        """
        Constructor for axis-aligned rectangular obstacle.

        Args:
            *points: **sequence of tuples**
                Sequence of points (as tuples) that define the vertices of the polygon
        """
        self.Polygon = Polygon(*points)
        self.plotting_object = mPolygon(list(points))

    def is_within_obs(self, node: Node) -> bool:
        """Takes a Node object and returns True if the node is within the polygon"""
        return self.Polygon.encloses_point(node.Point)

    def line_collision_check(self, node: Node) -> bool:
        """Return True if node's path to parent intersects this Obstacle"""
        if node.parent is None:  # root node case, there is no path to parent
            return False
        intersection_pts = node.path_to_parent[1].intersection(self.Polygon)
        return len(intersection_pts) > 0
