from abc import ABC, abstractmethod
from Node import Node
from sympy import Circle, Point, Polygon
from matplotlib.patches import Circle as mCircle
from matplotlib.patches import Polygon as mPolygon


class Obstacle(ABC):
    """
    Abstract base class for obstacles.

    **Methods**
    * is_within_obs: takes a Node object and returns True if the Node is within the Obstacle
    * line_collision_check: takes a list of Segment object and returns True if any Segment intersects the Obstacle
    """
    @abstractmethod
    def is_within_obs(self, node: Node) -> bool:
        """Return True if node is within this Obstacle"""
        return True

    @abstractmethod
    def line_collision_check(self, segments) -> bool:
        """Return True if any Segments in segments intersects this Obstacle"""
        return True


class CircularObstacle(Obstacle):
    """
    Class for 2D circular obstacles. This is a subclass of the Obstacle class.

    **Attributes**
    Circle: **sympy Circle**
        Sympy Circle object representing the obstacle.
    plotting_object: **Matplotlib pyplot patch**
        Matplotlib object representing the obstacle.

    **Methods**
    * is_within_obs: takes a Node object, and returns True if the Node is within the Obstacle
    * line_collision_check: takes a list of Segment object and returns True if any Segment intersects the Obstacle
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
        (x, y, theta) = map(float, node.Point.coordinates)
        tmp_point = Point(x, y)
        return self.Circle.center.distance(tmp_point) < self.Circle.radius

    def line_collision_check(self, segments) -> bool:
        """Return True if any Segments in segments intersects this Obstacle"""
        for segment in segments:
            intersection_pts = segment.intersection(self.Circle)
            if len(intersection_pts) > 0:
                return True
        return False


class PolygonObstacle(Obstacle):
    """
    Class for 2D polygonal obstacles. This is a subclass of the Obstacle class.

    **Attributes**
    Polygon: **sympy Polygon**
        Sympy Polygon object representing the obstacle.
    plotting_object: **Matplotlib pyplot patch**
        Matplotlib object representing the obstacle.

    **Methods**
    * is_within_obs: takes a Node object, and returns True if the Node is within the Obstacle
    * line_collision_check: takes a list of Segment object and returns True if any Segment intersects the Obstacle
    """
    def __init__(self, *points):
        """
        Constructor for Polygon obstacle.

        Args:
            *points: **sequence of tuples**
                Sequence of points (as tuples) that define the vertices of the polygon
        """
        self.Polygon = Polygon(*points)
        self.plotting_object = mPolygon(list(points))

    def is_within_obs(self, node: Node) -> bool:
        """Takes a Node object and returns True if the node is within the polygon"""
        (x, y, theta) = map(float, node.Point.coordinates)
        tmp_point = Point(x, y)
        return self.Polygon.encloses_point(tmp_point)

    def line_collision_check(self, segments) -> bool:
        """Return True if any Segments in segments intersects this Obstacle"""
        for segment in segments:
            intersection_pts = segment.intersection(self.Polygon)
            if len(intersection_pts) > 0:
                return True
        return False
