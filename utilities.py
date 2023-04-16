import numpy as np
from Node import XYThetaNode
from params import MAX_STEP, MIN_STEP, YAW_DIFF_THRESHOLD, PATH_LINSPACE_N, EUCLIDEAN_DIST_COEFF, HEADING_DIFF_COEFF
from sympy import Segment, Point
import matplotlib.pyplot as plt


def steer(near_node: XYThetaNode, rand_node: XYThetaNode):
    # if the nodes are too close together in the x-y plane, return early
    dist = near_node.dist_to_node(rand_node)
    if dist < MIN_STEP:
        return None

    near_node_coords = list(map(float, near_node.Point.coordinates))
    rand_node_coords = list(map(float, rand_node.Point.coordinates))

    # compute ideal path heading (line connecting near_node to rand_node), and if this heading is too different
    # from near_node's heading, then make it as far in the right direction as possible
    new_heading = rand_node.yaw
    # TODO: enforce heading difference limits (see theta1 and theta2 in sketch)

    # if the distance between nodes is too large, move as much as you can in the correct direction
    dx = rand_node_coords[0] - near_node_coords[0]
    dy = rand_node_coords[1] - near_node_coords[1]
    new_coords = [near_node_coords[0] + dx, near_node_coords[1] + dy]
    if dist > MAX_STEP:
        scale = float(MAX_STEP / dist)
        new_coords = [near_node_coords[0] + scale*dx, near_node_coords[1] + scale*dy]
    new_node = XYThetaNode(new_coords[0], new_coords[1], new_heading)

    # generate straight paths in state space to reach new_node
    x_path = np.linspace(near_node_coords[0], new_coords[0], num=PATH_LINSPACE_N)
    y_path = np.linspace(near_node_coords[1], new_coords[1], num=PATH_LINSPACE_N)
    theta_path = np.linspace(near_node.yaw, new_heading, num=PATH_LINSPACE_N)
    new_path = np.flip(np.vstack((x_path, y_path, theta_path)), 1)  # flipping to go backwards towards parent

    # generate Line object corresponding to the x-y components for collision checking
    p1 = Point(x_path[-1], y_path[-1])  # going backwards
    p2 = Point(x_path[0], y_path[0])
    new_path_collision_object = Segment(p1, p2)

    # populate attributes of new_node
    new_node.parent = near_node
    new_node.cost_to_come = near_node.cost_to_come + path_cost(new_path)
    new_node.path_to_parent = (new_path, new_path_collision_object)

    return new_node


def path_cost(path):
    euclidean_dist_cost = EUCLIDEAN_DIST_COEFF * np.sqrt((path[0, -1] - path[0, 0])**2 + (path[1, -1] - path[1, 0])**2)
    heading_diff_cost = HEADING_DIFF_COEFF * abs(path[2, -1] - path[2, 0])
    return euclidean_dist_cost + heading_diff_cost
