import numpy as np
from params import MAX_STEP, MIN_STEP
from sympy import Curve
from Node import XYThetaNode
from OtherUtilities import generate_trajectory_function


def steer(near_node: XYThetaNode, rand_node: XYThetaNode):
    # if the nodes are too close together in the x-y plane, return early
    dist = near_node.dist_to_node(rand_node)
    if dist < MIN_STEP:
        return None

    near_node_coords = np.array(list(map(float, near_node.Point.coordinates))).reshape(-1, 1)
    rand_node_coords = np.array(list(map(float, rand_node.Point.coordinates))).reshape(-1, 1)

    # if the distance between nodes is too large, move as much as you can in the correct direction
    dx = rand_node_coords[0] - near_node_coords[0]
    dy = rand_node_coords[1] - near_node_coords[1]
    if dist > MAX_STEP:
        scale = float(MAX_STEP / dist)
        new_coords = near_node_coords + np.array([scale*dx, scale*dy, 0]).reshape(-1, 1)
    else:
        new_coords = rand_node_coords

    # generate trajectory to new coordinates from near node and associated Curve object for collision checking
    path, collision_object = generate_trajectory_function(near_node_coords, new_coords)

    # populate attributes of new_node
    #new_node.parent = near_node
    #new_node.cost_to_come = near_node.cost_to_come + path_cost(new_path)
    #new_node.path_to_parent = (new_path, new_path_collision_object)

    return path, collision_object


def path_cost(path: Curve):
    return path.length


