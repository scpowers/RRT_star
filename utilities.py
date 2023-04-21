import numpy as np
from params import MAX_STEP, MIN_STEP, GOAL_DIST_THRESHOLD
from sympy import Curve
from Node import XYThetaNode
from OtherUtilities import generate_trajectory, generate_straight_path


def steer(near_node: XYThetaNode, rand_node: XYThetaNode, allow_new_thetaf=True):
    # if the nodes are too close together in the x-y plane, return early
    dist = near_node.dist_to_node(rand_node)
    if dist < MIN_STEP:
        return None, None

    near_node_coords = np.array(list(map(float, near_node.Point.coordinates))).reshape(-1, 1)
    rand_node_coords = np.array(list(map(float, rand_node.Point.coordinates))).reshape(-1, 1)

    # if the distance between nodes is too large, move as much as you can in the correct direction
    dx = rand_node_coords[0, 0] - near_node_coords[0, 0]
    dy = rand_node_coords[1, 0] - near_node_coords[1, 0]
    if dist > MAX_STEP:
        close_enough_to_goal = False
        scale = float(MAX_STEP / dist)
        new_coords = near_node_coords + np.array([scale*dx, scale*dy, 0]).reshape(-1, 1)
        new_coords[2, 0] = rand_node_coords[2, 0]
    else:
        new_coords = rand_node_coords
        close_enough_to_goal = True

    # generate trajectory to new coordinates from near node and associated Curve object for collision checking
    #path, collision_objects, controls, valid_path = generate_trajectory(near_node_coords, new_coords)
    new_coords_are_goal = rand_node.is_goal and close_enough_to_goal
    path, collision_objects, controls, valid_path = generate_straight_path(near_node_coords, new_coords,
                                                                           new_coords_are_goal, allow_new_thetaf)

    # this is hit if the generated controls are beyond the steering control limits
    if not valid_path:
        return None, None

    return path, collision_objects


def path_cost(path_collision_objects):
    return float(sum([segment.length for segment in path_collision_objects]))


