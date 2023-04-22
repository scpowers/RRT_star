from RRTBase import RRTBase
from steer_utilities import steer
import numpy as np
from params import GOAL_BIAS, N_SAMPLES
from Node import XYThetaNode
from steer_utilities import path_cost, angle_check
#from OtherUtilities import angle_check
from tqdm import tqdm


class RRT(RRTBase):
    """
    Class for rapidly-exploring random trees in their original form.

    **Attribute**
        T: **list, Node**
            List of Node objects representing the tree
        root: **Node**
            Node object representing the root configuration
        goal: **Node**
            Node object representing the goal configuration
        obs: **list, Obstacle**
            List of Obstacle objects in the environment
        goal_solution: **Node**
            Node object representing the actual discovered goal node with a parent node and cost-to-come

    **Methods**
        * add_node: add a node to the tree.
        * sample_free: return a random Node outside of any obstacles
        * is_node_free: return True if the node is not witin any obstacles
        * is_path_free: return True if the node's path to parent does not intersect any obstacles
        * get_nearest_node_idx: get index of the node closest to the given node within the tree
        * is_close_to_goal: return True if a given node is close enough to the goal to call them equivalent
        * build_tree: build the tree
        * visualize_tree: plot the tree's nodes and edges in the environment with obstacles

    """

    def add_node(self):
        """Add a node to the tree"""
        # goal biasing process
        rand = np.random.uniform()
        if rand < GOAL_BIAS:
            rand_node = self.goal
        else:
            rand_node = self.sample_free()

        nearest_node = self.T[self.get_nearest_node_idx(rand_node)]
        path, collision_objects = steer(nearest_node, rand_node)

        # skip if no valid path is found or the generated path goes through an obstacle
        if path is None or not self.is_path_free(collision_objects):
            return

        # finally create a new Node
        new_node = XYThetaNode(path[0, -1], path[1, -1], path[2, -1])
        new_node.parent = nearest_node
        new_node.path_to_parent = (path, collision_objects)
        new_node.cost_to_come = nearest_node.cost_to_come + path_cost(collision_objects)
        new_node.yaw = path[2, -1]

        # special case: generate path's endpoint is close to the goal
        if self.is_close_to_goal(new_node):
            # if no goal solution exists yet and the previous heading is sufficiently close to the goal heading, great
            if self.goal_solution is None and angle_check(self.goal.yaw, new_node.yaw):
                new_node.is_goal = True
                new_node.yaw = self.goal.yaw
                self.goal_solution = new_node
            else:  # if it's close to the goal but not a viable solution, don't add it to reduce clutter around it
                return
        # finally, add node to the tree
        self.T.append(new_node)

    def build_tree(self):
        """Build the tree"""
        for i in tqdm(range(N_SAMPLES)):
            self.add_node()
            if self.goal_solution is not None:
                print('found a path to the goal')
                break
        if self.goal_solution is None:
            print('No path to goal found.')




