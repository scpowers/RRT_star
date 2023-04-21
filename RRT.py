from RRTBase import RRTBase
from utilities import steer
import numpy as np
from params import GOAL_BIAS, N_SAMPLES
from Node import XYThetaNode
from utilities import path_cost
from OtherUtilities import angle_check
from tqdm import tqdm


class RRT(RRTBase):

    def add_node(self):
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
        for i in tqdm(range(N_SAMPLES)):
            self.add_node()
            if self.goal_solution is not None:
                break
        if self.goal_solution is None:
            print('No path to goal found.')
        else:
            print('Found a path to the goal!')




