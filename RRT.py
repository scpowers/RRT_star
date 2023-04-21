from RRTBase import RRTBase
from utilities import steer
import numpy as np
from params import GOAL_BIAS, MIN_STEP
from Node import XYThetaNode
from utilities import path_cost
from OtherUtilities import angle_check


class RRT(RRTBase):

    def add_node(self):
        # goal biasing process
        rand = np.random.uniform()
        if rand < GOAL_BIAS:
            rand_node = self.goal
        else:
            rand_node = self.sample_free()
            if self.is_close_to_goal(rand_node):  # if the randomly selected node is close to the goal, just use goal
                rand_node = self.goal

        nearest_node = self.T[self.get_nearest_node_idx(rand_node)]
        path, collision_objects = steer(nearest_node, rand_node)

        # skip if rand_node node is too close to nearest node or the generated path goes through an obstacle
        if path is None or not self.is_path_free(collision_objects):
            return

        # finally create a new Node
        new_node = XYThetaNode(path[0, -1], path[1, -1], path[2, -1])
        if new_node.dist_to_node(self.T[self.get_nearest_node_idx(new_node)]) < MIN_STEP:
            return
        new_node.parent = nearest_node
        new_node.path_to_parent = (path, collision_objects)
        new_node.cost_to_come = nearest_node.cost_to_come + path_cost(collision_objects)
        new_node.yaw = path[2, -1]

        # set flag saying that a solution has been found if the new node from steer is close to the goal node
        if self.goal_solution is None and self.is_close_to_goal(new_node) and angle_check(self.goal.yaw, new_node.yaw):
            self.goal_solution = new_node
            new_node.is_goal = True
            new_node.yaw = self.goal.yaw
        self.T.append(new_node)






