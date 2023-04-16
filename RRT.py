from RRTBase import RRTBase
from utilities import steer
import numpy as np
from params import GOAL_BIAS


class RRT(RRTBase):

    def add_node(self):
        rand = np.random.uniform()
        if rand < GOAL_BIAS:
            rand_node = self.goal
        else:
            rand_node = self.sample_free()
            if self.is_close_to_goal(rand_node):
                rand_node = self.goal
        nearest_node = self.T[self.get_nearest_node_idx(rand_node)]
        new_node = steer(nearest_node, rand_node)
        # skip if rand_node node is too close to nearest node or the generated path goes through an obstacle
        if new_node is None or not self.is_path_free(new_node):
            return
        # set flag saying that a solution has been found if the new node from steer is close to the goal node
        if self.is_close_to_goal(new_node) and self.goal_solution is None:
            self.goal_solution = new_node
            new_node.is_goal = True
        self.T.append(new_node)






