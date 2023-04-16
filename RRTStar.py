from RRTBase import RRTBase
from Node import Node
from utilities import steer
import numpy as np
from params import GOAL_BIAS, NEAR_DIST_THRESHOLD, GAMMA_STAR, MAX_STEP


class RRTStar(RRTBase):

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
        new_node = steer(nearest_node, rand_node)
        # skip if rand_node node is too close to nearest node or the generated path goes through an obstacle
        if new_node is None or not self.is_path_free(new_node):
            return

        # get the set of nodes that are within a given radius distance-wise of the new node
        nearby_idxs = self.get_nearby_node_idxs(new_node)

        # choose the best parent node for the new node, not just choosing the nearest one like RRT
        (new_node, best_parent_index) = self.choose_best_parent(rand_node, new_node, nearby_idxs)

        # set flag saying that a solution has been found if the new node from steer is close to the goal node
        if self.is_close_to_goal(new_node) and self.goal_solution is None:
            self.goal_solution = new_node
            new_node.is_goal = True
        self.T.append(new_node)

        # rewire
        self.rewire(new_node, nearby_idxs, best_parent_index)

    def get_nearby_node_idxs(self, node: Node):
        node_idxs = []
        for i, existing_node in enumerate(self.T):
            #dist_threshold = min(GAMMA_STAR * np.sqrt(np.log(len(self.T)) / len(self.T)), MAX_STEP)
            dist_threshold = NEAR_DIST_THRESHOLD
            if node.dist_to_node(existing_node) < dist_threshold:
                node_idxs.append(i)
        return node_idxs

    def choose_best_parent(self, rand_node, new_node, nearby_idxs):
        # choose best parent node for new_node by looking at each nearby node and their associated costs
        best_parent_index = self.get_nearest_node_idx(rand_node)  # this was the original nearest node
        best_new_node = new_node

        for idx in nearby_idxs:
            tmp_near_node = self.T[idx]
            tmp_new_node = steer(tmp_near_node, new_node)  # steering FROM the iterating near node TO the new node
            if tmp_new_node is None or not self.is_path_free(tmp_new_node):
                continue
            if tmp_new_node.cost_to_come < best_new_node.cost_to_come:
                best_parent_index = idx
                best_new_node = tmp_new_node

        # use the best parent in the vicinity
        """
        new_node.parent = best_parent
        new_node.cost_to_come = best_total_cost
        new_node.path_to_parent = best_path_to_parent

        return new_node, best_parent_index
        """
        return best_new_node, best_parent_index

    def rewire(self, new_node, nearby_idxs, best_parent_index):
        # rewire nodes in the vicinity
        for idx in nearby_idxs:
            if idx == best_parent_index:  # skip the best parent from above
                continue
            tmp_near_node = self.T[idx]
            tmp_new_node = steer(new_node, tmp_near_node)  # steering FROM new node TO the iterating near node
            if tmp_new_node is None or not self.is_path_free(tmp_new_node):
                continue
            if tmp_new_node.cost_to_come < tmp_near_node.cost_to_come:
                # actually cheaper to go to new_node first instead of the original parent of this iterating near node
                tmp_near_node.parent = tmp_new_node.parent
                tmp_near_node.cost_to_come = tmp_new_node.cost_to_come
                tmp_near_node.path_to_parent = tmp_new_node.path_to_parent
