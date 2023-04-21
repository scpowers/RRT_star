from RRTBase import RRTBase
from Node import *
from utilities import steer, path_cost
import numpy as np
from params import GOAL_BIAS, NEAR_DIST_THRESHOLD, GAMMA_STAR, MAX_STEP, N_SAMPLES
from OtherUtilities import angle_check
from tqdm import tqdm


class RRTStar(RRTBase):

    def add_node(self):
        # goal biasing process
        rand = np.random.uniform()
        if rand < GOAL_BIAS:
            rand_node = self.goal
        else:
            rand_node = self.sample_free()

        nearest_node_idx = self.get_nearest_node_idx(rand_node)
        nearest_node = self.T[nearest_node_idx]
        path, collision_objects = steer(nearest_node, rand_node)

        # skip if rand_node node is too close to nearest node or the generated path goes through an obstacle
        if path is None or not self.is_path_free(collision_objects):
            return

        # finally create a new Node
        new_node = XYThetaNode(path[0, -1], path[1, -1], path[2, -1])
        new_node.parent = nearest_node
        new_node.path_to_parent = (path, collision_objects)
        new_node.cost_to_come = nearest_node.cost_to_come + path_cost(collision_objects)

        # get the set of nodes that are within a given radius distance-wise of the new node
        nearby_idxs = self.get_nearby_node_idxs(new_node)

        # choose the best parent node for the new node, not just choosing the nearest one like RRT
        (new_node, best_parent_index) = self.choose_best_parent(rand_node, new_node, nearby_idxs)
        if new_node is None:
            return

        # set flag saying that a solution has been found if the new node from steer is close to the goal node
        if self.is_close_to_goal(new_node):
            if self.goal_solution is None and angle_check(self.goal.yaw, new_node.yaw):
                new_node.is_goal = True
                new_node.yaw = self.goal.yaw
                self.goal_solution = new_node
            else:
                return

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
        best_parent_index = None  # this was the original nearest node
        best_cost_to_come = np.inf
        best_path_to_parent = None

        for idx in nearby_idxs:
            # get the iterating near node
            tmp_near_node = self.T[idx]
            # attempt to steer FROM the iterating near node TO the new node
            path, collision_objects = steer(tmp_near_node, new_node)  #

            # skip if rand_node node is too close to nearest node or the generated path goes through an obstacle
            if path is None or not self.is_path_free(collision_objects):
                continue

            # compute new cost-to-come, coming from this iterating near node to the new node
            new_cost_to_come = tmp_near_node.cost_to_come + path_cost(collision_objects)

            # if this cost is better, set the iterating node as the best parent node
            if new_cost_to_come < best_cost_to_come:
                best_parent_index = idx
                best_cost_to_come = new_cost_to_come
                best_path_to_parent = (path, collision_objects)

        # use the best parent in the vicinity
        if best_parent_index is None:
            return None, None
        path = best_path_to_parent[0]
        new_node = XYThetaNode(path[0, -1], path[1, -1], path[2, -1])
        new_node.parent = self.T[best_parent_index]
        new_node.cost_to_come = best_cost_to_come
        new_node.path_to_parent = best_path_to_parent

        return new_node, best_parent_index

    def rewire(self, new_node, nearby_idxs, best_parent_index):
        # rewire nodes in the vicinity
        for idx in nearby_idxs:
            if idx == best_parent_index:  # skip the best parent from above
                continue
            tmp_near_node = self.T[idx]
            # attempt to steer FROM new node TO the iterating near node
            path, collision_objects = steer(new_node, tmp_near_node)

            # skip if rand_node node is too close to nearest node or the generated path goes through an obstacle
            if path is None or not self.is_path_free(collision_objects):
                continue

            # compute new cost-to-come, coming from the new node to this iterating near node
            new_cost_to_come = new_node.cost_to_come + path_cost(collision_objects)

            if new_cost_to_come < tmp_near_node.cost_to_come:
                # actually cheaper to go to new_node first instead of the original parent of this iterating near node
                tmp_near_node.parent = new_node
                tmp_near_node.cost_to_come = new_cost_to_come
                tmp_near_node.path_to_parent = (path, collision_objects)

    def build_tree(self):
        for i in tqdm(range(N_SAMPLES)):
            self.add_node()
        if self.goal_solution is None:
            print('No path to goal found.')
        else:
            print('Found a path to the goal!')
