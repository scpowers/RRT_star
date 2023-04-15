from RRTBase import RRTBase
from utilities import steer


class RRT(RRTBase):

    def add_node(self):
        while True:
            rand_node = self.sample_free()
            nearest_node = self.T[self.get_nearest_node_idx(rand_node)]
            new_node = steer(nearest_node, rand_node)
            if new_node is not None:
                break




