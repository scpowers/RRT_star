import numpy as np
from abc import ABC


class Node(ABC):
    """
    Base class for nodes.

    ** Attributes **
        state: list of state variables corresponding to this node
        parent: Node object that is this node's parent in the tree
        cost_to_come: cost to travel to this node via its parents from the root
        path_to_parent: array(s) of points describing the trajectory from the parent to this node in state space
    """
    def __init__(self, state):
        """
        Constructor for general node.

        Args:
            state: **list**
                list of state variables
        """
        self.state = state
        self.parent = None
        self.cost_to_come = np.inf
        self.path_to_parent = None


class XYThetaNode(Node):
    def __init__(self, state):
        """
        Constructor for x-y-theta node.

        Args:
            state: **list, double or int**
                list of state variables in the following order:
                    1. x
                    2. y
                    3. theta/yaw (heading on x-y plane)
        """
        super().__init__(state)
        self.x = state[0]
        self.y = state[1]
        self.yaw = state[2]
