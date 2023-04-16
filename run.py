from RRTBase import *
import numpy as np
from utilities import steer
import matplotlib.pyplot as plt
from RRT import RRT

circ_obs = CircularObstacle(4, 4, 1)
rect_obs = PolygonObstacle((3, 0), (4, 0), (4, 1), (3, 1))
circ2_obs = CircularObstacle(9, 9, 1)

"""
node1 = XYThetaNode(0.0, 0.0, 0.0)
print(f'node 1 is within first circular obstacle: {circ_obs.is_within_obs(node1)}')
print(f'node 1 is within second circular obstacle: {circ_obs.is_within_obs(node1)}')
print(f'node 1 is within polygon obstacle: {rect_obs.is_within_obs(node1)}')

obs_list = [circ_obs, rect_obs]
print(f'is node 1 within any obstacle: {np.any([obstacle.is_within_obs(node1) for obstacle in obs_list])}')

node2 = XYThetaNode(1.2, 1.2, 0.0)
print(f'distance from node 1 to node 2: {node1.dist_to_node(node2)}')
node3 = steer(node1, node2)
print(f'node3 path to parent intersects first circular obstacle: {circ_obs.line_collision_check(node3)}')
print(f'node3 path to parent intersects second circular obstacle: {circ2_obs.line_collision_check(node3)}')
print(f'node3 path to parent intersects polygon obstacle: {rect_obs.line_collision_check(node3)}')

fig, ax = plt.subplots()
ax.add_patch(circ_obs.plotting_object)
ax.add_patch(rect_obs.plotting_object)
ax.add_patch(circ2_obs.plotting_object)
plt.plot(node3.path_to_parent[0][0, :], node3.path_to_parent[0][1, :], color='k')
node1.plot_node()
node3.plot_node()
plt.xlim(-5, 5)
plt.ylim(-5, 5)
plt.axis('equal')
plt.show()
"""
"""
q_start = [5, 5, 0]
q_goal = [1, 8, 0]
"""
q_start = [0, 0, 0]
q_goal = [7, 7, 0]
obs = [circ_obs, rect_obs, circ2_obs]
#obs = []
rrt = RRT(q_start, q_goal, obs)
rrt.build_tree()
rrt.visualize_tree()
