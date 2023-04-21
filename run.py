#from RRTBase import *
import numpy as np
#from utilities import steer
import matplotlib.pyplot as plt
#from RRT import RRT
from Obstacle import *
from RRTStar import RRTStar
from RRT import RRT
from OtherUtilities import *
from utilities import path_cost
from Node import XYThetaNode

circ_obs = CircularObstacle(40, 20, 10)
rect_obs = PolygonObstacle((30, 0), (40, 0), (40, 10), (30, 10))
circ2_obs = CircularObstacle(90, 90, 5)

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
q_start = [5, 5, 0]
q_goal = [1, 8, 0]
"""

q_start = [0, 0, 0]
q_goal = [80, 30, 0]
#obs = [circ_obs, rect_obs, circ2_obs]
obs = [circ_obs]
#obs = []
rrt = RRT(q_start, q_goal, obs)
rrt.build_tree()
rrt.visualize_tree()

"""
state0 = np.array([0.0, 5.0, 0.0]).reshape(-1, 1)
statef = np.array([5.0, 2.5, 0.0]).reshape(-1, 1)
path, collision_objects, controls, valid_path = generate_trajectory_function(state0, statef)
print(f'is path valid? {valid_path}')
print(f'path cost: {path_cost(collision_objects)}')
print(f'circ object collision check: {circ_obs.line_collision_check(collision_objects)}')

plt.figure()
plt.plot(range(len(controls[0, :])), controls[0, :])
plt.plot(range(len(controls[1, :])), controls[1, :])
plt.show()

fig, ax = plt.subplots()
ax.add_patch(circ_obs.plotting_object)
plt.plot(path[0, :], path[1, :])
ax.set_aspect('equal')
plt.show()
"""

