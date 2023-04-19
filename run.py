#from RRTBase import *
import numpy as np
#from utilities import steer
import matplotlib.pyplot as plt
#from RRT import RRT
from Obstacle import *
from RRTStar import RRTStar
from OtherUtilities import *

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
q_start = [5, 5, 0]
q_goal = [1, 8, 0]
"""

"""
q_start = [0, 0, 0]
q_goal = [7, 7, 0]
obs = [circ_obs, rect_obs, circ2_obs]
#obs = []
rrt = RRTStar(q_start, q_goal, obs)
rrt.build_tree()
rrt.visualize_tree()
"""

state0 = np.array([0.0, 5.0, 0.0]).reshape(-1, 1)
statef = np.array([5.0, 2.5, -3*np.pi/4]).reshape(-1, 1)
T = 10.0

y0 = car_h(state0)
yf = car_h(statef)
dy0 = np.array([np.cos(state0[2]), np.sin(state0[2])]).reshape(-1, 1)
dyf = np.array([np.cos(statef[2]), np.sin(statef[2])]).reshape(-1, 1)

A = poly3_coeff(y0, dy0, yf, dyf, 0.0, T)

n_steps = 100
t1_vec = np.linspace(0, T, n_steps)
state_matrix = np.zeros((3, n_steps))

for t in range(n_steps):
    y = A @ lambdafunc(t1_vec[t])
    dy = A @ dlambdafunc(t1_vec[t])
    state_matrix[:, t] = psi(y, dy).flatten()

plt.plot(state_matrix[0, :], state_matrix[1, :])
plt.show()

