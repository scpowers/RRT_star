from RRTBase import *
import numpy as np

node = XYThetaNode([0, 0, 0])
circ_obs = CircularObstacle(1, 1, 0.1)
rect_obs = AxisAlignedRectObstacle(-1, -1, 1, 2)

print(f'node is within circular obstacle: {circ_obs.is_within_obs(node)}')
print(f'node is within rectangular obstacle: {rect_obs.is_within_obs(node)}')

obs_list = [circ_obs, rect_obs]
print(f'{np.any([obstacle.is_within_obs(node) for obstacle in obs_list])}')
