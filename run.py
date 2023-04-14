from RRTBase import *

node = XYThetaNode([0, 0, 0])
circ_obs = CircularObstacle(1, 1, 2)
rect_obs = AxisAlignedRectObstacle(-1, -1, 2, 2)

print(f'node is within circular obstacle: {circ_obs.is_within_obs(node)}')
print(f'node is within rectangular obstacle: {rect_obs.is_within_obs(node)}')
