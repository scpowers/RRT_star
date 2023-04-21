from Obstacle import *
from RRTStar import RRTStar
from RRT import RRT

circ_obs = CircularObstacle(40, 20, 10)
rect_obs = PolygonObstacle((30, 0), (40, 0), (40, 10), (30, 10))
circ2_obs = CircularObstacle(90, 90, 5)

q_start = [0, 0, 0]
q_goal = [60, 40, 0]
#obs = [circ_obs, rect_obs, circ2_obs]
obs = [circ_obs]
#obs = []
rrt = RRT(q_start, q_goal, obs)
rrt.build_tree()
rrt.visualize_tree()
