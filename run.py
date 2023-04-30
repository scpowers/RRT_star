from Obstacle import *
from RRT import RRT
import numpy as np

if __name__ == '__main__':
    # create Obstacle objects
    circ_obs = CircularObstacle(40, 20, 10)
    rect_obs = PolygonObstacle((60, 60), (75, 65), (85, 80), (70, 75))

    # specify initial and goal poses
    q_start = [0, 0, 0]
    q_goal = [50, 45, np.pi/6]

    # initialize tree
    obs = [circ_obs, rect_obs]
    rrt = RRT(q_start, q_goal, obs)

    # build and visualize the tree
    rrt.build_tree()
    rrt.visualize_tree()
