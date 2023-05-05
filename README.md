# RRT/RRT* for a simple car-like vehicle
### Project Overview
![example](https://user-images.githubusercontent.com/54383192/236500759-54a2878f-f571-4e6e-8ffa-f6eeef57be03.png)


This is the code for my Software Carpentry final project. It builds either RRTs or RRT*s
with drivable paths between nodes. An initial implementation generated feasible sub-trajectories
using the differential flatness property of a simple car-like model, but since sampling in state space
(x, y, yaw) proved to be computationally expensive, feasible trajectories are instead computed using
a maximum heading difference between nodes with a non-zero minimum distance between nodes. This allows the nodes
to be sampled in (x, y) space, which is substantially cheaper computationally. While this speed increase is substantial, the non-zero minimum distance between nodes can create dead zones, which can thereby sometimes prevent the algorithm from finding a solution to the goal.

### Dependencies
* numpy - ``conda install numpy``
* numba - ``conda install numba``
* matplotlib - ``conda install matplotlib``
* sympy - ``conda install sympy``
* tqdm - ``conda install -c conda-forge tqdm``

### Usage
To use this implementation in a main file that is executed:
1. create a list of Obstacle objects (either circular or polygonal)
2. specify an initial pose (x, y, yaw)
3. specify a final pose
4. create a tree object (either RRT or RRT*) using items 1-3
5. run tree.build() to construct the tree
6. run tree.visualize() to view the tree

An example usage is given in ``run.py``. 

**Note:** in the current implementation, an RRT will stop building the tree
as soon as a path to the goal is found, whereas an RRT* will attempt to add 
a fixed number of nodes (specified by the N_SAMPLES parameter in ``params.py``).
This is because a major advantage of RRT* is that it will rewire nodes if a cheaper
path becomes available, so the path to the goal can change accordingly. 

### Acknowledgements
* The RRT algorithm is from this paper: http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
* The RRT* algorithm is from this paper: https://arxiv.org/abs/1105.1186
* The high-level inheritance structure between RRTBase, RRT, and RRTStar is 
inspired by the following implementation: https://github.com/danny45s/motion-planning
