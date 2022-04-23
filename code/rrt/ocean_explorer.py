"""
Assignment #2 Template file
"""
import random
import math
import numpy as np
import matplotlib.pyplot as plt

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file rrt_planner.py. Your implementation
   can be tested by running RRT_DUBINS_PROBLEM.PY (check the main function).
2. Read all class and function documentation in RRT_DUBINS_PROBLEM carefully.
   There are plenty of helper function in the class to ease implementation.
3. Your solution must meet all the conditions specificed below.
4. Below are some do's and don'ts for this problem as well.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1. The solution loop must not run for more that a certain number of random iterations
   (Specified by a class member called MAX_ITER). This is mainly a safety
   measure to avoid time-out-related issues and will be set generously.
2. The planning function must return a list of nodes that represent a collision-free path
   from start node to the goal node. The path states (path_x, path_y, path_z)
   specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation for the node class to understand the terminology)
3. The returned path should have the start node at index 0 and goal node at index -1,
   while the parent node for node i from the list should be node i-1 from the list, ie,
   the path should be a valid list of nodes.
   (READ the documentation of the node to understand the terminology)
4. The node locations must not lie outside the map boundaries specified by
   RRT_DUBINS_PROBLEM.map_area.

DO(s) and DONT(s)
-------------------
1. Do not rename the file rrt_planner.py for submission.
2. Do not change change the PLANNING function signature.
3. Do not import anything other than what is already imported in this file.
4. You can write more function in this file in order to reduce code repitition
   but these function can only be used inside the PLANNING function.
   (since only the planning function will be imported)
"""

def sample_random_cmd_vel(rrt_dubins, rng):
    xmin = rrt_dubins.x_lim[0]
    xmax = rrt_dubins.x_lim[1]
    ymin = rrt_dubins.y_lim[0]
    ymax = rrt_dubins.y_lim[1]
    zmin = rrt_dubins.z_lim[0]
    zmax = rrt_dubins.z_lim[1]
    rvx = rng.uniform(xmin, xmax)
    rvy = rng.uniform(ymin, ymax)
    rvz = rng.uniform(zmin, zmax)
    cmd_vel = rrt_dubins.Node(rvx, rvy, rvz) # not really a node in any sense of the word, but it has .x .y and .z fields
    return cmd_vel

def visualize_3d_trajectory(rrt_dubins):
    ax = plt.axes(projection='3d')

    # Data for a three-dimensional line
    zline = np.linspace(0, 15, 1000)
    xline = np.sin(zline)
    yline = np.cos(zline)
    ax.plot3D(xline, yline, zline, 'gray')

    for node in rrt_dubins.node_list:
        ax.plot3D(node.path_x, node.path_y, node.path_z, 'red')
    plt.show()


def ocean_explorer(rrt_dubins, display_map=False, rng_seed = 43):
    """
        Execute RRT planning using Dubins-style paths. Make sure to populate the node_list.

        Inputs
        -------------
        rrt_dubins  - (RRT_DUBINS_PROBLEM) Class conatining the planning
                      problem specification
        display_map - (boolean) flag for animation on or off (OPTIONAL)

        Outputs
        --------------
        (list of nodes) This must be a valid list of connected nodes that form
                        a path from start to goal node

        NOTE: In order for rrt_dubins.draw_graph function to work properly, it is important
        to populate rrt_dubins.nodes_list with all valid RRT nodes.
    """
    # Initialize RNG
    seed = rng_seed # for best results ;)
    rng = np.random.default_rng(seed)

    # LOOP for max iterations
    i = 0
    while i < rrt_dubins.max_iter:
        i += 1

        # PLAN:
        # 1. pick a random node in the current tree (possibly biased somehow)
        # 2. pick a random cmd_vel
        # 3. Propogate that start node and cmd vel, add to tree
        # 4. repeat.

        # pick random node to extend from
        nodeidx = rng.integers(len(rrt_dubins.node_list))

        # pick a random cmd vel
        random_cmd_vel = sample_random_cmd_vel(rrt_dubins, rng)

        # propogate the new node
        added_new_node = rrt_dubins.propogate(rrt_dubins.node_list[nodeidx], random_cmd_vel)

        # check collision (TODO)

        # add new node to list
        rrt_dubins.node_list.append(added_new_node)

    if i == rrt_dubins.max_iter:
        print('reached max iterations')

    if display_map:
        visualize_3d_trajectory(rrt_dubins)


    # Return path, which is a list of nodes leading to the goal...
    return None
