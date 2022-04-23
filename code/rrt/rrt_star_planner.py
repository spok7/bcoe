"""
Assignment #2 Template file
"""
import random
import math
import numpy as np

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees* (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file rrt_star_planner.py. Your
   implementation can be tested by running RRT_DUBINS_PROBLEM.PY (check the 
   main function).
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
1. Do not rename the file rrt_star_planner.py for submission.
2. Do not change change the PLANNING function signature.
3. Do not import anything other than what is already imported in this file.
4. You can write more function in this file in order to reduce code repitition
   but these function can only be used inside the PLANNING function.
   (since only the planning function will be imported)
"""

def update_children(rrt_dubins, parent, delta):
    # TODO see if this pseudocode actually works right.
    parent.cost = parent.cost - delta
    for child in parent.children:
        update_children(rrt_dubins, child, delta)

def adopt_children(rrt_dubins, old_parent, new_parent):
    # TODO see if this actually works
    # In the course of rewiring a nearby node to go through a newly added (or modified) node, we must also update child
    # records.
    children = old_parent.children
    old_parent.children = [] # remove children from custody of terrible parent
    delta = old_parent.cost - new_parent.cost # how much the new parent is better by
    for child in children:
        update_children(rrt_dubins, child, delta)
    new_parent.children = children # put children in custody of new parent.

def euclid_distance(node1, node2):
    dx = node2.x - node1.x
    dy = node2.y - node1.y
    return math.hypot(dx, dy)

def rewire_nearby(rrt_dubins, new_node, radius):
    # Afer a new node is added to the tree, call this function.
    # We will examine all other nodes within a radius of the new node and determine if their cost via the new node might
    # be lower than their previous cost. If so, we will change the parent of that
    for node,i in zip(rrt_dubins.node_list, range(len(rrt_dubins.node_list))):
        if euclid_distance(new_node, node) < radius:
            old_cost = node.cost
            new_cost = rrt_dubins.calc_new_cost(new_node, node)
            if new_cost < old_cost:
                # we've found a new shorter path, yay (more work for us :/ )
                # don't bother deleting the node from the list, it will simply be superseded by the new, lower cost node
                same_node_via_lower_cost_path = rrt_dubins.propogate(new_node, node)
                if rrt_dubins.check_collision(same_node_via_lower_cost_path):
                    rrt_dubins.node_list.append(same_node_via_lower_cost_path)
                    adopt_children(rrt_dubins, node, same_node_via_lower_cost_path)
                    rrt_dubins.node_list.pop(i)
            pass

def random_sample(rrt_dubins, rng, goal_prob):
    # roll the dice
    dice = rng.uniform(0,1)
    if dice < goal_prob:
        return rrt_dubins.goal
    # else pick a random node
    xmin = rrt_dubins.x_lim[0]
    xmax = rrt_dubins.x_lim[1]
    ymin = rrt_dubins.y_lim[0]
    ymax = rrt_dubins.y_lim[1]
    xsam = rng.uniform(xmin, xmax)
    ysam = rng.uniform(ymin, ymax)
    qsam = rng.uniform(-3.14159, 3.14159)
    return rrt_dubins.Node(xsam, ysam, qsam)

def rrt_star_planner(rrt_dubins, display_map=False, rng_seed = 43):
    """
        Execute RRT* planning using Dubins-style paths. Make sure to populate the node_list.

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
    seed = rng_seed # 43 for best results ;)
    rng = np.random.default_rng(seed)

    # initialize children in node data structure
    for node in rrt_dubins.node_list:
        node.children = []

    # LOOP for max iterations
    i = 0
    while i < rrt_dubins.max_iter:
        i += 1

        # PLAN (RRT*)
        # Basically identical for RRT except for two things:
        # A. Instead of using the path distance to determine which node is nearest, we consider the total cost
        #       i.e. cost_factor = 0 instead of 1
        # B. We rewire "nearby" nodes already in the list if their total cost could be reduced by routing via new node

        # A. is trivial to implement assuming each node stores its associated cost with it; B is harder:
        # The idea is every time we update the cost of a node we need to recursively update the costs of all of its
        # children. We can do this by finding the difference between the new and old cost of the parent and subtracting
        # that off of the costs of all of the children recursively.
        # To do this, we need nodes to store their children in addition to their parents. Upon creation, each node must
        # initialize itself with an empty list of its children, and when added to the tree must append itself to the
        # child list of its new parent.
        # When a cost is updated, children's costs are updated with the delta recursively
        # When a node is rewired (i.e. adoped because it now has a different parent), it must be removed from the child
        #   list of the old parent.

        # Ideally, after every change to the costs of nodes in the graph, we would evaluate all nearby nodes for
        # potential adoption by the newly lower cost node. This would bring the algorithm more in line with actually
        # correct algorithms like A*. However this is not required by the RRT* algorithm given in the notes, so I won't
        # bother unless I'm feeling bored.

        # PLAN (RRT)
        # 1. Pick a random node in the allowable region. 10% of the time pick the goal node and see if it works.
        # 2. Find the closest node in terms of dubins path length of nodes already in the list
        # 3. Add that node to the list by propagating it from the closest one.
        # 4. Deal with obstacles and space limits later.

        # Set Goal Probabilty = 0.1
        # This determines what fraction of the time we simply try reaching the goal directly instead of a new rando
        goal_prob = 0.1
        # Cost Factor determines how much of the cost we subtract out when determining which node to add our node to
        # cost_factor = 1.0 is RRT style, where we attach the new node to the node in the tree that has the smallest
        #       dubins path length distance to the new node
        # cost_factor = 0.0 is RRT* style, where we attach the new node to the node in the tree that has the smallest
        #       total dubins path length from the start to the new node. RRT* also involves rewiring the tree.
        # For the example problem with seed 43, 1.0 yields total path cost = 57, while 0.0 yields path cost of 31
        cost_factor = 0.0 # RRT*-style nearest node selection

        nearby_radius = 5 # how far away from a node we should consider for rewiring.

        found_goal = False
        # node_list = rrt_dubins.node_list  # start out with whatever we already have in there
        random_new_node = random_sample(rrt_dubins, rng, goal_prob)
        mincost_new_node_to_add = None
        mincost = 100000000000000000000  # that looks big enough
        for node in rrt_dubins.node_list:
            cost = rrt_dubins.calc_new_cost(node,
                                            random_new_node) - cost_factor * node.cost  # note it's halfway to rrt* if we don't subtract off from_node.cost

            if cost < mincost:
                # potential new best node to branch from
                # check collision before accepting.
                added_new_node = rrt_dubins.propogate(node, random_new_node)
                if(added_new_node):
                    # assert(added_new_node.cost == cost)
                    # assert (added_new_node.cost > node.cost)

                    if rrt_dubins.check_collision(added_new_node):
                        # This node is good. If we don't find a better one, we can add it.
                        mincost = cost
                        mincost_new_node_to_add = added_new_node  # includes parent information
                        mincost_new_node_to_add.children = []
        if mincost_new_node_to_add:
            rrt_dubins.node_list.append(mincost_new_node_to_add)
            # assert(mincost_new_node_to_add.parent.cost < mincost_new_node_to_add.cost)
            mincost_new_node_to_add.parent.children.append(mincost_new_node_to_add)
            rewire_nearby(rrt_dubins, mincost_new_node_to_add, nearby_radius)
            if mincost_new_node_to_add.is_state_identical(rrt_dubins.goal):
                # we found a path to goal
                found_goal = True
                node = mincost_new_node_to_add
                lop = [node]
                while node.parent:
                    node = node.parent
                    lop.insert(0, node)
                # rrt_dubins.node_list = lop
        # rrt_dubins.node_list = node_list
        # else the node we picked this time apparently can't be reached by a dubins path from any node currently in the
        # Tree (node list). Oh well, I hope we get something better next time.


        # Generate a random vehicle state (x, y, z)

        # Add any addtional code you require for RRT*.
        
        # # Find an existing node nearest to the random vehicle state
        # new_node = rrt_dubins.propogate(rrt_dubins.Node(0,0,0), rrt_dubins.Node(1,1,0)) #example of usage
        #
        # # Check if the path between nearest node and random state has obstacle collision
        # # Add the node to nodes_list if it is valid
        # if rrt_dubins.check_collision(new_node):
        #     rrt_dubins.node_list.append(new_node) # Storing all valid nodes

        # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_dubins.draw_graph()

        # Check if new_node is close to goal
        if found_goal:
            # print("Iters:", i, ", number of nodes:", len(rrt_dubins.node_list))
            # print("Cost:", lop[-1].cost)
            pass
            # break

    if i == rrt_dubins.max_iter:
        print('reached max iterations')

    # Return path, which is a list of nodes leading to the goal...
    return lop
