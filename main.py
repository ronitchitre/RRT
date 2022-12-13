import numpy as np
import tree_lib
import constants

ic = np.array([0, 0, 1 / 2 ** 0.5, 1 / 2 ** 0.5, np.pi / 4])
initial_node = tree_lib.Node(x=ic[0:2], v=ic[2:4], theta=ic[4])
goal_point = np.array([5, 5])

def RRT(initial_node, final_point):
    tree = tree_lib.Tree(initial_node)
    doRRT = True
    while doRRT:
        rand_node = tree_lib.random_config(tree)
        nearest_node, nearest_node_distance = tree.find_nearest(rand_node)
        rand_node = tree_lib.new_config(rand_node, nearest_node, nearest_node_distance)
        neighbourhood = tree.get_nodes_in_region(rand_node)
        parent_node = tree.choose_parent(rand_node, neighbourhood)
        if parent_node is None:
            continue
        if tree_lib.is_obstacle_free(parent_node, rand_node):
            tree.insert_node(parent_node, rand_node)
            tree.rewire(neighbourhood, rand_node)
            if np.linalg.norm(rand_node.x - final_point) < 0.01:
                doRRT = False
    return tree

tree = RRT(initial_node, goal_point)