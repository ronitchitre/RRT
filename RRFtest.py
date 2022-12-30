import numpy as np
import matplotlib.pyplot as plt
from random import random
import forest_lib
import tree_lib
import RRTtest
import constants

ic = np.array([1, 1, 0, 1, np.pi / 2])
initial_node = tree_lib.Node(x=ic[0:2], v=ic[2:4], theta=ic[4])
recharge_point = np.array([0, 0])

initial_tree, path = RRTtest.RRT(initial_node, recharge_point)

forest = forest_lib.Forest(initial_tree)

robot_state = ic

while robot_state[1] < 1.03:
    initial_node = tree_lib.Node(x=robot_state[0:2], v=robot_state[2:4], theta=robot_state[4])
    new_tree = tree_lib.Tree(root_node=initial_node)
    forest.update_forest(new_tree)
    doRRT = True
    while doRRT:
        rand_node = tree_lib.random_config(new_tree, recharge_point)
        nearest_node, nearest_node_distance = new_tree.find_nearest(rand_node)
        rand_node = tree_lib.new_config(rand_node, nearest_node, nearest_node_distance)
        tree_neighbourhood = new_tree.get_nodes_in_region(rand_node)
        parent_node = new_tree.choose_parent(rand_node, tree_neighbourhood)
        if parent_node is None:
            continue
        if tree_lib.is_obstacle_free(parent_node, rand_node):
            new_tree.insert_node(parent_node, rand_node)
            new_tree.rewire(tree_neighbourhood, rand_node)
            p = random()
            if p <= constants.scan_forest_prob:
                forest_neighbourhood = forest.get_forest_neighbourhood(rand_node)
                forest.check_tree_connection(rand_node, forest_neighbourhood)
            if forest.checkgoal(recharge_point):
                doRRT = False
    # for node in new_tree.node_list:
    #     if node.id != 0:
    #         print(f"wrong {node.id}")
    print(len(new_tree.node_list))
    robot_state += constants.robot_velocity


            
