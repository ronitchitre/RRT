import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import random
import time
import forest_lib
import tree_lib
import RRTtest
import RRFtest
import constants

ic = np.array([0, 0, 0, 1, np.pi / 2])

def RRF(robot_state):
    tree_list = []
    path_list = []
    time_array = []
    initial_node = tree_lib.Node(x=robot_state[0:2], v=robot_state[2:4], theta=robot_state[4])
    start_time = time.time()
    initial_tree, path = RRTtest.RRT(initial_node, constants.recharge_points[0])
    end_time = time.time()
    print("initial tree made")
    time_array.append((end_time - start_time))
    tree_nodes = initial_tree.coord_list.copy()
    tree_list.append(tree_nodes)
    forest_neighbourhood = []

    forest = forest_lib.Forest(initial_tree)
    robot_state_ind = 0
    while robot_state_ind < 4:
        k = 0

        # fig, ax = plt.subplots()

        start_time = time.time()
        initial_node = tree_lib.Node(x=robot_state[0:2], v=robot_state[2:4], theta=robot_state[4])

        # x_data = initial_node.x[0]
        # y_data = initial_node.x[1]
        # scat = ax.scatter(x_data, y_data)
        # plt.xlim(0, constants.dimension_field[0])
        # plt.ylim(0, constants.dimension_field[1])
        # plt.show(block=False)

        new_tree = tree_lib.Tree(root_node=initial_node)
        forest.update_forest(new_tree)
        doRRT = True
        while doRRT:
            rand_node = tree_lib.random_config(new_tree, [constants.recharge_points[0]], robot_state[0:2], check_goal=True)
            nearest_node, nearest_node_distance = new_tree.find_nearest(rand_node)
            rand_node = tree_lib.new_config(rand_node, nearest_node, nearest_node_distance)
            tree_neighbourhood = new_tree.get_nodes_in_region(rand_node)
            parent_node = new_tree.choose_parent(rand_node, tree_neighbourhood)
            if parent_node is None or parent_node == "unsure":
                continue
            if tree_lib.is_obstacle_free(parent_node, rand_node):
                k += 1
                new_tree.insert_node(parent_node, rand_node)
                # new_tree.rewire(tree_neighbourhood, rand_node)
                p = random()
                if p <= constants.scan_forest_prob:
                    forest_neighbourhood = forest.get_path_neighbourhood(rand_node)
                    forest.check_tree_connection(rand_node, forest_neighbourhood, type)
                if forest.checkgoal(constants.recharge_points[0]):
                    doRRT = False
                if k >= constants.k_max:
                    doRRT = False
                    print(f"failed to find path in less than {constants.k_max} iterations")
                
                # x_data = [coord[0] for coord in new_tree.coord_list]
                # y_data = [coord[1] for coord in new_tree.coord_list]
                # scat.set_offsets(np.c_[x_data, y_data])
                # fig.canvas.draw()
                # plt.pause(0.1)

        if k < constants.k_max:
            recharge_point_node = new_tree.get_node_with_coord(constants.recharge_points[0])
            path = new_tree.get_path(recharge_point_node)
        end_time = time.time()
        time_array.append((end_time - start_time))

        if k < constants.k_max:
            plot_forest(forest, path)
        else:
            plot_forest(forest)
        if path is not None:
            path_list.append(path.copy())
        tree_list.append(new_tree.coord_list.copy())
        robot_state += constants.robot_velocity
        robot_state_ind += 1
    return tree_list, path_list, time_array
