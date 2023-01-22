import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import random
import time
import forest_lib
import tree_lib
import RRTtest
import constants

ic = np.array([0, 1, 0, 1, np.pi / 2])

def plot_forest(forest, path):
    old_tree = forest.tree_list[1]
    new_tree = forest.tree_list[0]
    old_tree_coords = np.array([np.array([x[0], x[1]]) for x in old_tree.coord_list])
    plt.scatter(old_tree_coords[:, 0], old_tree_coords[:, 1], label='old_tree', marker=".", c="purple", alpha=0.5)
    plt.plot(path[:, 0], path[:, 1], label='path', marker='o', color='blue', linewidth=2)
    for obstacle in constants.obstacle_line:
        obstacle_x = np.array([obstacle[0], obstacle[2]])
        obstacle_y = np.array([obstacle[1], obstacle[3]])
        plt.plot(obstacle_x, obstacle_y, label='obstacle', color='red', linewidth=5)
    new_tree_coords = np.array([np.array([x[0], x[1]]) for x in new_tree.coord_list])
    plt.scatter(new_tree_coords[:, 0], new_tree_coords[:, 1], label='new_tree', marker=".", c="yellow", alpha=1)

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('RRF path')
    plt.legend()
    plt.show()

def RRF(robot_state):
    tree_list = []
    path_list = []
    time_array = []
    initial_node = tree_lib.Node(x=robot_state[0:2], v=robot_state[2:4], theta=robot_state[4])
    start_time = time.time()
    initial_tree, path = RRTtest.RRT(initial_node, constants.recharge_point)
    end_time = time.time()
    print("initial tree made")
    time_array.append((end_time - start_time))
    tree_nodes = initial_tree.coord_list.copy()
    tree_list.append(tree_nodes)

    forest = forest_lib.Forest(initial_tree)
    while robot_state[1] < 1.03:
        fig, ax = plt.subplots()
        start_time = time.time()
        initial_node = tree_lib.Node(x=robot_state[0:2], v=robot_state[2:4], theta=robot_state[4])
        x_data = initial_node.x[0]
        y_data = initial_node.x[1]
        scat = ax.scatter(x_data, y_data)
        plt.xlim(0, constants.dimension_field[0])
        plt.ylim(0, constants.dimension_field[1])
        plt.show(block=False)
        new_tree = tree_lib.Tree(root_node=initial_node)
        forest.update_forest(new_tree)
        doRRT = True
        while doRRT:
            rand_node = tree_lib.random_config(new_tree, constants.recharge_point)
            nearest_node, nearest_node_distance = new_tree.find_nearest(rand_node)
            rand_node = tree_lib.new_config(rand_node, nearest_node, nearest_node_distance)
            tree_neighbourhood = new_tree.get_nodes_in_region(rand_node)
            parent_node = new_tree.choose_parent(rand_node, tree_neighbourhood)
            if parent_node is None or parent_node == "unsure":
                continue
            if tree_lib.is_obstacle_free(parent_node, rand_node):
                new_tree.insert_node(parent_node, rand_node)
                new_tree.rewire(tree_neighbourhood, rand_node)
                p = random()
                if p <= constants.scan_forest_prob:
                    forest_neighbourhood = forest.get_forest_neighbourhood(rand_node)
                    forest.check_tree_connection(rand_node, forest_neighbourhood)
                if forest.checkgoal(constants.recharge_point):
                    doRRT = False
                
                x_data = [coord[0] for coord in new_tree.coord_list]
                y_data = [coord[1] for coord in new_tree.coord_list]
                scat.set_offsets(np.c_[x_data, y_data])
                fig.canvas.draw()
                plt.pause(0.1)

        recharge_point_node = new_tree.get_node_with_coord(constants.recharge_point)
        path = new_tree.get_path(recharge_point_node)
        end_time = time.time()
        time_array.append((end_time - start_time))
        plot_forest(forest, path)
        path_list.append(path.copy())
        tree_list.append(new_tree.coord_list.copy())
        robot_state += constants.robot_velocity
    return tree_list, path_list, time_array

tree_list, path_list, time_array = RRF(ic)

plt.title("trees generated")
plt.xlabel("x")
plt.ylabel("y")
i = 0
for coord_list in tree_list:
    coords = np.array([np.array([x[0], x[1]]) for x in coord_list])
    plt.scatter(coords[:, 0], coords[:, 1], label=f'{i}th tree', marker=".")
    i += 1

plt.legend()
plt.show()

plt.title("paths generated")
plt.xlabel("x")
plt.ylabel("y")
i = 0

for path in path_list:
    plt.plot(path[:, 0], path[:, 1], label=f'{i}th path', marker='o', linewidth=2)   
    i += 1 

plt.legend()
plt.show()

# iterations = np.arange(0, 10, 1)
# time_array_0 = []
# time_array_1 = []
# time_array_2 = []
# time_array_3 = []

# for _1 in iterations:
#     ic_timetest = np.array([0, 1, 0, 1, np.pi / 2])
#     print(_1)
#     _2, _3, time_array = RRF(ic_timetest)
#     print(time_array)
#     time_array_0.append(time_array[0])
#     time_array_1.append(time_array[1])
#     time_array_2.append(time_array[2])
#     time_array_3.append(time_array[3])

# plt.title("time analysis")
# plt.xlabel("test case")
# plt.ylabel("time")
# plt.plot(iterations, time_array_0, marker = "o", label = "tree 0")
# plt.plot(iterations, time_array_1, marker = "o", label = "tree 1")
# plt.plot(iterations, time_array_2, marker = "o", label = "tree 2")
# plt.plot(iterations, time_array_3, marker = "o", label = "tree 3")
# plt.legend()
# plt.show()



            
