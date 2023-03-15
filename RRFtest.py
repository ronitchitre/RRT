import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from random import random
import time
import forest_lib
import tree_lib
import RRTtest
import constants

ic = np.array([0, 0, 0, 1, np.pi / 2])

def plot_forest(forest, path = None):
    new_tree = forest.tree_list[0]
    if len(forest.tree_list) > 1:
        old_tree = forest.tree_list[1]
        old_tree_coords = np.array([np.array([x[0], x[1]]) for x in old_tree.coord_list])
        plt.scatter(old_tree_coords[:, 0], old_tree_coords[:, 1], label='old_tree', marker=".", c="purple", alpha=0.5)
    if path is not None:
        path_coord_x = [node.x[0] for node in path if node.part_of_path]
        path_coord_y = [node.x[1] for node in path if node.part_of_path]
        plt.plot(path_coord_x, path_coord_y, label='path', marker='o', color='blue', linewidth=2)
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

def RRF(robot_state, end_y, type="path"):
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
    while robot_state[1] < end_y:
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
            if type == "path":
                rand_node = tree_lib.random_config(new_tree, [constants.recharge_points[0]], robot_state[0:2], check_goal=True, neighbourhood=path)
            if type == "tree":
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
                new_tree.rewire(tree_neighbourhood, rand_node)
                p = random()
                if p <= constants.scan_forest_prob:
                    forest_neighbourhood = forest.get_path_neighbourhood(rand_node)
                    forest.check_tree_connection(rand_node, forest_neighbourhood, type)
                if forest.checkgoal(constants.recharge_points[0]):
                    doRRT = False
                if k >= constants.k_max:
                    doRRT = False
                    print(f"failed to find path in less than {constants.k_max} iterations")
            new_tree.added_nodes = []
                
                # x_data = [coord[0] for coord in new_tree.coord_list]
                # y_data = [coord[1] for coord in new_tree.coord_list]
                # scat.set_offsets(np.c_[x_data, y_data])
                # fig.canvas.draw()
                # plt.pause(0.1)

        end_time = time.time()
        time_array.append((end_time - start_time))


        # if k < constants.k_max:
        #     plot_forest(forest, new_tree.path)
        # else:
        #     plot_forest(forest)


        if path is not None:
            path_list.append(path.copy())
        tree_list.append(new_tree.coord_list.copy())
        robot_state += constants.robot_velocity
    return tree_list, path_list, time_array

# tree_list, path_list, time_array = RRF(ic)

# plt.title("trees generated")
# plt.xlabel("x")
# plt.ylabel("y")
# i = 0
# for coord_list in tree_list:
#     coords = np.array([np.array([x[0], x[1]]) for x in coord_list])
#     plt.scatter(coords[:, 0], coords[:, 1], label=f'{i}th tree', marker=".")
#     i += 1

# plt.legend()
# plt.show()

# plt.title("paths generated")
# plt.xlabel("x")
# plt.ylabel("y")
# i = 0

# for path in path_list:
#     if path is not None:
#             path_coord_x = [node.x[0] for node in path]
#             path_coord_y = [node.x[1] for node in path]
#             plt.plot(path_coord_x, path_coord_y, label=f'{i}th path', marker='o', linewidth=2)   
#     i += 1 

# plt.legend()
# plt.show()

def RRFsim(robot_state, forest=None):
    initial_node = tree_lib.Node(x=robot_state[0:2], v=robot_state[2:4], theta=robot_state[4])
    k = 0
    if len(forest.tree_list) == 0:
        tree, path = RRTtest.RRTsim(initial_node, constants.recharge_points, constants.initial_power)
        forest.update_forest(tree)
        forest.goal = path[0].x
        return forest, path
    new_tree = tree_lib.Tree(root_node=initial_node)
    forest.update_forest(new_tree)
    old_tree = forest.tree_list[1]
    doRRT = True
    while doRRT:
        rand_node = tree_lib.random_config(new_tree, [forest.goal], robot_state, check_goal=True, neighbourhood=old_tree.path)
        nearest_node, nearest_node_distance = new_tree.find_nearest(rand_node)
        rand_node = tree_lib.new_config(rand_node, nearest_node, nearest_node_distance)
        tree_neighbourhood = new_tree.get_nodes_in_region(rand_node)
        parent_node = new_tree.choose_parent(rand_node, tree_neighbourhood)
        if parent_node is None or parent_node == "unsure":
            continue
        k += 1
        if tree_lib.is_obstacle_free(parent_node, rand_node):
            new_tree.insert_node(parent_node, rand_node)
            new_tree.rewire(tree_neighbourhood, rand_node)
            p = random()
            if p <= constants.scan_forest_prob:
                forest_neighbourhood = forest.get_path_neighbourhood(rand_node)
                forest.check_tree_connection(rand_node, forest_neighbourhood)
            if forest.checkgoal(forest.goal):
                doRRT = False
            elif k >= constants.k_max:
                doRRT = False
                print(f"failed to find path in less than {constants.k_max} iterations")
        new_tree.added_nodes = []
    if k < constants.k_max:
        return new_tree.path
    else:
        return None


if __name__ == "__main__":
    n_test = 10
    end_y = 0.8
    avg_time_path = np.zeros(10)
    avg_time_tree = np.zeros(10)
    steps = int(end_y / constants.robot_velocity[1]) + 1

    for i in range(n_test):
        ic_timetest = np.array([0, 0, 0, 1, np.pi / 2])
        _1, _2, time_array_iter = RRF(ic_timetest, end_y, type="path")
        avg_time_path += np.array(time_array_iter)
        print("path", time_array_iter)
    avg_time_path = avg_time_path / n_test

    for i in range(n_test):
        ic_timetest = np.array([0, 0, 0, 1, np.pi / 2])
        _1, _2, time_array_iter = RRF(ic_timetest, end_y, type="tree")
        avg_time_tree += np.array(time_array_iter)
        print("tree", time_array_iter)
    avg_time_tree = avg_time_tree / n_test

    y_coords = np.linspace(0, 10, 10)
    plt.plot(y_coords, avg_time_path, "-o", label="path")
    plt.plot(y_coords, avg_time_tree, "-o", label="tree")
    plt.xlabel("postion at which path found")
    plt.ylabel("time")
    plt.title("time averaged over test cases")
    plt.legend()
    plt.show()

    # tree_iters = np.arange(0, steps, 1)
    # for i in range(n_test):
    #     plt.plot(tree_iters, time_array[steps * i : steps * (i + 1)], label=f"{i}th tree", marker="o")
    # plt.xlabel('test cases')
    # plt.ylabel('time')
    # plt.title("time analysis")
    # plt.legend()
    # plt.show()
    



            
