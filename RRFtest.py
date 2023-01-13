import numpy as np
import matplotlib.pyplot as plt
from random import random
import forest_lib
import tree_lib
import RRTtest
import constants

ic = np.array([0, 1, 0, 1, np.pi / 2])
initial_node = tree_lib.Node(x=ic[0:2], v=ic[2:4], theta=ic[4])
recharge_point = np.array([0, 0])
tree_list = []
path_list = []

initial_tree, path = RRTtest.RRT(initial_node, recharge_point)
tree_nodes = initial_tree.coord_list.copy()
tree_list.append(tree_nodes)

forest = forest_lib.Forest(initial_tree)

robot_state = ic

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
        if parent_node is None or parent_node == "unsure":
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

    recharge_point_node = new_tree.get_node_with_coord(recharge_point)
    path = new_tree.get_path(recharge_point_node)
    plot_forest(forest, path)
    path_list.append(path.copy())
    tree_list.append(new_tree.coord_list.copy())
    robot_state += constants.robot_velocity

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




            
