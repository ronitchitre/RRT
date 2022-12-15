import numpy as np
import tree_lib
import constants
import matplotlib.pyplot as plt

ic = np.array([0, 0, 0, 1, np.pi / 2])
initial_node = tree_lib.Node(x=ic[0:2], v=ic[2:4], theta=ic[4])
goal_point = np.array([5, 5])

def RRT(initial_node, final_point):
    tree = tree_lib.Tree(initial_node)
    doRRT = True
    number_of_optimal = 0
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
            print(rand_node.x, rand_node.cost)
            if np.linalg.norm(rand_node.x - final_point) < 0.5:
                number_of_optimal += 1
                if number_of_optimal >= 3:
                    doRRT = False
                    path = tree.get_path(rand_node)
    return tree, path

def plot_path(path, tree, other_branches = False):
    if other_branches:
        coord_list = np.array([np.array([x[0], x[1]]) for x in tree.coord_list])
        plt.scatter(coord_list[:, 0], coord_list[:, 1], label='tree', marker=".", c="yellow")
    plt.plot(path[:, 0], path[:, 1], label='path', marker='o', color='blue', linewidth=2)
    for obstacle in constants.obstacle_line:
        obstacle_x = np.array([obstacle[0], obstacle[2]])
        obstacle_y = np.array([obstacle[1], obstacle[3]])
        plt.plot(obstacle_x, obstacle_y, label='obstacle', color='red', linewidth=5)

    
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('RRT* path')
    plt.legend()
    plt.show()

tree, path = RRT(initial_node, goal_point)
plot_path(path, tree, other_branches=True)
# print(tree.get_path(goal_point))