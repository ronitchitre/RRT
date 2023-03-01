import numpy as np
import tree_lib
import constants
import matplotlib.pyplot as plt

def RRT(initial_node, final_point, testing = False):
    tree = tree_lib.Tree(initial_node)
    doRRT = True
    test_nodes = []
    test_counter = 0
    k = 0
    if testing:
        with open("test_cases/bug.txt", "r") as file:
            for line in file:
                space = line.find(" ")
                x_coord = float(line[0:space])
                y_coord = float(line[(space + 1):])
                test_nodes.append(tree_lib.Node(x = np.array([x_coord, y_coord])))
    while doRRT:
        if not testing:
            rand_node = tree_lib.random_config(tree, [final_point], robot_state=initial_node.x)
        else:
            rand_node = test_nodes[test_counter]
            test_counter += 1
            if test_counter == len(test_nodes):
                testing = False
        nearest_node, nearest_node_distance = tree.find_nearest(rand_node)
        rand_node = tree_lib.new_config(rand_node, nearest_node, nearest_node_distance)
        neighbourhood = tree.get_nodes_in_region(rand_node)
        parent_node = tree.choose_parent(rand_node, neighbourhood)
        if parent_node is None:
            continue
        if tree_lib.is_obstacle_free(parent_node, rand_node):
            tree.insert_node(parent_node, rand_node)
            k += 1
            # tree.rewire(neighbourhood, rand_node)
            if np.linalg.norm(rand_node.x - final_point) <= 0.05:
                doRRT = False
                path = tree.get_path(rand_node)
        # if k >= constants.k_max:
        #     doRRT = False
        #     print(f"failed to find path in less than {constants.k_max} nodes")
        #     path = None
    return tree, path

def plot_tree(tree, path):
    if path is not None:
        path_coord_x = [node.x[0] for node in path]
        path_coord_y = [node.x[1] for node in path]
        plt.plot(path_coord_x, path_coord_y, label='path', marker='o', color='blue', linewidth=2)
    for obstacle in constants.obstacle_line:
        obstacle_x = np.array([obstacle[0], obstacle[2]])
        obstacle_y = np.array([obstacle[1], obstacle[3]])
        plt.plot(obstacle_x, obstacle_y, label='obstacle', color='red', linewidth=5)
    tree_coords = np.array([np.array([x[0], x[1]]) for x in tree.coord_list])
    plt.scatter(tree_coords[:, 0], tree_coords[:, 1], label='tree', marker=".", c="yellow", alpha=1)

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('RRT path')
    plt.legend()
    plt.show()


def reached_at_recharge(rand_node, power):
    for recharge_point in constants.recharge_points:
        if np.linalg.norm(recharge_point - rand_node.x) == 0 and rand_node.cost <= power:
            return True
    return False


def RRTsim(initial_node, final_points, power):
    tree = tree_lib.Tree(initial_node)
    doRRT = True
    k = 0
    while doRRT:
        rand_node = tree_lib.random_config(tree, final_points, robot_state=initial_node.x)
        nearest_node, nearest_node_distance = tree.find_nearest(rand_node)
        rand_node = tree_lib.new_config(rand_node, nearest_node, nearest_node_distance)
        neighbourhood = tree.get_nodes_in_region(rand_node)
        parent_node = tree.choose_parent(rand_node, neighbourhood)
        if parent_node is None:
            continue
        if tree_lib.is_obstacle_free(parent_node, rand_node):
            tree.insert_node(parent_node, rand_node)
            k += 1
            tree.rewire(neighbourhood, rand_node)
            if reached_at_recharge(rand_node, power):
                doRRT = False
                path = tree.get_path(rand_node)
        # if k >= constants.k_max:
        #     doRRT = False
        #     print(f"failed to find path in less than {constants.k_max} nodes")
        #     path = None
    return tree, path


if __name__ == "__main__":
    ic = np.array([0, 1, 0, 1, np.pi])
    initial_node = tree_lib.Node(x=ic[0:2], v=ic[2:4], theta=ic[4])
    goal_point = np.array([2, 2])
    tree, path = RRT(initial_node, goal_point, testing=False)
    plot_tree(tree, path)
    # print(tree.get_path(goal_point))


    # improvements
    # change distance function from euclidean to some other norm.
    # in random config and a probability p such that with probability p goal will be picked as node. 
