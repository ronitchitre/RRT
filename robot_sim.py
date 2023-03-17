import numpy as np
import matplotlib.pyplot as plt

import RRFtest
import RRTtest
import constants

ic_power = 600
robot_state = np.array([3, 0, 0, 1, np.pi / 2, ic_power])
trajectory = [np.array([3, 0, 3, 3]), np.array([3, 3, 0, 3])]

def plot_forest(forest, path = None):
    new_tree = forest.tree_list[0]
    if len(forest.tree_list) > 1:
        old_trees = forest.tree_list[1:]
        old_tree_coords = []
        for tree in old_trees:
           old_tree_coords.append(np.array([np.array([x[0], x[1]]) for x in tree.coord_list]))
        j = 0
        for tree_coords in old_tree_coords:
            plt.scatter(tree_coords[:, 0], tree_coords[:, 1], label=f'{j}th old_tree', marker=".", alpha=0.5)
            j += 1
    if path is not None:
        path_coord_x = [node.x[0] for node in path if node.part_of_path]
        path_coord_y = [node.x[1] for node in path if node.part_of_path]
        plt.plot(path_coord_x, path_coord_y, label='path', marker='o', color='blue', linewidth=2)
    for obstacle in constants.obstacle_line:
        obstacle_x = np.array([obstacle[0], obstacle[2]])
        obstacle_y = np.array([obstacle[1], obstacle[3]])

    for line in trajectory:
        line_x = np.array([line[0], line[2]])
        line_y = np.array([line[1], line[3]])
        plt.plot(line_x, line_y, label='robot trajectory', color='pink', linewidth=3, alpha = 0.7)
    new_tree_coords = np.array([np.array([x[0], x[1]]) for x in new_tree.coord_list])
    plt.scatter(new_tree_coords[:, 0], new_tree_coords[:, 1], label='new_tree', marker=".", c="yellow", alpha=1)

    plt.xlabel('x')
    plt.ylabel('y')
    plt.ylim(0, constants.dimension_field[1])
    plt.xlim(0, constants.dimension_field[0])
    plt.title('RRF path')
    plt.legend()
    plt.show()

def decide_if_recharge(robot_state, cost):
    power_required = cost * constants.distance_to_power
    if (robot_state[5] - power_required) < constants.safety_factor:
        return True
    return False


forest = RRFtest.forest_lib.Forest()

while True:
    RRFtest.RRFsim(robot_state, forest)
    path = forest.tree_list[0].path
    if path is None:
        cur_node = RRFtest.tree_lib.Node(robot_state[0:2], robot_state[2:4], robot_state[5])
        tree, path = RRTtest.RRTsim(cur_node, constants.recharge_points, robot_state[5])
        forest = RRFtest.forest_lib.Forest(tree)
        forest.goal = path[0].x
    if decide_if_recharge(robot_state, path[0].cost):
        new_power = ic_power - (1 * path[0].cost * constants.distance_to_power)
        print(f"recharge now {new_power}")
        cur_node = RRFtest.tree_lib.Node(robot_state[0:2], robot_state[2:4], new_power)
        tree, path = RRTtest.RRTsim(cur_node, constants.recharge_points, path[0].cost)
        forest = RRFtest.forest_lib.Forest(tree)
        forest.goal = path[0].x
        robot_state[5] = new_power
    plot_forest(forest, path)
    plt.draw()
    robot_state += constants.robot_velocity
    robot_state[5] += constants.robot_power_onion
    if robot_state[1] >= 3:
        constants.robot_velocity = constants.car_velocity * np.array([-1, 0, 0, 0, 0, 0])
    print(robot_state[5], path[0].cost)


