import numpy as np
import matplotlib.pyplot as plt

import RRFtest
import RRTtest
import constants

robot_state = np.array([0, 0, 0, 1, np.pi / 2, 20])

def plot_forest(forest, ax, path = None):
    new_tree = forest.tree_list[0]
    if len(forest.tree_list) > 1:
        old_tree = forest.tree_list[1]
        old_tree_coords = np.array([np.array([x[0], x[1]]) for x in old_tree.coord_list])
        ax.scatter(old_tree_coords[:, 0], old_tree_coords[:, 1], label='old_tree', marker=".", c="purple", alpha=0.5)
    if path is not None:
        path_coord_x = [node.x[0] for node in path]
        path_coord_y = [node.x[1] for node in path]
        ax.plot(path_coord_x, path_coord_y, label='path', marker='o', color='blue', linewidth=2)
    for obstacle in constants.obstacle_line:
        obstacle_x = np.array([obstacle[0], obstacle[2]])
        obstacle_y = np.array([obstacle[1], obstacle[3]])
        ax.plot(obstacle_x, obstacle_y, label='obstacle', color='red', linewidth=5)
    new_tree_coords = np.array([np.array([x[0], x[1]]) for x in new_tree.coord_list])
    ax.scatter(new_tree_coords[:, 0], new_tree_coords[:, 1], label='new_tree', marker=".", c="yellow", alpha=1)

    # ax.xlabel('x')
    # ax.ylabel('y')
    # ax.title('RRF path')
    # ax.legend()

def decide_if_recharge(robot_state, cost):
    power_required = cost * constants.distance_to_power
    if (robot_state[5] - power_required) < constants.safety_factor:
        return True
    return False


forest = RRFtest.forest_lib.Forest()
fig, ax = plt.subplots()
plt.show(block=False)
k = 0

while robot_state[1] < 6:
    RRFtest.RRFsim(robot_state, forest)
    path = forest.tree_list[0].path
    if path is None:
        continue
    if decide_if_recharge(robot_state, path[0].cost):
        print("battery ran out", path[0].cost)
        cur_node = RRFtest.tree_lib.Node(robot_state[0:2], robot_state[2:4], robot_state[5])
        final_node = constants.recharge_point
        tree, path = RRTtest.RRT(cur_node, final_node)
        forest = RRFtest.forest_lib.Forest(tree)
        robot_state[5] = 15
    ax.clear()
    plot_forest(forest, ax, path)
    plt.draw()
    plt.pause(0.1)
    robot_state[1] += constants.car_velocity
    robot_state[5] += constants.robot_power_onion

