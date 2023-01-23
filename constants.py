import numpy as np
pi = np.pi

car_length = 0.01
car_velocity = 0.01
max_ster_angle = 40 * pi / 180
max_turn_rate = np.tan(max_ster_angle) * car_velocity / car_length
robot_velocity = car_velocity * np.array([0, 1, 0, 0, 0])

N_for_rk4 = 5
step_size = 0.1
neighbour_radius = 0.15
k_max = 1000

forest_neighbour = 0.2
forest_trees = 2
scan_forest_prob = 0.5

dimension_field = np.array([4, 4])
obstacle_line = [np.array([1, 2, 1, 4])]
recharge_point = np.array([3, 3])

goal_prob = 0.3
neigh_prob = 0.2


def distance(node1, node2):
    # return abs(node1.x[0] - node2.x[0]) + abs(node1.x[1] - node2.x[1])
    return np.linalg.norm(node1.x -node2.x)