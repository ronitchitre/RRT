import numpy as np
pi = np.pi

car_length = 0.01
car_velocity = 0.01
max_ster_angle = 40 * pi / 180
max_turn_rate = np.tan(max_ster_angle) * car_velocity / car_length
robot_velocity = car_velocity * np.array([0, 1, 0, 0, 0])

N_for_rk4 = 5
step_size = 0.05
neighbour_radius = 0.08
k_max = 2500

forest_neighbour = 0.08
forest_trees = 2
scan_forest_prob = 0.0

dimension_field = np.array([4, 4])
obstacle_line = [np.array([3, 0, 3, 3])]
recharge_point = np.array([4, 1])

goal_prob = 0.1
neigh_prob = 0.5


def distance(node1, node2):
    # return abs(node1.x[0] - node2.x[0]) + abs(node1.x[1] - node2.x[1])
    return np.linalg.norm(node1.x -node2.x)