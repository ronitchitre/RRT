import numpy as np
pi = np.pi

car_length = 1
car_velocity = 0.1
max_ster_angle = 40 * pi / 180
max_turn_rate = np.tan(max_ster_angle) * car_velocity / car_length
robot_velocity = car_velocity * np.array([0, 1, 0, 0, 0, 0])
distance_to_power = 0.5
safety_factor = 80
robot_power_onion = -30
initial_power = 20

N_for_rk4 = 5
step_size = 0.05
neighbour_radius = 0.08
k_max = 2500

forest_neighbour = 0.08
forest_trees = 2
scan_forest_prob = 0.7

dimension_field = np.array([6, 6])
obstacle_line = []
# obstacle_line = [np.array([2, 0, 2, 2]), np.array([2, 2, 3, 2])]
# recharge_points = [np.array([3, 1])]
recharge_points = [np.array([1, 5]), np.array([5, 1])]

goal_prob = 0.2
neigh_prob = 0.0


def distance(node1, node2):
    # return abs(node1.x[0] - node2.x[0]) + abs(node1.x[1] - node2.x[1])
    return np.linalg.norm(node1.x -node2.x)