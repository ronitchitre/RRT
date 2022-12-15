import numpy as np
pi = np.pi

car_length = 0.01
car_velocity = 0.01
max_ster_angle = 40 * pi / 180
max_turn_rate = np.tan(max_ster_angle) * car_velocity / car_length

N_for_rk4 = 5
step_size = 0.05
neighbour_radius = 0.07

dimension_field = np.array([5, 5])
recharge_point = np.array([5, 5])
obstacle_line = [np.array([0.5, 0, 0.5, 0.5])]