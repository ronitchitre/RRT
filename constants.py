import numpy as np
pi = np.pi

car_length = 1
car_velocity = 1
max_ster_angle = 20 * pi / 180
max_turn_rate = np.tan(max_ster_angle) * car_velocity / car_length

dist_bw_node = 0.01
N_for_rk4 = 5
