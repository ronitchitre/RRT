import numpy as np
import math
import constants

pi = np.pi

def rk4method_for_car(ic_car, omega, times):
    N = len(times)
    h = times[1] - times[0]
    w_i = np.concatenate([ic_car.x, ic_car.v])
    for i in range(1, N):
        # t_i = times[0] + h * (i - 1)
        k1 = h * Car(w_i[0:3], w_i[3:6], 0).dynamics(omega)
        w_i_int = w_i + k1 / 2
        k2 = h * Car(w_i_int[0:3], w_i_int[3:6], 0).dynamics(omega)
        w_i_int = w_i + k2 / 2
        k3 = h * Car(w_i_int[0:3], w_i_int[3:6], 0).dynamics(omega)
        w_i_int = w_i + k3
        k4 = h * Car(w_i_int[0:3], w_i_int[3:6], 0).dynamics(omega)
        w_i = w_i + (k1 + 2 * k2 + 2 * k3 + k4) / 6
    return w_i[0:3], w_i[3:6]

def normalize(v):
    if np.linalg.norm(v) != 0:
        v = (v / np.linalg.norm(v))
    return v

def ang_between(a, b):
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    cross = np.cross(a, b)[2]
    return np.arcsin(cross / (norm_a * norm_b))

class Car:
    def __init__(self, x, v, theta):
        self.x = x
        self.v = v
        self.theta = theta

    def dynamics(self, omega):
        x_dot = self.v
        v_dot = np.cross(omega, self.v)
        return np.array([x_dot, v_dot])

    def propogate(self, des_node):
        des_direction = des_node - self.x
        alpha = ang_between(des_direction, self.v)
        dist_between_nodes = np.linalg.norm(des_direction)
        omega = 2 * constants.car_velocity * np.sin(alpha) / dist_between_nodes
        if abs(omega) > constants.max_turn_rate:
            return "des_node invalid"
        # distance = 2 * theta * (constants.car_velocity / omega)
        time_taken = 2 * alpha / omega
        distance = time_taken * constants.car_velocity
        times = np.linspace(0, time_taken, constants.N_for_rk4)
        new_x, new_v = rk4method_for_car(self, omega, times)
        new_theta = self.theta + self.ang_vel * time_taken
        new_car = Car(new_x, new_v, new_theta)
        return new_car, distance
        
        






