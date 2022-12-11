import numpy as np
import math
import constants

pi = np.pi

def rk4method_for_car(ic_car, times):
    N = len(times)
    h = (times[-1] - times[0]) / N
    w_i = np.concatenate([ic_car.r, ic_car.v])
    for i in range(1, N):
        # t_i = times[0] + h * (i - 1)
        k1 = h * Car(w_i[0:3], w_i[3:6]).dynamics()
        w_i_int = w_i + k1 / 2
        k2 = h * Car(w_i_int[0:3], w_i_int[3:6]).dynamics()
        w_i_int = w_i + k2 / 2
        k3 = h * Car(w_i_int[0:3], w_i_int[3:6]).dynamics()
        w_i_int = w_i + k3
        k4 = h * Car(w_i_int[0:3], w_i_int[3:6]).dynamics()
        w_i = w_i + (k1 + 2 * k2 + 2 * k3 + k4) / 6
    new_car = Car(w_i[0:3], w_i[3:6])
    return new_car

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
    def __init__(self, x, v, theta, ang_vel = 0):
        self.x = x
        self.v = v
        self.theta = theta
        self.ang_vel = ang_vel

    def dynamics(self):
        x_dot = self.v
        v_dot = np.cross(self.ang_vel, self.v)
        theta_dot = self.ang_vel
        return np.array([x_dot, v_dot, theta_dot])

    def propogate(self, des_node):
        des_direction = des_node - self.x
        theta = ang_between(des_direction, self.v)
        dist_between_nodes = np.linalg.norm(des_direction)
        omega = 2 * constants.car_velocity * np.sin(theta) / dist_between_nodes
        if abs(omega) > constants.max_turn_rate:
            return "des_node invalid"
        self.ang_vel = omega
        # distance = 2 * theta * (constants.car_velocity / omega)
        time_taken = 2 * theta / omega
        distance = time_taken * constants.car_velocity
        times = np.linspace(0, time_taken, constants.N_for_rk4)
        new_car = rk4method_for_car(self, times)
        return Car()
        
        






