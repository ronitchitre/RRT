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
        k1 = h * Car(w_i[0:2], w_i[2:4], 0).dynamics(omega)
        w_i_int = w_i + k1 / 2
        k2 = h * Car(w_i_int[0:2], w_i_int[2:4], 0).dynamics(omega)
        w_i_int = w_i + k2 / 2
        k3 = h * Car(w_i_int[0:2], w_i_int[2:4], 0).dynamics(omega)
        w_i_int = w_i + k3
        k4 = h * Car(w_i_int[0:2], w_i_int[2:4], 0).dynamics(omega)
        w_i = w_i + (k1 + 2 * k2 + 2 * k3 + k4) / 6
    return w_i[0:2], w_i[2:4]

def normalize(v):
    if np.linalg.norm(v) != 0:
        v = (v / np.linalg.norm(v))
    return v

def ang_between(a, b):
    a_complex = a[0] + a[1] * 1j
    b_complex = b[0] + b[1] * 1j
    return np.angle(b_complex * np.conj(a_complex))


def rotate(v, theta, length):
    v_to_complex = v[0] + v[1]*1j
    rotation = np.exp(1j * theta)
    new_v = length * v_to_complex * rotation
    return np.array([new_v.real, new_v.imag]) 

class Car:
    def __init__(self, x, v, theta):
        self.x = x
        self.v = normalize(v) * constants.car_velocity
        self.theta = theta

    def dynamics(self, omega):
        x_dot = self.v
        v_dot = np.cross(omega, self.v)
        return np.array([x_dot, v_dot])

    def propogate(self, des_node):
        des_direction = des_node.x - self.x
        alpha = ang_between(self.v, des_direction)
        dist_between_nodes = np.linalg.norm(des_direction)
        if math.isclose(dist_between_nodes, 0.0):
            return "invalid"
        omega = 2 * constants.car_velocity * np.sin(alpha) / dist_between_nodes
        new_x = des_node.x
        if abs(omega) > constants.max_turn_rate or abs(alpha) > np.pi / 2:
            if abs(alpha) > np.pi / 2:
                return "invalid"
            omega = np.sign(omega) * constants.max_turn_rate
            displacement = dist_between_nodes * np.array([np.cos(alpha), np.sin(alpha)])
            new_x = self.x + displacement
        # distance = 2 * theta * (constants.car_velocity / omega)
        if omega != 0:
            time_taken = 2 * alpha / omega
        else:
            time_taken = dist_between_nodes / constants.car_velocity
        distance = time_taken * constants.car_velocity
        new_v = rotate(self.v, 2 * alpha, constants.car_velocity)
        new_theta = self.theta + omega * time_taken
        new_car = Car(new_x, new_v, new_theta)
        return new_car, distance
        
        






