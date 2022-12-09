import numpy as np
import math
import constants

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

def normalize_vel(v):
    if np.linalg.norm(v) != 0:
        v = (v / np.linalg.norm(v)) * constants.car_velocity
    return v

class Car:
    def __init__(self, r, v, ster_ang):
        self.r = r
        self.v = normalize_vel(v)
        self.ster_ang = ster_ang
        self.length = constants.car_length

    def dynamics(self):
        r_dot = self.v
        ang_vel = constants.car_velocity * math.tan(self.ster_ang) / constants.car_length
        v_dot = np.cross(ang_vel, self.v)
        return np.array([r_dot, v_dot])

    def propogate(self, t_init, t_final, h):
        times = np.linspace(t_init, t_final, h)
        new_car = rk4method_for_car(self, times)
        self.x = new_car.x
        self.y = new_car.y


