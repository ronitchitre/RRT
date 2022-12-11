import numpy as np
import car

class Node(car.Car):
    def __init__(self, r, v, ster_ang, child):
        super().__init__(r, v, ster_ang)
        self.child = child
