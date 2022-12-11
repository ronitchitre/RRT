import numpy as np
import car

class Node(car.Car):
    def __init__(self, x, v, theta, parent=None, cost=0):
        super().__init__(x, v, theta)
        self.parent = parent
        self.cost = cost
        self.child_list = []
    
    def add_child(self, child_node):
        try:
            new_car, distance = self.propogate(child_node)
        except:
            return "child node can not be attached"
        cost_child = self.cost + distance
        child_node = Node(new_car.x, new_car.v, new_car.theta, parent=self, cost=cost_child) #might replace new_car.x with child_node.x
        self.child_list.append(child_node)
        return child_node 

    
