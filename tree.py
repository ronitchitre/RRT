import numpy as np
from random import uniform
import car
import constants

class Node(car.Car):
    def __init__(self, x, v = np.array([0, 0]), theta = 0, parent=None, cost=0):
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


class Tree():
    def __init__(self, root_node):
        self.root_node = root_node
        self.node_list = [root_node]

    def find_nearest(self, q_rand):
        nearest_node = self.root_node
        nearest_node_dist = np.linalg.norm(nearest_node.x - q_rand.x)
        for node in self.node_list[1:]:
            possible_dist = np.linalg.norm(node.x - q_rand.x)
            if possible_dist < nearest_node_dist:
                nearest_node = node
                nearest_node_dist = possible_dist
        return nearest_node, nearest_node_dist
    
    def get_nodes_in_region(self, center_node):
        neighbour_nodes = []
        for node in self.node_list:
            if np.linalg.norm(center_node.x - node.x) <= constants.neighbour_radius:
                neighbour_nodes.append(node)
        return neighbour_nodes
    
    def choose_parent(self, rand_node, neighbourhood):
        min_cost = float('inf')
        parent_node = "unsure"
        for node in neighbourhood:
            try:
                new_car, distance = node.propogate(rand_node)
            except:
                continue
            new_cost = node.cost + distance
            if new_cost < min_cost:
                min_cost = new_cost
                parent_node = node
        rand_node = Node(new_car.x, new_car.v, new_car.theta, parent=None, cost=min_cost)
        return parent_node, rand_node
    
    def insert_node(self, parent_node, child_node):
        parent_node.child_list.append(child_node)
        child_node.parent = parent_node
        self.node_list.append(child_node)
    
    def remove_connection(self, parent_node, child_node):
        parent_node.child_list.remove(child_node)
        child_node.parent = None 
        self.node_list.remove(child_node)

    def rewire(self, neighbourhood, child_node):
        for node in neighbourhood:
            current_cost = node.cost
            try:
                new_car, distance = child_node.propogate(node)
            except:
                continue
            new_cost = child_node.cost + distance
            if new_cost > current_cost:
                continue
            old_parent = node.parent
            self.remove_connection(old_parent, node)
            self.insert_node(child_node, node)





def random_config():
    x_coord = uniform(0, constants.dimension_field[0])
    y_coord = uniform(0, constants.dimension_field[1])
    return Node(np.array([x_coord, y_coord]))
    
def new_config(rand_node, nearest_node, nearest_node_dist):
    if nearest_node_dist <= constants.step_size:
        return rand_node
    coord_to_comp = (rand_node.x[0] - nearest_node.x[0]) + (rand_node.x[1] - nearest_node.x[1])*1j
    phase = np.angle(coord_to_comp)
    new_node_comp = constants.step_size * np.exp(1j * phase)
    rand_node.x = new_node_comp.real
    rand_node.y = new_node_comp.imag
    return rand_node


