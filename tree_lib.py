import numpy as np
from random import uniform
import car_lib
import constants
import obstacle_lib

class Node(car_lib.Car):
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
    
    def set_prop(self, x, v, theta, cost):
        self.x = x
        self.v = v
        self.theta = theta
        self.cost = cost


class Tree():
    def __init__(self, root_node):
        self.root_node = root_node
        self.node_list = [root_node]
        self.coord_list = [list(root_node.x)]

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
            path_possible = True
            try:
                new_car, distance = node.propogate(rand_node)
            except:
                path_possible = False
                continue
            new_cost = node.cost + distance
            if new_cost < min_cost:
                min_cost = new_cost
                parent_node = node
        if path_possible:
            rand_node.set_prop(new_car.x, new_car.v, new_car.theta, min_cost)
            return parent_node
    
    def insert_node(self, parent_node, child_node):
        parent_node.child_list.append(child_node)
        child_node.parent = parent_node
        self.node_list.append(child_node)
        self.coord_list.append(list(child_node.x))
    
    def remove_connection(self, parent_node, child_node):
        parent_node.child_list.remove(child_node)
        child_node.parent = None 
        self.node_list.remove(child_node)
        self.coord_list.remove(list(child_node.x))

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
    
    def get_path(self, cur_node):
        path = np.array([cur_node.x])
        while cur_node.parent is not None:
            cur_node = cur_node.parent
            path = np.vstack([path, cur_node.x])
        return path
    
    def get_sterile_nodes(self):
        sterile_nodes = []
        for node in self.node_list:
            if len(node.child_list) == 0:
                sterile_nodes.append(node)
        return sterile_nodes





def random_config(tree):
    x_coord = round(uniform(0, constants.dimension_field[0]), 3)
    y_coord = round(uniform(0, constants.dimension_field[1]), 3)
    new_coord = np.array([x_coord, y_coord])
    if list(new_coord) in tree.coord_list:
        return random_config(tree)
    return Node(x=np.array([x_coord, y_coord]))
    
def new_config(rand_node, nearest_node, nearest_node_dist):
    if nearest_node_dist <= constants.step_size:
        return rand_node
    coord_to_comp = (rand_node.x[0] - nearest_node.x[0]) + (rand_node.x[1] - nearest_node.x[1])*1j
    phase = np.angle(coord_to_comp)
    new_node_comp = constants.step_size * np.exp(1j * phase)
    rand_node.x = np.array([new_node_comp.real, new_node_comp.imag]) + nearest_node.x
    return rand_node

def is_obstacle_free(parent_node, child_node):
    obstacle_free = True
    for obstacle in constants.obstacle_line:
        if obstacle_lib.doIntersect(parent_node.x, child_node.x, obstacle[0:2], obstacle[2:4]):
            obstacle_free = False
    return obstacle_free




