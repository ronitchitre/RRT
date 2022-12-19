import numpy as np
import tree_lib
import constants

class Forest:
    def __init__(self, *trees):
        tree_list = []
        node_list = []
        for tree in trees:
            tree_list.append(tree)
            node_list += tree.node_list
        self.tree_list = tree_list
        self.node_list = node_list

    
    def add_tree(self, tree):
        self.tree_list.append(tree)
    
    def get_forest_neighbourhood(self, center_node):
        neighbour_nodes = []
        for node in self.node_list:
            if constants.distance(node, center_node) <= constants.forest_neighbour and node.id != center_node.id:
                neighbour_nodes.append(node)
        return neighbour_nodes
    

    def check_tree_connection(self, rand_node, neighbourhood):
        min_cost = float('inf')
        parent_node = "unsure"
        for node in neighbourhood:
            try:
                new_car, distance = rand_node.propogate(node)
                if distance < min_cost:
                    min_cost = distance
                    new_cost = rand_node.cost + distance
                    parent_node = tree_lib.Node(x = new_car.x, v=new_car.v, theta=new_car.theta, parent=rand_node, cost=new_cost, id=node.id)
                
            except:
                continue
        if parent_node != "unsure":
            self.tree_cutting(rand_node, parent_node)

    def tree_cutting(self, rand_node, parent_node):
        old_tree = self.tree_list[parent_node.id]
        new_tree = self.tree_list[rand_node.id]
        old_tree.remove_connection(parent_node.parent, parent_node)
    
    

