import numpy as np
import tree_lib
import constants

class Forest:
    def __init__(self, *trees):
        tree_list = []
        for tree in trees:
            tree_list.append(tree)
        self.tree_list = tree_list

    
    def add_tree(self, tree):
        self.tree_list.append(tree)
    
    def get_forest_neighbourhood(self, center_node):
        curent_tree = self.tree_list[center_node.id]
        neighbour_nodes = []
        for tree in self.tree_list:
            if tree.id != curent_tree.id:
                for node in tree.node_list:
                    if constants.distance(node, center_node) <= constants.forest_neighbour:
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

    def tree_cutting(self, new_node, link_node):
        old_tree = self.tree_list[link_node.id]
        new_tree = self.tree_list[new_node.id]
        old_tree.remove_connection(link_node.parent, link_node)
        old_tree.remove_subtree(link_node.parent)
        new_tree.insert_node(new_node, link_node)
    
    def update_forest(self, tree):
        if len(self.tree_list) < constants.forest_trees:
            self.add_tree(tree)
        else:
            self.tree_list = [tree] + self.tree_list
            self.tree_list = self.tree_list[:-1]
        for i in range(1, len(self.tree_list)):
            self.tree_list[i].id = i
    
    def checkgoal(self, goal):
        tree = self.tree_list[-1]
        if list(goal) in tree.coord_list:
            return True



    
    

