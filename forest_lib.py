import numpy as np
from random import random, uniform
import tree_lib
import constants

class Forest:
    def __init__(self, *trees):
        tree_list = []
        for tree in trees:
            tree_list.append(tree)
        self.tree_list = tree_list

    
    def add_tree(self, tree):
        self.tree_list = [tree] + self.tree_list
    
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
        distance_array = []
        car_array = []
        selected_node = False
        for node in neighbourhood:
            try:
                new_car, distance = rand_node.propogate(node)
            except:
                continue
            selected_node = True
            distance_array.append(distance)
            car_array.append(new_car)
        if selected_node:
            min_cost_index = distance_array.index(min(distance_array))
            min_distance = distance_array[min_cost_index]
            min_car = car_array[min_cost_index]
            link_node_old_tree = neighbourhood[min_cost_index]
            link_node_new_tree = tree_lib.Node(x=min_car.x, v=min_car.v, theta=min_car.theta)
            link_node_new_tree.cost = min_distance + rand_node.cost
            self.tree_building(rand_node, link_node_old_tree, link_node_new_tree)

    def tree_cutting(self, new_node, link_node):
        old_tree = self.tree_list[link_node.id]
        new_tree = self.tree_list[new_node.id]
        link_node_parent = link_node.parent
        if link_node.parent is None:
            return
        old_tree.remove_connection(link_node_parent, link_node)
        old_tree.remove_subtree(link_node)
        new_tree.insert_node(new_node, link_node, self)

    def tree_building(self, new_node, link_node_old_tree, link_node_new_tree):
        old_tree = self.tree_list[link_node_old_tree.id]
        new_tree = self.tree_list[new_node.id]
        new_tree.insert_node(new_node, link_node_new_tree, self)
        for child_node_old in link_node_old_tree.child_list:
            try:
                child_car_new, distance = link_node_new_tree.propogate(child_node_old)
            except:
                continue
            child_node_new = tree_lib.Node(x=child_car_new.x, v=child_car_new.v,
             theta=child_car_new.theta, cost=(link_node_new_tree.cost + distance))
            self.tree_building(link_node_new_tree, child_node_old, child_node_new)
    
    def update_forest(self, tree):
        if len(self.tree_list) < constants.forest_trees:
            self.add_tree(tree)
        else:
            self.tree_list = [tree] + self.tree_list
            self.tree_list = self.tree_list[:-1]
        for i in range(0, len(self.tree_list)):
            self.tree_list[i].id = i
            for node in self.tree_list[i].node_list:
                node.id = i
    
    def checkgoal(self, goal):
        tree = self.tree_list[0]
        if list(goal) in tree.coord_list:
            return True

    def random_config(self, final_point, check_goal=True):
        p = random()
        if check_goal and p <= constants.goal_prob:
            rand_node =  tree_lib.Node(x=final_point)
        else:
            x_coord = round(uniform(0, constants.dimension_field[0]), 3)
            y_coord = round(uniform(0, constants.dimension_field[1]), 3)
            new_coord = np.array([x_coord, y_coord])

            for tree in self.tree_list:
                if list(new_coord) in tree.coord_list:
                    return self.random_config(final_point, check_goal)
            rand_node = tree_lib.Node(x=np.array([x_coord, y_coord]))
        with open("test_cases/test.txt", "a") as file:
            file.write(f"{rand_node.x[0]} {rand_node.x[1]} \n")  
        return rand_node


    
    

