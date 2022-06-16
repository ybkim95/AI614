
from secrets import randbelow
from node import Node
from line import Line

from utils import *
from config import *

import numpy as np
from random import randint, random

def dis(n1, n2):
    if type(n1) != Node:
        n1 = Node(*n1)
    if type(n2) != Node:
        n2 = Node(*n2)
    return ((n1.x-n2.x)**2 + (n1.y-n2.y)**2)**0.5

class Tree:
    def __init__(self, config):
        start, end, obstacles = config
        self.start = Node(*start)
        self.end = Node(*end)
        self.obstacles = obstacles
        self.nodes = [self.start]
        self.edges = []
        self.vec2idx = {self.start:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}
        self.sx = self.end.x - self.start.x
        self.sy = self.end.y - self.start.y

    def extend(self, x_tree, x_rand):
        # print("extending from ({:.2f},{:.2f}) to ({:.2f},{:.2f}) ...".format(x_tree.x, x_tree.y, x_rand.x, x_rand.y))
        # 1) add node
        rand_node = (x_rand.x, x_rand.y) 
        adj_node = (x_tree.x, x_tree.y)

        vec = np.array(rand_node) - np.array(adj_node)
        length = np.linalg.norm(vec)
        vec = (vec / length) * min(STEP_SIZE, length)

        new_node = Node(adj_node[0]+vec[0], adj_node[1]+vec[1])
        try:
            new_idx = self.vec2idx[new_node]
            self.nodes.append(new_node)
        except:
            new_idx = len(self.nodes)
            self.nodes.append(new_node)
            self.vec2idx[new_node] = new_idx
            self.neighbors[new_idx] = []
        
        # print("nodes:")
        # for i in range(len(self.nodes)):
        #     print("({:.2f},{:.2f})".format(self.nodes[i].x, self.nodes[i].y))
        # print()

        # 2) add edge
        rand_node = Node(x_rand.x, x_rand.y) # tuple -> node
        try:
            rand_idx = self.vec2idx[x_rand]
            self.neighbors[rand_idx] = []
        except:
            rand_idx = len(self.nodes)
            self.nodes.append(rand_node)
            self.vec2idx[rand_node] = rand_idx
            self.neighbors[rand_idx] = []

        self.edges.append((rand_idx, new_idx))
        cost = dis(x_rand, new_node)
        self.neighbors[rand_idx].append((new_idx, cost))
        self.neighbors[new_idx].append((rand_idx, cost))
        return new_node, new_idx

    def rand_node(self):
        rand_idx = randint(0, len(self.nodes)-1)
        rand_node = self.nodes[rand_idx]
        # print("random node ({:.2f},{:.2f}) was selected from the tree".format(rand_node.x, rand_node.y))
        return rand_node #, rand_idx)

    def rand_node_heuristic(self):
        distance_info = sorted([(dis(node, self.end), node) for node in self.nodes], key = lambda x : x[0])
        return distance_info[0][1], self.vec2idx[distance_info[0][1]] # Node that is closest to the goal

    def rand_config(self, node):
        while True:
            posx = uniform(min(self.start.x, self.end.x), max(self.start.x, self.end.x))  #self.start.x - (self.sx / 2.) + random() * self.sx * 1.2 # 
            posy = uniform(min(self.start.y, self.end.y), max(self.start.y, self.end.y))  #self.start.y - (self.sy / 2.) + random() * self.sy * 1.2 #
            
            total = dis(node, Node(posx, posy))
            coord = (node.x + (node.x-posx)/total * uniform(-1,1), node.y + (node.y-posy)/total * uniform(-1,1)) 
            line = Line(node, Node(coord[0], coord[1]))

            if not thru_obs(line):
                # print("coord:", coord)
                return Node(coord[0], coord[1]) # idx?

    def add_edge(self, idx1, idx2, cost):
        # print("idx1:", idx1)
        # print("idx2:", idx2)
        
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def add_node(self, pos):
        try:
            # print("pos:", pos)s
            idx = self.vec2idx[pos]
        except:
            idx = len(self.nodes)
            self.nodes.append(pos)
            self.vec2idx[pos] = idx
            self.neighbors[idx] = []
        
        return idx


class Graph:
    def __init__(self, q_init):
        self.start = q_init[0]
        self.end = q_init[1]

        self.nodes = [Node(*q_init[0])]
        self.edges = []
        self.success = False

        self.vec2idx = {q_init[0]:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = q_init[1][0] - q_init[0][0]
        self.sy = q_init[1][1] - q_init[0][1]

    def add_node(self, pos):
        try:
            idx = self.vec2idx[pos]
        except:
            idx = len(self.nodes)
            self.nodes.append(pos)
            self.vec2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self):
        rx = random()
        ry = random()

        posx = self.start[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.start[1] - (self.sy / 2.) + ry * self.sy * 2
        try:
            node = Node(posx, posy)
            idx = self.vec2idx[Node(posx, posy)]
            return node, idx
        except:
            node = Node(posx, posy)
            self.vec2idx[node] = len(self.nodes)
            return Node(posx, posy), self.vec2idx[node] 
        # distance_info = sorted([(dis(node, self.end), node) for node in self.nodes], key = lambda x : x[0])

        # distance_info[0] = Node(*distance_info[0])

        # print("distance_info:", distance_info)
        
        # return distance_info[0], self.vec2idx[distance_info[0]] # Node that is closest to the goal
    
    def extend(self, x_tree, x_rand):
        # print("extending from ({:.2f},{:.2f}) to ({:.2f},{:.2f}) ...".format(x_tree.x, x_tree.y, x_rand.x, x_rand.y))
        # 1) add node
        # print("x_rand:", x_rand)
        # print("x_tree:", x_tree)
        # if type(x_rand) == Node:
        #     pass
        # elif type(x_rand[0]) != int:
        #     x_rand = x_rand[0]
        # if type(x_tree[0]) != int:
        #     x_tree = x_tree[0]
        
        if type(x_rand) == Node:
            rand_node = x_rand
        else:
            for i in x_rand:
                if type(i) == Node:
                    rand_node = i
                    break
            rand_node = Node(*x_rand)
        
        rand_node = (rand_node.x, rand_node.y)
            
        if type(x_tree) == Node: 
            adj_node = (x_tree.x, x_tree.y)
        else:
            adj_node = x_tree

        rand_node = (rand_node[0].x, rand_node[0].y)
        
        # print("rand_node:", rand_node, "adj_node:", adj_node)

        vec = np.array(rand_node) - np.array(adj_node)
        length = np.linalg.norm(vec) + 1e-6
        vec = (vec / length) * min(STEP_SIZE, length)

        new_node = Node(adj_node[0]+vec[0], adj_node[1]+vec[1])
        try:
            new_idx = self.vec2idx[new_node]
            self.nodes.append(new_node)
        except:
            new_idx = len(self.nodes)
            self.nodes.append(new_node)
            self.vec2idx[new_node] = new_idx
            self.neighbors[new_idx] = []
        
        # print("nodes:")
        # for i in range(len(self.nodes)):
        #     print("({:.2f},{:.2f})".format(self.nodes[i].x, self.nodes[i].y))
        # print()

        # 2) add edge
        if type(x_rand) == Node:
            rand_node = x_rand # tuple -> node
        else:
            rand_node = Node(*x_rand)
        try:
            rand_idx = self.vec2idx[x_rand]
            self.neighbors[rand_idx] = []
        except:
            rand_idx = len(self.nodes)
            self.nodes.append(rand_node)
            self.vec2idx[rand_node] = rand_idx
            self.neighbors[rand_idx] = []

        self.edges.append((rand_idx, new_idx))
        # print("x_rand:", x_rand, "new_node:", new_node)
        cost = dis(x_rand[0], new_node)
        self.neighbors[rand_idx].append((new_idx, cost))
        self.neighbors[new_idx].append((rand_idx, cost))
        return new_node, new_idx
