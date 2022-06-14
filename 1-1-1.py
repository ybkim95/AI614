import time
import numpy as np
from random import random, uniform, randint
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque


N_ITER = 300
STEP_SIZE = 0.1
RADIUS = 0.7
OBSTACLES = [
    # (x,y,r)
    (5,5, uniform(0.5,1.5)), 
    (9,3, uniform(0.5,1.5)), 
    (1.5,9, uniform(0.5,1.5)), 
    (1,4, uniform(0.5,1.5)), 
    (6,10, uniform(0.5,1.5)), 
    (6,0, uniform(0.5,1.5))
]
START = (1,1)
END = (8,6)


class Line:
    def __init__(self, p0, p1):
        p0 = (p0.x, p0.y)
        p1 = (p1.x, p1.y)
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Tree:
    def __init__(self, start, end, obstacles):
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
        # (1) add node
        rand_node = (x_rand.x, x_rand.y) 
        adj_node = (x_tree.x, x_tree.y)

        vec = np.array(rand_node) - np.array(adj_node)
        length = np.linalg.norm(vec)
        vec = (vec / length) * min(STEP_SIZE, length)

        new_node = Node(adj_node[0]+vec[0], adj_node[1]+vec[1])
        try:
            new_idx = self.vec2idx[new_node]
        except:
            new_idx = len(self.nodes)
            self.nodes.append(new_node)
            self.vec2idx[new_node] = new_idx
            self.neighbors[new_idx] = []

        # (2) add edge
        rand_node = Node(x_rand.x, x_rand.y)
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

        return (rand_node, rand_idx)


    def rand_config(self, node):
        
        while True:
            posx = uniform(0,10) #self.start.x - (self.sx / 2.) + random() * self.sx * 2
            posy = uniform(0,10) #self.start.y - (self.sy / 2.) + random() * self.sy * 2

            line = Line(node, Node(posx, posy))
            
            if not thru_obs(line):
                
                return Node(posx, posy)


def dis(p1, p2):
    p1 = (p1.x, p1.y)
    p2 = (p2.x, p2.y)
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5


def intersect(line, center, radius):
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True


def thru_obs(line):
    for obs in OBSTACLES:
        if intersect(line, obs[:2], obs[-1]):
            return True
    return False


def in_obs(node):
    for obs in OBSTACLES:
        if dis(Node(obs[0], obs[1]), node[0]) < obs[-1]:
            return True
    return False


def plot(ax, T, path=None):

    ax.set(xlim=(-2, 12), ylim=(-2, 12))

    px = [node.x for node in T.nodes]
    py = [node.y for node in T.nodes]

    for obs in OBSTACLES:
        circle = plt.Circle(obs[:2], obs[-1], color='black')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(T.start.x, T.start.y, c='green', s=150)
    # ax.scatter(T.end.x, T.end.y, c='red', s=150)
    ax.scatter(T.end.x, T.end.y, s=1000, facecolors='none', edgecolors='r', linestyle='--')

    lines = [((T.nodes[edge[0]].x,T.nodes[edge[0]].y), (T.nodes[edge[1]].x,T.nodes[edge[1]].y)) for edge in T.edges]
    lc = mc.LineCollection(lines, colors='blue', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [((path[i].x, path[i].y), (path[i+1].x, path[i+1].y)) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='magenta', linewidths=3)
        ax.add_collection(lc2)

    plt.draw()
    plt.pause(0.01)


def dijkstra(T):
    srcIdx = T.vec2idx[T.start]
    try:
        dstIdx = T.vec2idx[T.end]
    except:
        dstIdx = 0

    # build dijkstra
    nodes = list(T.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in T.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(T.nodes[curNode])
        curNode = prev[curNode]
    path.appendleft(T.nodes[curNode])
    return list(path)


def main():
    plt.ion() 
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set_title("An alternative sampling-based algorithm (2D)")
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False) 

    T = Tree(START, END, OBSTACLES)

    check = False

    for i in range(N_ITER):
        # print(i)

        x_tree = T.rand_node()
        if in_obs(x_tree):
            continue
        
        # print("x_tree: ({},{})".format(x_tree.x, x_tree.y))

        x_rand = T.rand_config(x_tree[0])

        x_new = T.extend(x_tree[0], x_rand)

        print("x_new: ({},{}), dist: {}".format(x_new[0].x, x_new[0].y))

        if dis(x_new[0], T.end) < RADIUS:
            T.vec2idx[T.end] = x_rand
            # end_idx = T.add_node(T.end)
            # T.add_edge(x_new[1], end_idx, dis(x_new, T.end))

            T.extend(x_new[0], T.end)

            path = dijkstra(T)
            path.append(x_new[0]) 
            
            if len(path) > 0:
                print("[INFO] Solution Found")
                # plot(ax, T, path)
                for i,p in enumerate(path):
                    print(i, "({},{})".format(p.x, p.y))
                check = True
            break
    
        plot(ax, T, dijkstra(T))

    if check:
        plot(ax, T, path)
        time.sleep(10)



main()