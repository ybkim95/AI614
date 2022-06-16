import numpy as np
from matplotlib import collections  as mc
import matplotlib.pyplot as plt

from node import Node
from config import *

obstacles = CONFIG[-1]

def dist_btw_points(a, b):
    distance = np.linalg.norm(np.array(b) - np.array(a))
    return distance

def Cost(G, a, b):
    # print("b:", b)
    # if type(a[0]) != int:
    #     a = (a[0].x,a[0].y)
    #     a = G.vec2idx[a]
    # else:
    a_idx = G.vec2idx[a] 
    

    # if type(b[0]) != int:
    #     b = (b[0].x, b[0].y)
    #     # print(b)
    #     b = G.vec2idx[b]
    # else:
    try:
        b_idx = G.vec2idx[b]
    except:
        b_idx = len(G.nodes)
        G.nodes.append(b)
        G.vec2idx[b] = b_idx
        G.neighbors[b_idx] = []
    
    G.edges.append((a_idx, b_idx))

    cost = 0
    # print("a_idx:", a_idx, "b_idx:", b_idx)
    while not b_idx == a_idx:
        p = G.edges[a_idx]
        p = (p[1][0].x, p[1][0].y)
        cost += dist_btw_points((b.x, b.y), p)
        b_idx = p
    return cost

def dis(p1, p2):
    # print(p1,p2)
    # if type(p1[0]) != int:
    #     if type(p1[0]) == Node:
    #         p1 = p1[0]
    
    # if type(p2[0]) != int:
    #     if type(p2[0]) == Node:
    #         p2 = p2[0]
    
    # if type(p1) == tuple:
    #     p1 = Node(*p1)
    # if type(p2) == tuple:
    #     p2 = Node(*p2)
    return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**0.5

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
    for obs in obstacles:
        if intersect(line, obs[:2], obs[-1]):
            return True
    return False

def in_obs(node):
    for obs in obstacles:
        if dis(Node(obs[0], obs[1]), node) < obs[-1]:
            return True
    return False

def plot(ax, T, path=None):

    ax.set(xlim=(-2, 12), ylim=(-2, 12))

    px = [node.x if type(node) == Node else node[0] for node in T.nodes]
    py = [node.y if type(node) == Node else node[1] for node in T.nodes]

    for obs in obstacles:
        circle = plt.Circle(obs[:2], obs[-1], color='black')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    if type(T.start) == Node:
        ax.scatter(T.start.x, T.start.y, c='green', s=150)
    else:
        ax.scatter(T.start[0], T.start[1], c='green', s=150)

    # ax.scatter(T.end.x, T.end.y, c='red', s=150)
    if type(T.end) == Node:
        ax.scatter(T.end.x, T.end.y, s=1000, facecolors='none', edgecolors='r', linestyle='--')
    else:
        ax.scatter(T.end[0], T.end[1], s=1000, facecolors='none', edgecolors='r', linestyle='--')


    # print("T.edges:", T.edges)
    # print("T.nodes:", T.nodes)
    # T.nodes[0] = Node(*T.nodes[0])
    # if type(T.nodes[0]) == Node: 
        # lines = [((T.nodes[edge[0]].x,T.nodes[edge[0]].y), (T.nodes[edge[1]].x,T.nodes[edge[1]].y)) for edge in T.edges]
    # type(T.nodes[0]) == tuple:
    lines = [((T.nodes[edge[0]][0],T.nodes[edge[0]][1]), (T.nodes[edge[1]][0],T.nodes[edge[1]][1])) for edge in T.edges]
    
    lc = mc.LineCollection(lines, colors='blue', linewidths=2)
    ax.add_collection(lc)


    # print("path:", path)
    if path is not None:
        if type(path[0]) == tuple:
            paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
            lc2 = mc.LineCollection(paths, colors='magenta', linewidths=3)
            ax.add_collection(lc2)

        else:
            paths = [((path[i].x, path[i].y), (path[i+1].x,path[i+1].y)) for i in range(len(path)-1)]
            lc2 = mc.LineCollection(paths, colors='magenta', linewidths=3)
            ax.add_collection(lc2)

    plt.draw()
    plt.pause(0.01)

def Intersection(line, center, radius):
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

def distance(x, y):
    # print("x:", x, "y:", y)
    if type(x) == Node:
        x = (x.x, x.y)
    else:
        for i in x:
            if type(i) == Node:
                x = (i.x, i.y)
    if type(y) == Node:
        y = (y.x, y.y)
    else:
        for i in y:
            if type(i) == Node:
                y = (i.x, i.y)
    # print("x:", x, "y:", y)
        
    return np.linalg.norm(np.array(x) - np.array(y))

def isInObstacle(vec, obstacles):
    for tmp in obstacles:
        obs = tmp[:-1]
        if type(vec) != tuple:
            vec = (vec.x, vec.y)
        else:
            vec = vec[0]
            if type(vec) == Node:
                vec = (vec.x, vec.y)
                
        radius = tmp[-1]
        if distance(obs, vec) < radius:
            return True
    return False

def isThruObstacle(line, obstacles):
    for tmp in obstacles:
        obs = tmp[:-1]
        radius = tmp[-1]
        if Intersection(line, obs, radius):
            return True
    return False