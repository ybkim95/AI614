'''
MIT License
Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.  
'''

from tabnanny import check
import numpy as np
from random import random, uniform
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
import time
from config import *


class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn

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
    return np.linalg.norm(np.array(x) - np.array(y))

def isInObstacle(vex, obstacles):
    for tmp in obstacles:
        obs = tmp[:-1]
        radius = tmp[-1]
        if distance(obs, vex) < radius:
            return True
    return False

def isThruObstacle(line, obstacles):
    for tmp in obstacles:
        obs = tmp[:-1]
        radius = tmp[-1]
        if Intersection(line, obs, radius):
            return True
    return False

def nearest(G, vex, obstacles):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.vertices):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx

def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
    return newvex

def window(startpos, endpos):
    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    return winx, winy, width, height

def isInWindow(pos, winx, winy, width, height):
    if winx < pos[0] < winx+width and \
        winy < pos[1] < winy+height:
        return True
    else:
        return False

class Graph:
    def __init__(self, q_init):
        self.start = q_init[0]
        self.goal = q_init[1]

        self.vertices = [q_init[0]]
        self.edges = []
        self.success = False

        self.vex2idx = {q_init[0]:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = q_init[1][0] - q_init[0][0]
        self.sy = q_init[1][1] - q_init[0][1]

    def add_vertex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
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
        return posx, posy


def RRT(config, stepSize):
    q_init = config[:-1]
    obstacles = config[-1]

    start = time.time()

    plt.ion() 
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set_title("1.3 RRT")
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False) 

    # (0) G.init(q_init)
    G = Graph(q_init)
    
    num_steps = 0
    radius = max(tmp[-1] for tmp in obstacles)
    for _ in range(N_ITER):
        # (1) q_rand
        q_rand= G.randomPosition()
        
        if isInObstacle(q_rand, obstacles):
            continue
        
        # (2) q_near
        q_near, near_idx = nearest(G, q_rand, obstacles)
        if q_near is None:
            continue

        # (3) q_new
        q_new = newVertex(q_rand, q_near, stepSize)

        # (4) G.add_vertex(q_new)
        new_idx = G.add_vertex(q_new)

        # (5) G.add_edge(q_near, q_new)
        G.add_edge(new_idx, near_idx, distance(q_new, q_near))

        # end condition check
        if distance(q_new, G.goal) < radius:
            end_idx = G.add_vertex(G.goal)
            G.add_edge(new_idx, end_idx, distance(q_new, G.goal))
            G.success = True
            if dijkstra(G) is not None:
                print("[INFO] Path Found")
                plot(fig, ax, G, obstacles, dijkstra(G))
                print("[RRT] Execution Time:", time.time() - start)
                print("[RRT] Number of Steps:", num_steps)
            
            fig.savefig("1-3.png")
            break
            
        num_steps += 1
        
        plot(fig, ax, G, obstacles, dijkstra(G))
        

def dijkstra(G):
    srcIdx = G.vex2idx[G.start]
    try:
        dstIdx = G.vex2idx[G.goal]
    except:
        dstIdx = 0

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)

def plot(fig, ax, G, obstacles, path=None):

    ax.set(xlim=(-2, 12), ylim=(-2, 12))

    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]

    for tmp in obstacles:
        obs = tmp[:-1]
        radius = tmp[-1] 
        circle = plt.Circle(obs, radius, color='black')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.start[0], G.start[1], c='green', s=150)
    # ax.scatter(G.goal[0], G.goal[1], c='red', s=150)

    ax.scatter(G.goal[0], G.goal[1], s=1000, facecolors='none', edgecolors='r', linestyle='--')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='blue', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='magenta', linewidths=3)
        ax.add_collection(lc2)

    plt.draw()
    plt.pause(0.01)
             
stepSize = 0.7

G = RRT(CONFIG, stepSize)
# G = RRT_star(startpos, endpos, obstacles, n_iter, stepSize)

# if G.success:
#     path = dijkstra(G)
#     print(path)
#     plot(G, obstacles, radius, path)
# else:
#     plot(G, obstacles, radius)