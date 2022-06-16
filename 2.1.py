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
from dijkstra import dijkstra
from utils import plot#, isInObstacle, isThruObstacle
from config import START, END, RRT_OBSTACLES


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

def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(G.nodes):
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
        self.end = q_init[1]

        self.nodes = [q_init[0]]
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
        return posx, posy


def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    plt.ion() 
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set_title("2.1 RRT*")
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False) 
    start = time.time()
    G = Graph((startpos, endpos))

    rewire_time = []
    num_steps = 0
    for _ in range(n_iter):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        q_new = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_node(q_new)
        dist = distance(q_new, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        mid1 = time.time()
        # update nearby nodes distance (if shorter)
        for vex in G.nodes:
            if vex == q_new:
                continue

            dist = distance(vex, q_new)
            if dist > radius:
                continue

            line = Line(vex, q_new)
            if isThruObstacle(line, obstacles):
                continue

            idx = G.vec2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

            dist = distance(q_new, G.end)
        
        
        if distance(q_new, G.end) < radius:
            print("[INFO] Path Found")   
            endidx = G.add_node(G.end)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
            except:
                G.distances[endidx] = G.distances[newidx]+dist

            G.success = True
            if dijkstra(G) is not None:
                print("[INFO] Path Found")
                # plot(ax, G, dijkstra(G))
                print("[RRT*] Execution Time:", time.time() - start)
                print("[RRT*] Number of Steps:", num_steps)
            
            fig.savefig("2-1.png")
            break
        rewire_time.append(time.time()-mid1)
    
        num_steps +=1
        # plot(ax, G, dijkstra(G))
    
    print("[RRT*] Portion of REWIRE:", sum(rewire_time)/(time.time()-start)*100)


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        # plot(G, obstacles, radius, path)
        return path


radius = 0.7                
#obstacles = [(0,0)]
startpos = (1,1)#check_overlap(obstacles)
endpos = (8,8)#check_overlap(obstacles + [startpos])
n_iter = 300
stepSize = 0.7
q_init = (startpos, endpos)

G = RRT_star(START, END, RRT_OBSTACLES, n_iter, radius, stepSize)
# G = RRT_star(q_init, obstacles, n_iter, radius, stepSize)

# if G.success:
#     path = dijkstra(G)
#     print(path)
#     plot(G, obstacles, radius, path)
# else:
#     plot(G, obstacles, radius)