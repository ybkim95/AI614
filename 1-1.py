from tabnanny import check
import numpy as np
from random import random, uniform, randint
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
from collections import deque
import time


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


def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(line, obstacles, radius):
    for obs in obstacles:
        if Intersection(line, obs, radius):
            return True
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


def rand_conf(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    # print(1)
    # print(vex)
    while True:
        rand_idx = randint(0, len(G.vertices)-1)
        v = G.vertices[rand_idx]
        # print(G.vertices)s
        line = Line(v, vex)
            
        if not isThruObstacle(line, obstacles, radius):
            
            Nidx = rand_idx
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


def plot(fig, ax, G, obstacles, radius, path=None):

    ax.set(xlim=(-2, 12), ylim=(-2, 12))

    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]

    for obs in obstacles:
        circle = plt.Circle(obs, radius, color='black')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.start[0], G.start[1], c='green', s=150)
    ax.scatter(G.goal[0], G.goal[1], c='red', s=150)

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='blue', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='magenta', linewidths=3)
        ax.add_collection(lc2)

    plt.draw()
    plt.pause(0.01)


def run(q_init, obstacles, n_iter, radius, stepSize):

    plt.ion() 
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set_title("An alternative sampling-based algorithm (2D)")
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False) 

    # (0) G.init(q_init)
    G = Graph(q_init)

    for _ in range(n_iter):
        # (1) q_rand
        q_tree= G.randomPosition() # FIXME to q_rand
        
        if isInObstacle(q_tree, obstacles, radius):
            continue
        
        # (2) q_rand
        q_rand, rand_idx = rand_conf(G, q_tree, obstacles, radius) # FIXME to q_tree
        if q_rand is None:
            continue

        # print(2)

        # (3) q_new
        q_new = newVertex(q_tree, q_rand, stepSize)

        # print(3)

        # (4) G.add_vertex(q_new)
        new_idx = G.add_vertex(q_new)

        # print(4)

        # (5) G.add_edge(q_near, q_new)
        G.add_edge(new_idx, rand_idx, distance(q_new, q_rand))

        # print(5)

        # end condition check
        if distance(q_new, G.goal) < radius:
            end_idx = G.add_vertex(G.goal)
            G.add_edge(new_idx, end_idx, distance(q_new, G.goal))
            G.success = True
            if dijkstra(G) is not None:
                print("[INFO] Solution Found")
                plot(fig, ax, G, obstacles, radius, dijkstra(G))
                time.sleep(10)
            break
        
        plot(fig, ax, G, obstacles, radius, dijkstra(G))


radius = 1                
obstacles = [(5,5), (9,3), (1.5,9), (1,4), (6,10), (6,0)]#[check_overlap(obstacles) for _ in range(5)]
startpos = (1,1)#check_overlap(obstacles)
endpos = (9,9)#check_overlap(obstacles + [startpos])
n_iter = 200
stepSize = 0.7
q_init = (startpos, endpos)

G = run(q_init, obstacles, n_iter, radius, stepSize)