import matplotlib.pyplot as plt
from matplotlib import collections  as mc

from collections import deque
import numpy as np
from random import random
import time

from tree import Tree
from config import *
from utils import plot, isThruObstacle
from dijkstra import dijkstra
from line import Line
from node import Node


def distance(x, y):
    if type(x) == Node:
        x = (x.x, x.y)
    if type(y) == Node:
        y = (y.x, y.y)
    return np.linalg.norm(np.array(x) - np.array(y))

def isInObstacle(vex, obstacles):
    for tmp in obstacles:
        obs = tmp[:-1]
        radius = tmp[-1]
        if distance(obs, vex) < radius:
            return True
    return False

def nearest(T, vex, obstacles):
    Nvex = None
    Nidx = None
    minDist = float("inf")

    for idx, v in enumerate(T.nodes):
        line = Line(v, vex)
        if isThruObstacle(line, obstacles):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx

def newVertex(rand_node, near_node, stepSize):
    if type(near_node) == Node:
        near_node = (near_node.x, near_node.y)
    dir = np.array(rand_node) - np.array(near_node)
    length = np.linalg.norm(dir)
    dir = (dir / length) * min (stepSize, length)

    newvex = (near_node[0]+dir[0], near_node[1]+dir[1])
    return newvex

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
    T = Tree(CONFIG)
    
    num_steps = 0
    radius = max(tmp[-1] for tmp in obstacles)
    for _ in range(N_ITER):
        # (1) q_rand
        q_rand= T.rand_pos()
        
        if isInObstacle(q_rand, obstacles):
            continue
        
        # (2) q_near
        q_near, near_idx = nearest(T, q_rand, obstacles)
        if q_near is None:
            continue

        # (3) q_new
        q_new = newVertex(q_rand, q_near, stepSize)

        # (4) G.add_vertex(q_new)
        new_idx = T.add_node(q_new)

        # (5) G.add_edge(q_near, q_new)
        T.add_edge(new_idx, near_idx, distance(q_new, q_near))

        if distance(q_new, T.end) < radius:
            end_idx = T.add_node(T.end)
            T.add_edge(new_idx, end_idx, distance(q_new, T.end))
            T.success = True
            if dijkstra(T) is not None:
                print("[INFO] Path Found")
                plot(ax, T, dijkstra(T))
                print("[RRT] Execution Time:", time.time() - start)
                print("[RRT] Number of Steps:", num_steps)
            
            fig.savefig("1-3.png")
            break
            
        num_steps += 1
        
        plot(ax, T, dijkstra(T))
             
stepSize = 0.7

T = RRT(CONFIG, stepSize)