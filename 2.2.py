import numpy as np

import matplotlib.pyplot as plt
from matplotlib import collections  as mc

from collections import deque
import time
from random import random
import math

from config import *
from utils import *
from dijkstra import dijkstra
from line import Line1
from tree import Graph

# FIXME
def Near(V, x_rand):
    # print("V:", V)
    # print("x_rand:", x_rand)
    X_near = []
    n = len(V)
    d = 2
    for x_ in V:
        # print("dis(x_rand, x_):", dis(x_rand[0], x_))
        # print("less than", GAMMA * (math.log(n)/n)**(1/d))
        if dis(x_rand[0], x_) <= GAMMA * (math.log(n)/n)**(1/d):
            X_near.append(x_)
    return X_near

def ChooseParent(G, X_near, x_rand):
    x_init = G.start

    minCost = float("inf")
    x_min = None
    sigma_min = None

    # print("X_near:", X_near)
    # print("x_rand:", x_rand)
    if type(x_rand[0]) == Node:
        pass
    else:
        x_rand = Node(*x_rand[0])

    for x_near in X_near:
        sigma = G.extend(x_near, x_rand)

        if Cost(G, x_init, x_near) + Cost(G, x_init, sigma) < minCost:
            minCost = Cost(G, x_init, x_near) + Cost(G, x_init, sigma)
            x_min = x_near
            x_rand.parent = x_near
            sigma_min = sigma
    return x_min, sigma_min

def Parent(x_near):
    return x_near.parent
    

def CollisionFree(G, sigma):
    cnt = True
    for vec in G.nodes:
        if type(vec) != tuple:
            vec = (vec.x, vec.y)
        
        # print("vec:", vec, "sigma:", sigma)
        for i in vec:
            if type(i) == Node:
                vec = i
                break
        for j in sigma:
            if type(j) == Node:
                sigma = j
                break
        
        if type(vec) == Node:
            vec = (vec.x, vec.y)
        if type(sigma) == Node:
            sigma = (sigma.x, sigma.y)
        
        # print("vec:", vec, "sigma:", sigma)
        line = Line1(vec, sigma)
        if isThruObstacle(line, CONFIG[-1]):
            cnt = False
    
    return cnt

def Rewire(G, X_near, x_rand):
    for x_near in X_near:
        sigma = G.extend(x_rand, x_near)[0]
        if Cost(G, G.start, x_rand) + Cost(G, G.start, sigma) < Cost(G, G.start, x_near):
            if CollisionFree(G, sigma):
                x_parent = Parent(G, x_near)
                G.edges.remove((G.vec2idx(x_parent), G.vec2idx(x_near)))
                G.edges.append((G.vec2idx(x_rand), G.vec2idx(x_near)))
    return (G.nodes, G.edges)
    


def newnode(randvec, nearvec, stepSize):
    dirn = np.array(randvec) - np.array(nearvec)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvec = (nearvec[0]+dirn[0], nearvec[1]+dirn[1])
    return newvec


def RRT_star(config, stepSize):
    q_init = config[:-1]
    obstacles = config[-1]
    start = time.time()

    plt.ion() 
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set_title("2.1 RRT*")
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False) 

    G = Graph(q_init)

    num_steps = 0

    for _ in range(N_ITER):
        x_rand= G.randomPosition()
        if isInObstacle(x_rand, obstacles):
            continue

        # print("G.nodes:", G.nodes)
        # print("x_rand:", x_rand)
        X_near = Near(G.nodes, x_rand) # FIXME

        # print("X_near:", X_near)

        # ChooseParent
        x_min, sigma_min = ChooseParent(G, X_near, x_rand)

        if CollisionFree(G, x_rand):
            G.nodes.append(x_rand[0])
            # print("G.nodes:", G.nodes)
            G.edges.append((x_min, x_rand))
            G.nodes, G.edges = Rewire(G, X_near, x_rand)

        if x_min is None:
            # print("continue")
            continue
        
        if distance(x_min, G.end) < 0.7:
            print("[INFO] Path Found")   

            G.success = True
            if dijkstra(G) is not None:
                print("[INFO] Path Found")
                plot(ax, G, dijkstra(G))
                print("[RRT*] Execution Time:", time.time() - start)
                print("[RRT*] Number of Steps:", num_steps)
            
            fig.savefig("2-2.png")
            break
    
        num_steps +=1
        plot(ax, G, dijkstra(G))
    
stepSize = 0.7
G = RRT_star(CONFIG, stepSize)