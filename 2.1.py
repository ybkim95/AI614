import numpy as np
import matplotlib.pyplot as plt
import time

from dijkstra import dijkstra
from config import *
from utils import *
from line import Line
from tree import Graph2


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

def newVertex(randvex, nearvex):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (STEP_SIZE, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
    return newvex

def RRT_star(startpos, endpos, obstacles):
    plt.ion() 
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set_title("2.1 RRT*")
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False) 
    start = time.time()
    G = Graph2((startpos, endpos))

    rewire_time = []
    num_steps = 0
    for _ in range(N_ITER):
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles)
        if nearvex is None:
            continue

        q_new = newVertex(randvex, nearvex)

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
            if dist > RADIUS:
                continue

            line = Line(vex, q_new)
            if isThruObstacle(line, obstacles):
                continue

            idx = G.vec2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist

            dist = distance(q_new, G.end)
        
        
        if distance(q_new, G.end) < RADIUS:
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
                path = dijkstra(G)
                plot(ax, G, path)
                print("[RRT*] Execution Time:", time.time() - start)
                print("[RRT*] Number of Steps:", num_steps)
            
            fig.savefig("figures/2-1.png")
            break
        rewire_time.append(time.time()-mid1)
    
        num_steps +=1
        plot(ax, G, dijkstra(G))
    
    print("[RRT*] Portion of REWIRE:", sum(rewire_time)/(time.time()-start)*100)


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        plot(G, obstacles, radius, path)
        return path

G = RRT_star(START, END, RRT_OBSTACLES)