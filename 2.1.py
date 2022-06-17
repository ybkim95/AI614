import matplotlib.pyplot as plt
import time

from dijkstra import dijkstra
from config import *
from utils import *
from line import Line
from tree import Graph2


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

        nearvex, nearidx = near(G, randvex, obstacles)
        if nearvex is None:
            continue

        q_new = new_node(randvex, nearvex)

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

G = RRT_star(START, END, RRT_OBSTACLES)