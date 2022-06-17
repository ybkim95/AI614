import matplotlib.pyplot as plt
import time

from tree import Tree
from config import *
from utils import plot
from dijkstra import dijkstra

from utils import dis1, obs_checker, near, new_node

def RRT(config):
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
        
        if obs_checker(q_rand, obstacles):
            continue
        
        # (2) q_near
        q_near, near_idx = near(T, q_rand, obstacles)
        if q_near is None:
            continue

        # (3) q_new
        q_new = new_node(q_rand, q_near, STEP_SIZE)

        # (4) G.add_vertex(q_new)
        new_idx = T.add_node(q_new)

        # (5) G.add_edge(q_near, q_new)
        T.add_edge(new_idx, near_idx, dis1(q_new, q_near))

        if dis1(q_new, T.end) < radius:
            end_idx = T.add_node(T.end)
            T.add_edge(new_idx, end_idx, dis1(q_new, T.end))
            T.success = True
            if dijkstra(T) is not None:
                print("[INFO] Path Found")
                plot(ax, T, dijkstra(T))
                print("[RRT] Execution Time:", time.time() - start)
                print("[RRT] Number of Steps:", num_steps)
            
            fig.savefig("figures/1-3.png")
            break
            
        num_steps += 1
        plot(ax, T, dijkstra(T))
             
T = RRT(CONFIG)