import time
import numpy as np
import matplotlib.pyplot as plt

from utils import *
from config import *
from tree import Tree
from dijkstra import dijkstra


def main():
    plt.ion() 
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set_title("1.1 An alternative sampling-based algorithm (2D)")
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False) 
    start = time.time()

    # 1. Initialize a tree with  initial configuration
    T = Tree(CONFIG)

    check = False
    num_steps = 0
    for i in range(N_ITER):
        # 2. Select a random node from the tree, called x_tree
        x_tree = T.rand_node()
        if in_obs(x_tree):
            continue

        # 3. Select a random configuration, called x_rand, 
        #    from the collision-free configuration space
        x_rand = T.rand_config(x_tree)

        # 4. Extend from x_tree to x_rand (i.e. x_new)
        x_new = T.extend(x_tree, x_rand)

        if dis(x_new[0], T.end) < RADIUS:
            end_idx = T.add_node(T.end)

            # 5. Add the new nodes from the extend operation to the tree
            T.add_edge(x_new[1], end_idx, dis(x_new[0], T.end))

            path = dijkstra(T)
            
            # 6. If a goal configuration is added to the tree, terminate. 
            #    Solution found!
            if len(path) > 0:
                execution_time = time.time() - start
                print("[INFO] Solution Found")
                plot(ax, T, [])
                for i,p in enumerate(path):
                    print(i, "({},{})".format(p.x, p.y))
                check = True
                fig.savefig("figures/1-1.png")
                print("Execution Time:", execution_time)
                print("Number of Steps:", num_steps)
                break
            else:
                # 7. Otherwise, go to step 2 
                continue
        
        num_steps += 1
    
        plot(ax, T, [])
    
    if check:
        plot(ax, T, path)
        time.sleep(10)
    print("Number of Steps:", num_steps)


main()