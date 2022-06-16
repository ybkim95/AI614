import time
import numpy as np
import matplotlib.pyplot as plt

from utils import *
from config import *
from tree import Tree
from dijkstra import dijkstra


def main():
    start = time.time()
    plt.ion() 
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set_title("1.2 Exploiting domain knowledge")
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False) 

    # 1. Initialize a tree with  initial configuration
    T = Tree(CONFIG)

    # print("Init Tree:")
    # for i in range(len(T.nodes)):
        # print("({:.2f},{:.2f})".format(T.nodes[i].x,T.nodes[i].y))
    # print()

    check = False
    num_steps = 0
    for i in range(N_ITER):
        # 2. Select a random node from the tree, called x_tree
        x_tree, tree_idx = T.rand_node_heuristic()
        if in_obs(x_tree):
            continue
        # print("x_tree: ({:.2f},{:.2f})".format(x_tree.x, x_tree.y))

        # 3. Select a random configuration, called x_rand, 
        #    from the collision-free configuration space
        x_rand = T.rand_config(x_tree)
        # print("x_rand: ({:.2f},{:.2f})".format(x_rand.x, x_rand.y))

        # 4. Extend from x_tree to x_rand (i.e. x_new)
        x_new = T.extend(x_tree, x_rand)
        new_idx = T.add_node(x_new[0])
        # print("x_new: ({:.2f},{:.2f})".format(x_new[0].x, x_new[0].y))
        # print("x_new: ({},{}), dist: {}".format(x_new[0].x, x_new[0].y, dis(x_new[0], T.end)))

        # print("dis:", dis(x_new[0], T.end))
        # print()s

        T.add_edge(new_idx, tree_idx, dis(x_new[0], x_tree))

        if dis(x_new[0], T.end) < RADIUS:
            # T.vec2idx[T.end] = x_rand # ERROR idx를 넣어줘야 함
            end_idx = T.add_node(T.end)

            # 5. Add the new nodes from the extend operation to the tree
            T.add_edge(x_new[1], end_idx, dis(x_new[0], T.end))
            # T.extend(x_new, T.end)

            path = dijkstra(T)
            # path.append(x_new) 

            # print("path:", path)
            
            # 6. If a goal configuration is added to the tree, terminate. 
            #    Solution found!
            if len(path) > 0:
                execution_time = time.time() - start
                print("[INFO] Solution Found")
                # print("path:", path)
                plot(ax, T, path)
                # for i,p in enumerate(path):
                    # print(i, "({},{})".format(p.x, p.y))
                check = True
                fig.savefig("1-2.png")
                print("Execution Time:", execution_time)
                print("Number of Steps:", num_steps)
                break
            else:
                # 7. Otherwise, go to step 2 
                continue
    
        plot(ax, T, dijkstra(T))
        num_steps += 1
    
    if check:
        plot(ax, T, path)
        time.sleep(2)


main()