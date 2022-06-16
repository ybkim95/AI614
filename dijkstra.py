from collections import deque

# def dijkstra(T):
#     srcIdx = T.vec2idx[T.start]
#     try:
#         dstIdx = T.vec2idx[T.end]
#     except:
#         dstIdx = 0

#     # build dijkstra
#     nodes = list(T.neighbors.keys())
#     dist = {node: float('inf') for node in nodes}
#     prev = {node: None for node in nodes}
#     dist[srcIdx] = 0

#     while nodes:
#         curNode = min(nodes, key=lambda node: dist[node])
#         nodes.remove(curNode)
#         if dist[curNode] == float('inf'):
#             break

#         for neighbor, cost in T.neighbors[curNode]:
#             newCost = dist[curNode] + cost
#             if newCost < dist[neighbor]:
#                 dist[neighbor] = newCost
#                 prev[neighbor] = curNode

#     # retrieve path
#     path = deque()
#     curNode = dstIdx
#     # print("curNode: ({:.2f},{:.2f})".format(curNode.x, curNode.y))
#     while prev[curNode] is not None:
#         path.appendleft(curNode) #(T.nodes[curNode])
#         curNode = prev[curNode]
#     path.appendleft(curNode) #(T.nodes[curNode])
#     return list(path)


def dijkstra(G):
    srcIdx = G.vec2idx[G.start]
    try:
        dstIdx = G.vec2idx[G.end]
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
        path.appendleft(G.nodes[curNode])
        curNode = prev[curNode]
    path.appendleft(G.nodes[curNode])
    return list(path)