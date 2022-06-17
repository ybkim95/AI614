from collections import deque

def dijkstra(T):
    src_idx = T.vec2idx[T.start]
    try:
        dst_idx = T.vec2idx[T.end]
    except:
        dst_idx = 0

    nodes = list(T.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[src_idx] = 0

    while nodes:
        cur_node = min(nodes, key=lambda node: dist[node])
        nodes.remove(cur_node)
        if dist[cur_node] == float('inf'):
            break

        for neighbor, cost in T.neighbors[cur_node]:
            new_cost = dist[cur_node] + cost
            if new_cost < dist[neighbor]:
                dist[neighbor] = new_cost
                prev[neighbor] = cur_node

    path = deque()
    cur_node = dst_idx
    while prev[cur_node] is not None:
        path.appendleft(T.nodes[cur_node])
        cur_node = prev[cur_node]
    path.appendleft(T.nodes[cur_node])
    return list(path)