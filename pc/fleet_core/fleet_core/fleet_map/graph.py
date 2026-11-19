# ---- 다익스트라 경로 탐색 ----
from __future__ import annotations

import heapq
import math


# ---- start에서 goal까지 최단 edge 순서 찾기 ----
def shortest_path(site, start: str, goal: str, cost=None) -> list | None:
    if start == goal:
        return []
    price = cost or (lambda edge: edge.price_m)
    best = {start: 0.0}
    came: dict[str, object] = {}
    todo = [(0.0, start)]
    while todo:
        spent, node = heapq.heappop(todo)
        if node == goal:
            break
        if spent > best.get(node, math.inf):
            continue
        for edge in site.edges_from(node):
            step = price(edge)
            if math.isinf(step):
                continue
            total = spent + step
            if total < best.get(edge.b, math.inf) - 1e-12:
                best[edge.b] = total
                came[edge.b] = edge
                heapq.heappush(todo, (total, edge.b))
    if goal not in came:
        return None
    way = []
    node = goal
    while node != start:
        way.append(came[node])
        node = came[node].a
    return way[::-1]


# ---- start에서 goal까지 실제 최단 길이 ----
def distance(site, start: str, goal: str) -> float:
    way = shortest_path(site, start, goal, lambda edge: edge.length)
    return math.inf if way is None else sum(edge.length for edge in way)
