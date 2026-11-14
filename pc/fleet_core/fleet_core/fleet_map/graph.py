# ---- 다익스트라 경로 탐색 ----
from __future__ import annotations

import heapq
import math


# ---- start에서 goal까지 최단 링크 순서 찾기 ----
def route(site, start: str, goal: str, cost=None) -> list | None:
    if start == goal:
        return []
    price = cost or (lambda link: link.price_m)
    best = {start: 0.0}
    came: dict[str, object] = {}
    todo = [(0.0, start)]
    while todo:
        spent, node = heapq.heappop(todo)
        if node == goal:
            break
        if spent > best.get(node, math.inf):
            continue
        for link in site.links_from(node):
            step = price(link)
            if math.isinf(step):
                continue
            total = spent + step
            if total < best.get(link.b, math.inf) - 1e-12:
                best[link.b] = total
                came[link.b] = link
                heapq.heappush(todo, (total, link.b))
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
    way = route(site, start, goal, lambda link: link.length)
    return math.inf if way is None else sum(link.length for link in way)
