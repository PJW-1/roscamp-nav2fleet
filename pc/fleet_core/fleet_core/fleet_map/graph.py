"""지점 그래프에서 가장 싼 길."""
from __future__ import annotations

import heapq
import math


def route(site, start: str, goal: str, cost=None) -> list | None:
    """start에서 goal까지의 연결들. 같은 지점이면 [], 길이 없으면 None.

    cost(link)가 없으면 길이로 잰다. 값이 inf인 연결은 쓰지 않는다. 같은 값이면 지점 이름
    순서로 정해 같은 지도에서 늘 같은 길이 나온다.
    """
    if start == goal:
        return []
    price = cost or (lambda link: link.length)
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
