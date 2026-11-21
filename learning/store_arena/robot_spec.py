"""Scripted fleet robots without a simulator (stage 4c): Pode-sized boxes that drive the fleet map's
HALL node graph as fleet_manager routes do, wait for our robot, back off if both would wait for
good, and the fleet intent (future route points) the observation gets.

A robot's episode is a chain of routes: from a start node to a goal node by the shortest path over
the map's `nav:` links (straight lines between nodes, as Nav2 goals between nodes), a pause at the
goal, then the next route. All of it is laid out at reset as one polyline with vertex times, so the
robot's position is a function of its own clock; waiting stops the clock and backing off runs it
backwards.
"""

from dataclasses import dataclass

import numpy as np
import torch
from mover_spec import MoverShape, point_segment_distance, surface_gap

ROBOT = MoverShape("robot", "box", (0.12, 0.112, 0.20), 0.15)  # Pode footprint; Gazebo nav_bench


@dataclass(frozen=True)
class NodeGraph:
    names: tuple[str, ...]
    xy: np.ndarray  # (K, 2)
    path: np.ndarray  # (K, K, P, 2) shortest path a -> b, node points, padded with b
    npts: np.ndarray  # (K, K) points used (1 when a == b)
    length: np.ndarray  # (K, K) m
    path_node: np.ndarray  # (K, K, P) node index of each path point, padded with b


def node_graph(site, zone: str = "HALL") -> NodeGraph:
    """Nodes of `zone` that have a nav link, and the all-pairs shortest paths over those links."""
    links = [
        e for e in site.edges() if e.mode == "nav_goal"
        and site.nodes[e.a].zone == zone and site.nodes[e.b].zone == zone
    ]
    names = tuple(sorted({e.a for e in links} | {e.b for e in links}))
    k = len(names)
    idx = {n: i for i, n in enumerate(names)}
    xy = np.array([(site.nodes[n].x, site.nodes[n].y) for n in names])
    dist = np.full((k, k), np.inf)
    nxt = np.full((k, k), -1)
    for i in range(k):
        dist[i, i], nxt[i, i] = 0.0, i
    for e in links:
        i, j = idx[e.a], idx[e.b]
        d = float(np.hypot(*(xy[j] - xy[i])))
        if d < dist[i, j]:
            dist[i, j], nxt[i, j] = d, j
    for m in range(k):  # Floyd-Warshall
        for i in range(k):
            for j in range(k):
                if dist[i, m] + dist[m, j] < dist[i, j] - 1e-12:
                    dist[i, j], nxt[i, j] = dist[i, m] + dist[m, j], nxt[i, m]
    if not np.isfinite(dist).all():
        raise ValueError(f"{zone} nav graph is not connected")
    seqs = {}
    for i in range(k):
        for j in range(k):
            s = [i]
            while s[-1] != j:
                s.append(int(nxt[s[-1], j]))
            seqs[i, j] = s
    p = max(len(s) for s in seqs.values())
    path = np.zeros((k, k, p, 2))
    npts = np.zeros((k, k), dtype=np.int64)
    path_node = np.zeros((k, k, p), dtype=np.int64)
    for (i, j), s in seqs.items():
        pts = xy[s]
        path[i, j] = np.concatenate((pts, np.repeat(pts[-1:], p - len(s), 0)))
        npts[i, j] = len(s)
        path_node[i, j] = s + [j] * (p - len(s))
    return NodeGraph(names, xy, path, npts, dist, path_node)


def graph_tensors(g: NodeGraph, device="cpu") -> dict:
    """The graph arrays chain_route and sample_chains take, as tensors."""
    return {
        "xy": torch.as_tensor(g.xy, dtype=torch.float32, device=device),
        "path": torch.as_tensor(g.path, dtype=torch.float32, device=device),
        "npts": torch.as_tensor(g.npts, device=device),
        "path_node": torch.as_tensor(g.path_node, device=device),
    }


def sample_chains(
    graph_t: dict, n: int, legs: int, banned: torch.Tensor, gen=None, start_banned=None
):
    """Node indices (n, legs + 1) of a route chain per env: a start node and `legs` goals, each
    different from the one before, none in `banned` (n, K) bool (our start and goal, other robots'
    starts). Env rows where fewer than two nodes are allowed get a chain that stays on one node.
    start_banned (n, K): further nodes the start may not be (where any other node is allowed)."""
    allowed = (~banned).float()
    w = allowed
    if start_banned is not None:
        ws = allowed * (~start_banned).float()
        w = torch.where(ws.sum(-1, keepdim=True) > 0, ws, allowed)
    w = w + 1e-9 * (w.sum(-1, keepdim=True) == 0)  # multinomial needs a positive row
    nodes = [torch.multinomial(w, 1, generator=gen)[:, 0]]
    for _ in range(legs):
        wl = allowed.clone()
        wl[torch.arange(n), nodes[-1]] = 0.0
        stay = wl.sum(-1) == 0
        wl[stay, nodes[-1][stay]] = 1.0
        nodes.append(torch.multinomial(wl, 1, generator=gen)[:, 0])
    return torch.stack(nodes, -1)


def chain_route(graph_t: dict, chain: torch.Tensor, speed: torch.Tensor, pause: torch.Tensor):
    """Lay out route chains as polylines with vertex times.
    chain (n, L+1) node indices, speed (n,) m/s, pause (n, L) s at each goal.
    Returns dict: pts (n, V, 2), time (n, V) s, arc (n, V) m, leg_end_arc (n, V) m (arc length of
    the end of the route a vertex belongs to: what fleet_manager knows of the route), V = 1 +
    L * (P + 1). Each route is padded with its goal; one extra copy of the goal holds the pause."""
    path = graph_t["path"]
    n, legs = chain.shape[0], chain.shape[1] - 1
    p = path.shape[2]
    pts = [graph_t["xy"][chain[:, 0]][:, None]]
    vnode = [chain[:, :1]]
    for leg in range(legs):
        seg = path[chain[:, leg], chain[:, leg + 1]]  # (n, P, 2), starts at the leg's start node
        pts.append(seg)
        pts.append(seg[:, -1:])  # the pause copy
        if "path_node" in graph_t:
            sn = graph_t["path_node"][chain[:, leg], chain[:, leg + 1]]
            vnode += [sn, sn[:, -1:]]
    pts = torch.cat(pts, 1)  # (n, 1 + L * (P + 1), 2); each leg repeats its start node first
    step = (pts[:, 1:] - pts[:, :-1]).norm(dim=-1)  # (n, V - 1)
    dt = step / speed[:, None].clamp(min=1e-6)
    pause_slot = torch.arange(1, legs + 1, device=chain.device) * (p + 1)  # index of the copy
    dt[:, pause_slot - 1] += pause
    z = torch.zeros(n, 1, device=chain.device)
    time = torch.cat((z, dt.cumsum(-1)), -1)
    arc = torch.cat((z, step.cumsum(-1)), -1)
    leg_of = torch.cat(
        (torch.zeros(1, dtype=torch.long), torch.arange(legs).repeat_interleave(p + 1))
    ).to(chain.device)
    end_arc = arc[:, pause_slot]  # (n, L)
    leg_end_arc = end_arc[:, leg_of]
    route = {
        "pts": pts,
        "time": time,
        "arc": arc,
        "leg_end_arc": leg_end_arc,
        "heading": route_heading(pts),
    }
    if "path_node" in graph_t:  # node bookkeeping for the node reservation rule
        vnode = torch.cat(vnode, 1)  # (n, V) node of every vertex
        goal_of = torch.cat((chain[:, :1], chain[:, 1:][:, leg_of[1:]]), 1)
        is_goal = vnode == goal_of
        is_goal[:, 0] = False  # the start node is not a goal
        # node of the first later vertex at another place (-1: none), for passing through
        moved = (pts[:, 1:] - pts[:, :-1]).norm(dim=-1) > 1e-6  # (n, V - 1)
        nxt = torch.full_like(vnode, -1)
        for v in range(vnode.shape[1] - 2, -1, -1):
            nxt[:, v] = torch.where(moved[:, v], vnode[:, v + 1], nxt[:, v + 1])
        route.update(vnode=vnode, is_goal=is_goal, next_node=nxt)
    return route


def route_nodes_held(node_arc: torch.Tensor, done_arc: torch.Tensor, release_m: float):
    """(n, K) nodes our robot holds: node_arc (n, K) path length at which its route passes each
    node (inf: it does not), done_arc (n,) path length behind it."""
    return torch.isfinite(node_arc) & (node_arc + release_m > done_arc[:, None])


def robot_nodes(route: dict, t: torch.Tensor):
    """Nodes a robot holds at clock t: the node it is at or last left (n,), and the node at the
    end of the link it is driving (n,; -1 when it stands at a node)."""
    _, i = _interp(route["time"], t, route["arc"][..., None])
    rows = torch.arange(len(t), device=t.device)
    moving = (route["pts"][rows, i] - route["pts"][rows, i - 1]).norm(dim=-1) > 1e-6
    in_link = moving & (t > route["time"][rows, i - 1] + 1e-6)
    cur = route["vnode"][rows, i - 1]
    return cur, torch.where(in_link, route["vnode"][rows, i], torch.full_like(cur, -1))


def _interp(xs: torch.Tensor, x: torch.Tensor, ys: torch.Tensor):
    """Piecewise-linear ys(x) per row; xs (n, V) nondecreasing, x (n,), ys (n, V, D) -> (n, D),
    and the index i of the segment's end vertex (n,)."""
    v = xs.shape[1]
    i = torch.searchsorted(xs.contiguous(), x[:, None].contiguous(), right=True)[:, 0]
    i = i.clamp(1, v - 1)
    rows = torch.arange(xs.shape[0], device=xs.device)
    x0, x1 = xs[rows, i - 1], xs[rows, i]
    f = ((x - x0) / (x1 - x0).clamp(min=1e-9)).clamp(0.0, 1.0)
    return ys[rows, i - 1] + f[:, None] * (ys[rows, i] - ys[rows, i - 1]), i


def robot_pose(route: dict, t: torch.Tensor):
    """Position (n, 2), yaw (n,) and arc length travelled (n,) at clock t (n,)."""
    ys = torch.cat((route["pts"], route["arc"][..., None]), -1)
    out, i = _interp(route["time"], t, ys)
    pos, arc = out[:, :2], out[:, 2]
    yaw = route["heading"][torch.arange(len(t), device=t.device), i - 1]
    return pos, yaw, arc


def route_heading(pts: torch.Tensor) -> torch.Tensor:
    """(n, V - 1) heading of each segment of pts (n, V, 2); a zero-length one keeps the heading
    before it (leading ones take the first real heading)."""
    d = pts[:, 1:] - pts[:, :-1]
    h = torch.atan2(d[..., 1], d[..., 0])
    moving = d.norm(dim=-1) > 1e-6
    for j in range(1, h.shape[1]):
        h[:, j] = torch.where(moving[:, j], h[:, j], h[:, j - 1])
    for j in range(h.shape[1] - 2, -1, -1):
        h[:, j] = torch.where(moving[:, : j + 1].any(-1), h[:, j], h[:, j + 1])
    return h


def intent_points(route: dict, t: torch.Tensor, spacing_m: float, points: int) -> torch.Tensor:
    """(n, points, 2) points 1..points * spacing_m ahead of the robot along its current route
    (clamped at the route's goal: fleet_manager knows the route it gave, not the next one)."""
    _, _, arc = robot_pose(route, t)
    ys = torch.cat((route["arc"][..., None], route["leg_end_arc"][..., None]), -1)
    out, _ = _interp(route["time"], t, ys)
    end = out[:, 1]
    ahead = arc[:, None] + spacing_m * torch.arange(1, points + 1, device=t.device)[None]
    ahead = torch.minimum(ahead, end[:, None])
    res = []
    for k in range(points):
        p, _ = _interp(route["arc"], ahead[:, k], route["pts"])
        res.append(p)
    return torch.stack(res, 1)


def robot_step(
    route,
    t,
    wait,
    back,
    dt,
    our_xy,
    yield_m,
    wait_s,
    backoff_s,
    retreat=False,
    lock_m=0.0,
    blocked=None,
    node_m=0.12,
):
    """One control step of a scripted robot's clock t, waiting time wait and backing-off time left
    back (all (n,)). It waits while its next step would bring its centre within yield_m of our
    robot's centre and closer (the Gazebo nav_bench robot waits at 0.25 m whatever its direction).
    After waiting wait_s it backs off along its route: for backoff_s, or with retreat back to the
    last route node at least 5 cm behind it; then drives on.
    lock_m > 0: a link lock as fleet_manager's lanes. Before driving into the next link (crossing a
    node) it waits at the node while our robot's centre is within lock_m of that link.
    blocked (n, K) bool: node reservation as fleet_manager's. Nodes reserved by others may only be
    driven through: before a link it waits at its node unless the link's end node is free, or is
    not one of its goals and the node after it is free. Held at a link it waits on its node. Nor
    does it yield within node_m of a taken node: it drives on and yields past it. Route vertices
    must carry nodes (chain_route with graph_tensors).
    Returns (t, wait, back)."""
    t_end = route["time"][:, -1]
    backing = back > 0
    direction = torch.where(backing, -1.0, 1.0)
    t1 = torch.minimum((t + direction * dt).clamp(min=0.0), t_end)
    if retreat:  # stop at the node the retreat goes to
        t1 = torch.where(backing, torch.maximum(t1, t - back), t1)
    p0, _, arc0 = robot_pose(route, t)
    p1, _, _ = robot_pose(route, t1)
    g0, g1 = (p0 - our_xy).norm(dim=-1), (p1 - our_xy).norm(dim=-1)
    held = (g1 < yield_m) & (g1 < g0 - 1e-9)
    locked = torch.zeros_like(held)
    rows = torch.arange(len(t), device=t.device)
    _, i1 = _interp(route["time"], t1, route["arc"][..., None])
    a, b = route["pts"][rows, i1 - 1], route["pts"][rows, i1]
    moving = (b - a).norm(dim=-1) > 1e-6
    entering = t <= route["time"][rows, i1 - 1] + 1e-6  # at or before the link's first node
    if lock_m > 0:
        d = point_segment_distance(our_xy, a[:, None], b[:, None])[:, 0]
        locked = ~backing & entering & moving & (d < lock_m)
    t_lock = t
    if blocked is not None:

        def taken(node):
            return blocked.gather(1, node.clamp(min=0)[:, None])[:, 0] & (node >= 0)

        after = route["next_node"][rows, i1]
        end_taken = taken(route["vnode"][rows, i1])
        no_stop = end_taken & (route["is_goal"][rows, i1] | (after < 0) | taken(after))
        res = ~backing & entering & moving & no_stop
        locked = locked | res
        # a step that would cross the node stops on it, not just short of it
        t_lock = torch.where(res, route["time"][rows, i1 - 1], t)
        # no yield stop within node_m of a node someone else holds
        _, i0 = _interp(route["time"], t, route["arc"][..., None])
        on_taken = torch.zeros_like(held)
        for v in (i0 - 1, i0):
            near = (p0 - route["pts"][rows, v]).norm(dim=-1) < node_m
            on_taken = on_taken | (near & taken(route["vnode"][rows, v]))
        held = held & ~on_taken
    held = held & ~locked  # a lock wait is not a yield wait: it never starts a back-off
    wait = torch.where(held, wait + dt, torch.where(locked, wait, torch.zeros_like(wait)))
    start_back = held & (wait >= wait_s)
    t_old = t
    t = torch.where(locked, t_lock, torch.where(held, t, t1))
    if retreat:  # back to the last vertex at least 5 cm of route behind
        behind = route["arc"] <= (arc0 - 0.05)[:, None]
        k = (behind.long().cumsum(-1) * behind).argmax(-1)  # last True (0 if none)
        target = torch.where(behind.any(-1), route["time"].gather(1, k[:, None])[:, 0], 0.0)
        new_back = (t - target).clamp(min=0.0)
    else:
        new_back = torch.full_like(back, backoff_s)
    # a retreat ends at its node, however long our robot holds it on the way
    used = (t_old - t).clamp(min=0.0) if retreat else torch.full_like(back, dt)
    back = torch.where(start_back, new_back, (back - used).clamp(min=0.0))
    wait = torch.where(start_back, torch.zeros_like(wait), wait)
    return t, wait, back


def robot_gap(our_xy: torch.Tensor, pos: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """(n,) our robot centre to the scripted robot's box surface."""
    return surface_gap(ROBOT, our_xy, pos, yaw)


def to_body(points: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """points (n, ..., 2) in the env frame -> our robot frame (x forward, y left)."""
    d = points - xy.view(-1, *([1] * (points.dim() - 2)), 2)
    c = torch.cos(yaw).view(-1, *([1] * (points.dim() - 2)))
    s = torch.sin(yaw).view(-1, *([1] * (points.dim() - 2)))
    return torch.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]), -1)

