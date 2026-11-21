"""Routes in the store arena without a simulator: start/goal nodes, global paths, static obstacles,
and following a path (progress, remaining distance, lookahead point).

Global path: like the robot's Nav2 global planner (NavFn, Dijkstra) on its global costmap
(pode/pode_nav_config/params/nav2_params.yaml): 2.5 cm cells, cells within the inscribed radius
of an obstacle are blocked, inflation cost 252 * exp(-scaling * (d - inscribed)) out to the
inflation radius, NavFn traversal cost 50 + 0.8 * cost. Here an 8-connected Dijkstra over cell
centres (NavFn propagates a potential on 4 neighbours and descends its gradient, so its paths
are a little smoother). The real global costmap also has a lidar obstacle layer, so the static
boxes of an episode are part of the map its path is planned on.

Everything is planned once when the env is built: for every allowed (start, goal) node pair a set
of obstacle layouts, each with its own path, stored as padded tensors (route = pair x layout).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import torch
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

LETHAL = 254
INSCRIBED = 253


@dataclass
class Routes:
    """Padded route tensors. R routes = pairs x layouts (route r = pair r // M, layout r % M)."""

    node_names: list[str]
    pairs: list[tuple[int, int]]  # (start node, goal node) indices into node_names
    node_xy: np.ndarray  # (nodes, 2)
    layouts: int  # M
    obstacle_sizes: tuple[float, ...]  # side of the square box in each slot [m]
    obstacle_xy: np.ndarray  # (R, K, 2) box centres; inactive slots NaN
    path: np.ndarray  # (R, L, 2), padded with the goal
    npts: np.ndarray  # (R,)
    cum: np.ndarray  # (R, L) arc length from the start, padded with the total
    total: np.ndarray  # (R,)

    @property
    def count(self) -> int:
        return len(self.total)

    def start_goal(self, r: int) -> tuple[np.ndarray, np.ndarray]:
        s, g = self.pairs[r // self.layouts]
        return self.node_xy[s], self.node_xy[g]


def point_rect_distance(px: np.ndarray, py: np.ndarray, rects: np.ndarray) -> np.ndarray:
    """Distance from points (...) to the nearest of rects (B, 4); 0 inside."""
    x0, y0, x1, y1 = (rects[:, i] for i in range(4))
    dx = np.maximum(np.maximum(x0 - px[..., None], 0.0), px[..., None] - x1)
    dy = np.maximum(np.maximum(y0 - py[..., None], 0.0), py[..., None] - y1)
    return np.sqrt(dx * dx + dy * dy).min(axis=-1)


class Grid:
    """Cell centres over the arena bounds and the Nav2-like cost of each cell."""

    def __init__(self, bounds, cell: float):
        x0, y0, x1, y1 = bounds
        self.cell = cell
        self.x0, self.y0 = x0, y0
        self.nx = int(math.ceil((x1 - x0) / cell - 1e-9))
        self.ny = int(math.ceil((y1 - y0) / cell - 1e-9))
        ix, iy = np.meshgrid(np.arange(self.nx), np.arange(self.ny), indexing="ij")
        self.cx = x0 + (ix + 0.5) * cell
        self.cy = y0 + (iy + 0.5) * cell
        self.bounds = bounds
        # 8-connected neighbour offsets and step lengths [cells]
        self.offsets = [(1, 0), (0, 1), (1, 1), (1, -1)]

    def index(self, xy) -> int:
        i = min(max(int((xy[0] - self.x0) / self.cell), 0), self.nx - 1)
        j = min(max(int((xy[1] - self.y0) / self.cell), 0), self.ny - 1)
        return i * self.ny + j

    def centre(self, flat: np.ndarray) -> np.ndarray:
        return np.stack((self.cx.ravel()[flat], self.cy.ravel()[flat]), -1)

    def costs(self, rects: np.ndarray, inscribed: float, inflation: float, scaling: float):
        """Nav2 costmap value per cell for obstacles rects (B, 4); outside bounds is lethal."""
        d = point_rect_distance(self.cx, self.cy, rects)
        c = np.where(
            d <= inflation, 252.0 * np.exp(-scaling * np.maximum(d - inscribed, 0.0)), 0.0
        )
        c = np.where(d <= inscribed, INSCRIBED, c)
        c = np.where(d <= 0.0, LETHAL, c)
        return c

    def graph(self, cost: np.ndarray):
        """Sparse 8-connected graph; edge weight = mean NavFn cell cost x step length [cells]."""
        free = cost < INSCRIBED
        trav = 50.0 + 0.8 * cost
        idx = np.arange(self.nx * self.ny).reshape(self.nx, self.ny)
        rows, cols, w = [], [], []
        for di, dj in self.offsets:
            i0, i1 = max(0, -di), self.nx - max(0, di)
            j0, j1 = max(0, -dj), self.ny - max(0, dj)
            a = idx[i0:i1, j0:j1].ravel()
            b = idx[i0 + di : i1 + di, j0 + dj : j1 + dj].ravel()
            ok = free.ravel()[a] & free.ravel()[b]
            a, b = a[ok], b[ok]
            step = math.hypot(di, dj)
            ww = 0.5 * (trav.ravel()[a] + trav.ravel()[b]) * step
            rows += [a, b]
            cols += [b, a]
            w += [ww, ww]
        n = self.nx * self.ny
        data = (np.concatenate(w), (np.concatenate(rows), np.concatenate(cols)))
        return coo_matrix(data, (n, n)).tocsr()


def plan(grid: Grid, cost: np.ndarray, start, goal) -> np.ndarray | None:
    """Global path start -> goal as (L, 2) points (start, cell centres, goal), or None."""
    s, g = grid.index(start), grid.index(goal)
    if cost.ravel()[s] >= INSCRIBED or cost.ravel()[g] >= INSCRIBED:
        return None
    dist, pred = dijkstra(grid.graph(cost), indices=g, return_predecessors=True)
    if not np.isfinite(dist[s]):
        return None
    cells = [s]
    while cells[-1] != g:
        cells.append(int(pred[cells[-1]]))
    pts = grid.centre(np.array(cells))
    return np.concatenate((np.asarray(start)[None], pts[1:-1], np.asarray(goal)[None]), 0)


def hall_nodes(site, excluded_kinds: str) -> tuple[list[str], np.ndarray]:
    """Nodes inside zone HALL whose kind is not in the comma-separated excluded_kinds."""
    skip = {k.strip() for k in excluded_kinds.split(",") if k.strip()}
    names = [k for k, n in site.nodes.items() if n.zone == "HALL" and n.kind not in skip]
    return names, np.array([(site.nodes[k].x, site.nodes[k].y) for k in names])


def box_rect(xy, side: float) -> np.ndarray:
    h = 0.5 * side
    return np.array((xy[0] - h, xy[1] - h, xy[0] + h, xy[1] + h))


def sample_layout(rng, spec, bounds, static_rects, path0, start, goal, sizes) -> np.ndarray:
    """Box centres (K, 2) for one episode layout; NaN = slot not used. Each used box sits on the
    obstacle-free path (with probability obstacle_on_path_prob, shifted sideways by up to
    obstacle_lateral_m) or anywhere in the arena, clear of walls/shelves, other boxes and, by
    obstacle_node_clear_m, of the start and goal."""
    k = len(sizes)
    out = np.full((k, 2), np.nan)
    n_used = int(rng.integers(0, int(spec.obstacle_max) + 1))
    x0, y0, x1, y1 = bounds
    se = np.array([start, goal])
    seg = np.diff(path0, axis=0)
    cum = np.concatenate(([0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))))
    for slot in rng.permutation(k)[:n_used]:
        side = sizes[slot]
        for _ in range(50):
            if rng.random() < spec.obstacle_on_path_prob and cum[-1] > 0:
                s = rng.uniform(0.0, cum[-1])
                j = min(int(np.searchsorted(cum, s)), len(path0) - 1)
                xy = path0[j] + rng.uniform(-spec.obstacle_lateral_m, spec.obstacle_lateral_m, 2)
            else:
                xy = np.array((rng.uniform(x0, x1), rng.uniform(y0, y1)))
            r = box_rect(xy, side)
            if point_rect_distance(se[:, 0], se[:, 1], r[None]).min() < spec.obstacle_node_clear_m:
                continue
            others = [box_rect(out[i], sizes[i]) for i in range(k) if not np.isnan(out[i, 0])]
            if rects_overlap(r, np.array(list(static_rects) + others)):
                continue
            out[slot] = xy
            break
    return out


def rects_overlap(r: np.ndarray, rects: np.ndarray) -> bool:
    if len(rects) == 0:
        return False
    hit = (r[0] < rects[:, 2]) & (r[2] > rects[:, 0]) & (r[1] < rects[:, 3]) & (r[3] > rects[:, 1])
    return bool(hit.any())


def build_routes(spec, site, static_rects: np.ndarray) -> Routes:
    """All routes for the arena: allowed node pairs x spec.obstacle_layouts layouts, each with the
    global path planned around its boxes. Deterministic for a given spec (spec.route_seed)."""
    rng = np.random.default_rng(int(spec.route_seed))
    names, node_xy = hall_nodes(site, spec.excluded_node_kinds)
    grid = Grid(site.bounds(), spec.grid_m)
    sizes = tuple(float(s) for s in spec.obstacle_sizes)
    cost0 = grid.costs(static_rects, spec.inscribed_m, spec.inflation_m, spec.cost_scaling)
    pairs, obstacle_xy, paths = [], [], []
    for a in range(len(names)):
        for b in range(len(names)):
            if a == b or np.hypot(*(node_xy[a] - node_xy[b])) < spec.min_start_goal_m:
                continue
            path0 = plan(grid, cost0, node_xy[a], node_xy[b])
            if path0 is None:
                continue
            pairs.append((a, b))
            for m in range(int(spec.obstacle_layouts)):
                for _ in range(20):  # resample the layout until a path exists
                    if m == 0:  # layout 0 of every pair has no boxes
                        boxes = np.full((len(sizes), 2), np.nan)
                    else:
                        boxes = sample_layout(
                            rng, spec, site.bounds(), static_rects, path0, node_xy[a], node_xy[b],
                            sizes,
                        )
                    used = [
                        box_rect(boxes[i], sizes[i])
                        for i in range(len(sizes))
                        if not np.isnan(boxes[i, 0])
                    ]
                    if not used:
                        p = path0
                    else:
                        rects = np.concatenate((static_rects, np.array(used)))
                        cost = grid.costs(
                            rects, spec.inscribed_m, spec.inflation_m, spec.cost_scaling
                        )
                        p = plan(grid, cost, node_xy[a], node_xy[b])
                    if p is not None:
                        break
                else:
                    boxes, p = np.full((len(sizes), 2), np.nan), path0
                obstacle_xy.append(boxes)
                paths.append(p)
    lmax = max(len(p) for p in paths)
    r = len(paths)
    path = np.zeros((r, lmax, 2))
    cum = np.zeros((r, lmax))
    npts = np.array([len(p) for p in paths])
    for i, p in enumerate(paths):
        seg = np.diff(p, axis=0)
        c = np.concatenate(([0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))))
        path[i, : len(p)] = p
        path[i, len(p) :] = p[-1]
        cum[i, : len(p)] = c
        cum[i, len(p) :] = c[-1]
    return Routes(
        node_names=names,
        pairs=pairs,
        node_xy=node_xy,
        layouts=int(spec.obstacle_layouts),
        obstacle_sizes=sizes,
        obstacle_xy=np.array(obstacle_xy),
        path=path,
        npts=npts,
        cum=cum,
        total=cum[:, -1].copy(),
    )


def follow(
    path: torch.Tensor,
    cum: torch.Tensor,
    npts: torch.Tensor,
    prog: torch.Tensor,
    xy: torch.Tensor,
    lookahead: float,
    window: int,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Per env: path (N, L, 2), cum (N, L), npts (N,), progress index prog (N,), robot xy (N, 2).

    Returns (new progress index, remaining distance, lookahead point). The progress index never
    goes back: it moves to the nearest path point among prog .. prog + window. The robot is
    projected onto the segment from that point to the next; remaining distance = path length after
    the projection + distance from the robot to it. The lookahead point is the first path point at
    least `lookahead` metres of path beyond the progress point (the goal near the end)."""
    n, lmax = cum.shape
    j = torch.arange(lmax, device=xy.device)[None, :]
    d = (path - xy[:, None, :]).norm(dim=-1)
    ok = (j >= prog[:, None]) & (j <= prog[:, None] + window) & (j < npts[:, None])
    near = torch.where(ok, d, torch.full_like(d, math.inf)).argmin(dim=-1)
    prog = torch.maximum(prog, near)
    rows = torch.arange(n, device=xy.device)
    p0 = path[rows, prog]
    nxt = torch.minimum(prog + 1, npts - 1)
    seg = path[rows, nxt] - p0
    seg_len = seg.norm(dim=-1)
    t = ((xy - p0) * seg).sum(-1) / seg_len.clamp(min=1e-9) ** 2
    t = torch.where(seg_len > 0, t.clamp(0.0, 1.0), torch.zeros_like(t))
    proj = p0 + t[:, None] * seg
    total = cum[rows, npts - 1]
    remaining = total - cum[rows, prog] - t * seg_len + (xy - proj).norm(dim=-1)
    target = cum[rows, prog] + lookahead
    la = torch.searchsorted(cum.contiguous(), target[:, None]).squeeze(-1)
    la = torch.minimum(la, npts - 1)
    return prog, remaining, path[rows, la]
