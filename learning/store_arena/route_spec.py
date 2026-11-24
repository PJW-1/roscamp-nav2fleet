"""시뮬레이터 없이 쓰는 매장 아레나 경로: 시작/목표 노드, 전역 경로, 정적 장애물, 경로 추종.

전역 경로는 로봇의 Nav2 전역 코스트맵(2.5 cm 셀)에서 8방향 Dijkstra 로 계획한다.
env 생성 때 모든 (시작, 목표) 쌍 x 장애물 배치마다 한 번 계획해 패딩 텐서로 둔다 (route = 쌍 x 배치).
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
    """패딩된 경로 텐서. R 경로 = 쌍 x 배치 (경로 r = 쌍 r // M, 배치 r % M)."""

    node_names: list[str]
    pairs: list[tuple[int, int]]  # (시작 노드, 목표 노드) 인덱스
    node_xy: np.ndarray  # (nodes, 2)
    layouts: int  # M
    obstacle_sizes: tuple[float, ...]  # 슬롯별 정사각 박스 한 변 [m]
    obstacle_xy: np.ndarray  # (R, K, 2) 박스 중심, 안 쓰는 슬롯은 NaN
    path: np.ndarray  # (R, L, 2), 목표로 채움
    npts: np.ndarray  # (R,)
    cum: np.ndarray  # (R, L) 시작부터 호장, 전체 길이로 채움
    total: np.ndarray  # (R,)

    @property
    def count(self) -> int:
        return len(self.total)

    def start_goal(self, r: int) -> tuple[np.ndarray, np.ndarray]:
        s, g = self.pairs[r // self.layouts]
        return self.node_xy[s], self.node_xy[g]


def point_rect_distance(px: np.ndarray, py: np.ndarray, rects: np.ndarray) -> np.ndarray:
    """점 (...) 에서 가장 가까운 rect (B, 4) 까지 거리, 안쪽은 0."""
    x0, y0, x1, y1 = (rects[:, i] for i in range(4))
    dx = np.maximum(np.maximum(x0 - px[..., None], 0.0), px[..., None] - x1)
    dy = np.maximum(np.maximum(y0 - py[..., None], 0.0), py[..., None] - y1)
    return np.sqrt(dx * dx + dy * dy).min(axis=-1)


class Grid:
    """아레나 범위의 셀 중심과 셀별 Nav2 식 비용."""

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
        # 8방향 이웃 오프셋과 걸음 길이 [셀]
        self.offsets = [(1, 0), (0, 1), (1, 1), (1, -1)]

    def index(self, xy) -> int:
        i = min(max(int((xy[0] - self.x0) / self.cell), 0), self.nx - 1)
        j = min(max(int((xy[1] - self.y0) / self.cell), 0), self.ny - 1)
        return i * self.ny + j

    def centre(self, flat: np.ndarray) -> np.ndarray:
        return np.stack((self.cx.ravel()[flat], self.cy.ravel()[flat]), -1)

    def costs(self, rects: np.ndarray, inscribed: float, inflation: float, scaling: float):
        """장애물 rects (B, 4) 에 대한 셀별 Nav2 코스트맵 값, 범위 밖은 치명적."""
        d = point_rect_distance(self.cx, self.cy, rects)
        c = np.where(
            d <= inflation, 252.0 * np.exp(-scaling * np.maximum(d - inscribed, 0.0)), 0.0
        )
        c = np.where(d <= inscribed, INSCRIBED, c)
        c = np.where(d <= 0.0, LETHAL, c)
        return c

    def graph(self, cost: np.ndarray):
        """희소 8방향 그래프: 간선 가중치 = 평균 NavFn 셀 비용 x 걸음 길이."""
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
            if di and dj:  # 대각선: 양옆 셀도 비어야 막힌 모서리를 안 자름
                ok &= free[i0 + di : i1 + di, j0:j1].ravel()
                ok &= free[i0:i1, j0 + dj : j1 + dj].ravel()
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
    """시작에서 목표까지 전역 경로 (L, 2), 없으면 None."""
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
    """zone HALL 안에서 excluded_kinds(쉼표 구분)가 아니고 via(나눈 goal 중간 지점)도 아닌 노드."""
    skip = {k.strip() for k in excluded_kinds.split(",") if k.strip()}
    names = [k for k, n in site.nodes.items()
             if n.zone == "HALL" and n.kind not in skip and not getattr(n, "via", False)]
    return names, np.array([(site.nodes[k].x, site.nodes[k].y) for k in names])


def box_rect(xy, side: float) -> np.ndarray:
    h = 0.5 * side
    return np.array((xy[0] - h, xy[1] - h, xy[0] + h, xy[1] + h))


def sample_layout(rng, spec, bounds, static_rects, path0, start, goal, sizes) -> np.ndarray:
    """한 에피소드 배치의 박스 중심 (K, 2), NaN = 안 쓰는 슬롯.
    박스는 장애물 없는 경로 위(obstacle_on_path_prob, 옆 이동 obstacle_lateral_m) 또는 임의 위치에 놓고 벽, 선반, 다른 박스, 시작/목표에서 띄운다."""
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
    """아레나의 모든 경로: 허용 노드 쌍 x 배치, 박스를 피한 전역 경로 포함. spec.route_seed 로 결정적."""
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
                for _ in range(20):  # 경로가 생길 때까지 배치를 다시 뽑음
                    if m == 0:  # 모든 쌍의 배치 0 은 박스 없음
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
    """env 별 경로 추종: path (N, L, 2), cum (N, L), npts (N,), 진행 인덱스 prog (N,), 로봇 xy (N, 2).

    반환 (새 진행 인덱스, 남은 거리, lookahead 점). 진행 인덱스는 prog .. prog + window 중 가장 가까운 점으로만 앞으로 간다.
    lookahead 점은 진행점에서 경로로 lookahead m 이상 앞의 첫 점 (끝에서는 목표)."""
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
