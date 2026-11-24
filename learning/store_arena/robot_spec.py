"""시뮬레이터 없이 쓰는 스크립트 플릿 로봇(4c): HALL 노드 그래프를 달리는 Pode 크기 박스.

경로는 시작 노드에서 목표 노드까지 nav 링크 최단 경로이고, 목표에서 쉬고 다음 경로로 이어진다.
리셋 때 꼭짓점 시간이 있는 폴리라인으로 깔아 두며, 기다림은 시계를 멈추고 후퇴는 거꾸로 돈다.
"""

from dataclasses import dataclass

import numpy as np
import torch
from arena_spec import ts
from mover_spec import MoverShape, point_segment_distance, surface_gap

# Pode 발자국 (Gazebo nav_bench), 최고속도는 nav_speed.yaml
ROBOT = MoverShape("robot", "box", (0.12, 0.112, 0.20), ts.NAV_SPEED_MPS)


@dataclass(frozen=True)
class NodeGraph:
    names: tuple[str, ...]
    xy: np.ndarray  # (K, 2)
    path: np.ndarray  # (K, K, P, 2) a -> b 최단 경로 노드점, b 로 채움
    npts: np.ndarray  # (K, K) 쓰인 점 수 (a == b 면 1)
    length: np.ndarray  # (K, K) m
    path_node: np.ndarray  # (K, K, P) 경로점의 노드 인덱스, b 로 채움


def node_graph(site, zone: str = "HALL") -> NodeGraph:
    """zone 의 nav 링크가 있는 노드와 그 사이 모든 쌍 최단 경로."""
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
    """그래프 배열을 텐서로."""
    return {
        "xy": torch.as_tensor(g.xy, dtype=torch.float32, device=device),
        "path": torch.as_tensor(g.path, dtype=torch.float32, device=device),
        "npts": torch.as_tensor(g.npts, device=device),
        "path_node": torch.as_tensor(g.path_node, device=device),
    }


def sample_chains(
    graph_t: dict, n: int, legs: int, banned: torch.Tensor, gen=None, start_banned=None
):
    """env 별 경로 체인의 노드 인덱스 (n, legs + 1): 시작과 목표 legs 개, 앞과 달라야 하고 banned (n, K) 는 제외.
    허용 노드가 둘 미만인 행은 한 노드에 머무는 체인. start_banned (n, K) 는 시작에서만 추가 제외."""
    allowed = (~banned).float()
    w = allowed
    if start_banned is not None:
        ws = allowed * (~start_banned).float()
        w = torch.where(ws.sum(-1, keepdim=True) > 0, ws, allowed)
    w = w + 1e-9 * (w.sum(-1, keepdim=True) == 0)  # multinomial 은 양수 행이 필요
    nodes = [torch.multinomial(w, 1, generator=gen)[:, 0]]
    for _ in range(legs):
        wl = allowed.clone()
        wl[torch.arange(n), nodes[-1]] = 0.0
        stay = wl.sum(-1) == 0
        wl[stay, nodes[-1][stay]] = 1.0
        nodes.append(torch.multinomial(wl, 1, generator=gen)[:, 0])
    return torch.stack(nodes, -1)


def chain_route(graph_t: dict, chain: torch.Tensor, speed: torch.Tensor, pause: torch.Tensor):
    """경로 체인을 꼭짓점 시간이 있는 폴리라인으로 깐다.
    chain (n, L+1) 노드, speed (n,) m/s, pause (n, L) s. 돌려주는 dict: pts, time, arc, leg_end_arc (경로 끝까지의 호장), V = 1 + L * (P + 1).
    경로는 목표로 채우고 목표 사본 하나가 정지 시간을 담는다."""
    path = graph_t["path"]
    n, legs = chain.shape[0], chain.shape[1] - 1
    p = path.shape[2]
    pts = [graph_t["xy"][chain[:, 0]][:, None]]
    vnode = [chain[:, :1]]
    for leg in range(legs):
        seg = path[chain[:, leg], chain[:, leg + 1]]  # (n, P, 2), 구간 시작 노드부터
        pts.append(seg)
        pts.append(seg[:, -1:])  # 정지용 사본
        if "path_node" in graph_t:
            sn = graph_t["path_node"][chain[:, leg], chain[:, leg + 1]]
            vnode += [sn, sn[:, -1:]]
    pts = torch.cat(pts, 1)  # (n, 1 + L * (P + 1), 2)
    step = (pts[:, 1:] - pts[:, :-1]).norm(dim=-1)  # (n, V - 1)
    dt = step / speed[:, None].clamp(min=1e-6)
    pause_slot = torch.arange(1, legs + 1, device=chain.device) * (p + 1)  # 사본 인덱스
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
    if "path_node" in graph_t:  # 노드 예약 규칙용 노드 정보
        vnode = torch.cat(vnode, 1)  # (n, V) 꼭짓점별 노드
        goal_of = torch.cat((chain[:, :1], chain[:, 1:][:, leg_of[1:]]), 1)
        is_goal = vnode == goal_of
        is_goal[:, 0] = False  # 시작 노드는 목표 아님
        # 다음에 다른 위치가 되는 첫 꼭짓점의 노드 (-1: 없음), 통과용
        moved = (pts[:, 1:] - pts[:, :-1]).norm(dim=-1) > 1e-6  # (n, V - 1)
        nxt = torch.full_like(vnode, -1)
        for v in range(vnode.shape[1] - 2, -1, -1):
            nxt[:, v] = torch.where(moved[:, v], vnode[:, v + 1], nxt[:, v + 1])
        route.update(vnode=vnode, is_goal=is_goal, next_node=nxt)
    return route


def route_nodes_held(node_arc: torch.Tensor, done_arc: torch.Tensor, release_m: float):
    """(n, K) 우리 로봇이 잡은 노드: node_arc (n, K) 경로가 노드를 지나는 호장 (inf: 안 지남), done_arc (n,) 지나온 길이."""
    return torch.isfinite(node_arc) & (node_arc + release_m > done_arc[:, None])


def robot_nodes(route: dict, t: torch.Tensor):
    """시계 t 에서 로봇이 잡은 노드 (n,) 각 -1 은 없음: 있는 곳/떠난 곳, 가는 링크 끝, 목표가 아니면 그 다음 노드."""
    _, i = _interp(route["time"], t, route["arc"][..., None])
    rows = torch.arange(len(t), device=t.device)
    moving = (route["pts"][rows, i] - route["pts"][rows, i - 1]).norm(dim=-1) > 1e-6
    in_link = moving & (t > route["time"][rows, i - 1] + 1e-6)
    cur = route["vnode"][rows, i - 1]
    none = torch.full_like(cur, -1)
    nxt = torch.where(in_link, route["vnode"][rows, i], none)
    passing = in_link & ~route["is_goal"][rows, i]
    return cur, nxt, torch.where(passing, route["next_node"][rows, i], none)


def _interp(xs: torch.Tensor, x: torch.Tensor, ys: torch.Tensor):
    """행별 구간선형 보간 ys(x): xs (n, V), x (n,), ys (n, V, D) -> (n, D) 와 끝 꼭짓점 인덱스 (n,)."""
    v = xs.shape[1]
    i = torch.searchsorted(xs.contiguous(), x[:, None].contiguous(), right=True)[:, 0]
    i = i.clamp(1, v - 1)
    rows = torch.arange(xs.shape[0], device=xs.device)
    x0, x1 = xs[rows, i - 1], xs[rows, i]
    f = ((x - x0) / (x1 - x0).clamp(min=1e-9)).clamp(0.0, 1.0)
    return ys[rows, i - 1] + f[:, None] * (ys[rows, i] - ys[rows, i - 1]), i


def robot_pose(route: dict, t: torch.Tensor):
    """시계 t 의 위치 (n, 2), yaw (n,), 이동 호장 (n,)."""
    ys = torch.cat((route["pts"], route["arc"][..., None]), -1)
    out, i = _interp(route["time"], t, ys)
    pos, arc = out[:, :2], out[:, 2]
    yaw = route["heading"][torch.arange(len(t), device=t.device), i - 1]
    return pos, yaw, arc


def route_heading(pts: torch.Tensor) -> torch.Tensor:
    """pts (n, V, 2) 구간별 진행 방향 (n, V - 1); 길이 0 은 앞 방향 유지."""
    d = pts[:, 1:] - pts[:, :-1]
    h = torch.atan2(d[..., 1], d[..., 0])
    moving = d.norm(dim=-1) > 1e-6
    for j in range(1, h.shape[1]):
        h[:, j] = torch.where(moving[:, j], h[:, j], h[:, j - 1])
    for j in range(h.shape[1] - 2, -1, -1):
        h[:, j] = torch.where(moving[:, : j + 1].any(-1), h[:, j], h[:, j + 1])
    return h


def intent_points(route: dict, t: torch.Tensor, spacing_m: float, points: int) -> torch.Tensor:
    """로봇 앞 경로를 따라 1..points * spacing_m 지점 (n, points, 2), 목표에서 멈춤."""
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
    """스크립트 로봇의 제어 한 스텝: 시계 t, 대기 시간 wait, 후퇴 남은 시간 back (모두 (n,)).

    다음 스텝에 우리 로봇 중심과 yield_m 이내로 가까워지면 기다리고, wait_s 기다리면 경로를 따라 후퇴한 뒤 다시 간다.
    lock_m > 0 이면 우리 로봇이 다음 링크에 가까울 때 노드에서 대기한다.
    blocked (n, K) 는 fleet_manager 식 노드 예약: 예약된 노드는 지나가기만 하고 링크 끝이 비어야 들어간다.
    경로 꼭짓점에 노드가 있어야 한다 (chain_route + graph_tensors). 반환 (t, wait, back)."""
    t_end = route["time"][:, -1]
    backing = back > 0
    direction = torch.where(backing, -1.0, 1.0)
    t1 = torch.minimum((t + direction * dt).clamp(min=0.0), t_end)
    if retreat:  # 후퇴 목적지 노드에서 멈춤
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
    entering = t <= route["time"][rows, i1 - 1] + 1e-6  # 링크 첫 노드 이전
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
        # 노드를 넘을 스텝은 바로 앞이 아니라 노드에서 멈춤
        t_lock = torch.where(res, route["time"][rows, i1 - 1], t)
        # 남이 잡은 노드 node_m 이내에서는 양보 정지 안 함
        _, i0 = _interp(route["time"], t, route["arc"][..., None])
        on_taken = torch.zeros_like(held)
        for v in (i0 - 1, i0):
            near = (p0 - route["pts"][rows, v]).norm(dim=-1) < node_m
            on_taken = on_taken | (near & taken(route["vnode"][rows, v]))
        held = held & ~on_taken
    held = held & ~locked  # 잠금 대기는 양보 대기가 아님: 후퇴를 시작하지 않음
    wait = torch.where(held, wait + dt, torch.where(locked, wait, torch.zeros_like(wait)))
    start_back = held & (wait >= wait_s)
    t_old = t
    t = torch.where(locked, t_lock, torch.where(held, t, t1))
    if retreat:  # 경로상 5 cm 이상 뒤의 마지막 꼭짓점으로
        behind = route["arc"] <= (arc0 - 0.05)[:, None]
        k = (behind.long().cumsum(-1) * behind).argmax(-1)  # 마지막 True (없으면 0)
        target = torch.where(behind.any(-1), route["time"].gather(1, k[:, None])[:, 0], 0.0)
        new_back = (t - target).clamp(min=0.0)
    else:
        new_back = torch.full_like(back, backoff_s)
    # 후퇴는 우리 로봇이 길을 잡고 있어도 자기 노드에서 끝남
    used = (t_old - t).clamp(min=0.0) if retreat else torch.full_like(back, dt)
    back = torch.where(start_back, new_back, (back - used).clamp(min=0.0))
    wait = torch.where(start_back, torch.zeros_like(wait), wait)
    return t, wait, back


def robot_gap(our_xy: torch.Tensor, pos: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """(n,) 우리 로봇 중심에서 스크립트 로봇 박스 표면까지 거리."""
    return surface_gap(ROBOT, our_xy, pos, yaw)


def to_body(points: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """env 프레임 점 (n, ..., 2) 을 우리 로봇 프레임 (x 앞, y 왼쪽) 으로."""
    d = points - xy.view(-1, *([1] * (points.dim() - 2)), 2)
    c = torch.cos(yaw).view(-1, *([1] * (points.dim() - 2)))
    s = torch.sin(yaw).view(-1, *([1] * (points.dim() - 2)))
    return torch.stack((c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]), -1)

