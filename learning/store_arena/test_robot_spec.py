"""스크립트 플릿 로봇 테스트(시뮬 없이): HALL 그래프, 경로 체인, 이동, 대기, 후퇴, intent 점."""

import math
import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import mover_spec as M  # noqa: E402
import robot_spec as RS  # noqa: E402


@pytest.fixture(scope="module")
def graph():
    site = A.load_site(A.ARENA)
    g = RS.node_graph(site)
    static = np.array([(b.x0, b.y0, b.x1, b.y1) for b in A.arena_boxes(A.ARENA, site)])
    return site, g, static


def _t(g):
    return {
        "xy": torch.tensor(g.xy, dtype=torch.float32),
        "path": torch.tensor(g.path, dtype=torch.float32),
        "npts": torch.tensor(g.npts),
    }


def test_graph_is_the_hall_nav_links(graph):
    site, g, static = graph
    assert set(g.names) == {"E-1", "E-2", "E-3", "M_E", "O_IN", "O_OUT", "Q2"}  # Q1: 고정 경로
    assert np.isfinite(g.length).all()
    # 모든 직선 링크는 지나가는 로봇 몸체(반폭 0.056)가 벽/선반에서 떨어짐
    k = len(g.names)
    for i in range(k):
        for j in range(k):
            pts = g.path[i, j, : g.npts[i, j]]
            for a, b in zip(pts[:-1], pts[1:], strict=True):
                s = np.linspace(0, 1, 50)[:, None]
                p = torch.tensor(a + s * (b - a), dtype=torch.float32)[None]
                d = M.point_rect_distance(p, torch.tensor(static, dtype=torch.float32)[None])
                assert d.min() > 0.056
    # O_OUT -> Q2 는 선반이 아니라 마당 입구를 지남
    i, j = g.names.index("O_OUT"), g.names.index("Q2")
    assert g.npts[i, j] >= 3


def test_chains_avoid_banned_nodes_and_never_repeat(graph):
    _, g, _ = graph
    gt = _t(g)
    n, k = 500, len(g.names)
    banned = torch.zeros(n, k, dtype=torch.bool)
    banned[:, 0] = True
    banned[torch.arange(n), torch.randint(1, k, (n,))] = True
    ch = RS.sample_chains(gt, n, 4, banned, torch.Generator().manual_seed(0))
    assert not banned.gather(1, ch).any()
    assert (ch[:, 1:] != ch[:, :-1]).all()


def _line_route(speed=0.1, pause=1.0):
    # 두 노드 그래프: A (0, 0) -- B (1, 0), 체인 A -> B -> A
    gt = {
        "xy": torch.tensor([[0.0, 0.0], [1.0, 0.0]]),
        "path": torch.tensor([[[[0.0, 0.0], [0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0]]],
                              [[[1.0, 0.0], [0.0, 0.0]], [[1.0, 0.0], [1.0, 0.0]]]]),
        "npts": torch.tensor([[1, 2], [2, 1]]),
    }
    chain = torch.tensor([[0, 1, 0]])
    return RS.chain_route(gt, chain, torch.tensor([speed]), torch.tensor([[pause, pause]]))


def test_robot_drives_its_chain_with_pauses():
    r = _line_route()
    # A -> B 10 s, 정지 1 s, B -> A 10 s, 정지 1 s
    for t, x, yaw in ((0.0, 0.0, 0.0), (5.0, 0.5, 0.0), (10.5, 1.0, 0.0), (16.0, 0.5, math.pi)):
        pos, h, _ = RS.robot_pose(r, torch.tensor([t]))
        assert pos[0].tolist() == pytest.approx([x, 0.0], abs=1e-5)
        assert abs(math.remainder(h.item() - yaw, 2 * math.pi)) < 1e-5
    pos, _, _ = RS.robot_pose(r, torch.tensor([99.0]))
    assert pos[0].tolist() == pytest.approx([0.0, 0.0], abs=1e-5)  # 마지막 목표에 머뭄


def test_intent_points_stop_at_the_current_goal():
    r = _line_route()
    p = RS.intent_points(r, torch.tensor([5.0]), 0.2, 3)  # x 0.5 에서 B 로 가는 중
    assert p[0, :, 0].tolist() == pytest.approx([0.7, 0.9, 1.0], abs=1e-5)  # B 를 안 넘음
    p = RS.intent_points(r, torch.tensor([10.5]), 0.2, 3)  # B 에서 정지: 경로 끝
    assert p[0, :, 0].tolist() == pytest.approx([1.0, 1.0, 1.0], abs=1e-5)


def test_robot_waits_backs_off_and_drives_on():
    r = _line_route(speed=0.15, pause=0.0)
    our = torch.tensor([[0.6, 0.0]])  # 가는 길에 서 있음
    t, wait, back = torch.zeros(1), torch.zeros(1), torch.zeros(1)
    gaps, backed = [], False
    for _ in range(200):
        t, wait, back = RS.robot_step(r, t, wait, back, 0.05, our, 0.25, 3.0, 2.0)
        pos, _, _ = RS.robot_pose(r, t)
        gaps.append((pos - our).norm().item())
        backed |= bool(back.item() > 0)
    assert min(gaps) >= 0.25 - 1e-6  # 우리 로봇 중심에 0.25 m 안으로 안 감
    assert backed and max(gaps[-60:]) > 0.25 + 0.15  # 한때 0.15 m 이상 물러남
    # 우리 로봇에서 멀어지는 로봇은 가까워도 멈추지 않음
    t2, _, _ = RS.robot_step(r, torch.tensor([3.0]), wait * 0, back * 0, 0.05,
                             torch.tensor([[0.3, 0.0]]), 0.25, 3.0, 2.0)
    assert t2.item() == pytest.approx(3.05)


def _three_node_route(speed=0.15):
    # A (0, 0) -> B (1, 0) -> C (2, 0) 한 경로 (B 는 가는 길의 노드)
    gt = {
        "xy": torch.tensor([[0.0, 0.0], [2.0, 0.0]]),
        "path": torch.tensor([[[[0.0, 0.0]] * 3, [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]],
                              [[[2.0, 0.0], [1.0, 0.0], [0.0, 0.0]], [[2.0, 0.0]] * 3]]),
        "npts": torch.tensor([[1, 3], [3, 1]]),
    }
    return RS.chain_route(gt, torch.tensor([[0, 1]]), torch.tensor([speed]), torch.zeros(1, 1))


def test_retreat_goes_back_to_the_node_before():
    r = _three_node_route()
    our = torch.tensor([[1.6, 0.0]])  # B -> C 링크 위 길목
    t, wait, back = torch.zeros(1), torch.zeros(1), torch.zeros(1)
    xs = []
    for _ in range(400):
        t, wait, back = RS.robot_step(r, t, wait, back, 0.05, our, 0.25, 3.0, 2.0, retreat=True)
        xs.append(RS.robot_pose(r, t)[0][0, 0].item())
    xs = np.array(xs)
    peak = int(np.argmax(xs > 1.34))  # 우리 로봇 0.25 m 앞에서 첫 정지
    assert xs.max() == pytest.approx(1.35, abs=0.01)
    assert xs[peak:].min() == pytest.approx(1.0, abs=0.01)  # 노드 B 까지만 물러남


def test_edge_lock_holds_the_robot_at_the_node():
    r = _three_node_route()
    our = torch.tensor([[1.5, 0.08]])  # B -> C 링크 옆, 0.25 m 양보에 걸리지 않음
    t, wait, back = torch.zeros(1), torch.zeros(1), torch.zeros(1)
    xs = []
    for _ in range(200):
        t, wait, back = RS.robot_step(r, t, wait, back, 0.05, our, 0.25, 99.0, 2.0, lock_m=0.15)
        xs.append(RS.robot_pose(r, t)[0][0, 0].item())
    assert max(xs) == pytest.approx(1.0, abs=0.01)  # 우리 로봇이 B -> C 위면 B 에서 대기
    t, wait, back = RS.robot_step(r, t, wait, back, 0.05, torch.tensor([[5.0, 5.0]]), 0.25,
                                  99.0, 2.0, lock_m=0.15)
    assert RS.robot_pose(r, t)[0][0, 0].item() > 1.0  # 링크가 비면 감


def test_timed_backoff_steps_a_whole_dt_as_before():
    r = _three_node_route()
    t, _, back = RS.robot_step(r, torch.tensor([5.0]), torch.zeros(1), torch.tensor([0.02]),
                               0.05, torch.tensor([[9.0, 9.0]]), 0.25, 3.0, 2.0)
    assert t.item() == pytest.approx(4.95) and back.item() == 0.0


def test_edge_lock_also_holds_at_the_route_start():
    r = _three_node_route()
    t, _, _ = RS.robot_step(r, torch.zeros(1), torch.zeros(1), torch.zeros(1), 0.05,
                            torch.tensor([[0.5, 0.1]]), 0.25, 3.0, 2.0, lock_m=0.15)
    assert t.item() == 0.0


class _Node:
    def __init__(self, x, y):
        self.x, self.y, self.zone = x, y, "HALL"


class _Edge:
    def __init__(self, a, b):
        self.a, self.b, self.mode = a, b, "nav_goal"


class _Site:
    """노드 A (0, 0) - B (1, 0) - C (2, 0) - D (3, 0) 일렬, 이웃끼리 링크."""

    nodes = {n: _Node(i, 0.0) for i, n in enumerate("ABCD")}

    def edges(self):
        pairs = ("AB", "BC", "CD")
        return [_Edge(a, b) for a, b in pairs] + [_Edge(b, a) for a, b in pairs]


def _line_graph():
    g = RS.node_graph(_Site())
    return g, RS.graph_tensors(g)


def test_route_vertices_know_their_nodes_and_goals():
    g, gt = _line_graph()
    a, b, c, d = (g.names.index(x) for x in "ABCD")
    r = RS.chain_route(gt, torch.tensor([[a, c, a]]), torch.tensor([0.1]), torch.ones(1, 2))
    pts_node = r["vnode"][0].tolist()
    # 모든 꼭짓점은 이름 붙은 노드 위
    for v, node in enumerate(pts_node):
        assert r["pts"][0, v].tolist() == pytest.approx(g.xy[node].tolist())
    goals = [pts_node[v] for v in range(len(pts_node)) if r["is_goal"][0, v]]
    assert set(goals) == {a, c} and b not in goals and d not in goals
    # robot_nodes: 링크 A -> B 에서는 A, B 와 B 를 지나므로 C 까지 잡음; B -> C 에서는 B 와 목표 C; C 정지 중엔 C 만
    held = RS.robot_nodes(r, torch.tensor([5.0]))
    assert [h.item() for h in held] == [a, b, c]
    held = RS.robot_nodes(r, torch.tensor([15.0]))
    assert [h.item() for h in held] == [b, c, -1]
    held = RS.robot_nodes(r, torch.tensor([20.5]))
    assert [h.item() for h in held] == [c, -1, -1]


def _run(r, blocked, steps=400, our=(9.0, 9.0)):
    t, wait, back = torch.zeros(1), torch.zeros(1), torch.zeros(1)
    xs = []
    for _ in range(steps):
        t, wait, back = RS.robot_step(r, t, wait, back, 0.05, torch.tensor([our]), 0.25, 3.0,
                                      2.0, blocked=blocked)
        xs.append(RS.robot_pose(r, t)[0][0, 0].item())
    return xs


def test_reservation_waits_at_its_node_before_a_taken_goal():
    g, gt = _line_graph()
    a, b, c = (g.names.index(x) for x in "ABC")
    r = RS.chain_route(gt, torch.tensor([[a, c]]), torch.tensor([0.15]), torch.zeros(1, 1))
    blocked = torch.zeros(1, len(g.names), dtype=torch.bool)
    blocked[0, c] = True  # 목표가 잡힘: 거기 멈출 수 없음
    blocked[0, b] = True  # B 도 잡힘: A 다음에 멈출 빈 노드 없음
    assert max(_run(r, blocked)) == pytest.approx(0.0, abs=1e-6)  # A 에 머뭄
    blocked[0, b] = False
    xs = _run(r, blocked)  # B 로 가서 대기: B 위에서 (스텝이 B 에서 끝나지 않음)
    assert max(xs) == pytest.approx(1.0, abs=1e-6) and xs[-1] == pytest.approx(1.0, abs=1e-6)


def test_waiting_robot_holds_only_the_node_it_waits_at():
    g, gt = _line_graph()
    a, b, c = (g.names.index(x) for x in "ABC")
    r = RS.chain_route(gt, torch.tensor([[a, c]]), torch.tensor([0.15]), torch.zeros(1, 1))
    blocked = torch.zeros(1, len(g.names), dtype=torch.bool)
    blocked[0, c] = True
    t, wait, back = torch.zeros(1), torch.zeros(1), torch.zeros(1)
    for _ in range(400):
        t, wait, back = RS.robot_step(r, t, wait, back, 0.05, torch.tensor([[9.0, 9.0]]), 0.25,
                                      3.0, 2.0, blocked=blocked)
    assert [h.item() for h in RS.robot_nodes(r, t)] == [b, -1, -1]


def test_no_yield_stop_on_a_taken_node():
    g, gt = _line_graph()
    a, b, c = (g.names.index(x) for x in "ABC")
    r = RS.chain_route(gt, torch.tensor([[a, c]]), torch.tensor([0.15]), torch.zeros(1, 1))
    blocked = torch.zeros(1, len(g.names), dtype=torch.bool)
    blocked[0, b] = True  # B 잡힘: 우리 로봇이 가까워도 B 를 통과
    xs = _run(r, blocked, our=(1.2, 0.0))
    # B 를 node_m (0.12) 지난 뒤 우리 로봇 앞에서 양보
    assert 1.0 + 0.12 <= max(xs) < 1.2  # (이후 후퇴: robot_backoff_s)
    assert max(_run(r, torch.zeros_like(blocked), our=(1.2, 0.0))) < 1.0  # B 가 비면 앞에서 양보


def test_route_nodes_are_ours_until_we_are_past_them():
    node_arc = torch.tensor([[0.5, float("inf"), 2.0]])  # 노드 1 은 우리 경로 위에 없음
    held = RS.route_nodes_held(node_arc, torch.tensor([0.8]), 0.2)
    assert held.tolist() == [[False, False, True]]


def test_reservation_passes_through_a_taken_node_it_need_not_stop_at():
    g, gt = _line_graph()
    a, b, c = (g.names.index(x) for x in "ABC")
    r = RS.chain_route(gt, torch.tensor([[a, c]]), torch.tensor([0.15]), torch.zeros(1, 1))
    blocked = torch.zeros(1, len(g.names), dtype=torch.bool)
    blocked[0, b] = True  # B 잡힘, 그러나 빈 목표 C 로 가며 지나가기만 함
    assert max(_run(r, blocked)) == pytest.approx(2.0, abs=1e-6)


def test_to_body_and_robot_gap():
    pts = torch.tensor([[[1.0, 1.0]]])
    b = RS.to_body(pts, torch.tensor([[1.0, 0.0]]), torch.tensor([math.pi / 2]))
    assert b[0, 0].tolist() == pytest.approx([1.0, 0.0], abs=1e-6)  # 정면 1 m
    g = RS.robot_gap(torch.tensor([[0.5, 0.0]]), torch.zeros(1, 2), torch.zeros(1))
    assert g.item() == pytest.approx(0.5 - 0.06)


def test_start_banned_moves_the_start_off_our_nodes():
    g, gt = _line_graph()
    banned = torch.zeros(64, 4, dtype=torch.bool)
    ours = torch.zeros(64, 4, dtype=torch.bool)
    ours[:, :3] = True  # A, B, C 는 우리 경로: 로봇은 D 에서 출발
    chain = RS.sample_chains(gt, 64, 2, banned, start_banned=ours)
    assert (chain[:, 0] == 3).all() and (chain[:, 1] != 3).all()


def test_robot_with_priority_does_not_wait_for_us():
    r = _line_route()
    t = torch.zeros(1)
    for _ in range(160):  # 우리 로봇은 0.6 에 서 있음: yield_m 0 이면 그 위로 감
        t, _, _ = RS.robot_step(r, t, torch.zeros(1), torch.zeros(1), 0.05,
                                torch.tensor([[0.6, 0.0]]), torch.zeros(1), 3.0, 2.0)
    assert RS.robot_pose(r, t)[0][0, 0].item() > 0.6
