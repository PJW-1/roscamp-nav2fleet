"""Scripted fleet robots without a simulator: the HALL nav graph, route chains, motion along them,
waiting for our robot, backing off, and the fleet intent points."""

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
    assert set(g.names) == {"E-1", "E-2", "E-3", "M_E", "O_IN", "O_OUT", "Q2"}  # Q1: fixed path
    assert np.isfinite(g.length).all()
    # every straight link keeps a passing robot body (half width 0.056) off walls and shelves
    k = len(g.names)
    for i in range(k):
        for j in range(k):
            pts = g.path[i, j, : g.npts[i, j]]
            for a, b in zip(pts[:-1], pts[1:], strict=True):
                s = np.linspace(0, 1, 50)[:, None]
                p = torch.tensor(a + s * (b - a), dtype=torch.float32)[None]
                d = M.point_rect_distance(p, torch.tensor(static, dtype=torch.float32)[None])
                assert d.min() > 0.056
    # O_OUT -> Q2 goes through the yard entrances, not through a shelf
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
    # a two-node graph: A (0, 0) -- B (1, 0); chain A -> B -> A
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
    # A -> B 10 s, pause 1 s, B -> A 10 s, pause 1 s
    for t, x, yaw in ((0.0, 0.0, 0.0), (5.0, 0.5, 0.0), (10.5, 1.0, 0.0), (16.0, 0.5, math.pi)):
        pos, h, _ = RS.robot_pose(r, torch.tensor([t]))
        assert pos[0].tolist() == pytest.approx([x, 0.0], abs=1e-5)
        assert abs(math.remainder(h.item() - yaw, 2 * math.pi)) < 1e-5
    pos, _, _ = RS.robot_pose(r, torch.tensor([99.0]))
    assert pos[0].tolist() == pytest.approx([0.0, 0.0], abs=1e-5)  # stays at its last goal


def test_intent_points_stop_at_the_current_goal():
    r = _line_route()
    p = RS.intent_points(r, torch.tensor([5.0]), 0.2, 3)  # at x 0.5 heading to B
    assert p[0, :, 0].tolist() == pytest.approx([0.7, 0.9, 1.0], abs=1e-5)  # not past B
    p = RS.intent_points(r, torch.tensor([10.5]), 0.2, 3)  # pausing at B: the route is done
    assert p[0, :, 0].tolist() == pytest.approx([1.0, 1.0, 1.0], abs=1e-5)


def test_robot_waits_backs_off_and_drives_on():
    r = _line_route(speed=0.15, pause=0.0)
    our = torch.tensor([[0.6, 0.0]])  # standing on its way
    t, wait, back = torch.zeros(1), torch.zeros(1), torch.zeros(1)
    gaps, backed = [], False
    for _ in range(200):
        t, wait, back = RS.robot_step(r, t, wait, back, 0.05, our, 0.25, 3.0, 2.0)
        pos, _, _ = RS.robot_pose(r, t)
        gaps.append((pos - our).norm().item())
        backed |= bool(back.item() > 0)
    assert min(gaps) >= 0.25 - 1e-6  # never drove within 0.25 m of our robot's centre
    assert backed and max(gaps[-60:]) > 0.25 + 0.15  # backed off at least 0.15 m at some point
    # a robot driving away from ours is not held, however close
    t2, _, _ = RS.robot_step(r, torch.tensor([3.0]), wait * 0, back * 0, 0.05,
                             torch.tensor([[0.3, 0.0]]), 0.25, 3.0, 2.0)
    assert t2.item() == pytest.approx(3.05)


def _three_node_route(speed=0.15):
    # A (0, 0) -> B (1, 0) -> C (2, 0) as one route (B is a node on the way)
    gt = {
        "xy": torch.tensor([[0.0, 0.0], [2.0, 0.0]]),
        "path": torch.tensor([[[[0.0, 0.0]] * 3, [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]],
                              [[[2.0, 0.0], [1.0, 0.0], [0.0, 0.0]], [[2.0, 0.0]] * 3]]),
        "npts": torch.tensor([[1, 3], [3, 1]]),
    }
    return RS.chain_route(gt, torch.tensor([[0, 1]]), torch.tensor([speed]), torch.zeros(1, 1))


def test_retreat_goes_back_to_the_node_before():
    r = _three_node_route()
    our = torch.tensor([[1.6, 0.0]])  # in the way on the B -> C link
    t, wait, back = torch.zeros(1), torch.zeros(1), torch.zeros(1)
    xs = []
    for _ in range(400):
        t, wait, back = RS.robot_step(r, t, wait, back, 0.05, our, 0.25, 3.0, 2.0, retreat=True)
        xs.append(RS.robot_pose(r, t)[0][0, 0].item())
    xs = np.array(xs)
    peak = int(np.argmax(xs > 1.34))  # first stop, 0.25 m before our robot
    assert xs.max() == pytest.approx(1.35, abs=0.01)
    assert xs[peak:].min() == pytest.approx(1.0, abs=0.01)  # back to node B, not further


def test_edge_lock_holds_the_robot_at_the_node():
    r = _three_node_route()
    our = torch.tensor([[1.5, 0.08]])  # beside the B -> C link, not in the way of a 0.25 m yield
    t, wait, back = torch.zeros(1), torch.zeros(1), torch.zeros(1)
    xs = []
    for _ in range(200):
        t, wait, back = RS.robot_step(r, t, wait, back, 0.05, our, 0.25, 99.0, 2.0, lock_m=0.15)
        xs.append(RS.robot_pose(r, t)[0][0, 0].item())
    assert max(xs) == pytest.approx(1.0, abs=0.01)  # waits at B while our robot is on B -> C
    t, wait, back = RS.robot_step(r, t, wait, back, 0.05, torch.tensor([[5.0, 5.0]]), 0.25,
                                  99.0, 2.0, lock_m=0.15)
    assert RS.robot_pose(r, t)[0][0, 0].item() > 1.0  # drives on once the link is free


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
    """A line of nodes A (0, 0) - B (1, 0) - C (2, 0) - D (3, 0), links between neighbours."""

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
    # every vertex sits on the node it is named after
    for v, node in enumerate(pts_node):
        assert r["pts"][0, v].tolist() == pytest.approx(g.xy[node].tolist())
    goals = [pts_node[v] for v in range(len(pts_node)) if r["is_goal"][0, v]]
    assert set(goals) == {a, c} and b not in goals and d not in goals
    # robot_nodes: in the link A -> B it holds A and has B next; pausing at C it holds C only
    cur, nxt = RS.robot_nodes(r, torch.tensor([5.0]))
    assert (cur.item(), nxt.item()) == (a, b)
    cur, nxt = RS.robot_nodes(r, torch.tensor([20.5]))
    assert (cur.item(), nxt.item()) == (c, -1)


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
    blocked[0, c] = True  # its goal is taken: it may not stop there
    blocked[0, b] = True  # B is taken too: no free node to stop at after A
    assert max(_run(r, blocked)) == pytest.approx(0.0, abs=1e-6)  # stays at A
    blocked[0, b] = False
    xs = _run(r, blocked)  # goes to B and waits there: on B itself (steps do not end at B)
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
    cur, nxt = RS.robot_nodes(r, t)
    assert (cur.item(), nxt.item()) == (b, -1)


def test_no_yield_stop_on_a_taken_node():
    g, gt = _line_graph()
    a, b, c = (g.names.index(x) for x in "ABC")
    r = RS.chain_route(gt, torch.tensor([[a, c]]), torch.tensor([0.15]), torch.zeros(1, 1))
    blocked = torch.zeros(1, len(g.names), dtype=torch.bool)
    blocked[0, b] = True  # B taken: it drives through B even with our robot near ...
    xs = _run(r, blocked, our=(1.2, 0.0))
    # ... and yields only once it is node_m (0.12) past B, short of our robot
    assert 1.0 + 0.12 <= max(xs) < 1.2  # (then it backs off: robot_backoff_s)
    assert max(_run(r, torch.zeros_like(blocked), our=(1.2, 0.0))) < 1.0  # free B: yields before


def test_route_nodes_are_ours_until_we_are_past_them():
    node_arc = torch.tensor([[0.5, float("inf"), 2.0]])  # node 1 is not on our route
    held = RS.route_nodes_held(node_arc, torch.tensor([0.8]), 0.2)
    assert held.tolist() == [[False, False, True]]


def test_reservation_passes_through_a_taken_node_it_need_not_stop_at():
    g, gt = _line_graph()
    a, b, c = (g.names.index(x) for x in "ABC")
    r = RS.chain_route(gt, torch.tensor([[a, c]]), torch.tensor([0.15]), torch.zeros(1, 1))
    blocked = torch.zeros(1, len(g.names), dtype=torch.bool)
    blocked[0, b] = True  # B taken, but it only passes B on its way to the free goal C
    assert max(_run(r, blocked)) == pytest.approx(2.0, abs=1e-6)


def test_to_body_and_robot_gap():
    pts = torch.tensor([[[1.0, 1.0]]])
    b = RS.to_body(pts, torch.tensor([[1.0, 0.0]]), torch.tensor([math.pi / 2]))
    assert b[0, 0].tolist() == pytest.approx([1.0, 0.0], abs=1e-6)  # 1 m straight ahead
    g = RS.robot_gap(torch.tensor([[0.5, 0.0]]), torch.zeros(1, 2), torch.zeros(1))
    assert g.item() == pytest.approx(0.5 - 0.06)


def test_start_banned_moves_the_start_off_our_nodes():
    g, gt = _line_graph()
    banned = torch.zeros(64, 4, dtype=torch.bool)
    ours = torch.zeros(64, 4, dtype=torch.bool)
    ours[:, :3] = True  # A, B, C are on our route: robots start at D
    chain = RS.sample_chains(gt, 64, 2, banned, start_banned=ours)
    assert (chain[:, 0] == 3).all() and (chain[:, 1] != 3).all()


def test_robot_with_priority_does_not_wait_for_us():
    r = _line_route()
    t = torch.zeros(1)
    for _ in range(160):  # our robot stands on its line at 0.6: yield_m 0 drives into it
        t, _, _ = RS.robot_step(r, t, torch.zeros(1), torch.zeros(1), 0.05,
                                torch.tensor([[0.6, 0.0]]), torch.zeros(1), 3.0, 2.0)
    assert RS.robot_pose(r, t)[0][0, 0].item() > 0.6
