"""Routes without a simulator: node rules, global paths, obstacle layouts, path following."""

import dataclasses
import math
import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import route_spec as R  # noqa: E402

SPEC = dataclasses.replace(A.ARENA, obstacle_layouts=6)


@pytest.fixture(scope="module")
def built():
    site = A.load_site(SPEC)
    static = np.array([(b.x0, b.y0, b.x1, b.y1) for b in A.arena_boxes(SPEC, site)])
    return site, static, R.build_routes(SPEC, site, static)


def test_nodes_are_hall_without_parking(built):
    site, _, r = built
    assert r.node_names == ["O_IN", "O_OUT", "M_E", "Q1", "Q2", "E-1", "E-2", "E-3"]
    for name in r.node_names:
        assert site.nodes[name].zone == "HALL" and site.nodes[name].kind != "parking"
    names, _ = R.hall_nodes(site, "")
    assert {"P-1", "P-2", "P-3"} <= set(names)


def test_nodes_leave_room_to_turn(built):
    """Episodes start at a node with a random yaw: the rear corner (0.0845 m) must be clear."""
    _, static, r = built
    d = R.point_rect_distance(r.node_xy[:, 0], r.node_xy[:, 1], static)
    assert d.min() > 0.0845


def test_pairs_follow_the_distance_rule(built):
    _, _, r = built
    n = len(r.node_names)
    allowed = {
        (a, b)
        for a in range(n)
        for b in range(n)
        if a != b and np.hypot(*(r.node_xy[a] - r.node_xy[b])) >= SPEC.min_start_goal_m
    }
    assert set(r.pairs) == allowed  # every allowed pair has a path in the empty arena
    assert r.count == len(r.pairs) * SPEC.obstacle_layouts


def test_paths_are_clear_connected_and_end_at_the_nodes(built):
    _, static, r = built
    for i in range(r.count):
        p = r.path[i, : r.npts[i]]
        s, g = r.start_goal(i)
        assert np.allclose(p[0], s) and np.allclose(p[-1], g)
        steps = np.hypot(*np.diff(p, axis=0).T)
        # cell centre to cell centre inside; the nodes sit anywhere in their end cells
        assert steps[1:-1].max() <= math.sqrt(2) * SPEC.grid_m + 1e-9
        assert max(steps[0], steps[-1]) <= 1.5 * math.sqrt(2) * SPEC.grid_m + 1e-9
        boxes = [
            R.box_rect(r.obstacle_xy[i, k], r.obstacle_sizes[k])
            for k in range(len(r.obstacle_sizes))
            if not np.isnan(r.obstacle_xy[i, k, 0])
        ]
        rects = np.concatenate([static] + [np.array(boxes)] * bool(boxes))
        # interior points are cell centres outside the inscribed radius of every obstacle
        clear = R.point_rect_distance(p[1:-1, 0], p[1:-1, 1], rects)
        assert clear.min() > SPEC.inscribed_m
        assert np.isclose(r.total[i], steps.sum())


def test_layouts_keep_boxes_apart_and_clear_of_the_nodes(built):
    _, static, r = built
    used = ~np.isnan(r.obstacle_xy[..., 0])
    assert not used[:: SPEC.obstacle_layouts].any()  # layout 0 of every pair is empty
    assert used.sum(1).max() <= SPEC.obstacle_max and used.any()
    for i in range(r.count):
        s, g = r.start_goal(i)
        rects = [
            R.box_rect(r.obstacle_xy[i, k], r.obstacle_sizes[k]) for k in np.flatnonzero(used[i])
        ]
        for j, b in enumerate(rects):
            assert not R.rects_overlap(b, static)
            assert not R.rects_overlap(b, np.array(rects[:j] + rects[j + 1 :]))
            d = R.point_rect_distance(np.array([s[0], g[0]]), np.array([s[1], g[1]]), b[None])
            assert d.min() >= SPEC.obstacle_node_clear_m


def test_a_box_on_the_way_bends_the_path(built):
    """Same pair: some layout's path is longer than the empty layout's (boxes are on the way)."""
    _, _, r = built
    m = SPEC.obstacle_layouts
    longer = [
        (r.total[k * m + 1 : (k + 1) * m] > r.total[k * m] + 0.02).any()
        for k in range(len(r.pairs))
    ]
    assert np.mean(longer) > 0.5


def test_plan_avoids_a_wall_with_a_gap():
    """A 1 x 1 m room split by a wall at x 0.5 with one gap at y 0.8..1.0: the path goes through
    the gap, as far from the gap edge as the inflation makes worth it."""
    grid = R.Grid((0.0, 0.0, 1.0, 1.0), 0.025)
    walls = np.array([(0.48, 0.0, 0.52, 0.8)])
    cost = grid.costs(walls, 0.07, 0.15, 15.0)
    p = R.plan(grid, cost, (0.2, 0.2), (0.8, 0.2))
    assert p is not None
    crossing = p[np.argmin(np.abs(p[:, 0] - 0.5))]
    assert crossing[1] > 0.8 + 0.07
    walls = np.array([(0.48, 0.0, 0.52, 0.95)])  # gap 0.05 m: narrower than the robot
    assert R.plan(grid, grid.costs(walls, 0.07, 0.15, 15.0), (0.2, 0.2), (0.8, 0.2)) is None


def test_build_is_deterministic_and_follows_overrides(built):
    site, static, r = built
    again = R.build_routes(SPEC, site, static)
    assert np.array_equal(again.obstacle_xy, r.obstacle_xy, equal_nan=True)
    _, arena = A.split_overrides(["min_start_goal_m=1.0", "obstacle_sizes=0.1,0.1,0.1"])
    spec = dataclasses.replace(SPEC, **arena, obstacle_layouts=2)
    other = R.build_routes(spec, site, static)
    assert len(other.pairs) < len(r.pairs) and other.obstacle_sizes == (0.1, 0.1, 0.1)


def test_follow_progress_remaining_and_lookahead():
    path = torch.tensor([[[0.1 * i, 0.0] for i in range(11)] + [[1.0, 0.0]] * 3])  # 0..1 m, padded
    cum = torch.tensor([[0.1 * i for i in range(11)] + [1.0] * 3])
    npts = torch.tensor([11])
    xy = torch.tensor([[0.42, 0.03]])
    prog, rem, la = R.follow(path, cum, npts, torch.tensor([0]), xy, 0.4, 20)
    assert prog.item() == 4
    assert rem.item() == pytest.approx(1.0 - 0.42 + 0.03, abs=1e-6)  # projected onto 0.4..0.5
    assert la[0].tolist() == pytest.approx([0.8, 0.0])
    # progress never goes back, and the window caps how far it may jump
    prog2, _, _ = R.follow(path, cum, npts, prog, torch.tensor([[0.0, 0.0]]), 0.4, 20)
    assert prog2.item() == 4
    prog3, _, _ = R.follow(path, cum, npts, torch.tensor([0]), torch.tensor([[1.0, 0.0]]), 0.4, 3)
    assert prog3.item() == 3
    # near the end the lookahead is the goal
    _, rem4, la4 = R.follow(path, cum, npts, prog, torch.tensor([[0.95, 0.0]]), 0.4, 20)
    assert la4[0].tolist() == pytest.approx([1.0, 0.0]) and rem4.item() == pytest.approx(0.05)
