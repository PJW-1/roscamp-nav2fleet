"""Movers without a simulator: legs stay in free space away from start and goal, and the motion
goes back and forth with pauses."""

import dataclasses
import math
import os
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import arena_spec as A  # noqa: E402
import mover_spec as M  # noqa: E402
import route_spec as R  # noqa: E402

SPEC = dataclasses.replace(A.ARENA, obstacle_layouts=4)


def test_shapes_match_the_gazebo_benchmark():
    person, cart = M.MOVERS
    assert (person.kind, person.size, person.speed) == ("cylinder", (0.04, 0.25), 0.10)
    assert (cart.kind, cart.size, cart.speed) == ("box", (0.14, 0.09, 0.20), 0.06)
    assert min(m.height for m in M.MOVERS) > 0.125 + 0.05  # above the lidar plane
    assert cart.radius == pytest.approx(0.5 * math.hypot(0.14, 0.09))


@pytest.fixture(scope="module")
def batch():
    site = A.load_site(SPEC)
    static = np.array([(b.x0, b.y0, b.x1, b.y1) for b in A.arena_boxes(SPEC, site)])
    r = R.build_routes(SPEC, site, static)
    idx = np.arange(r.count)
    rects = []
    for i in idx:
        boxes = [
            R.box_rect(r.obstacle_xy[i, k], r.obstacle_sizes[k])
            for k in range(len(r.obstacle_sizes))
            if not np.isnan(r.obstacle_xy[i, k, 0])
        ]
        boxes += [(9.0, 9.0, 9.1, 9.1)] * (len(r.obstacle_sizes) - len(boxes))  # padding
        rects.append(np.concatenate((static, np.array(boxes))))
    ends = np.array([r.start_goal(i) for i in idx])
    t = {
        "rects": torch.tensor(np.array(rects), dtype=torch.float32),
        "path": torch.tensor(r.path, dtype=torch.float32),
        "npts": torch.tensor(r.npts),
        "start": torch.tensor(ends[:, 0], dtype=torch.float32),
        "goal": torch.tensor(ends[:, 1], dtype=torch.float32),
    }
    return site, t


@pytest.mark.parametrize("shape", M.MOVERS, ids=lambda s: s.name)
def test_legs_are_free_and_clear_of_start_and_goal(batch, shape):
    site, t = batch
    gen = torch.Generator().manual_seed(0)
    legs = M.sample_legs(
        t["rects"], site.bounds(), t["path"], t["npts"], t["start"], t["goal"], shape, SPEC, gen
    )
    act = legs["active"]
    assert 0.6 < act.float().mean() < 0.8  # mover_prob 0.75, nearly every env has a valid leg
    s = torch.linspace(0, 1, 50)
    a, b = legs["a"][act], legs["b"][act]
    pts = a[:, None] + s[None, :, None] * (b - a)[:, None]  # (n, 50, 2)
    clear = M.point_rect_distance(pts, t["rects"][act])
    assert clear.min() > shape.radius  # the whole leg, not only the 9 checked points
    for key in ("start", "goal"):
        d = (pts - t[key][act][:, None]).norm(dim=-1).amin(-1)
        assert d.min() > SPEC.mover_node_clear_m
    length = (b - a).norm(dim=-1)
    assert length.min() >= SPEC.mover_leg_m[0] - 1e-5 and length.max() <= SPEC.mover_leg_m[1] + 1e-5
    lo, hi = SPEC.mover_speed_scale
    assert legs["speed"].min() >= shape.speed * lo and legs["speed"].max() <= shape.speed * hi


def test_motion_back_and_forth_with_pauses():
    legs = {
        "a": torch.tensor([[0.0, 0.0], [0.0, 0.0]]),
        "b": torch.tensor([[1.0, 0.0], [0.0, 1.0]]),
        "speed": torch.tensor([0.5, 0.0]),
        "pause": torch.tensor([1.0, 1.0]),
        "phase": torch.tensor([0.0, 0.3]),
    }
    # env 0: pause 0..1 s at a, out 1..3 s, pause 3..4 s at b, back 4..6 s, cycle 6 s
    cases = [(0.5, 0.0, 0.0), (2.0, 0.5, 0.5), (3.5, 1.0, 0.0), (5.0, 0.5, -0.5), (8.0, 0.5, 0.5)]
    for tt, x, vx in cases:
        pos, yaw, vel = M.mover_pose(legs, torch.tensor([tt, tt]))
        assert pos[0].tolist() == pytest.approx([x, 0.0], abs=1e-5)
        assert vel[0].tolist() == pytest.approx([vx, 0.0], abs=1e-5)
        assert yaw[0].item() == pytest.approx(0.0)
        assert pos[1].tolist() == pytest.approx([0.0, 0.0]) and vel[1].abs().max() == 0
    # the phase starts the mover part-way through its cycle
    legs["phase"] = torch.tensor([0.5, 0.0])
    pos, _, _ = M.mover_pose(legs, torch.tensor([0.0, 0.0]))
    assert pos[0].tolist() == pytest.approx([1.0, 0.0])  # 3 s into the cycle: at b
