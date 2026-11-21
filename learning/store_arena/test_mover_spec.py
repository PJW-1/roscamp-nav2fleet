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


def test_leg_check_catches_a_corner_between_samples():
    """A box corner 0.035 m beside an 0.8 m leg, halfway between two of 9 evenly spaced points:
    the checked clearance must not exceed the true one by more than the 0.01 m margin."""
    a, b = torch.tensor([[[0.0, 0.0]]]), torch.tensor([[[0.8, 0.0]]])
    rects = torch.tensor([[[0.05 - 0.2, 0.035, 0.05, 0.3]]])  # corner at (0.05, 0.035)
    clear = M.leg_clearance(a, b, rects)
    assert clear.item() <= 0.035 + M.LEG_CHECK_MARGIN_M + 1e-6


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
    # both ends (where it pauses) beside the route: it crosses the route but does not stay on it
    valid = torch.arange(t["path"].shape[1])[None] < t["npts"][act][:, None]
    for end in (a, b):
        d = (end[:, None] - t["path"][act]).norm(dim=-1)
        d = torch.where(valid, d, torch.full_like(d, 9.0)).amin(-1)
        assert d.min() - shape.radius >= SPEC.mover_end_path_clear_m - 1e-5
    # and a good share of legs still cross the route (middle of the leg on the path)
    mid = 0.5 * (a + b)
    dm = (mid[:, None] - t["path"][act]).norm(dim=-1)
    dm = torch.where(valid, dm, torch.full_like(dm, 9.0)).amin(-1)
    assert (dm < 0.05).float().mean() > 0.2


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


def test_mover_waits_for_the_robot_and_goes_on_when_it_leaves():
    person = M.MOVERS[0]
    legs = {
        "a": torch.tensor([[0.0, 0.0], [0.0, 0.0]]),
        "b": torch.tensor([[1.0, 0.0], [1.0, 0.0]]),
        "speed": torch.tensor([0.1, 0.1]),
        "pause": torch.tensor([0.0, 0.0]),
        "phase": torch.tensor([0.0, 0.0]),
    }
    dt, yield_m = 0.05, 0.15
    robot = torch.tensor([[0.5, 0.0], [0.5, 0.0]])  # env 0 on the leg; env 1 the robot leaves
    t, wait = torch.zeros(2), torch.zeros(2)
    for k in range(200):
        if k == 100:
            robot[1] = torch.tensor([0.5, 1.0])
        t, wait, _ = M.yield_step(legs, t, dt, person, robot, yield_m, wait, math.inf)
        pos, _, _ = M.mover_pose(legs, t)
        gap = M.surface_gap(person, robot, pos, torch.zeros(2))
        assert gap[0] >= yield_m - 1e-6  # never walks closer than yield_m into the robot
    assert pos[0, 0] == pytest.approx(0.5 - person.size[0] - yield_m, abs=0.006)  # waiting
    assert pos[1, 0] > 0.5  # went on once the robot was gone
    # a mover walking away from a robot close behind it is not held
    t2, _, _ = M.yield_step(
        legs, torch.tensor([0.5, 0.5]), dt, person, torch.zeros(2, 2), yield_m, wait, math.inf
    )
    assert t2.tolist() == pytest.approx([0.55, 0.55])
    # a per-env yield distance: env 1 (0, a Gazebo-like mover) walks into the robot
    on_leg = torch.tensor([[0.5, 0.0], [0.5, 0.0]])
    t3, _, _ = M.yield_step(legs, torch.tensor([3.1, 3.1]), dt, person, on_leg,
                            torch.tensor([0.15, 0.0]), wait * 0, math.inf)
    assert t3.tolist() == pytest.approx([3.1, 3.15])


def test_surface_gap_of_a_turned_box():
    cart = M.MOVERS[1]
    xy = torch.tensor([[0.0, 0.5]])
    g0 = M.surface_gap(cart, xy, torch.zeros(1, 2), torch.zeros(1))
    g1 = M.surface_gap(cart, xy, torch.zeros(1, 2), torch.tensor([math.pi / 2]))
    assert g0.item() == pytest.approx(0.5 - 0.045) and g1.item() == pytest.approx(0.5 - 0.07)


def test_turn_back_mirrors_the_mover_onto_the_other_way():
    legs = {
        "a": torch.zeros(3, 2),
        "b": torch.tensor([[1.0, 0.0]] * 3),
        "speed": torch.tensor([0.5, 0.5, 0.5]),
        "pause": torch.tensor([1.0, 1.0, 1.0]),
        "phase": torch.zeros(3),
    }
    # cycle 6 s: out 1..3 s, back 4..6 s; t 2.0 going out, 5.5 coming back, 0.5 pausing at a
    t = torch.tensor([2.0, 5.5, 0.5])
    p0, _, v0 = M.mover_pose(legs, t)
    p1, _, v1 = M.mover_pose(legs, M.turn_back(legs, t))
    assert p1.flatten().tolist() == pytest.approx(p0.flatten().tolist(), abs=1e-5)  # same place
    assert v1[:2].flatten().tolist() == pytest.approx((-v0[:2]).flatten().tolist(), abs=1e-5)
    assert v1[2].abs().max() == 0  # a pausing mover stays paused


def test_waiting_mover_turns_back_after_mover_wait_s():
    person = M.MOVERS[0]
    legs = {
        "a": torch.tensor([[0.0, 0.0]]),
        "b": torch.tensor([[1.0, 0.0]]),
        "speed": torch.tensor([0.1]),
        "pause": torch.tensor([0.0]),
        "phase": torch.tensor([0.0]),
    }
    dt, robot = 0.05, torch.tensor([[0.5, 0.0]])  # the robot stands on the leg
    t, wait = torch.zeros(1), torch.zeros(1)
    turned_at = None
    for k in range(200):
        t, wait, turned = M.yield_step(legs, t, dt, person, robot, 0.15, wait, 2.0)
        if turned.any() and turned_at is None:
            turned_at = k
    pos, _, _ = M.mover_pose(legs, t)
    first_hold = int((0.5 - 0.04 - 0.15) / (0.1 * dt))  # steps to reach the yield distance
    assert turned_at == pytest.approx(first_hold + 2.0 / dt, abs=2)
    assert pos[0, 0] < 0.2  # walked back toward a
    # only a mover that is actually held turns back, even with wait_s 0
    far = torch.tensor([[0.5, 5.0]])
    t1, _, turned = M.yield_step(legs, torch.tensor([1.0]), dt, person, far, 0.15, wait * 0, 0.0)
    assert not turned.any() and t1.item() == pytest.approx(1.05)
    # a zero-length leg stays finite and in place
    legs0 = dict(legs, b=legs["a"].clone())
    assert torch.isfinite(M.turn_back(legs0, torch.tensor([1.0]))).all()
