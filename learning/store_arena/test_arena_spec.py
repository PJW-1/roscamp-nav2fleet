"""Tests of the store arena boxes and the 2D ray cast, without Isaac.
Run: python -m pytest learning/store_arena -q"""

import math
import os
import sys

import pytest
import torch
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena_spec as arena  # noqa: E402

MAP = os.path.join(arena.REPO, "pc", "fleet_core", "fleet_core", "data", "fleet_map_v7.yaml")


def rect(b):
    return (b.x0, b.y0, b.x1, b.y1)


def test_boxes_follow_yaml_bounds_and_measured_shelves():
    with open(MAP, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    x0, y0, x1, y1 = data["bounds"]
    boxes = arena.arena_boxes()
    walls = [b for b in boxes if b.kind == "wall"]
    shelves = [b for b in boxes if b.kind == "shelf"]
    t = arena.gz_arena.WALL_M
    # inner faces of the four walls are the bounds, walls are WALL_M thick
    assert sorted(rect(w) for w in walls) == sorted(
        [
            (x0 - t, y0 - t, x1 + t, y0),
            (x0 - t, y1, x1 + t, y1 + t),
            (x0 - t, y0, x0, y1),
            (x1, y0, x1 + t, y1),
        ]
    )
    assert [rect(s) for s in shelves] == [tuple(r) for r in data["shelves"]]
    # the Gazebo-only correction differs from the measured shelves and is not used
    assert data["sim_shelves"] != data["shelves"]
    assert [rect(s) for s in shelves] != [tuple(r) for r in data["sim_shelves"]]
    assert all(w.height_m == arena.ARENA.wall_height_m for w in walls)
    assert all(s.height_m == arena.ARENA.shelf_height_m for s in shelves)


def test_boxes_are_read_from_the_yaml_at_run_time(tmp_path):
    """A changed yaml (as after a tape-measure check) changes the boxes; sim_shelves never does."""
    text = open(MAP, encoding="utf-8").read()
    text = text.replace("- [0.68, 0.365, 0.99, 0.405]   # 남쪽 진열대, 서쪽",
                        "- [0.70, 0.36, 0.99, 0.40]   # moved", 1)
    text = text.replace("sim_shelves:\n  - [0.63, 0.77, 1.56, 0.793]",
                        "sim_shelves:\n  - [0.63, 0.77, 1.50, 0.79]", 1)
    assert "# moved" in text and "1.50, 0.79]" in text
    path = tmp_path / "fleet_map_moved.yaml"
    path.write_text(text, encoding="utf-8")
    spec = arena.make_arena_spec({"layout": str(path)})
    shelves = [rect(b) for b in arena.arena_boxes(spec) if b.kind == "shelf"]
    assert shelves[1] == (0.70, 0.36, 0.99, 0.40)
    assert shelves[0] == (0.63, 0.77, 1.56, 0.803)  # measured, not the edited sim_shelves


def test_split_overrides_routes_fields():
    task, arena_o = arena.split_overrides(
        ["w_time=-0.1", "env_spacing=4", "layout=v7", "goal_dist_range=0.5,1"]
    )
    assert task == {"w_time": -0.1, "goal_dist_range": (0.5, 1.0)}
    assert arena_o == {"env_spacing": 4.0, "layout": "v7"}
    with pytest.raises(TypeError):
        arena.make_arena_spec({"no_such_field": 1.0})


def test_raycast_from_arena_point_matches_hand_values():
    rects = arena.boxes_tensor(arena.arena_boxes())
    # lidar at (0.30, 0.55), robot facing +x: ray 180 ahead runs along the middle aisle to the east
    # wall (1.98), ray 0 behind hits the west wall (0.0), ray 270 left the north wall (0.98)
    r = arena.raycast_rects(torch.tensor([[0.30, 0.55]]), torch.tensor([0.0]), rects, max_range=5.0)
    assert r[0, 180].item() == pytest.approx(1.68, abs=1e-5)
    assert r[0, 0].item() == pytest.approx(0.30, abs=1e-5)
    assert r[0, 270].item() == pytest.approx(0.43, abs=1e-5)
    assert r[0, 90].item() == pytest.approx(0.55, abs=1e-5)
    # facing +y at (0.85, 0.55): shelf A south face 0.77 ahead, south shelf north face 0.405 behind
    r = arena.raycast_rects(
        torch.tensor([[0.85, 0.55]]), torch.tensor([math.pi / 2]), rects, max_range=5.0
    )
    assert r[0, 180].item() == pytest.approx(0.22, abs=1e-5)
    assert r[0, 0].item() == pytest.approx(0.145, abs=1e-5)


def test_raycast_range_limits_and_inside_start():
    rects = torch.tensor([[1.0, -1.0, 1.1, 1.0]])
    r = arena.raycast_rects(torch.zeros(1, 2), torch.zeros(1), rects, max_range=2.0)
    assert r[0, 180].item() == pytest.approx(1.0, abs=1e-6)
    assert math.isinf(r[0, 0].item())  # behind: nothing
    r = arena.raycast_rects(torch.zeros(1, 2), torch.zeros(1), rects, max_range=0.9)
    assert math.isinf(r[0, 180].item())  # beyond max_range
    # wall face 0.03 m ahead: the ray starts 0.05 out, inside the box, and returns its far side
    r = arena.raycast_rects(torch.tensor([[0.97, 0.0]]), torch.zeros(1), rects, max_range=2.0)
    assert r[0, 180].item() == pytest.approx(0.13, abs=1e-6)



def test_scripted_robot_defaults_are_the_user_decision():
    # 2026-11-22: 12 routes + retreat to the node before; 4c (ppo_h) used --set robot_routes=4
    # --set robot_retreat=0
    assert (arena.ARENA.robot_routes, arena.ARENA.robot_retreat) == (12, 1)
    old = arena.make_arena_spec({"robot_routes": 4, "robot_retreat": 0})
    assert (old.robot_routes, old.robot_retreat) == (4, 0)


def test_footprint_clearance_to_rects_boxes_and_cylinders():
    half = (0.06, 0.056)  # Pode footprint 0.12 x 0.112
    xy, yaw = torch.tensor([[0.0, 0.0]]), torch.zeros(1)
    wall = torch.tensor([[[0.065, -1.0, 0.2, 1.0]]])  # face 5 mm ahead of the front edge
    assert arena.footprint_clearance(xy, yaw, half, wall).item() == pytest.approx(0.005, abs=1e-6)
    # turned 45 deg the corner reaches sqrt(0.06^2 + 0.056^2) = 0.0821 ahead: overlaps
    assert arena.footprint_clearance(xy, torch.tensor([math.pi / 4]), half, wall).item() == 0.0
    # a wall corner pointing into the side: the rect corner, not a footprint corner, is nearest
    corner = torch.tensor([[[-0.01, 0.059, 0.3, 0.5]]])
    c = arena.footprint_clearance(xy, yaw, half, corner)
    assert c.item() == pytest.approx(0.003, abs=1e-6)
    # a turned box (a cart) and a cylinder (a person of radius 0.04)
    g = arena.box_clearance(xy, yaw, half, torch.tensor([[0.2, 0.0]]), torch.zeros(1),
                            (0.07, 0.045))
    assert g.item() == pytest.approx(0.2 - 0.07 - 0.06, abs=1e-6)
    c = arena.cylinder_clearance(xy, yaw, half, torch.tensor([[0.0, 0.2]]), 0.04)
    assert c.item() == pytest.approx(0.2 - 0.04 - 0.056, abs=1e-6)


def test_near_wall_penalty_is_linear_and_bounded():
    c = torch.tensor([0.0, 0.025, 0.05, 0.3])
    p = arena.near_wall_penalty(c, near_m=0.05, weight=-0.1)
    assert p.tolist() == pytest.approx([-0.1, -0.05, 0.0, 0.0])


def test_corner_start_routes_and_away_heading():
    rects = torch.tensor([[0.0, -0.1, 3.0, 0.0]])  # south wall face at y = 0
    start = torch.tensor([[1.66, 0.15], [1.0, 0.8]])
    assert arena.near_wall_starts(start, rects, 0.2).tolist() == [True, False]
    path = torch.tensor([[[1.66, 0.15], [1.5, 0.15], [1.0, 0.15]]])  # heads west
    yaw = arena.away_heading(path, torch.zeros(1))
    assert yaw.item() == pytest.approx(0.0, abs=1e-6)  # faces east, as the Gazebo Q2 case


def test_cross_overlap_without_corners_inside_is_zero_clearance():
    half = (0.06, 0.056)
    xy, yaw = torch.tensor([[0.0, 0.0]]), torch.zeros(1)
    bar = torch.tensor([[[-0.2, -0.01, 0.2, 0.01]]])  # a thin bar through the footprint
    assert arena.footprint_clearance(xy, yaw, half, bar).item() == 0.0
    g = arena.box_clearance(xy, yaw, half, torch.tensor([[0.0, 0.0]]), torch.zeros(1),
                            (0.2, 0.01))
    assert g.item() == 0.0


def test_graze_flags_are_booleans_also_when_off():
    c = torch.tensor([0.0, 0.004, 0.006])
    assert arena.graze_flags(c, 0.005).tolist() == [True, True, False]
    off = arena.graze_flags(c, 0.0)
    assert off.dtype == torch.bool and not off.any()
