"""매장 아레나 박스와 2D 레이캐스트 테스트 (Isaac 없이).
실행: python -m pytest learning/store_arena -q"""

import math
import os
import sys

import pytest
import torch
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena_spec as arena  # noqa: E402
import route_spec as R  # noqa: E402

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
    # 벽 안쪽 면이 bounds, 벽 두께는 WALL_M
    assert sorted(rect(w) for w in walls) == sorted(
        [
            (x0 - t, y0 - t, x1 + t, y0),
            (x0 - t, y1, x1 + t, y1 + t),
            (x0 - t, y0, x0, y1),
            (x1, y0, x1 + t, y1),
        ]
    )
    assert [rect(s) for s in shelves] == [tuple(r) for r in data["shelves"]]
    assert all(w.height_m == arena.ARENA.wall_height_m for w in walls)
    assert all(s.height_m == arena.ARENA.shelf_height_m for s in shelves)


def test_boxes_are_read_from_the_yaml_at_run_time(tmp_path):
    """yaml 이 바뀌면 박스가 바뀌지만 sim_shelves 는 영향 없음."""
    text = open(MAP, encoding="utf-8").read()
    text = text.replace("- [0.704, 0.364, 1.014, 0.404]     # 남쪽 진열대, 서쪽",
                        "- [0.75, 0.36, 0.99, 0.40]   # moved", 1)
    text = text.replace("sim_shelves:\n  - [0.654, 0.754, 1.574, 0.794]",
                        "sim_shelves:\n  - [0.654, 0.754, 1.50, 0.79]", 1)
    assert "# moved" in text and "1.50, 0.79]" in text
    path = tmp_path / "fleet_map_moved.yaml"
    path.write_text(text, encoding="utf-8")
    spec = arena.make_arena_spec({"layout": str(path)})
    shelves = [rect(b) for b in arena.arena_boxes(spec) if b.kind == "shelf"]
    assert shelves[1] == (0.75, 0.36, 0.99, 0.40)
    assert shelves[0] == (0.654, 0.754, 1.574, 0.794)  # 실측값, 고친 sim_shelves 아님


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
    # (0.30, 0.55) 에서 +x 를 봄: 레이 180 은 동쪽 벽, 0 은 서쪽 벽, 270 은 북쪽 벽
    r = arena.raycast_rects(torch.tensor([[0.30, 0.55]]), torch.tensor([0.0]), rects, max_range=5.0)
    assert r[0, 180].item() == pytest.approx(1.68, abs=1e-5)
    assert r[0, 0].item() == pytest.approx(0.30, abs=1e-5)
    assert r[0, 270].item() == pytest.approx(0.43, abs=1e-5)
    assert r[0, 90].item() == pytest.approx(0.55, abs=1e-5)
    # (0.85, 0.55) 에서 +y 를 봄: 앞 선반 A 남쪽 면 0.754, 뒤 남쪽 선반 북쪽 면 0.404
    r = arena.raycast_rects(
        torch.tensor([[0.85, 0.55]]), torch.tensor([math.pi / 2]), rects, max_range=5.0
    )
    assert r[0, 180].item() == pytest.approx(0.204, abs=1e-5)
    assert r[0, 0].item() == pytest.approx(0.146, abs=1e-5)


def test_raycast_range_limits_and_inside_start():
    rects = torch.tensor([[1.0, -1.0, 1.1, 1.0]])
    r = arena.raycast_rects(torch.zeros(1, 2), torch.zeros(1), rects, max_range=2.0)
    assert r[0, 180].item() == pytest.approx(1.0, abs=1e-6)
    assert math.isinf(r[0, 0].item())  # 뒤: 없음
    r = arena.raycast_rects(torch.zeros(1, 2), torch.zeros(1), rects, max_range=0.9)
    assert math.isinf(r[0, 180].item())  # max_range 밖
    # 벽 면이 0.03 m 앞: 레이는 0.05 에서 시작해 박스 안이므로 먼 면을 돌려줌
    r = arena.raycast_rects(torch.tensor([[0.97, 0.0]]), torch.zeros(1), rects, max_range=2.0)
    assert r[0, 180].item() == pytest.approx(0.13, abs=1e-6)



def test_episode_time_and_travel_waits_follow_the_one_top_speed():
    """0.25 m/s 기준 시간은 0.25 / v 배가 되고, 정지 시간은 그대로."""
    import robot_spec

    ts = arena.ts
    k = 0.25 / ts.NAV_SPEED_MPS
    a = arena.ARENA
    assert a.episode_s == pytest.approx(30.0 * k)
    assert a.mover_wait_s == pytest.approx(2.0 * k)
    assert a.robot_wait_s == pytest.approx(3.0 * k)
    assert a.robot_backoff_s == pytest.approx(2.0 * k)
    assert a.mover_pause_s == (0.0, 1.0) and a.robot_pause_s == (0.0, 2.0)
    assert robot_spec.ROBOT.speed == ts.NAV_SPEED_MPS == ts.ACTION_LIMITS.v_max


def test_scripted_robot_defaults_are_the_user_decision():
    # 12 경로 + 직전 노드로 후퇴가 기본, 4c 는 robot_routes=4, robot_retreat=0
    assert (arena.ARENA.robot_routes, arena.ARENA.robot_retreat) == (12, 1)
    assert arena.ARENA.robot_reserve == 1
    # 대역 Pode 는 최고속도를 못 넘음: 배율 1.0 이하
    assert arena.ARENA.robot_speed_scale[1] <= 1.0
    old = arena.make_arena_spec({"robot_routes": 4, "robot_retreat": 0})
    assert (old.robot_routes, old.robot_retreat) == (4, 0)


def test_footprint_clearance_to_rects_boxes_and_cylinders():
    half = (0.06, 0.056)  # Pode 발자국 0.12 x 0.112
    xy, yaw = torch.tensor([[0.0, 0.0]]), torch.zeros(1)
    wall = torch.tensor([[[0.065, -1.0, 0.2, 1.0]]])  # 앞 모서리 5 mm 앞의 면
    assert arena.footprint_clearance(xy, yaw, half, wall).item() == pytest.approx(0.005, abs=1e-6)
    # 45 deg 돌리면 모서리가 sqrt(0.06^2 + 0.056^2) = 0.0821 앞까지 닿아 겹침
    assert arena.footprint_clearance(xy, torch.tensor([math.pi / 4]), half, wall).item() == 0.0
    # 옆을 향한 벽 모서리: 가장 가까운 것은 발자국이 아니라 rect 모서리
    corner = torch.tensor([[[-0.01, 0.059, 0.3, 0.5]]])
    c = arena.footprint_clearance(xy, yaw, half, corner)
    assert c.item() == pytest.approx(0.003, abs=1e-6)
    # 돌린 박스(수레)와 원기둥(반지름 0.04 사람)
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
    rects = torch.tensor([[0.0, -0.1, 3.0, 0.0]])  # 남쪽 벽 면 y = 0
    start = torch.tensor([[1.66, 0.15], [1.0, 0.8]])
    assert arena.near_wall_starts(start, rects, 0.2).tolist() == [True, False]
    path = torch.tensor([[[1.66, 0.15], [1.5, 0.15], [1.0, 0.15]]])  # 서쪽으로
    yaw = arena.away_heading(path, torch.zeros(1))
    assert yaw.item() == pytest.approx(0.0, abs=1e-6)  # 동쪽을 봄


def test_cross_overlap_without_corners_inside_is_zero_clearance():
    half = (0.06, 0.056)
    xy, yaw = torch.tensor([[0.0, 0.0]]), torch.zeros(1)
    bar = torch.tensor([[[-0.2, -0.01, 0.2, 0.01]]])  # 발자국을 가로지르는 얇은 막대
    assert arena.footprint_clearance(xy, yaw, half, bar).item() == 0.0
    g = arena.box_clearance(xy, yaw, half, torch.tensor([[0.0, 0.0]]), torch.zeros(1),
                            (0.2, 0.01))
    assert g.item() == 0.0


def test_graze_flags_are_booleans_also_when_off():
    c = torch.tensor([0.0, 0.004, 0.006])
    assert arena.graze_flags(c, 0.005).tolist() == [True, True, False]
    off = arena.graze_flags(c, 0.0)
    assert off.dtype == torch.bool and not off.any()


def test_corner_start_nodes_of_the_store_arena():
    site = arena.load_site()
    names, xy = R.hall_nodes(site, arena.ARENA.excluded_node_kinds)
    rects = arena.boxes_tensor(arena.arena_boxes(arena.ARENA, site))
    near = arena.near_wall_starts(torch.as_tensor(xy, dtype=torch.float32), rects,
                                  arena.ARENA.corner_start_m)
    # M_E(1.645, 0.55)는 남쪽 진열대 동쪽 것 북동 모서리에서 0.183 m라 들지 않는다
    assert sorted(n for n, c in zip(names, near.tolist(), strict=True) if c) == [
        "E-1", "E-3", "Q1", "Q2"]


def test_footprint_box_is_off_centre_like_the_body():
    # base_link 몸체 x -0.0745..0.0387, y +-0.047: 뒤쪽이 앞쪽보다 멀리 나옴
    a = arena.ARENA
    c, half = arena.footprint_box(torch.zeros(1, 2), torch.zeros(1), a.footprint_x, a.footprint_y)
    assert half == pytest.approx((0.0566, 0.047)) and c[0].tolist() == pytest.approx([-0.0179, 0])
    behind = torch.tensor([[[-0.3, -1.0, -0.08, 1.0]]])  # 뒤 모서리 5.5 mm 뒤의 면
    cl = arena.footprint_clearance(c, torch.zeros(1), half, behind)
    assert cl.item() == pytest.approx(0.0055, abs=1e-6)
    # 돌리면(yaw pi) 뒤쪽이 +x 를 향함
    c2, _ = arena.footprint_box(torch.zeros(1, 2), torch.tensor([math.pi]), a.footprint_x,
                                a.footprint_y)
    assert c2[0].tolist() == pytest.approx([0.0179, 0.0], abs=1e-6)


def test_handover_start_turns_towards_the_path_with_the_chosen_error():
    torch.manual_seed(0)
    n = 20000
    path_dir = (torch.rand(n) * 2 - 1) * math.pi
    yaw, w = arena.handover_start(path_dir, torch.rand(n, 4))
    err = torch.atan2(torch.sin(yaw - path_dir), torch.cos(yaw - path_dir))
    assert torch.all(torch.sign(w) == -torch.sign(err))  # w 는 오차를 0 쪽으로 돌림
    assert set(w.abs().unique().tolist()) == {0.5, 1.0}
    a = err.abs().rad2deg()
    normal = a < 30
    assert torch.all((a[normal] >= 16 - 1e-3) & (a[normal] <= 26 + 1e-3))
    assert torch.all((a[~normal] >= 40 - 1e-3) & (a[~normal] <= 160 + 1e-3))
    assert abs(normal.float().mean().item() - 0.5) < 0.02
    assert abs((err > 0).float().mean().item() - 0.5) < 0.02


def test_path_heading_points_along_the_path_start():
    path = torch.tensor([[[0.0, 0.0], [0.05, 0.0], [0.05, 0.2], [0.05, 0.4]]])
    assert torch.allclose(arena.path_heading(path), torch.tensor([math.atan2(0.2, 0.05)]))
