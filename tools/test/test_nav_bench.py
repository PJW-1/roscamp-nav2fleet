import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "nav_bench"))

import metrics as mt  # noqa: E402
import movers as mv  # noqa: E402

CONFIG = ROOT / "tools" / "nav_bench" / "scenarios.yaml"


# ---- 설정 ----
def test_the_config_loads_and_every_scenario_names_known_movers():
    cfg = mv.load(CONFIG)
    assert set(cfg["scenarios"]) == {"a_hall", "b_static", "c_moving", "d_robot"}
    assert cfg["controllers"] == ["FollowPathGoal", "FollowPathMPPI"]


def test_every_mover_stands_above_the_lidar_plane():
    for m in mv.load(CONFIG)["movers"].values():
        height = m.size[2] if m.shape == "box" else m.size[1]
        assert height >= 0.20, m.name


def test_every_mover_path_stays_inside_the_arena_and_off_the_shelves():
    cfg = mv.load(CONFIG)
    statics = mt.arena_rects()[4:]
    for m in cfg["movers"].values():
        length = mv.path_length(m.path) if len(m.path) > 1 else 0.0
        for k in range(21):
            pose = mv.pose_at(m, length * k / 20)
            assert 0.0 < pose[0] < 1.98 and 0.0 < pose[1] < 0.98, m.name
            shape = mv.mover_shape(m, pose)
            assert min(mv.poly_dist(shape, r) for r in statics) > 0.0, m.name


def test_a_bad_mover_is_refused(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(CONFIG.read_text(encoding="utf-8").replace("shape: cylinder", "shape: cone"),
                   encoding="utf-8")
    with pytest.raises(ValueError, match="shape"):
        mv.load(bad)


# ---- 움직임 ----
def test_a_mover_goes_back_and_forth_along_its_path():
    m = mv.Mover("p", "box", (0.1, 0.1, 0.2), ((0.0, 0.0), (1.0, 0.0)), 0.5)
    assert mv.pose_at(m, 0.5)[:2] == pytest.approx((0.5, 0.0))
    assert mv.pose_at(m, 1.5)[:2] == pytest.approx((0.5, 0.0))
    assert mv.pose_at(m, 1.5)[2] == pytest.approx(math.pi)
    assert mv.pose_at(m, 2.25)[:2] == pytest.approx((0.25, 0.0))


def test_a_one_way_mover_stops_at_the_end_and_yields_to_the_robot():
    m = mv.Mover("r", "box", (0.1, 0.1, 0.2), ((0.0, 0.0), (1.0, 0.0)), 0.5, once=True,
                 yield_m=0.25)
    assert mv.pose_at(m, 5.0)[:2] == pytest.approx((1.0, 0.0))
    assert mv.step(m, 0.2, 0.1, (0.3, 0.0), mv.pose_at(m, 0.2)) == 0.2
    assert mv.step(m, 0.2, 0.1, (0.9, 0.0), mv.pose_at(m, 0.2)) == pytest.approx(0.25)


# ---- 지표 ----
def test_sign_changes_skip_the_deadband():
    assert mt.sign_changes([0.1, -0.1, 0.01, -0.2, 0.3], 0.02) == 2
    assert mt.sign_changes([0.1, 0.005, -0.005, 0.2], 0.02) == 0


def test_a_robot_that_stands_ten_seconds_counts_as_blocked_once():
    truth = [[t * 0.5, 0.0, 0.0, 0.0] for t in range(30)]
    assert mt.blocked_events(truth, 10.0, 0.05) == 1
    moving = [[t * 0.5, t * 0.02, 0.0, 0.0] for t in range(30)]
    assert mt.blocked_events(moving, 10.0, 0.05) == 0


def test_the_body_distance_sees_a_shelf_touch_as_a_collision():
    body = mt.footprint()
    statics = [mv.rect(0.0, 0.0, 1.0, 0.1)]
    near = mt.body_clearance([[0.0, 0.5, 0.2, 0.0]], body, statics, [], [])
    assert near[0][1] == pytest.approx(0.2 - 0.056 - 0.1, abs=1e-6)
    touch = mt.body_clearance([[0.0, 0.5, 0.15, 0.0]], body, statics, [], [])
    assert touch[0][1] == 0.0
    assert mt.events_below(near + touch + near + touch, 0.005) == 2


def test_the_body_distance_includes_moving_obstacles():
    body = mt.footprint()
    m = mv.Mover("p", "cylinder", (0.04, 0.25), ((1.0, 0.5),), 0.0)
    track = [[0.0, [1.0, 0.5, 0.0]]]
    out = mt.body_clearance([[0.0, 0.8, 0.5, 0.0]], body, [], [m], track)
    assert out[0][1] == pytest.approx(1.0 - 0.04 - 0.84, abs=2e-3)
