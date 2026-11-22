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
    assert set(cfg["scenarios"]) == {"a_hall", "b_static", "c_moving", "d_robot", "e_corridor"}
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


def test_two_crossing_rectangles_touch_even_with_every_corner_outside():
    long_x = mv.rect(-1.0, -0.1, 1.0, 0.1)
    long_y = mv.rect(-0.1, -1.0, 0.1, 1.0)
    assert mv.poly_dist(long_x, long_y) == 0.0
    assert mv.poly_dist(long_x, mv.rect(-1.0, 0.3, 1.0, 0.4)) == pytest.approx(0.2)


def test_the_metrics_count_only_the_leg_between_goal_sent_and_result():
    cfg = mv.load(CONFIG)
    rec = {"scenario": "a_hall", "controller": "FollowPathGoal", "repeat": 0, "status": "SUCCEEDED",
           "t_start": 10.0, "t_end": 20.0, "movers_names": [], "movers": [],
           "truth": [[5.0, 0.0, 0.0, 0.0], [10.0, 1.0, 0.55, 0.0], [20.0, 1.645, 0.55, 0.0]],
           "cmd": [[5.0, 0.0, 0.5], [6.0, 0.0, -0.5], [12.0, 0.1, 0.3]], "scan_min": [[5.0, 0.01]]}
    row = mt.leg_metrics(rec, cfg, mt.footprint(), mt.arena_rects())
    assert row["doridori"] == 0 and row["lidar_min_m"] is None
    assert row["path_m"] == pytest.approx(0.645) and row["ok"]


# ---- 3단계: 공정성, 통로 띠, 시간 나누기 ----
def test_the_corridor_scenario_leaves_a_gap_wide_enough_to_stay_in_the_corridor():
    cfg = mv.load(CONFIG)
    low, _ = cfg["corridor_band"]
    for name in cfg["scenarios"]["e_corridor"]:
        m = cfg["movers"][name]
        half = (m.size[1] / 2) if m.shape == "box" else m.size[0]
        for _, y in m.path:
            assert y - half - low >= 0.30 - 1e-9, name


def test_the_run_order_is_abba_so_ten_repeats_start_five_times_with_each():
    names = ["A", "B"]
    orders = [mt.run_order(r, names) for r in range(10)]
    assert orders[:4] == [["A", "B"], ["B", "A"], ["B", "A"], ["A", "B"]]
    assert sum(o[0] == "A" for o in orders) == 5


def test_a_start_pose_off_by_more_than_the_tolerance_is_flagged():
    start = {"x": 0.49, "y": 0.625, "yaw_deg": 110}
    assert not mt.pose_off((0.50, 0.63, math.radians(113)), start, 0.03, 5.0)
    assert mt.pose_off((0.53, 0.625, math.radians(110)), start, 0.03, 5.0)
    assert mt.pose_off((0.49, 0.625, math.radians(100)), start, 0.03, 5.0)
    assert not mt.pose_off((0.49, 0.625, math.radians(110 - 360)), start, 0.03, 5.0)


def test_the_band_check_fails_when_any_point_leaves_the_corridor():
    assert mt.in_band([(0.5, 0.62), (1.0, 0.56), (1.6, 0.55)], (0.405, 0.77))
    assert not mt.in_band([(0.5, 0.62), (1.0, 0.18), (1.6, 0.55)], (0.405, 0.77))
    assert mt.in_band([], (0.405, 0.77)) is None


def test_the_time_split_separates_waiting_turning_stopping_and_driving():
    cmd = [[0.0, 0.0, 0.0], [1.0, 0.0, 1.0], [1.1, 0.0, 1.0], [1.2, 0.0, 0.0], [1.3, 0.1, 0.0],
           [1.4, 0.1, 0.0], [1.5, 0.0, 0.0]]
    split = mt.time_split(cmd, 0.0, 1.5, 0.3)
    assert split["wait_s"] == pytest.approx(1.0)
    assert split["turn_s"] == pytest.approx(0.2)
    assert split["stop_s"] == pytest.approx(0.1)
    assert split["drive_s"] == pytest.approx(0.2)
    assert split["drive_vx"] == pytest.approx(0.1)
    tail = mt.time_split([[0.0, 0.0, 0.0], [1.0, 0.1, 0.0]], 0.0, 3.0, 0.3)
    assert tail["drive_s"] == pytest.approx(2.0)


def test_only_plans_inside_the_leg_window_decide_the_band():
    cfg = mv.load(CONFIG)
    rec = {"scenario": "a_hall", "controller": "FollowPathGoal", "repeat": 0, "status": "SUCCEEDED",
           "t_start": 10.0, "t_end": 20.0, "movers_names": [], "movers": [],
           "plans": [[9.0, [[1.0, 0.18]]], [10.5, [[1.0, 0.56]]]],
           "truth": [[10.0, 0.5, 0.62, 0.0], [20.0, 1.645, 0.55, 0.0]], "cmd": [], "scan_min": []}
    assert mt.leg_metrics(rec, cfg, mt.footprint(), mt.arena_rects())["plan_in_band"] is True
    rec["plans"] = [[9.0, [[1.0, 0.56]]]]
    assert mt.leg_metrics(rec, cfg, mt.footprint(), mt.arena_rects())["plan_in_band"] is None


def test_a_leg_records_its_start_pose_band_and_time_split():
    cfg = mv.load(CONFIG)
    rec = {"scenario": "a_hall", "controller": "FollowPathGoal", "repeat": 0, "status": "SUCCEEDED",
           "t_start": 10.0, "t_end": 20.0, "movers_names": [], "movers": [],
           "start_pose": [0.50, 0.62, math.radians(108)], "replaced": True,
           "plans": [[10.1, [[0.5, 0.62], [1.0, 0.56], [1.645, 0.55]]]],
           "truth": [[10.0, 0.5, 0.62, 0.0], [20.0, 1.645, 0.55, 0.0]],
           "cmd": [[10.0, 0.0, 0.0], [11.0, 0.1, 0.0], [12.0, 0.1, 0.0]], "scan_min": []}
    row = mt.leg_metrics(rec, cfg, mt.footprint(), mt.arena_rects())
    assert row["start_err_m"] == pytest.approx(math.hypot(0.01, 0.005))
    assert row["start_err_deg"] == pytest.approx(2.0)
    assert row["replaced"] is True
    assert row["plan_in_band"] and row["truth_in_band"] and row["in_corridor"]
    assert row["drive_s"] == pytest.approx(9.0)


def test_the_table_keeps_legs_outside_the_corridor_on_their_own_row_with_rates():
    import report
    cfg = mv.load(CONFIG)
    base = {"scenario": "e_corridor", "controller": "FollowPathGoal", "ok": True, "time_s": 30.0,
            "path_m": 1.5, "lidar_min_m": 0.1, "body_min_m": 0.1, "doridori": 6,
            "collisions": 1, "blocked": 0, "cpu_pct": 10.0, "cmd_dt_p95_ms": 52.0,
            "plan_in_band": True, "truth_in_band": True, "in_corridor": True}
    rows = [base, dict(base, in_corridor=False, truth_in_band=False), base]
    text = report.table(rows, cfg)
    corridor = [ln for ln in text.splitlines() if "| 통로 |" in ln]
    outside = [ln for ln in text.splitlines() if "| 밖 |" in ln]
    assert len(corridor) == 1 and len(outside) == 1
    assert "2/2" in corridor[0] and "| 2 (2.0/분, 0.67/m) |" in corridor[0]


# ---- 세 컨트롤러 ----
@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_controllers_rotate_so_each_takes_every_place_and_follows_every_other_equally(n):
    names = [chr(ord("A") + i) for i in range(n)]
    orders = [mt.run_order(r, names) for r in range(2 * n)]
    for place in range(n):
        assert sorted(o[place] for o in orders) == sorted(names * 2)
    pairs = [(o[i], o[i + 1]) for o in orders for i in range(n - 1)]
    counts = {pairs.count((a, b)) for a in names for b in names if a != b}
    assert len(counts) == 1
    assert [mt.run_order(r, ["A", "B"]) for r in range(4)] == [
        ["A", "B"], ["B", "A"], ["B", "A"], ["A", "B"]]


def test_welch_gives_the_difference_its_standard_error_and_t():
    diff, se, t = mt.welch([10.0, 12.0, 14.0], [7.0, 8.0, 9.0])
    assert diff == pytest.approx(4.0)
    assert se == pytest.approx(math.sqrt(4.0 / 3 + 1.0 / 3))
    assert t == pytest.approx(diff / se)
    assert mt.welch([1.0], [2.0, 3.0]) == (None, None, None)


def test_the_report_takes_the_controllers_from_the_legs_and_names_rl(tmp_path):
    import report
    cfg = mv.load(CONFIG)
    base = {"scenario": "a_hall", "controller": "FollowPathGoal", "ok": True, "time_s": 10.0,
            "path_m": 1.5, "lidar_min_m": 0.1, "body_min_m": 0.1, "doridori": 6,
            "collisions": 0, "blocked": 0, "cpu_pct": 10.0, "cmd_dt_p95_ms": 52.0,
            "plan_in_band": True, "truth_in_band": True, "in_corridor": True, "drive_vx": 0.2}
    rows = [dict(base, time_s=t) for t in (10.0, 11.0)] + \
           [dict(base, controller="FollowPathRL", time_s=t) for t in (6.0, 7.0)]
    cfg = report.with_controllers(cfg, rows)
    assert cfg["controllers"] == ["FollowPathGoal", "FollowPathMPPI", "FollowPathRL"]
    assert report.label("FollowPathRL") == "RL" and report.label("FollowPathGoal") == "RPP"
    assert report.label("FollowPathRLShim") == "RL+shim"
    assert report.label("FollowPathRLAlone") == "RL alone"
    assert report.label("FollowPathRLShimC") == "RL+shim c"
    assert report.label("FollowPathRLShimACD") == "RL+shim acd"
    assert any("| FollowPathRL |" in ln for ln in report.table(rows, cfg).splitlines())
    diffs = report.diff_table(rows, cfg)
    line = next(ln for ln in diffs.splitlines() if ln.startswith("| a_hall | RL - RPP | 시간 s"))
    assert "-4.00 ± 0.71" in line and "t -5.7" in line
    pytest.importorskip("matplotlib")
    report.figure(rows, cfg, tmp_path / "compare.png")
    assert (tmp_path / "compare.png").stat().st_size > 0


def test_the_track_plot_draws_a_config_with_one_scenario(tmp_path):
    import report
    pytest.importorskip("matplotlib")
    cfg = {"scenarios": {"a_hall": {}}, "controllers": ["FollowPathGoal"]}
    raw = [{"scenario": "a_hall", "controller": "FollowPathGoal",
            "truth": [[0.0, 0.2, 0.2, 0.0], [0.1, 0.3, 0.2, 0.0]]}]
    report.tracks(raw, cfg, tmp_path / "tracks.png")
    assert (tmp_path / "tracks.png").stat().st_size > 0
