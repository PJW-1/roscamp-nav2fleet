"""fleet_check: 로봇 사이 최근접 거리, 정지, 쥔 구역 밖 위치. ROS 없음."""
import math

import pytest

pytest.importorskip("fleet_core")
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_gz_sim.fleet_check import FleetCheck  # noqa: E402

SITE = load_fleet_map("d2")
ROBOTS = ["pode1", "pode2", "pode3"]
W = SITE.node_pose("W")
ZONE_W = SITE.nodes["W"].zone  # WORK_DOCK


def check(**kw):
    return FleetCheck(SITE, ROBOTS, **kw)


def test_min_distance_uses_samples_taken_at_about_the_same_time():
    c = check()
    c.on_truth("pode1", 1.00, 0.0, 0.0, 0.0, 0.0)
    c.on_truth("pode2", 1.02, 0.5, 0.0, 0.0, 0.0)
    c.on_truth("pode3", 1.03, 3.0, 0.0, 0.0, 0.0)
    c.on_truth("pode1", 5.00, 0.45, 0.0, 0.0, 0.0)  # pode2의 마지막 표본은 4 s 전
    s = c.summary()
    assert s["min_distance_m"] == pytest.approx(0.5)
    assert s["min_distance_pair"] == ["pode1", "pode2"]
    assert s["min_distance_at_s"] == pytest.approx(1.02)


def test_min_distance_compares_every_sample_inside_the_sync_window():
    """pode1이 표본 둘을 보낸 뒤 pode2가 하나를 보낸다: 먼저 온 쪽이 더 가까운 쌍이다."""
    c = check(sync_s=0.1)
    c.on_truth("pode1", 1.00, 0.30, 0.0, 0.0, 0.0)
    c.on_truth("pode1", 1.03, 0.60, 0.0, 0.0, 0.0)
    c.on_truth("pode2", 1.05, 0.0, 0.0, 0.0, 0.0)
    assert c.summary()["min_distance_m"] == pytest.approx(0.30)


def test_a_late_sample_still_meets_the_samples_of_its_time():
    """토픽이 늦게 처리되어 pode2의 0.05 s 표본이 pode1의 0.30 s 표본보다 뒤에 올 수 있다."""
    c = check(sync_s=0.1)
    c.on_truth("pode1", 0.00, 0.20, 0.0, 0.0, 0.0)
    c.on_truth("pode1", 0.30, 0.90, 0.0, 0.0, 0.0)
    c.on_truth("pode2", 0.05, 0.0, 0.0, 0.0, 0.0)
    assert c.summary()["min_distance_m"] == pytest.approx(0.20)


def test_no_pair_seen_together_gives_no_distance():
    c = check()
    c.on_truth("pode1", 1.0, 0.0, 0.0, 0.0, 0.0)
    assert c.summary()["min_distance_m"] is None


def drive(c, robot, t0, t1, speed, dt=0.1, turn=0.0):
    t = t0
    while t < t1 - 1e-9:
        c.on_truth(robot, t, 0.0, 0.0, speed, turn)
        t += dt
    return t


def test_a_stop_is_counted_once_after_moving_and_standing_long_enough():
    c = check(stop_min_s=0.5)
    t = drive(c, "pode1", 0.0, 2.0, 0.2)
    t = drive(c, "pode1", t, t + 0.3, 0.0)  # 너무 짧음
    t = drive(c, "pode1", t, t + 1.0, 0.2)
    t = drive(c, "pode1", t, t + 3.0, 0.0)  # 아무리 길어도 정지 한 번
    s = c.summary()["robots"]["pode1"]
    assert s["stops"] == 1


def test_turning_on_the_spot_is_not_a_stop():
    c = check(stop_min_s=0.5)
    t = drive(c, "pode1", 0.0, 1.0, 0.2)
    drive(c, "pode1", t, t + 2.0, 0.0, turn=0.8)
    assert c.summary()["robots"]["pode1"]["stops"] == 0


def test_standing_from_the_start_is_not_a_stop():
    c = check(stop_min_s=0.5)
    drive(c, "pode1", 0.0, 5.0, 0.0)
    assert c.summary()["robots"]["pode1"]["stops"] == 0


def test_stops_while_the_robot_reports_moving_are_counted_apart():
    c = check(stop_min_s=0.5)
    c.on_state("pode1", "MOVING")
    t = drive(c, "pode1", 0.0, 1.0, 0.2)
    t = drive(c, "pode1", t, t + 1.0, 0.0)
    c.on_state("pode1", "IDLE")
    t = drive(c, "pode1", t, t + 1.0, 0.2)
    drive(c, "pode1", t, t + 1.0, 0.0)
    s = c.summary()["robots"]["pode1"]
    assert s["stops"] == 2 and s["stops_while_moving"] == 1


def test_positions_count_as_outside_only_after_the_fleet_publishes_zones():
    c = check()
    c.on_truth("pode1", 1.0, *W, 0.0, 0.0)
    assert c.summary()["robots"]["pode1"]["outside_held_samples"] == 0
    c.on_zones({ZONE_W: "pode1"})
    c.on_truth("pode1", 1.1, W[0] + 2.0, W[1] + 2.0, 0.0, 0.0)
    c.on_truth("pode1", 1.2, *W, 0.0, 0.0)
    c.on_truth("pode1", 1.3, W[0] + 0.01, W[1], 0.0, 0.0)
    s = c.summary()["robots"]["pode1"]
    assert s["outside_held_samples"] == 1 and s["zone_samples"] == 3
    assert c.summary()["outside_held_samples"] == 1


def test_holding_no_zone_is_counted_apart_from_outside():
    """주차 칸에서 기다리는 로봇은 구역을 쥐지 않아도 '밖'이 아니다."""
    c = check()
    c.on_zones({ZONE_W: "pode2"})
    c.on_truth("pode1", 1.0, *W, 0.0, 0.0)
    s = c.summary()
    assert s["robots"]["pode1"]["outside_held_samples"] == 0
    assert s["robots"]["pode1"]["no_zone_held_samples"] == 1
    assert s["no_zone_held_samples"] == 1 and s["zone_samples"] == 1


def test_far_from_every_held_zone_is_outside():
    c = check()
    c.on_zones({ZONE_W: "pode1"})
    c.on_truth("pode1", 1.0, W[0] + 2.0, W[1] + 2.0, 0.0, 0.0)
    assert c.summary()["robots"]["pode1"]["outside_held_samples"] == 1


def a_point_off_zone_w(lo, hi):
    for r in range(1, 60):
        for k in range(16):
            angle = k * math.pi / 8
            p = (W[0] + 0.01 * r * math.cos(angle), W[1] + 0.01 * r * math.sin(angle))
            if lo < SITE.distance_to_zone(*p, ZONE_W) < hi:
                return p
    raise AssertionError("no such point")


def test_margin_forgives_a_few_centimetres():
    off = a_point_off_zone_w(0.02, 0.08)
    strict, loose = check(margin_m=0.0), check(margin_m=0.10)
    for c in (strict, loose):
        c.on_zones({ZONE_W: "pode1"})
        c.on_truth("pode1", 1.0, *off, 0.0, 0.0)
    assert strict.summary()["robots"]["pode1"]["outside_held_samples"] == 1
    assert loose.summary()["robots"]["pode1"]["outside_held_samples"] == 0


def test_summary_reports_time_spans_and_real_time_factor():
    c = check()
    c.on_truth("pode1", 10.0, 0.0, 0.0, 0.0, 0.0, wall=100.0)
    c.on_truth("pode1", 20.0, 0.0, 0.0, 0.0, 0.0, wall=120.0)
    s = c.summary()
    assert s["sim_span_s"] == pytest.approx(10.0)
    assert s["wall_span_s"] == pytest.approx(20.0)
    assert s["real_time_factor"] == pytest.approx(0.5)


def test_unknown_robots_are_ignored():
    c = check()
    c.on_truth("pode9", 1.0, 0.0, 0.0, 0.0, 0.0)
    c.on_state("pode9", "MOVING")
    assert set(c.summary()["robots"]) == set(ROBOTS)


# ---- 여러 로봇이 쥔 구역(shared, lane) ----


def test_a_robot_named_among_several_holders_holds_the_zone():
    """ZoneView.holder는 shared 구역의 로봇들을 쉼표로 잇는다(TRAFFIC_CONTRACT 3)."""
    c = check()
    c.on_zones({"HALL": "pode1,pode3", ZONE_W: "pode2"})
    hall = SITE.node_pose("W_Q")
    c.on_truth("pode1", 1.0, *hall, 0.0, 0.0)
    c.on_truth("pode3", 1.0, *hall, 0.0, 0.0)
    c.on_truth("pode2", 1.0, *hall, 0.0, 0.0)  # 도크만 쥐고 HALL에 서 있음
    robots = c.summary()["robots"]
    assert robots["pode1"]["outside_held_samples"] == 0
    assert robots["pode3"]["outside_held_samples"] == 0
    assert robots["pode2"]["outside_held_samples"] == 1


def test_a_name_that_only_starts_like_a_holder_is_not_one():
    c = check()
    c.on_zones({ZONE_W: "pode10"})
    c.on_truth("pode1", 1.0, *W, 0.0, 0.0)
    assert c.summary()["robots"]["pode1"]["no_zone_held_samples"] == 1


# keepout 침범 깊이: 마당에 있는 몸이 쥐지 않은 좁은 구역에 걸친 깊이

BODY = [(0.06, 0.06), (-0.06, 0.06), (-0.06, -0.06), (0.06, -0.06)]


def body_check(**kw):
    return FleetCheck(SITE, ROBOTS, body=BODY, **kw)


def test_a_body_corner_inside_a_zone_the_robot_does_not_hold_is_a_keepout_intrusion():
    """J_W는 x 0.30..0.50, y 0.79..0.99. 마당 (0.40, 0.76)에서 동쪽을 보는 로봇의 북쪽 모서리는 y 0.82라 0.03 m 안이다."""
    c = body_check()
    c.on_zones({"HALL": "pode1"})
    c.on_truth("pode1", 1.0, 0.40, 0.60, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] == 0.0
    c.on_truth("pode1", 2.0, 0.40, 0.76, 0.0, 0.0, yaw=0.0)
    s = c.summary()
    assert s["keepout_depth_m"] == pytest.approx(0.03)
    assert (s["keepout_zone"], s["keepout_robot"], s["keepout_at_s"]) == ("J_W", "pode1", 2.0)
    assert s["robots"]["pode1"]["keepout_samples"] == 1
    assert s["robots"]["pode1"]["keepout_depth_m"] == pytest.approx(0.03)
    assert s["robots"]["pode1"]["keepout_checked_samples"] == 2
    assert s["robots"]["pode2"]["keepout_depth_m"] is None, "never seen in a yard"


def test_the_keepout_depth_follows_the_heading():
    c = body_check()
    c.on_zones({"HALL": "pode1"})
    c.on_truth("pode1", 1.0, 0.40, 0.72, 0.0, 0.0, yaw=math.pi / 4)
    assert c.summary()["keepout_depth_m"] == pytest.approx(
        0.72 + 0.06 * math.sqrt(2) - 0.79, abs=1e-3)


def test_a_zone_the_robot_holds_or_has_just_left_is_no_intrusion():
    c = body_check(left_s=5.0)
    c.on_zones({"HALL": "pode1", "J_W": "pode1"})
    c.on_truth("pode1", 1.0, 0.40, 0.76, 0.0, 0.0, yaw=0.0)
    c.on_zones({"HALL": "pode1", "J_W": ""})
    c.on_truth("pode1", 4.0, 0.40, 0.76, 0.0, 0.0, yaw=0.0)  # 꼬리가 아직 떠나는 중
    assert c.summary()["keepout_depth_m"] == 0.0
    c.on_truth("pode1", 7.0, 0.40, 0.76, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] == pytest.approx(0.03)


def test_a_robot_on_a_fixed_line_is_not_measured_against_the_next_zone():
    """중심이 J_W(마당 아님) 안이면 코가 거기서 하는 일은 고정 경로 주행이다."""
    c = body_check()
    c.on_zones({"J_W": "pode1"})
    c.on_truth("pode1", 1.0, 0.46, 0.89, 0.0, 0.0, yaw=0.0)  # 코가 EXPRESS에 0.02 m 들어감
    assert c.summary()["keepout_depth_m"] is None, "not measured there"
    c.on_truth("pode2", 1.0, 0.40, 0.60, 0.0, 0.0, yaw=0.0)  # 다른 로봇, 마당 안
    assert c.summary()["keepout_depth_m"] == 0.0


def test_the_keepout_depth_counts_only_after_the_fleet_publishes_zones():
    """측정하지 못한 것은 '침입 없음'이 아니다: 판정이 통과시키면 안 된다."""
    c = body_check()
    c.on_truth("pode1", 1.0, 0.40, 0.76, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] is None
    c.on_zones({"HALL": "pode1"})
    c.on_truth("pode1", 1.1, 0.40, 0.60, 0.0, 0.0)  # 방향 없음
    assert c.summary()["keepout_depth_m"] is None
    c.on_truth("pode1", 1.2, 0.40, 0.60, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] == 0.0


def test_a_zone_held_only_between_two_pose_samples_still_counts_as_just_left():
    c = body_check(left_s=5.0)
    c.on_zones({"HALL": "pode1"})
    c.on_truth("pode1", 1.0, 0.40, 0.60, 0.0, 0.0, yaw=0.0)
    c.on_zones({"HALL": "pode1", "J_W": "pode1"})
    c.on_zones({"HALL": "pode1", "J_W": ""})
    c.on_truth("pode1", 1.1, 0.40, 0.76, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] == 0.0
    c.on_truth("pode1", 6.5, 0.40, 0.76, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] == pytest.approx(0.03)


def test_the_keepout_depth_is_not_rounded_down_to_the_limit():
    """0.0204 m는 0.02 m 한도에서 0.020으로 읽히면 안 된다."""
    c = body_check()
    c.on_zones({"HALL": "pode1"})
    c.on_truth("pode1", 1.0, 0.40, 0.79 + 0.0204 - 0.06, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] > 0.0203


def test_a_zone_corner_that_cuts_into_a_side_of_the_body_is_measured():
    """얇은 구역이 오른쪽 면으로 0.01 m 들어오지만 몸 모서리는 모두 밖이다."""
    from types import SimpleNamespace

    from fleet_core.fleet_map.model import SHARED
    zones = {"YARD": SimpleNamespace(id="YARD", kind=SHARED, rects=((-1.0, -1.0, 0.05, 1.0),)),
             "THIN": SimpleNamespace(id="THIN", kind="dock", rects=((0.05, -0.01, 0.10, 0.01),))}
    site = SimpleNamespace(
        zones=zones,
        zone_at=lambda x, y: next((z.id for z in zones.values() for x0, y0, x1, y1 in z.rects
                                   if x0 <= x <= x1 and y0 <= y <= y1), None),
        distance_to_zone=SITE.distance_to_zone.__func__.__get__(SimpleNamespace(zones=zones)))
    c = FleetCheck(site, ["pode1"], body=BODY)
    c.on_zones({"YARD": "pode1"})
    c.on_truth("pode1", 1.0, 0.0, 0.0, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] == pytest.approx(0.01, abs=1e-3)


def test_without_a_body_the_keepout_depth_is_not_measured():
    c = check()
    c.on_zones({"HALL": "pode1"})
    c.on_truth("pode1", 1.0, 0.40, 0.76, 0.0, 0.0, yaw=0.0)
    assert c.summary()["keepout_depth_m"] is None


def test_the_body_is_the_footprint_nav2_runs_with():
    from pathlib import Path

    import yaml
    from fleet_gz_sim.fleet_check import footprint_of
    repo = Path(__file__).resolve().parents[3]
    nav = yaml.safe_load((repo / "pode/pode_nav_config/params/nav2_params.yaml")
                         .read_text(encoding="utf-8"))
    body = footprint_of(nav)
    assert len(body) >= 4 and all(len(p) == 2 for p in body)
    assert max(abs(y) for _, y in body) == pytest.approx(0.056)
    assert footprint_of({}) is None
