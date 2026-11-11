"""안전 수치 두 개(너무 이른 구역 풀기, 너무 가까운 두 로봇). ROS 없이 돈다."""
import pytest
from fake_robots.safety_meter import SafetyMeter
from fleet_core.fleet_map import load_fleet_map

SITE = load_fleet_map("d2")  # J_W: x 0.30..0.50, y 0.79..0.99


def meter(**kw):
    return SafetyMeter(SITE, 0.08, **kw)


def test_robots_apart_count_nothing():
    m = meter()
    m.on_pose("r1", 0.0, 0.0)
    m.on_pose("r2", 1.0, 0.0)
    m.tick(0.0)
    assert m.near == 0
    assert m.min_gap_m == pytest.approx(1.0)


def test_coming_close_counts_once_per_time_for_each_pair():
    m = meter()
    m.on_pose("r1", 0.0, 0.0)
    m.on_pose("r2", 0.1, 0.0)
    m.tick(0.0)
    m.tick(0.1)
    assert m.near == 1
    m.on_pose("r2", 0.5, 0.0)
    m.tick(0.2)
    m.on_pose("r2", 0.1, 0.0)
    m.tick(0.3)
    assert m.near == 2
    assert m.min_gap_m == pytest.approx(0.1)


def test_a_robot_off_the_course_does_not_count():
    m = meter(on_course=lambda robot: robot != "gone")
    m.on_pose("r1", 0.0, 0.0)
    m.on_pose("gone", 0.0, 0.0)
    m.tick(0.0)
    assert m.near == 0


def test_a_release_with_the_body_next_to_the_zone_is_early():
    m = meter()
    m.on_pose("r1", 0.40, 0.89)
    m.on_zones(0.0, {"J_W": "r1"})
    m.on_pose("r1", 0.40, 0.79 - 0.07)
    m.on_zones(1.0, {"J_W": ""})
    assert m.early == 1
    assert m.min_release_m == pytest.approx(0.07)


def test_a_release_past_the_radius_is_not_early():
    m = meter()
    m.on_zones(0.0, {"J_W": "r1"})
    m.on_pose("r1", 0.40, 0.65)
    m.on_zones(1.0, {"J_W": ""})
    assert (m.early, m.releases) == (0, 1)


def test_one_of_several_holders_leaving_a_shared_zone_is_a_release_of_that_robot():
    m = meter()
    m.on_pose("r1", 0.40, 0.89)
    m.on_pose("r3", 1.0, 0.4)
    m.on_zones(0.0, {"HALL": "r1,r3"})
    m.on_zones(1.0, {"HALL": "r3,#blocked"})
    assert (m.releases, m.early) == (1, 0)
    assert m.min_release_m == pytest.approx(0.89 - 0.79)  # HALL에는 진열대 옆 띠(y 0.79까지)도 든다


def test_summary_is_plain_data_with_nothing_measured():
    summary = meter().summary()
    assert summary["early"] == 0 and summary["near"] == 0
    assert summary["min_gap_m"] is None and summary["min_release_m"] is None
