"""실물 시험용 PC 도킹 대역의 판정과 두 대 실물 시나리오. ROS 없이 돈다."""
from pathlib import Path

import pytest
from fake_robots.dock_wait import (
    REACH_M,
    dock_done,
    dock_outcome,
    dock_phase,
    dock_wait_problems,
    pose_fresh,
)
from fake_robots.scenario import load_scenario, scenario_map_file
from fleet_core.traffic.site import load_site

SCENARIO = Path(__file__).parents[1] / "scenarios" / "real_two_orders.yaml"
GZ_V7 = Path(__file__).parents[3] / "sim" / "fleet_gz_sim" / "scenarios" / "gz_v7.yaml"


# ---- 단계와 끝남 ----
def test_phase_goes_approach_align_done():
    assert dock_phase(0.0, 3.0) == "APPROACH"
    assert dock_phase(1.4, 3.0) == "APPROACH"
    assert dock_phase(1.6, 3.0) == "ALIGN"
    assert dock_phase(3.0, 3.0) == "DONE"


def test_done_only_after_the_wait():
    assert not dock_done(2.99, 3.0)
    assert dock_done(3.0, 3.0)
    assert dock_done(0.0, 0.0)


# ---- 위치 판정 ----
def test_outcome_inside_reach_succeeds_with_cm_errors():
    ok, ex, ey, eyaw = dock_outcome((1.90, 0.48, 0.3), (1.88, 0.495))
    assert ok
    assert ex == pytest.approx(2.0)
    assert ey == pytest.approx(-1.5)
    assert eyaw == 0.0


def test_outcome_on_the_reach_edge_succeeds_and_beyond_fails():
    assert dock_outcome((1.88 + REACH_M, 0.495), (1.88, 0.495))[0]
    assert not dock_outcome((1.88 + REACH_M + 0.01, 0.495), (1.88, 0.495))[0]


def test_state_must_be_recent():
    assert pose_fresh(10.0, 11.5)
    assert not pose_fresh(10.0, 12.5)
    assert not pose_fresh(None, 12.5)


# ---- launch에서 켜도 되는지 ----
def test_no_problems_for_real_robots_on_wall_clock():
    assert dock_wait_problems(["pode1", "pode2"], [], sim_clock=False) == []


def test_refused_on_sim_clock():
    found = dock_wait_problems(["pode1"], [], sim_clock=True)
    assert len(found) == 1 and "use_sim_clock" in found[0]


def test_refused_for_a_robot_that_is_also_faked():
    found = dock_wait_problems(["pode1", "pode2"], ["pode2", "pode3"], sim_clock=False)
    assert len(found) == 1 and "pode2" in found[0]


def test_refused_for_bad_names():
    assert dock_wait_problems(["pode1;rm"], [], sim_clock=False)
    assert dock_wait_problems(["2pode"], [], sim_clock=False)


# ---- 두 대 실물 시나리오 ----
def test_real_two_orders_is_v7_two_robots_wall_speed():
    assert scenario_map_file(SCENARIO) == "v7"
    site = load_site("v7")
    scn = load_scenario(str(SCENARIO), site)
    assert sorted(scn.robots) == ["pode1", "pode2"]
    assert scn.robots["pode1"].start == "P-1"
    assert scn.robots["pode2"].start == "P-2"
    assert scn.speed_factor == 1.0
    assert scn.dock.duration_s == 3.0
    assert scn.faults == []


def test_real_two_orders_are_the_first_four_of_gz_v7_shifted_20_s():
    """사용자 결정(11-18): 시뮬(gz_v7)과 같게, 건수만 4건. 로봇을 띄울 시간으로 20 s 늦춘다."""
    site = load_site("v7")
    real = load_scenario(str(SCENARIO), site)
    sim = load_scenario(str(GZ_V7), site)
    assert len(real.orders) == 4
    assert [o.at_s for o in real.orders] == [20.0, 40.0, 60.0, 80.0]
    assert [(o.id, o.items) for o in real.orders] == [(o.id, o.items) for o in sim.orders[:4]]
    assert [o.at_s for o in real.orders] == [o.at_s + 20.0 for o in sim.orders[:4]]
    assert real.arm == sim.arm
    assert real.dock == sim.dock
    assert real.stock == sim.stock


# ---- 세 대 실물 시나리오 ----
THREE = Path(__file__).parents[1] / "scenarios" / "real_three_orders.yaml"


def test_real_three_orders_is_v7_three_robots_wall_speed():
    assert scenario_map_file(THREE) == "v7"
    site = load_site("v7")
    scn = load_scenario(str(THREE), site)
    assert sorted(scn.robots) == ["pode1", "pode2", "pode3"]
    assert [scn.robots[r].start for r in ("pode1", "pode2", "pode3")] == ["P-1", "P-2", "P-3"]
    assert scn.speed_factor == 1.0
    assert scn.dock.duration_s == 3.0
    assert scn.faults == []


def test_real_three_orders_equal_real_two_orders_except_pode3():
    site = load_site("v7")
    two = load_scenario(str(SCENARIO), site)
    three = load_scenario(str(THREE), site)
    assert [(o.at_s, o.id, o.items) for o in three.orders] == [
        (o.at_s, o.id, o.items) for o in two.orders]
    assert three.arm == two.arm
    assert three.dock == two.dock
    assert three.stock == two.stock
    assert three.expect == two.expect
    for r in ("pode1", "pode2"):
        assert three.robots[r] == two.robots[r]
