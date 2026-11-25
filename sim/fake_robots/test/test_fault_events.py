"""사건으로 시작하고 끝나는 고장: 로봇 속도가 바뀌어도 시나리오가 노린 순간에 난다. ROS가 필요 없다."""
import math

import pytest
from fake_robots.motion import REINSTATE_AFTER_S, FaultSchedule
from fake_robots.scenario import ScenarioError, parse_scenario
from fake_robots.scenario_cfg import Fault
from fleet_core.fleet_map import load_fleet_map


@pytest.fixture(scope="module")
def site():
    return load_fleet_map("d2")


def scenario(*faults):
    return {"seed": 1, "speed_factor": 1.0, "timeout_s": 100,
            "stock": {"arm_left": {"A": 2}, "arm_right": {"C": 1}},
            "orders": [{"at_s": 0, "id": "O-1", "items": {"A": 1}}],
            "robots": {"pode1": {"start": "P-1"}, "pode2": {"start": "P-2"}},
            "faults": list(faults), "expect": {"orders_shipped_min": 1}}


def test_a_stuck_fault_starts_its_delay_at_the_robots_own_event():
    f = Fault(at_s=4.0, robot="pode1", type="stuck", after_event="DOCKED",
              after_detail="at OUTBOUND")
    s = FaultSchedule([f], "pode1")
    assert not s.stuck(1000.0)
    s.on_event(50.0, "pode1", "DOCKED", "at WORK: x 1 cm")
    s.on_event(55.0, "pode2", "DOCKED", "at OUTBOUND: x 1 cm")
    assert not s.stuck(1000.0), "another robot's dock or another dock does not start it"
    s.on_event(66.0, "pode1", "DOCKED", "at OUTBOUND: x 1 cm")
    s.on_event(90.0, "pode1", "DOCKED", "at OUTBOUND: x 1 cm")
    assert (s.stuck(69.9), s.stuck(70.0)) == (False, True)
    assert s.start_of(f) == 70.0, "only the first matching event counts"


def test_a_central_event_naming_the_robot_matches_too():
    f = Fault(at_s=1.5, robot="pode2", type="operator_remove", after_event="ORDER_SHIPPED")
    s = FaultSchedule([f], "pode2")
    s.on_event(40.0, "central", "ORDER_SHIPPED", "O-002 robot=pode1 placed=B:1")
    assert not s.removed(100.0)
    s.on_event(48.0, "central", "ORDER_SHIPPED", "O-001 robot=pode2 placed=A:1")
    assert (s.removed(49.4), s.removed(49.5)) == (False, True)


def test_an_obstacle_stays_until_the_fleet_marks_the_way_blocked():
    f = Fault(at_s=20.0, robot="pode1", type="blocked", duration_s=120.0,
              until_event="ZONE_MARKED_BLOCKED")
    s = FaultSchedule([f], "pode1")
    assert (s.obstacle(19.9), s.obstacle(20.0), s.obstacle(60.0)) == (False, True, True)
    s.on_event(10.0, "pode1", "ZONE_MARKED_BLOCKED", "before the object, does not count")
    assert s.obstacle(60.0)
    s.on_event(70.0, "pode1", "ZONE_MARKED_BLOCKED", "W_Q->N_W in HALL")
    assert (s.obstacle(69.9), s.obstacle(70.0)) == (True, False)
    assert s.end_of(f) == 70.0


def test_without_the_event_the_obstacle_ends_at_its_duration():
    f = Fault(at_s=20.0, robot="pode1", type="blocked", duration_s=120.0,
              until_event="ZONE_MARKED_BLOCKED")
    s = FaultSchedule([f], "pode1")
    assert (s.obstacle(139.9), s.obstacle(140.0)) == (True, False)


def test_a_fault_that_never_fires_starts_at_infinity():
    f = Fault(at_s=1.0, robot="pode1", type="stuck", after_event="DOCKED")
    assert FaultSchedule([f], "pode1").start_of(f) == math.inf


def test_a_put_back_waits_for_a_removal_that_fires_late():
    gone = Fault(at_s=2.5, robot="pode2", type="operator_remove", after_event="ORDER_SHIPPED")
    back = Fault(at_s=160.0, robot="pode2", type="operator_reinstate", start="P-1")
    s = FaultSchedule([gone, back], "pode2")
    assert (s.removed(170.0), s.put_back(170.0)) == (False, None), "nothing to put back yet"
    s.on_event(200.0, "central", "ORDER_SHIPPED", "O-001 robot=pode2 placed=A:1")
    assert (s.removed(202.5), s.put_back(202.5)) == (True, None)
    assert s.start_of(back) == 202.5 + REINSTATE_AFTER_S, "the operator gets time to clear it"
    assert (s.removed(212.4), s.put_back(212.6)) == (True, "P-1")


def test_a_removal_on_time_keeps_the_put_back_time():
    gone = Fault(at_s=2.5, robot="pode2", type="operator_remove", after_event="ORDER_SHIPPED")
    back = Fault(at_s=160.0, robot="pode2", type="operator_reinstate", start="P-1")
    s = FaultSchedule([gone, back], "pode2")
    s.on_event(50.0, "central", "ORDER_SHIPPED", "O-001 robot=pode2 placed=A:1")
    assert s.start_of(back) == 160.0


def test_a_timed_removal_keeps_a_put_back_soon_after_it():
    gone = Fault(at_s=0.5, robot="pode2", type="operator_remove")
    back = Fault(at_s=2.0, robot="pode2", type="operator_reinstate", start="P-1")
    assert FaultSchedule([gone, back], "pode2").start_of(back) == 2.0


def test_an_event_removal_is_the_only_removal_of_its_robot(site):
    with pytest.raises(ScenarioError):
        parse_scenario(scenario(
            {"at_s": 1, "robot": "pode1", "type": "operator_remove", "after_event": "DOCKED"},
            {"at_s": 50, "robot": "pode1", "type": "operator_reinstate", "start": "P-1"},
            {"at_s": 80, "robot": "pode1", "type": "operator_remove"}), site)


def test_event_fields_are_read(site):
    sc = parse_scenario(scenario(
        {"at_s": 4, "robot": "pode1", "type": "stuck", "after_event": "DOCKED",
         "after_detail": "at OUTBOUND"},
        {"at_s": 20, "robot": "pode2", "type": "blocked", "duration_s": 120,
         "until_event": "ZONE_MARKED_BLOCKED"}), site)
    stuck, blocked = sc.faults
    assert (stuck.after_event, stuck.after_detail, stuck.until_event) == (
        "DOCKED", "at OUTBOUND", "")
    assert (blocked.after_event, blocked.until_event) == ("", "ZONE_MARKED_BLOCKED")


@pytest.mark.parametrize("fault", [
    {"at_s": 1, "robot": "pode1", "type": "comm_loss", "duration_s": 2, "after_event": "DOCKED"},
    {"at_s": 1, "robot": "pode1", "type": "stuck", "until_event": "DOCKED"},
    {"at_s": 1, "robot": "pode1", "type": "stuck", "after_detail": "at OUTBOUND"},
    {"at_s": -1, "robot": "pode1", "type": "stuck", "after_event": "DOCKED"},
])
def test_event_fields_only_where_they_mean_something(site, fault):
    with pytest.raises(ScenarioError):
        parse_scenario(scenario(fault), site)
