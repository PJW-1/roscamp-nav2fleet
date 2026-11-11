"""TRAFFIC_CONTRACT 7절 불변식: 가짜 grants를 넣어 위반을 모으는지 본다."""
import math

from fake_robots.world.invariants import (
    CAPACITY,
    COMMAND,
    LANE_WAY,
    OUTSIDE,
    STANDING,
    TOUCH,
    Invariants,
)
from fake_robots.world.recording import FakeGrants
from fake_robots.world.robot import SimRobot
from fleet_core.common.commands import MoveCmd, RetreatCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map

SITE = load_fleet_map("d2")
PARAMS = load_params()


def _check(inv, grants, robots, t=0.1, views=None, quiet=None):
    inv.check(t, grants, robots, views or {}, quiet or {})
    return [v.code for v in inv.violations]


def test_capacity_and_one_robot_per_dock_zone():
    inv, g = Invariants(SITE, PARAMS), FakeGrants()
    g.give("pinky1", "J_W")
    assert _check(inv, g, {}) == []
    g.give("pinky2", "J_W")
    assert _check(inv, g, {}) == [CAPACITY]
    assert _check(inv, g, {}, t=0.2) == [CAPACITY]  # 이어지는 위반은 한 번만


def test_a_robot_outside_its_held_zones():
    inv, g = Invariants(SITE, PARAMS), FakeGrants()
    g.give("pinky1", "HALL")
    robots = {"pinky1": SimRobot("pinky1", *SITE.node_pose("N_W"))}
    assert _check(inv, g, robots) == []
    robots["pinky1"].y = 0.85  # J_W 안
    assert _check(inv, g, robots, t=0.2) == [OUTSIDE]
    inv2 = Invariants(SITE, PARAMS)
    off = {"pinky1": {"on_map": True, "fault": "OUT_OF_SERVICE"}}
    assert _check(inv2, g, robots, views=off) == []


def test_two_robots_in_the_lane_driving_opposite_ways():
    inv = Invariants(SITE, PARAMS)
    a, b = SimRobot("pinky1", 0.8, 0.89), SimRobot("pinky2", 1.2, 0.89)
    _check(inv, None, {"pinky1": a, "pinky2": b})
    a.x += 0.01
    b.x += 0.01
    assert _check(inv, None, {"pinky1": a, "pinky2": b}, t=0.2) == []
    b.x -= 0.02
    a.x += 0.01
    assert _check(inv, None, {"pinky1": a, "pinky2": b}, t=0.3) == [LANE_WAY]


def test_standing_in_a_lane_or_junction_unless_blocked():
    inv = Invariants(SITE, PARAMS)
    a = SimRobot("pinky1", *SITE.node_pose("X_W"))
    b = SimRobot("pinky2", 1.0, 0.89)
    robots = {"pinky1": a, "pinky2": b}
    for k in range(1, 60):
        _check(inv, None, robots, t=k * 0.1, quiet={"pinky2": True})
    assert [(v.code, v.robot) for v in inv.violations] == [(STANDING, "pinky1")]


def test_a_robot_creeping_slowly_through_the_lane_is_not_standing():
    inv = Invariants(SITE, PARAMS)
    a = SimRobot("pinky1", 0.6, 0.89)
    for k in range(1, 100):
        a.x += 0.004  # 0.04 m/s
        _check(inv, None, {"pinky1": a}, t=k * 0.1)
    assert inv.violations == []


def test_a_one_point_follow_path_is_checked_from_where_the_robot_stands():
    inv, g = Invariants(SITE, PARAMS), FakeGrants()
    g.give("pinky1", "HALL")
    xw = SITE.node_pose("X_W")
    cmd = MoveCmd("pinky1", 1, "X_W", *xw, 0.0)
    inv.check_command(0.1, g, cmd, "follow_path", SITE.node_pose("N_W"))
    assert [v.code for v in inv.violations] == [COMMAND]


def test_robots_closer_than_two_radii():
    inv = Invariants(SITE, PARAMS)
    a, b = SimRobot("pinky1", 1.0, 0.4), SimRobot("pinky2", 1.0 + 0.15, 0.4)
    assert _check(inv, None, {"pinky1": a, "pinky2": b}) == [TOUCH]


def test_commands_must_stay_in_held_zones():
    inv, g = Invariants(SITE, PARAMS), FakeGrants()
    g.give("pinky1", "HALL")
    x, y = SITE.node_pose("N_E")
    inv.check_command(0.1, g, MoveCmd("pinky1", 1, "N_E", x, y, 0.0, mode="nav_goal"),
                      "nav_goal")
    assert inv.violations == []
    xw = SITE.node_pose("X_W")
    inv.check_command(0.1, g, MoveCmd("pinky1", 2, "X_W", *xw, 0.0, mode="nav_goal"),
                      "nav_goal")
    path = ((*SITE.node_pose("N_W"), 0.0), (*xw, 0.0))
    inv.check_command(0.1, g, MoveCmd("pinky1", 3, "X_W", *xw, 0.0, path=path), "follow_path")
    g.give("pinky1", "J_W")
    inv.check_command(0.1, g, MoveCmd("pinky1", 4, "X_W", *xw, 0.0, path=path), "follow_path")
    back = ((*SITE.node_pose("W"), 0.0), (*xw, 0.0))
    inv.check_command(0.1, g, RetreatCmd("pinky1", 5, 0.22, 0.05, back), "retreat")
    assert [(v.code, v.detail.split()[1]) for v in inv.violations] == [
        (COMMAND, "2"), (COMMAND, "3"), (COMMAND, "5")]
    assert math.isclose(SITE.node_pose("W")[1], xw[1])
