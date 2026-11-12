"""TRAFFIC_CONTRACT 7절 불변식: 가짜 grants를 넣어 위반을 모으는지 본다."""
import math

from fake_robots.world.invariants import (
    CAPACITY,
    COMMAND,
    DOCK_TWO,
    LANE_WAY,
    NODE,
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


def test_a_short_stay_in_a_shared_zone_not_held_is_only_counted():
    """Nav2 goal은 공유 구역을 가로질러 갈 수 있다. 교통 층은 자리가 나면 그 구역을 잡는다.
    자리가 있는데도 2초 넘게 안 잡히면 그때 3번 위반이다."""
    inv, g = Invariants(SITE, PARAMS), FakeGrants()
    g.give("pinky1", "J_W")
    robots = {"pinky1": SimRobot("pinky1", *SITE.node_pose("P-1"))}  # HALL 안
    assert _check(inv, g, robots, t=0.1) == []
    assert _check(inv, g, robots, t=2.0) == []
    assert inv.shared_outside == 1
    assert _check(inv, g, robots, t=2.2) == [OUTSIDE]
    g.give("pinky1", "HALL")
    _check(inv, g, robots, t=2.3)
    g.take("pinky1", "HALL")
    _check(inv, g, robots, t=2.4)
    assert inv.shared_outside == 2


def test_a_stay_in_a_full_shared_zone_is_no_violation():
    inv, g = Invariants(SITE, PARAMS), FakeGrants()
    g.give("pinky1", "J_W")
    for i in range(SITE.zones["HALL"].capacity):
        g.give(f"other{i}", "HALL")
    robots = {"pinky1": SimRobot("pinky1", *SITE.node_pose("P-1"))}
    for t in (0.1, 1.0, 2.5, 4.0):
        assert _check(inv, g, robots, t=t) == []


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


def test_two_robots_inside_one_dock_zone_by_true_position():
    """추가 지시서 2의 2절: 도크 구역(도크, 갈림목)에 실제로 두 대. 쥔 것과 상관없이 본다."""
    inv = Invariants(SITE, PARAMS)
    robots = {"pinky1": SimRobot("pinky1", 0.31, 0.89), "pinky2": SimRobot("pinky2", 0.49, 0.89)}
    assert _check(inv, None, robots) == [DOCK_TWO]  # 둘 다 J_W 안, 몸은 닿지 않는다
    robots["pinky2"].x = 0.52  # EXPRESS로 나갔다
    assert _check(inv, None, robots, t=0.2) == [DOCK_TWO]


def test_one_node_reserved_by_two_robots():
    inv, g = Invariants(SITE, PARAMS), FakeGrants()
    g.give("pinky1", "HALL")
    g.give("pinky2", "HALL")
    g.robot_nodes = {"pinky1": ("W_Q",), "pinky2": ("O_Q",)}
    robots = {n: SimRobot(n, *SITE.node_pose(p)) for n, p in (("pinky1", "P-1"),
                                                             ("pinky2", "P-3"))}
    assert _check(inv, g, robots) == []
    g.robot_nodes["pinky2"] = ("O_Q", "W_Q")
    assert _check(inv, g, robots, t=0.2) == [NODE]


def test_a_robot_standing_on_a_node_another_robot_reserved():
    inv, g = Invariants(SITE, PARAMS), FakeGrants()
    g.give("pinky1", "HALL")
    g.give("pinky2", "HALL")
    g.robot_nodes = {"pinky2": ("W_Q",)}
    robots = {"pinky1": SimRobot("pinky1", *SITE.node_pose("W_Q")),
              "pinky2": SimRobot("pinky2", *SITE.node_pose("P-3"))}
    robots["pinky1"].run = object()  # 지나가는 중이면 보지 않는다
    assert _check(inv, g, robots) == []
    robots["pinky1"].run = None
    assert _check(inv, g, robots, t=0.2) == [NODE]
