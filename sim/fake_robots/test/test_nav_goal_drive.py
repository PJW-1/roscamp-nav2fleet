"""ROS 가짜 Pode의 nav_goal 흉내(NavGoalDrive). ROS 없이 Motion 위에서 돈다."""
import math

from fake_robots.motion import Motion
from fake_robots.nav_goal_drive import NavGoalDrive
from fake_robots.world.free_space import FreeSpace
from fleet_core.fleet_map import load_fleet_map

SITE = load_fleet_map("d2")
RADIUS = 0.08
SPACE = FreeSpace(SITE, RADIUS)
DT = 0.1


def drive(nav, motion, others=(), until_s=30.0, speed=0.2):
    """노드의 틱처럼: update, 앞이 비면 간다. (끝 시각, 이유)."""
    t = 0.0
    while t < until_s:
        reason = nav.update(t, motion, others)
        if reason:
            return t, reason
        if nav.arrived(motion):
            return t, "ARRIVED"
        if nav.clear_ahead(motion, speed * DT, others):
            motion.step(DT, speed)
        t = round(t + DT, 6)
    return t, None


def test_a_reachable_goal_is_reached_on_a_planned_way():
    here, there = SITE.node_pose("P-1"), SITE.node_pose("P-3")
    m = Motion(*here)
    nav = NavGoalDrive(SPACE, RADIUS, there, 0.0, here)
    t, how = drive(nav, m)
    assert how == "ARRIVED" and math.dist((m.x, m.y), there) < 1e-6
    assert not nav.blocked(t)


def test_a_goal_off_the_free_space_is_aborted_soon():
    here = SITE.node_pose("P-1")
    m = Motion(*here)
    nav = NavGoalDrive(SPACE, RADIUS, (-1.0, -1.0), 0.0, here)
    t, how = drive(nav, m)
    assert how == "NAV_ABORTED" and t <= 1.0


def test_a_robot_on_the_straight_line_is_driven_round():
    here, there = SITE.node_pose("P-1"), SITE.node_pose("P-3")
    other = (1.0, 0.15, RADIUS)  # 주차 줄 한가운데에 선 로봇
    m = Motion(*here)
    nav = NavGoalDrive(SPACE, RADIUS, there, 0.0, here)
    closest = math.inf
    t = 0.0
    while t < 30.0 and not nav.arrived(m):
        assert nav.update(t, m, [other]) is None
        if nav.clear_ahead(m, 0.02, [other]):
            m.step(DT, 0.2)
        closest = min(closest, math.dist((m.x, m.y), other[:2]))
        t = round(t + DT, 6)
    assert nav.arrived(m) and closest >= 2 * RADIUS


def test_a_robot_standing_on_the_goal_blocks_after_ten_seconds():
    """목표 위에 선 로봇은 비킬 것으로 보고 다가가, 닿기 전에 서서 기다린다."""
    here, there = SITE.node_pose("P-1"), SITE.node_pose("P-2")
    other = (*there, RADIUS)
    m = Motion(*here)
    nav = NavGoalDrive(SPACE, RADIUS, there, 0.0, here)
    t, how = drive(nav, m, [other], until_s=8.0)
    assert how is None and not nav.blocked(t)
    while t < 11.0:  # 지난 시각을 묻지 않는다(보고용 막힘은 상태로 남는다)
        assert nav.update(t, m, [other]) is None
        if nav.clear_ahead(m, 0.2 * DT, [other]):
            m.step(DT, 0.2)
        t = round(t + DT, 6)
    assert nav.blocked(t)
    assert math.dist((m.x, m.y), other[:2]) >= 2 * RADIUS


def test_blocked_clears_only_after_unblock_m_from_where_it_started():
    """Pode와 같이(unblock_m, 2026-11-17) 몇 cm 꿈틀대서는 BLOCKED가 풀리지 않는다."""
    here, there = SITE.node_pose("P-1"), SITE.node_pose("P-3")
    m = Motion(*here)
    nav = NavGoalDrive(SPACE, RADIUS, there, 0.0, here)
    t = 0.0
    while t <= 10.05:
        nav.update(t, m, [])
        t = round(t + DT, 6)
    assert nav.blocked(t)
    x0 = m.x
    for dx in (0.06, 0.10, 0.04, 0.0, 0.08):
        m.x = x0 + dx
        nav.update(t, m, [])
        assert nav.blocked(t), dx
        t = round(t + DT, 6)
    m.x = x0 + 0.16
    nav.update(t, m, [])
    assert not nav.blocked(t)
