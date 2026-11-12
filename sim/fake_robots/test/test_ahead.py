"""앞에 있는 다른 로봇을 보고 서기(ahead.py). ROS 없이 Motion 위에서 돈다."""
import math

import pytest
from fake_robots.ahead import PathWatch, nothing_ahead
from fake_robots.motion import Motion
from fake_robots.nav_goal_drive import NavGoalDrive
from fake_robots.world.free_space import FreeSpace
from fake_robots.world.runs import FOLLOW_BLOCKED_S
from fleet_core.fleet_map import load_fleet_map

SITE = load_fleet_map("d2")
RADIUS = 0.08
SPACE = FreeSpace(SITE, RADIUS)
DT = 0.05
SPEED = 0.2
STEP = SPEED * DT


def on_the_line(*nodes):
    """고정 선의 첫 지점에 서서 path 모드로 끝 지점을 향하는 Motion."""
    points = [SITE.node_pose(n) for n in nodes]
    m = Motion(*points[0])
    m.set_target(*points[-1], via=points[1:-1], start=points[0])
    return m


def test_a_robot_on_the_line_ahead_stops_a_path_robot_until_it_leaves():
    m = on_the_line("X_E", "N_E")  # 아래(-y)로 간다
    x, y = SITE.node_pose("N_E")
    ahead = [(x, y + 0.05, RADIUS)]
    watch = PathWatch(RADIUS)
    assert not watch.clear_ahead(0.0, m, STEP, ahead)
    assert not watch.blocked(FOLLOW_BLOCKED_S - 0.1)
    assert not watch.clear_ahead(FOLLOW_BLOCKED_S, m, STEP, ahead)
    assert watch.blocked(FOLLOW_BLOCKED_S)
    assert watch.clear_ahead(FOLLOW_BLOCKED_S + DT, m, STEP, [])
    assert not watch.blocked(FOLLOW_BLOCKED_S + DT)


def test_a_back_up_restarts_the_standing_time():
    """자기 복구로 후진한 시간은 서 있던 시간이 아니다."""
    m = on_the_line("X_E", "N_E")
    x, y = SITE.node_pose("N_E")
    ahead = [(x, y + 0.05, RADIUS)]
    watch = PathWatch(RADIUS)
    assert not watch.clear_ahead(0.0, m, STEP, ahead)
    watch.moved()
    assert not watch.clear_ahead(0.8, m, STEP, ahead)
    assert not watch.blocked(FOLLOW_BLOCKED_S + 0.5)
    assert watch.blocked(0.8 + FOLLOW_BLOCKED_S)


def test_a_robot_beside_or_behind_does_not_stop_a_path_robot():
    """가까이 있어도 가면 멀어지는 로봇 앞에서는 서지 않는다(붙은 둘이 떨어질 수 있어야 한다)."""
    m = on_the_line("X_E", "N_E")
    beside = (m.x + 2 * RADIUS, m.y, RADIUS)
    behind = (m.x, m.y + 2 * RADIUS, RADIUS)
    assert nothing_ahead(m, RADIUS, STEP, [beside, behind])
    assert PathWatch(RADIUS).clear_ahead(0.0, m, STEP, [beside, behind])


def test_a_robot_that_is_not_moving_has_nothing_ahead():
    here = SITE.node_pose("N_E")
    assert nothing_ahead(Motion(*here), RADIUS, STEP, [(*here, RADIUS)])


@pytest.mark.parametrize("leaves_at", [4.4, 4.6, 4.8, 5.0, 5.2, 5.4, 5.6])
def test_a_robot_coming_down_the_line_waits_for_one_crossing_below_it(leaves_at):
    """R3: 고정 선 X_E > N_E를 내려오는 로봇과, N_E 바로 아래를 지나 O_Q로 가는 Nav2 goal 로봇."""
    down = on_the_line("X_E", "N_E")
    watch = PathWatch(RADIUS)
    start, there = SITE.node_pose("N_W"), SITE.node_pose("O_Q")
    across = Motion(*start)
    nav = NavGoalDrive(SPACE, RADIUS, there, 0.0, start)
    closest, t = math.inf, 0.0
    while t < 20.0 and (down.moving or not nav.arrived(across)):
        if t >= leaves_at and watch.clear_ahead(t, down, STEP, [(across.x, across.y, RADIUS)]):
            down.step(DT, SPEED)
        other = [(down.x, down.y, RADIUS)]
        assert nav.update(t, across, other) is None
        if nav.clear_ahead(across, STEP, other):
            across.step(DT, SPEED)
        closest = min(closest, math.dist((down.x, down.y), (across.x, across.y)))
        t = round(t + DT, 6)
    assert not down.moving and nav.arrived(across), "one of them never arrived"
    assert closest >= 2 * RADIUS, f"bodies overlap: {closest:.3f} m apart"
