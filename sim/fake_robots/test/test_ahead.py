"""앞에 있는 다른 로봇을 보고 서기(ahead.py). ROS 없이 Motion 위에서 돈다."""
import math

import pytest
from fake_robots.ahead import PathWatch, nothing_ahead, seen, where_now, yields
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


def test_a_moving_robot_is_placed_where_it_is_now_not_where_it_was_last_seen():
    """ROS 가짜 Pode는 남의 자세를 10 Hz로 받는다. 0.2 m/s로 오는 로봇은 받은 자세보다 0.03 m쯤
    가까이 있다. 마지막 두 자세로 지금 자리를 어림한다."""
    view = seen(seen(None, 1.00, 0.50, 3.0), 1.02, 0.50, 3.1)
    assert where_now(view, 3.1) == pytest.approx((1.02, 0.50))
    assert where_now(view, 3.25) == pytest.approx((1.05, 0.50))


def test_a_standing_robot_stays_where_it_was_seen():
    view = seen(seen(None, 1.00, 0.50, 3.0), 1.00, 0.50, 3.1)
    assert where_now(view, 9.0) == pytest.approx((1.00, 0.50))
    assert where_now(seen(None, 1.00, 0.50, 3.0), 3.2) == pytest.approx((1.00, 0.50))


def test_a_robot_lifted_away_or_long_silent_is_not_carried_on():
    """들어낸 로봇은 한 번에 멀리 뛴다. 그 속도로 밀지 않는다. 소식이 끊긴 로봇도
    0.3초까지만 민다."""
    lifted = seen(seen(None, 1.00, 0.50, 3.0), -5.0, -5.0, 3.1)
    assert where_now(lifted, 3.3) == pytest.approx((-5.0, -5.0))
    late = seen(seen(None, 1.00, 0.50, 3.0), 1.02, 0.50, 3.1)
    assert where_now(late, 60.0) == pytest.approx((1.08, 0.50))
    gap = seen(seen(None, 1.00, 0.50, 3.0), 1.20, 0.50, 4.0)
    assert where_now(gap, 4.2) == pytest.approx((1.20, 0.50))


def test_a_robot_yields_to_one_that_moves_close_by():
    """가까이(0.26 m)에서 방금(1초 안) 움직인 로봇이 있으면 양보하는 쪽은 선다. 누가 양보하는지는 노드가 이름 순서로 정한다."""
    moving = seen(seen(None, 1.00, 0.50, 3.0), 1.02, 0.50, 3.1)
    assert yields(1.25, 0.50, [moving], 3.1)
    assert not yields(1.30, 0.50, [moving], 3.1), "the next parking bay is 0.28 m away"
    assert yields(1.25, 0.50, [moving], 4.0)
    assert not yields(1.25, 0.50, [moving], 4.2), "it has stood for a second"
    standing = seen(seen(None, 1.00, 0.50, 3.0), 1.00, 0.50, 3.1)
    assert not yields(1.25, 0.50, [standing], 3.1)
    assert not yields(1.25, 0.50, [], 3.1)


def test_two_poses_at_the_same_instant_give_no_speed_to_carry_on():
    """한 시계 걸음에 자세 둘이 오면 속도를 잴 수 없으니 앞의 속도를 물려주지 않는다."""
    moving = seen(seen(None, 1.00, 0.50, 3.0), 1.02, 0.50, 3.1)
    stopped = seen(moving, 1.03, 0.50, 3.1)
    assert where_now(stopped, 3.3) == pytest.approx((1.03, 0.50))


def test_a_clock_that_went_back_does_not_keep_a_robot_yielding():
    """시뮬 시계가 되감기면 남은 움직인 시각 때문에 선 로봇에게 양보하지 않는다."""
    moving = seen(seen(None, 1.00, 0.50, 30.0), 1.02, 0.50, 30.1)
    assert not yields(1.25, 0.50, [moving], 2.0)
    standing = seen(moving, 1.02, 0.50, 2.0)
    assert not yields(1.25, 0.50, [standing], 30.5), "the old time comes round again"


@pytest.mark.parametrize("late_ticks", [3, 4])
@pytest.mark.parametrize("starts_at", [0.0, 1.5, 3.0, 4.2, 4.8, 6.0])
def test_two_nav_robots_that_see_each_other_late_pass_without_overlap(late_ticks, starts_at):
    """O_Q로 가는 로봇과 N_E에서 W_Q로 떠나는 로봇이 마당에서 마주쳐도, 어림하고 한쪽이 양보하면 겹치지 않고 둘 다 닿는다."""
    a, b = Motion(*SITE.node_pose("N_W")), Motion(*SITE.node_pose("N_E"))
    nav_a = NavGoalDrive(SPACE, RADIUS, SITE.node_pose("O_Q"), 0.0, (a.x, a.y))
    nav_b, of_a, of_b, sent = None, None, None, []
    closest, t, tick = math.inf, 0.0, 0
    while t < 60.0 and not (nav_a.arrived(a) and nav_b is not None and nav_b.arrived(b)):
        if tick % 2 == 0:
            sent.append((tick + late_ticks, (a.x, a.y), (b.x, b.y)))
        while sent and sent[0][0] <= tick:
            _, at_a, at_b = sent.pop(0)
            of_a, of_b = seen(of_a, *at_a, t), seen(of_b, *at_b, t)
        if nav_b is None and t >= starts_at:
            nav_b = NavGoalDrive(SPACE, RADIUS, SITE.node_pose("W_Q"), t, (b.x, b.y))
        sees_b = [] if of_b is None else [(*where_now(of_b, t + DT), RADIUS)]
        sees_a = [] if of_a is None else [(*where_now(of_a, t + DT), RADIUS)]
        nav_a.update(t, a, sees_b)
        go_a = nav_a.clear_ahead(a, STEP, sees_b)
        go_b = False
        if nav_b is not None:
            nav_b.update(t, b, sees_a)
            go_b = (nav_b.clear_ahead(b, STEP, sees_a)
                    and not yields(b.x, b.y, [of_a] if of_a else [], t))
        if go_a:
            a.step(DT, SPEED)
        if go_b:
            b.step(DT, SPEED)
        closest = min(closest, math.dist((a.x, a.y), (b.x, b.y)))
        t, tick = round(t + DT, 6), tick + 1
    assert closest >= 2 * RADIUS, f"bodies overlap: {closest:.3f} m apart"
    assert t < 60.0, "they blocked each other and never arrived"
