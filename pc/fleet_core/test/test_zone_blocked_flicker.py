"""BLOCKED가 깜빡여도 한 막힘 일화로 센다(관제 쪽 이력 현상).

로봇(pode_fsm)은 좁은 곳에서 RECOVERING, BLOCKED, 짧은 MOVING을 번갈아 보고한다. Nav2 goal이면
5 cm만 움직여도 BLOCKED가 풀린다. 짧은 틈마다 막힘 시계를 다시 세면 blocked_mark_s(10 s)에
닿지 못해 표시도 다른 길도 없이 제자리에서 꿈틀댄다. 같은 goal 안에서 실제로 나아가거나
(blocked_reset_m), 막힘 아닌 보고가 오래 이어지거나(blocked_reset_s), goal이 바뀔 때만 다시 센다.
"""
from dataclasses import replace

import pytest
from fleet_core.common.params import load_params
from zone_harness import Harness

ROBOT = "pode2"
# 긴 틈을 보는 시험: 멈춤 감시와 출발 감시가 goal을 거두지 않게 한다
NO_STALL = replace(load_params(), stall_s=1000.0, start_limit_s=0.0)


def called(params=None):
    """pode2가 이름으로 불려 Nav2 goal을 받고 제자리에 서 있다(속도 0)."""
    world = Harness(speed=0.0, params=params)
    world.step()
    world.call("WORK", ROBOT)
    world.step()
    assert world.moves(ROBOT)
    return world


def flicker(world, cycle, start=None):
    """ROBOT은 world.now 기준 cycle [(상태, 초), ...]를 되풀이해 보고한다. 다른 로봇은 그대로다."""
    plain = Harness._state.__get__(world)
    t0 = world.now if start is None else start
    period = sum(s for _, s in cycle)

    def state(bot):
        if bot.name != ROBOT:
            return plain(bot)
        t = (world.now - t0) % period
        for name, seconds in cycle:
            if t < seconds - 1e-9:
                return name
            t -= seconds
        return cycle[-1][0]
    world._state = state


def marked(world):
    return world.codes(ROBOT).count("ZONE_MARKED_BLOCKED")


def first_mark_at(world, limit_s):
    """limit_s 안에 처음 표시된 때까지 돌리고 걸린 시간을 돌려준다(없으면 None)."""
    t0 = world.now
    if world.run_until(lambda: marked(world) > 0, limit_s):
        return world.now - t0
    return None


def wiggle(world, dx):
    bot = world.bots[ROBOT]
    bot.x += dx


# ---- 깜빡여도 표시된다 ----

def test_blocked_with_short_moving_gaps_is_marked_after_the_mark_time():
    world = called()
    p = world.params
    flicker(world, [("BLOCKED", 3.0), ("MOVING", 0.5)])
    took = first_mark_at(world, 3 * p.blocked_mark_s)
    assert took is not None, "the timer restarted on every short gap"
    # 일화 첫 BLOCKED부터 blocked_mark_s 뒤, 그때 MOVING이면 다음 BLOCKED에서
    assert p.blocked_mark_s - 0.2 <= took <= p.blocked_mark_s + 1.0
    assert "REROUTE" in world.codes(ROBOT)
    assert world.codes(ROBOT).count("BLOCKED_WAIT") == 1
    # 다시 막힌 횟수는 표시 사건에 붙는다(진단용)
    mark = next(e.detail for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert mark.endswith(", after 3 short gaps")  # 0, 3.5, 7, 10.5 s에 BLOCKED


def test_recovering_inside_an_episode_does_not_restart_the_timer():
    world = called()
    p = world.params
    # 로봇 FSM 흉내: 1 s 서 있다가 RECOVERING 2 s, BLOCKED 2 s, 꿈틀거림(MOVING) 0.5 s
    flicker(world, [("BLOCKED", 2.0), ("MOVING", 0.5), ("RECOVERING", 2.0)])
    took = first_mark_at(world, 3 * p.blocked_mark_s)
    assert took is not None
    assert took <= p.blocked_mark_s + 3.0
    assert world.codes(ROBOT).count("BLOCKED_WAIT") == 1


def test_small_wiggles_during_the_gaps_do_not_count_as_progress():
    world = called()
    p = world.params
    flicker(world, [("BLOCKED", 3.0), ("MOVING", 0.5)])
    for i in range(round(3 * p.blocked_mark_s / 0.5)):
        if marked(world):
            break
        wiggle(world, 0.04 if i % 2 == 0 else -0.04)  # 5 cm보다 작은 앞뒤 꿈틀거림
        world.run(0.5)
    assert marked(world) == 1


def test_the_mark_waits_for_blocked_to_come_back():
    """기다린 시간이 넘었어도 지금 막힘이 아니면 표시하지 않고, BLOCKED가 돌아오면 곧 한다."""
    world = called()
    p = world.params
    flicker(world, [("BLOCKED", 3.0), ("MOVING", 9.0)])  # 9 s 틈은 blocked_reset_s보다 짧다
    world.run(p.blocked_mark_s + 1.0)  # 일화 11 s: MOVING 중
    assert marked(world) == 0
    world.run(1.5)  # 12 s에 BLOCKED가 돌아온다
    assert marked(world) == 1


@pytest.mark.parametrize("gap_s", [9.0, 11.0])
def test_nav_goal_re_block_gaps_with_small_wiggles_keep_the_episode(gap_s):
    """Nav2 goal 로봇은 5 cm 움직이면 BLOCKED가 풀리고, 다시 BLOCKED가 되기까지 blocked_after_s
    (10 s)가 걸린다(pode_fsm nav_goal.py). 그 사이의 틈도 일화를 끝내지 않는다."""
    world = called()
    p = world.params
    assert gap_s < p.blocked_reset_s
    flicker(world, [("BLOCKED", 1.0), ("MOVING", gap_s)])
    t0 = world.now
    for i in range(round(3 * (1.0 + gap_s) / 0.5)):
        if marked(world):
            break
        wiggle(world, 0.045 if i % 2 == 0 else -0.045)  # 5 cm 안쪽 꿈틀거림
        world.run(0.5)
    assert marked(world) == 1
    # 일화 첫 BLOCKED부터 blocked_mark_s를 넘긴 뒤 처음 돌아온 BLOCKED에서
    assert world.now - t0 <= 2 * (1.0 + gap_s) + 0.6


# ---- 다시 세는 때 ----

def test_real_progress_restarts_the_timer():
    world = called()
    p = world.params
    flicker(world, [("BLOCKED", 3.0), ("MOVING", 0.5)])
    world.run(3.2)  # 첫 틈(MOVING) 안
    wiggle(world, p.blocked_reset_m + 0.05)  # 일화 시작점에서 실제로 나아갔다
    world.run(0.3)
    took = first_mark_at(world, 3 * p.blocked_mark_s)
    assert took is not None
    # 새 일화는 3.5 s에 시작한다: 처음부터 10 s가 아니라 3.5 + 10 s
    assert took >= p.blocked_mark_s - 0.2


def test_a_long_gap_restarts_the_timer():
    world = called(NO_STALL)
    p = world.params
    flicker(world, [("BLOCKED", 3.0), ("MOVING", p.blocked_reset_s + 1.0)])
    world.run(3 * (3.0 + p.blocked_reset_s + 1.0) - 1.0)  # 네 번째 BLOCKED 전까지
    assert marked(world) == 0
    assert world.codes(ROBOT).count("BLOCKED_WAIT") == 3, "one per episode"


def test_a_new_goal_restarts_the_timer():
    world = called()
    p = world.params
    flicker(world, [("BLOCKED", 3.0), ("MOVING", 0.5)])
    world.run(6.6)  # 두 번째 틈 안
    m = world.traffic._robots[ROBOT]
    m.goal = replace(m.goal, id=m.goal.id + 1000)  # 틈 사이에 goal이 바뀌었다
    world.run(0.2)
    took = first_mark_at(world, 3 * p.blocked_mark_s)
    assert took is not None
    assert took >= p.blocked_mark_s - 0.5, "the old goal's wait was carried over"


# ---- 끝낼 때가 막힘이 돌아오는 보고와 겹쳐도 끝낸다 ----

def test_progress_reached_on_the_report_that_blocks_again_restarts_the_timer():
    world = called()
    p = world.params
    flicker(world, [("BLOCKED", 3.0), ("MOVING", 0.5)])
    world.run(3.2)  # 첫 틈 안
    wiggle(world, p.blocked_reset_m - 0.01)  # 아직 기준 안
    world.run(0.2)  # 마지막 MOVING 보고
    wiggle(world, 0.03)  # BLOCKED가 돌아오는 보고에서 기준을 넘는다
    took = first_mark_at(world, 3 * p.blocked_mark_s)
    assert took is not None
    # 새 일화는 BLOCKED가 돌아온 보고에서 시작한다(다음 틈까지 미루지 않는다)
    assert p.blocked_mark_s - 0.2 <= took <= p.blocked_mark_s + 1.0


def test_a_gap_that_reaches_the_reset_time_on_the_blocking_report_restarts_the_timer():
    world = called(NO_STALL)
    p = world.params
    gap = p.blocked_reset_s
    flicker(world, [("BLOCKED", 3.0), ("MOVING", gap)], start=0.0)  # 시각이 정확히 떨어지게
    world.run(3.0 + gap)  # 막힘 아닌 보고가 gap 동안 이어진 뒤 BLOCKED
    assert marked(world) == 0, "the old episode's wait was carried over"
    assert world.codes(ROBOT).count("BLOCKED_WAIT") == 2
    assert "GOAL_STALLED" not in world.codes(ROBOT)
