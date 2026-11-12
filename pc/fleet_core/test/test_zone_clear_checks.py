"""지나가는 로봇이 확인하면 막힘 표시를 지운다(추가 지시서 3의 4-3)."""
from fleet_core.common.commands import NO_ANSWER, CheckClearCmd
from zone_harness import Harness

POINT = (1.0, 0.65)  # 마당(HALL) 안, N_W-N_E 선 위
KEYS = [("N_W", "N_E"), ("N_E", "N_W")]


def world_with_a_mark(starts=None):
    """pinky1이 POINT에서 막혀 표시를 냈다. 관제는 아직 묻지 않았다."""
    world = Harness(starts=starts or {"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    t = world.traffic
    mark = t._mark_point(t._robots["pinky1"], POINT, KEYS, world.now)
    return world, t, mark


def put(world, robot, xy):
    world.bots[robot].x, world.bots[robot].y = xy
    world.report()


def asks(t):
    return [c for c in t.take_output() if isinstance(c, CheckClearCmd)]


def test_a_robot_passing_within_range_is_asked_once_per_interval_and_the_reporter_never():
    world, t, mark = world_with_a_mark()
    put(world, "pinky1", (1.0, 0.60))  # 표시를 낸 로봇은 묻지 않는다
    put(world, "pinky2", (1.5, 0.65))  # 0.5 m: 안
    put(world, "pinky3", (1.75, 0.30))  # 밖
    t.tick_progress(world.now + 4.9)
    assert asks(t) == [], "not before check_interval_s after the mark"
    t.tick_progress(world.now + 5.0)
    [cmd] = asks(t)
    assert (cmd.robot, cmd.x, cmd.y, cmd.radius_m) == ("pinky2", 1.0, 0.65, 0.10)
    t.tick_progress(world.now + 9.9)
    assert asks(t) == [], "one question per mark per interval"


def test_two_clear_answers_in_a_row_remove_the_mark():
    world, t, mark = world_with_a_mark()
    put(world, "pinky2", (1.3, 0.65))
    world.now = 5.0
    t.tick_progress(world.now)
    [first] = asks(t)
    t.on_clear_check("pinky2", first.check_id, "clear")
    assert ("N_W", "N_E") in t._blocked, "one clear is not enough"
    world.now = 10.0
    t.tick_progress(world.now)
    [second] = asks(t)
    t.on_clear_check("pinky2", second.check_id, "clear")
    assert not t._marks and not any(key in t._blocked for key in KEYS)
    [event] = [e for e in t.take_output() if getattr(e, "code", "") == "BLOCK_CLEARED"]
    assert event.robot == "pinky2" and "marked 10 s ago" in event.detail


def test_a_blocked_answer_restarts_the_clock_and_resets_the_clear_count():
    world, t, mark = world_with_a_mark()
    put(world, "pinky2", (1.3, 0.65))
    world.now = 5.0
    t.tick_progress(world.now)
    t.on_clear_check("pinky2", asks(t)[0].check_id, "clear")
    world.now = 50.0
    t.tick_progress(world.now)
    t.on_clear_check("pinky2", asks(t)[0].check_id, "blocked")
    assert mark.clears == 0 and mark.until == 110.0 and t._blocked[KEYS[0]] == 110.0
    assert "BLOCK_CONFIRMED" in [getattr(e, "code", "") for e in t.take_output()]
    world.now = 70.0
    t.tick_progress(world.now)  # 예전이면 60 s에 지웠다
    assert t._marks and KEYS[0] in t._blocked


def test_late_and_strange_answers_change_nothing():
    world, t, mark = world_with_a_mark()
    put(world, "pinky2", (1.3, 0.65))
    world.now = 5.0
    t.tick_progress(world.now)
    cmd = asks(t)[0]
    t.on_clear_check("pinky2", cmd.check_id, "unknown")
    t.on_clear_check("pinky2", cmd.check_id, "clear")  # 이미 답했다
    t.on_clear_check("pinky3", 999, "clear")
    assert mark.clears == 0 and mark.until == 60.0


def clear_once(world, t, now):
    world.now = now
    t.tick_progress(now)
    t.on_clear_check("pinky2", asks(t)[0].check_id, "clear")


def test_an_unknown_answer_breaks_the_run_of_clears():
    """clear, unknown, clear는 이어진 두 번이 아니다(실패도 fleet_manager가 unknown으로 준다)."""
    world, t, mark = world_with_a_mark()
    put(world, "pinky2", (1.3, 0.65))
    clear_once(world, t, 5.0)
    world.now = 10.0
    t.tick_progress(world.now)
    t.on_clear_check("pinky2", asks(t)[0].check_id, "unknown")
    assert mark.clears == 0
    clear_once(world, t, 15.0)
    assert t._marks and KEYS[0] in t._blocked, "only one clear in a row"


def test_no_answer_breaks_the_run_and_puts_the_silent_robot_behind_others():
    world, t, mark = world_with_a_mark()
    put(world, "pinky2", (1.3, 0.65))  # 0.3 m: 가장 가깝다
    put(world, "pinky3", (1.5, 0.65))  # 0.5 m
    clear_once(world, t, 5.0)
    world.now = 10.0
    t.tick_progress(world.now)
    [silent] = asks(t)
    assert silent.robot == "pinky2"  # 답하지 않는다
    world.now = 15.0
    t.tick_progress(world.now)
    [cmd] = asks(t)
    assert mark.clears == 0, "a question with no answer breaks the run"
    assert cmd.robot == "pinky3", "the robot that did not answer goes behind the others"
    t.on_clear_check("pinky2", silent.check_id, "clear")  # 늦은 답은 세지 않는다
    assert mark.clears == 0


def test_a_robot_that_could_not_be_asked_stays_behind_the_others():
    """서비스가 없거나 부르기가 실패하면 fleet_manager가 NO_ANSWER를 준다. 답이 아니다."""
    world, t, mark = world_with_a_mark()
    put(world, "pinky2", (1.3, 0.65))  # 가장 가깝지만 서버가 없다
    put(world, "pinky3", (1.5, 0.65))
    for now, robot in ((5.0, "pinky2"), (10.0, "pinky3"), (15.0, "pinky3")):
        world.now = now
        t.tick_progress(now)
        [cmd] = asks(t)
        assert cmd.robot == robot, now
        if robot == "pinky2":
            t.on_clear_check("pinky2", cmd.check_id, NO_ANSWER)
            assert mark.check == 0 and "pinky2" in mark.silent
        else:
            t.on_clear_check("pinky3", cmd.check_id, "clear")
    assert not t._marks, "pinky3's two clears remove it"


def test_a_new_block_report_drops_the_question_in_flight():
    """표시를 새로 고치는 막힘 보고가 오면, 그 전에 보낸 질문의 clear 답은 세지 않는다."""
    world, t, mark = world_with_a_mark()
    put(world, "pinky2", (1.3, 0.65))
    world.now = 5.0
    t.tick_progress(world.now)
    [cmd] = asks(t)
    again = t._mark_point(t._robots["pinky1"], POINT, KEYS, 6.0)
    assert again is mark
    t.on_clear_check("pinky2", cmd.check_id, "clear")
    assert mark.clears == 0 and mark.check == 0


def test_nobody_passing_drops_the_mark_after_the_retry_time_and_a_repeat_doubles_it():
    world, t, mark = world_with_a_mark()
    m = t._robots["pinky1"]
    t.tick_progress(60.0)
    assert not t._marks and KEYS[0] not in t._blocked
    again = t._mark_point(m, (1.05, 0.66), KEYS, 70.0)
    assert again.retry_s == 120.0 and again.until == 190.0
    t.tick_progress(190.0)
    third = t._mark_point(m, POINT, KEYS, 200.0)
    assert third.retry_s == 240.0
    t.tick_progress(440.0)
    assert t._mark_point(m, POINT, KEYS, 450.0).retry_s == 300.0, "capped"


def test_a_cleared_mark_forgets_its_history():
    world, t, mark = world_with_a_mark()
    m = t._robots["pinky1"]
    t.tick_progress(60.0)
    again = t._mark_point(m, POINT, KEYS, 70.0)
    put(world, "pinky2", (1.3, 0.65))
    for now in (75.0, 80.0):
        world.now = now
        t.tick_progress(now)
        t.on_clear_check("pinky2", asks(t)[0].check_id, "clear")
    assert not t._marks and again.id not in t._marks
    assert t._mark_point(m, POINT, KEYS, 90.0).retry_s == 60.0
