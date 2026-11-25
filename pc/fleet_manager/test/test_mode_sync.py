"""운영 모드 맞추기(IR-73): /system/mode를 받기 전에는 아무것도 안 한다. 받으면 로봇마다 set_mode를 부르고,
다시 연결되면 다시 부르며, 모드가 다른 채 TRIES번 부르고도 다르면 그 Pode를 배정에서 뺀다. 팔은 빼지 않는다."""
from fleet_manager.mode_sync import ANSWER_S, NO_SERVER_S, RECONNECT_GAP_S, RETRY_S, TRIES, ModeSync

READY = lambda robot: True  # noqa: E731


def sync():
    return ModeSync(["pode1", "pode2"], ["arm_ship"])


def test_nothing_happens_before_the_system_mode_comes():
    s = sync()
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    for t in range(0, 20):
        assert s.due(float(t), READY) == []
    assert s.changes() == []


def test_the_system_mode_is_sent_to_every_robot_heard_from():
    s = sync()
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    s.on_state("arm_ship", "UNKNOWN", "a1", 0.0)
    assert s.on_system_mode("TEST")
    calls = s.due(0.1, READY)
    assert sorted(r for r, _ in calls) == ["arm_ship", "pode1"]
    assert len({rid for _, rid in calls}) == 2


def test_a_robot_that_takes_the_mode_is_not_asked_again():
    s = sync()
    s.on_system_mode("TEST")
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    [(robot, rid)] = s.due(0.0, READY)
    s.on_answer(robot, rid)
    for t in range(1, 30):
        s.on_state("pode1", "TEST", "b1", t * 0.2)
        assert s.due(t * 0.2, READY) == []
    assert s.changes() == []


def test_no_second_call_while_the_answer_is_pending():
    s = sync()
    s.on_system_mode("TEST")
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    assert len(s.due(0.0, READY)) == 1
    s.on_state("pode1", "UNKNOWN", "b1", 0.2)
    assert s.due(RETRY_S + 0.1, READY) == []


def test_a_robot_that_stays_in_another_mode_is_blocked_then_freed():
    s = sync()
    s.on_system_mode("TEST")
    t, asked = 0.0, 0
    while t < (TRIES + 2) * RETRY_S:
        s.on_state("pode1", "NORMAL", "b1", t)
        for robot, rid in s.due(t, READY):
            asked += 1
            s.on_answer(robot, rid)
        t += 0.2
    assert asked >= TRIES + 1
    [(robot, blocked, detail)] = s.changes()
    assert (robot, blocked) == ("pode1", True)
    assert "NORMAL" in detail and "TEST" in detail
    assert s.changes() == []
    s.on_state("pode1", "TEST", "b1", t)
    assert s.changes() == [("pode1", False, "")]


def test_it_is_not_blocked_before_the_tries_run_out():
    s = sync()
    s.on_system_mode("TEST")
    t = 0.0
    while t < TRIES * RETRY_S - 0.5:
        s.on_state("pode1", "NORMAL", "b1", t)
        for robot, rid in s.due(t, READY):
            s.on_answer(robot, rid)
        t += 0.2
    assert s.changes() == []


def test_a_robot_without_the_service_is_blocked_too():
    s = sync()
    s.on_system_mode("TEST")
    t = 0.0
    while t < (TRIES + 2) * RETRY_S:
        s.on_state("pode1", "", "b1", t)
        assert s.due(t, lambda robot: False) == []
        t += 0.2
    assert [(r, b) for r, b, _ in s.changes()] == [("pode1", True)]
    assert "pode1" in s.no_server


def test_an_arm_is_asked_but_never_blocked():
    s = sync()
    s.on_system_mode("TEST")
    t, asked = 0.0, 0
    while t < (TRIES + 2) * RETRY_S:
        s.on_state("arm_ship", "", "a1", t)
        asked += len(s.due(t, READY))
        t += 0.2
    assert asked >= TRIES + 1
    assert s.changes() == []


def test_a_reconnect_asks_again_even_when_the_mode_matches():
    s = sync()
    s.on_system_mode("TEST")
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    [(robot, rid)] = s.due(0.0, READY)
    s.on_answer(robot, rid)
    s.on_state("pode1", "TEST", "b1", 0.2)
    gap = 0.2 + RECONNECT_GAP_S + 0.1
    s.on_state("pode1", "TEST", "b1", gap)
    assert [r for r, _ in s.due(gap, READY)] == ["pode1"]


def test_a_new_boot_id_asks_again():
    s = sync()
    s.on_system_mode("TEST")
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    [(robot, rid)] = s.due(0.0, READY)
    s.on_answer(robot, rid)
    s.on_state("pode1", "TEST", "b1", 0.2)
    s.on_state("pode1", "TEST", "b2", 0.4)
    assert [r for r, _ in s.due(0.4, READY)] == ["pode1"]


def test_a_lost_robot_is_not_asked_until_it_comes_back():
    s = sync()
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    s.on_system_mode("TEST")
    assert s.due(RECONNECT_GAP_S + 1.0, READY) == []


def test_an_unanswered_call_is_sent_again_later():
    s = sync()
    s.on_system_mode("TEST")
    t, asked = 0.0, []
    while t < 10.0:
        s.on_state("pode1", "UNKNOWN", "b1", t)
        asked += s.due(t, READY)
        t += 0.2
    assert len(asked) >= 2
    assert len({rid for _, rid in asked}) == len(asked)


def test_a_new_system_mode_asks_everyone_again():
    s = sync()
    s.on_system_mode("TEST")
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    [(robot, rid)] = s.due(0.0, READY)
    s.on_answer(robot, rid)
    s.on_state("pode1", "TEST", "b1", 0.2)
    assert not s.on_system_mode("TEST")
    assert s.on_system_mode("NORMAL")
    assert [r for r, _ in s.due(0.4, READY)] == ["pode1"]


def test_a_pode_without_the_service_is_not_blocked_before_ir32_wait():
    """IR-32: 서비스는 5초 이상 기다린다. 부르지도 못한 채 그보다 일찍 빼지 않는다."""
    s = sync()
    s.on_system_mode("TEST")
    t = 0.0
    while t < NO_SERVER_S - 0.2:
        s.on_state("pode1", "", "b1", t)
        s.due(t, lambda robot: False)
        t += 0.2
    assert s.changes() == []


def test_request_ids_differ_after_a_fleet_restart():
    """로봇은 같은 request_id에 옛 답만 돌려준다. 관제가 다시 켜져도 id가 겹치지 않아야 새 모드가 반영된다."""
    first = []
    for _ in range(2):
        s = sync()
        s.on_system_mode("TEST")
        s.on_state("pode1", "UNKNOWN", "b1", 0.0)
        [(_, rid)] = s.due(0.0, READY)
        first.append(rid)
    assert first[0] != first[1]


def test_an_expired_call_is_reported_once_for_cleanup():
    s = sync()
    s.on_system_mode("TEST")
    s.on_state("pode1", "UNKNOWN", "b1", 0.0)
    assert len(s.due(0.0, READY)) == 1
    s.on_state("pode1", "UNKNOWN", "b1", 0.2)
    s.due(0.2, READY)
    assert s.expired == []
    t = ANSWER_S + 0.1
    s.on_state("pode1", "UNKNOWN", "b1", t)
    s.due(t, READY)
    assert s.expired == ["pode1"]
    s.due(t + 0.1, READY)
    assert s.expired == []


def test_an_old_no_server_time_does_not_block_later():
    """모드가 맞는 동안 서버가 없던 시각은 남기지 않는다. 나중에 서버가 사라지면 그때부터 5초를 기다린다."""
    s = sync()
    s.on_system_mode("TEST")
    no = lambda robot: False  # noqa: E731
    t = 0.0
    while t < 10.0:
        s.on_state("pode1", "TEST", "b1", t)
        s.due(t, no)
        t += 0.2
    end = t + NO_SERVER_S - 0.4
    while t < end:
        s.on_state("pode1", "NORMAL", "b1", t)
        s.due(t, no)
        t += 0.2
    assert s.changes() == []


def _no_server_run(s, start, end, step, boot=lambda t: "b1", mode="NORMAL"):
    t = start
    while t < end:
        s.on_state("pode1", mode, boot(t), t)
        s.due(t, lambda robot: False)
        t += step
    return t


def test_short_gaps_do_not_restart_the_no_server_wait():
    """같은 boot_id로 1.5초를 조금 넘게 끊겨 들어와도 서버 없음 5초는 이어서 센다."""
    s = sync()
    s.on_system_mode("TEST")
    _no_server_run(s, 0.0, NO_SERVER_S + 2.0, RECONNECT_GAP_S + 0.1)
    assert s.changes() == [("pode1", True, "reports NORMAL, system TEST")]


def test_a_reboot_restarts_the_no_server_wait():
    """boot_id가 바뀌면 새 프로그램이라 서버 없음 5초를 처음부터 센다."""
    s = sync()
    s.on_system_mode("TEST")
    t = _no_server_run(s, 0.0, NO_SERVER_S - 1.0, 0.2)
    t = _no_server_run(s, t, t + NO_SERVER_S - 0.6, 0.2, boot=lambda _: "b2")
    assert s.changes() == []
    _no_server_run(s, t, t + RETRY_S + 0.5, 0.2, boot=lambda _: "b2")  # 판단은 부르는 차례마다
    assert s.changes() == [("pode1", True, "reports NORMAL, system TEST")]
