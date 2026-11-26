"""브리지 보고 대기열(BridgeOutbox)과 Hello 순서(Handshake): 가짜 시계로 본다 (IR-22·23·40·41)."""
import pytest
from fleet_manager.goal_tracker import BridgeOutbox, Handshake, backoff_s

TRIP = ("trip", "T-1")


def outbox(cap=4, rand=lambda: 0.5):
    box = BridgeOutbox(first_s=1.0, max_s=10.0, reply_limit_s=3.0, max_in_flight=cap, rand=rand)
    return box


def ready(cap=4):
    box = outbox(cap)
    box.set_epoch(7)
    return box


# ---- 간격: 1 → 2 → 4 → 8 → 10 s, ±20 % ----
def test_the_backoff_doubles_from_one_second_up_to_ten():
    assert [backoff_s(a, 1.0, 10.0, 1.0) for a in range(1, 7)] == [1, 2, 4, 8, 10, 10]


def test_the_backoff_stays_at_the_cap_after_thousands_of_failures():
    # 브리지가 몇 시간 꺼져 있어도 간격 계산이 넘치지 않는다
    assert backoff_s(5000, 1.0, 10.0, 1.0) == 10.0


@pytest.mark.parametrize("rand, factor", [(0.0, 0.8), (1.0, 1.2)])
def test_the_backoff_is_jittered_by_twenty_percent(rand, factor):
    box = BridgeOutbox(1.0, 10.0, 3.0, 4, rand=lambda: rand)
    box.set_epoch(1)
    box.put(TRIP, "a")
    [(token, *_)] = box.due(0.0)
    box.answered(token, False, 0.0)
    assert box.due(factor * 1.0 - 1e-6) == []
    assert len(box.due(factor * 1.0 + 1e-6)) == 1


# ---- Hello 전에는 아무것도 보내지 않는다 (IR-22) ----
def test_nothing_goes_out_before_the_epoch_and_everything_waits():
    box = outbox()
    box.put(TRIP, "a")
    box.put(("event", "e1"), "ev")
    assert box.due(0.0) == []
    assert box.waiting() == 2
    box.set_epoch(7)
    sent = box.due(0.0)
    assert [(payload, epoch) for _, payload, epoch, _ in sent] == [("a", 7), ("ev", 7)]


# ---- 같은 건은 최신 보고로 바꿔 끼우고 n은 늘어난다 (IR-23) ----
def test_a_waiting_trip_report_is_replaced_by_the_newest_with_a_new_n():
    box = outbox()
    n1 = box.put(TRIP, "assigned")
    n2 = box.put(TRIP, "filling")
    assert n2 > n1
    box.set_epoch(7)
    [(_, payload, epoch, n)] = box.due(0.0)
    assert (payload, epoch, n) == ("filling", 7, n2)


def test_one_report_per_trip_is_in_flight_and_the_next_waits_for_the_answer():
    box = ready()
    box.put(TRIP, "assigned")
    [(token, *_)] = box.due(0.0)
    box.put(TRIP, "filling")
    box.put(TRIP, "transporting")
    assert box.due(0.5) == [], "the trip already has a report in flight"
    box.answered(token, True, 0.6)
    [(_, payload, _, n)] = box.due(0.6)
    assert payload == "transporting" and n == 3


def test_other_trips_do_not_wait_for_a_trip_in_flight():
    box = ready()
    box.put(TRIP, "a")
    box.due(0.0)
    box.put(("trip", "T-2"), "b")
    assert [p for _, p, _, _ in box.due(0.1)] == ["b"]


# ---- 실패와 응답 한도: 1 → 2 → 4 s 뒤 다시, 포기하지 않는다 (IR-40·41) ----
def test_a_failed_report_is_sent_again_on_the_backoff_schedule_forever():
    box = ready()
    box.put(TRIP, "a")
    sent_at = []
    now = 0.0
    while now < 120.0:
        for token, payload, _, n in box.due(now):
            sent_at.append(round(now, 1))
            box.answered(token, False, now)
        now = round(now + 0.1, 1)
    gaps = [round(b - a, 1) for a, b in zip(sent_at, sent_at[1:])]
    assert gaps[:5] == [1.0, 2.0, 4.0, 8.0, 10.0]
    assert set(gaps[4:]) == {10.0}, "it never gives up and never waits more than 10 s"


def test_a_resend_keeps_the_same_n_so_the_bridge_can_say_duplicate():
    box = ready()
    n = box.put(TRIP, "a")
    [(token, *_)] = box.due(0.0)
    box.answered(token, False, 0.0)
    [(_, _, _, again)] = box.due(1.0)
    assert again == n


def test_a_newer_report_replaces_a_failed_one():
    box = ready()
    box.put(TRIP, "old")
    [(token, *_)] = box.due(0.0)
    box.put(TRIP, "new")
    box.answered(token, False, 0.0)
    [(_, payload, _, _)] = box.due(1.0)
    assert payload == "new"


def test_a_request_past_the_reply_limit_counts_as_failed():
    box = ready()
    box.put(TRIP, "a")
    [(token, *_)] = box.due(0.0)
    assert box.expire(2.9) == []
    assert box.expire(3.0) == [token]
    assert box.due(3.9) == []
    [(again, *_)] = box.due(4.0)
    box.answered(token, True, 4.1)  # 버린 요청의 늦은 답은 아무것도 끝내지 않는다
    assert box.waiting() == 1
    box.answered(again, True, 4.2)
    assert box.waiting() == 0


def test_a_delivered_report_resets_the_backoff():
    box = ready()
    box.put(TRIP, "a")
    [(token, *_)] = box.due(0.0)
    box.answered(token, False, 0.0)
    [(token, *_)] = box.due(1.0)
    box.answered(token, True, 1.1)
    box.put(TRIP, "b")
    assert len(box.due(1.1)) == 1, "a delivered trip sends its next report at once"


# ---- 노드 하나의 동시 요청 상한 (IR-41) ----
def test_requests_in_flight_are_capped_and_the_rest_keep_their_order():
    box = ready(cap=2)
    for i in range(4):
        box.put(("event", f"e{i}"), i)
    assert [p for _, p, _, _ in box.due(0.0)] == [0, 1]
    assert box.due(0.1) == []
    tokens = box.in_flight()
    box.answered(tokens[0], True, 0.2)
    assert [p for _, p, _, _ in box.due(0.2)] == [2]


# ---- Hello → ListOpenTrips 순서와 재시도 ----
def test_the_handshake_asks_hello_then_open_trips_and_retries_on_the_backoff():
    hs = Handshake(1.0, 10.0, 3.0, rand=lambda: 0.5)
    assert hs.due(0.0) == "hello"
    hs.sent(0.0)
    assert hs.due(0.5) is None, "one call at a time"
    hs.failed(0.5)
    assert hs.due(1.4) is None
    assert hs.due(1.5) == "hello"
    hs.sent(1.5)
    assert hs.expired(4.4) is False
    assert hs.expired(4.5) is True
    assert hs.due(6.4) is None
    assert hs.due(6.5) == "hello"
    hs.sent(6.5)
    hs.hello_ok(7)
    assert hs.epoch == 7
    assert hs.due(6.6) == "list"
    hs.sent(6.6)
    hs.list_ok()
    assert hs.due(100.0) is None and hs.done
