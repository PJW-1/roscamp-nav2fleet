import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import arrival_check as ac  # noqa: E402


def seg(s, mode, x=0.0, y=0.0):
    return {"seq_id": s, "mode": mode, "x": x, "y": y}


def order(t, gid, upd, base):
    return {"t": t, "out": "MoveOrderCmd", "robot": "p", "goal_id": gid, "order_id": "o",
            "update_id": upd, "base": base}


def test_segment_ends_follow_updates():
    # 창은 조각이 시작한(앞 조각 끝이 보고된) 때보다 LOOK_BACK_S 앞에서 연다
    rows = [order(1.0, 1, 0, [seg(1, "follow_path"), seg(2, "nav_goal", 1.0, 0.0)]),
            {"t": 2.0, "in": "on_move_feedback", "args": ["p", 1, 1]},
            order(3.0, 2, 1, [seg(3, "nav_goal", 2.0, 0.0)]),
            {"t": 3.1, "in": "on_move_result", "args": ["p", 1, False, "SUPERSEDED", 1]},
            {"t": 5.0, "in": "on_move_feedback", "args": ["p", 2, 2]},
            {"t": 9.0, "in": "on_move_result", "args": ["p", 2, True, "", 3]}]
    b = ac.LOOK_BACK_S
    assert ac.segment_ends(rows) == [("p", 1.0, 0.0, 2.0 - b, 5.0, True),
                                     ("p", 2.0, 0.0, 5.0 - b, 9.0, True)]


def test_segment_ends_keep_a_nav_goal_that_failed():
    rows = [order(1.0, 1, 0, [seg(1, "nav_goal", 1.0, 0.0), seg(2, "follow_path")]),
            {"t": 6.0, "in": "on_move_result", "args": ["p", 1, False, "NAV_ABORTED", 0]}]
    assert ac.segment_ends(rows) == [("p", 1.0, 0.0, 1.0 - ac.LOOK_BACK_S, 6.0, False)]


def test_segment_ends_keep_a_failure_reported_after_an_update():
    rows = [order(1.0, 1, 0, [seg(1, "nav_goal", 1.0, 0.0)]),
            order(3.0, 2, 1, [seg(2, "follow_path")]),
            {"t": 3.1, "in": "on_move_result", "args": ["p", 1, False, "NAV_ABORTED", 0]}]
    assert ac.segment_ends(rows) == [("p", 1.0, 0.0, 1.0 - ac.LOOK_BACK_S, 3.1, False)]


def test_segment_ends_keep_a_cancel_of_the_previous_order():
    rows = [order(1.0, 1, 0, [seg(1, "nav_goal", 1.0, 0.0)]),
            {"t": 4.0, "out": "MoveOrderCmd", "robot": "p", "goal_id": 2, "order_id": "o2",
             "update_id": 0, "base": [seg(1, "follow_path")]},
            {"t": 4.1, "in": "on_move_result", "args": ["p", 1, False, "CANCELED", 0]}]
    assert ac.segment_ends(rows) == [("p", 1.0, 0.0, 1.0 - ac.LOOK_BACK_S, 4.1, False)]


def test_lags_ignore_passing_the_goal_before_the_segment():
    # 앞 조각에서 t=2에 목적지를 지나갔고, 이 조각(창 4.5~9)에서는 t=8에 닿았다
    truth = {"p": [(2.0, 1.0, 0.0), (5.0, 0.5, 0.0), (8.0, 1.0, 0.0), (9.0, 1.0, 0.0)]}
    done, near_fail = ac.lags([("p", 1.0, 0.0, 4.5, 9.0, True)], truth)
    assert done == [1.0] and near_fail == 0


def test_lags_skip_early_handover_and_count_failures_near_the_goal():
    truth = {"p": [(t / 10, t / 10, 0.0) for t in range(0, 100)]}  # x = t
    segs = [("p", 1.0, 0.0, 0.0, 5.0, True),   # 0.94에 닿고 5.0에 끝: 4.06 s
            ("p", 2.0, 0.0, 0.0, 1.5, True),   # 닿기 전에 끝남: 뺀다
            ("p", 3.0, 0.0, 0.0, 8.0, False)]  # 닿은 뒤 실패
    done, near_fail = ac.lags(segs, truth)
    assert len(done) == 1 and abs(done[0] - 4.1) < 0.11 and near_fail == 1
