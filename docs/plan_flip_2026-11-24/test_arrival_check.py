import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import arrival_check as ac  # noqa: E402


def seg(s, mode, x=0.0, y=0.0):
    return {"seq_id": s, "mode": mode, "x": x, "y": y}


def order(t, gid, upd, base):
    return {"t": t, "out": "MoveOrderCmd", "robot": "p", "goal_id": gid, "order_id": "o",
            "update_id": upd, "base": base}


def test_segment_ends_follow_updates_and_look_back_one_segment():
    # 끝 보고가 늦을 수 있어 창은 앞 조각이 시작한 때부터다
    rows = [order(1.0, 1, 0, [seg(1, "follow_path"), seg(2, "nav_goal", 1.0, 0.0)]),
            {"t": 2.0, "in": "on_move_feedback", "args": ["p", 1, 1]},
            order(3.0, 2, 1, [seg(3, "nav_goal", 2.0, 0.0)]),
            {"t": 3.1, "in": "on_move_result", "args": ["p", 1, False, "SUPERSEDED", 1]},
            {"t": 5.0, "in": "on_move_feedback", "args": ["p", 2, 2]},
            {"t": 9.0, "in": "on_move_result", "args": ["p", 2, True, "", 3]}]
    assert ac.segment_ends(rows) == [("p", 1.0, 0.0, 1.0, 5.0, True), ("p", 2.0, 0.0, 2.0, 9.0, True)]


def test_segment_ends_keep_a_nav_goal_that_failed():
    rows = [order(1.0, 1, 0, [seg(1, "nav_goal", 1.0, 0.0), seg(2, "follow_path")]),
            {"t": 6.0, "in": "on_move_result", "args": ["p", 1, False, "NAV_ABORTED", 0]}]
    assert ac.segment_ends(rows) == [("p", 1.0, 0.0, 1.0, 6.0, False)]


def test_lags_skip_early_handover_and_count_failures_near_the_goal():
    truth = {"p": [(t / 10, t / 10, 0.0) for t in range(0, 100)]}  # x = t
    segs = [("p", 1.0, 0.0, 0.0, 5.0, True),   # 0.94에 닿고 5.0에 끝: 4.06 s
            ("p", 2.0, 0.0, 0.0, 1.5, True),   # 닿기 전에 끝남: 뺀다
            ("p", 3.0, 0.0, 0.0, 8.0, False)]  # 닿은 뒤 실패
    done, near_fail = ac.lags(segs, truth)
    assert len(done) == 1 and abs(done[0] - 4.1) < 0.11 and near_fail == 1
