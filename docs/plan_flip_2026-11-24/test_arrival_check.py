import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import arrival_check as ac  # noqa: E402


def seg(s, mode, x=0.0, y=0.0):
    return {"seq_id": s, "mode": mode, "x": x, "y": y}


def test_segment_ends_follow_updates():
    rows = [{"t": 1.0, "out": "MoveOrderCmd", "robot": "p", "goal_id": 1, "order_id": "o", "update_id": 0,
             "base": [seg(1, "follow_path"), seg(2, "nav_goal", 1.0, 0.0)]},
            {"t": 2.0, "in": "on_move_feedback", "args": ["p", 1, 1]},
            {"t": 3.0, "out": "MoveOrderCmd", "robot": "p", "goal_id": 2, "order_id": "o", "update_id": 1,
             "base": [seg(3, "nav_goal", 2.0, 0.0)]},
            {"t": 3.1, "in": "on_move_result", "args": ["p", 1, False, "SUPERSEDED", 1]},
            {"t": 5.0, "in": "on_move_feedback", "args": ["p", 2, 2]},
            {"t": 9.0, "in": "on_move_result", "args": ["p", 2, True, "", 3]}]
    assert ac.segment_ends(rows) == [("p", 1.0, 0.0, 2.0, 5.0), ("p", 2.0, 0.0, 5.0, 9.0)]


def test_lags_skip_segments_handed_over_early():
    truth = {"p": [(t / 10, t / 10, 0.0) for t in range(0, 100)]}  # x = t
    # 1.0에 닿은 시각 0.94, 끝 5.0 -> 4.06 s; 2.0 조각은 1.5에 끝나 닿기 전이다
    out = ac.lags([("p", 1.0, 0.0, 0.0, 5.0), ("p", 2.0, 0.0, 0.0, 1.5)], truth)
    assert len(out) == 1 and abs(out[0] - 4.1) < 0.11
