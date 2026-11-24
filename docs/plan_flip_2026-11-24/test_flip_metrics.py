"""flip_metrics의 순수 함수 시험(판 폴더 없이)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import flip_metrics as fm  # noqa: E402

GOAL = (2.0, 0.0)


def line(pts):
    return [(float(x), float(y)) for x, y in pts]


def route_a():
    # 아래로 도는 길: (0,0) -> (0,-1) -> (2,-1) -> (2,0)
    return line([(0, 0), (0, -1), (2, -1), GOAL])


def dense(pts, step=0.05):
    out = []
    for a, b in zip(pts, pts[1:]):
        k = max(1, int(np.hypot(b[0] - a[0], b[1] - a[1]) / step))
        out += [(a[0] + (b[0] - a[0]) * i / k, a[1] + (b[1] - a[1]) * i / k) for i in range(k)]
    return out + [pts[-1]]


A = dense(route_a())
B = dense(line([(0, 0), (0, 1), (2, 1), GOAL]))
A_TAIL = A[10:]


def test_flip_count_alternating():
    # A, B, A, A: 뒤집힘 2번
    plans = [(0, A), (1, B), (2, A), (3, A_TAIL)]
    flips, off = fm.count_flips(plans)
    assert len(flips) == 2 and off == 0


def test_same_route_not_flip():
    flips, off = fm.count_flips([(0, A), (1, A_TAIL), (2, A_TAIL[5:])])
    assert not flips and off == 0


def test_off_route_counted_separately():
    far = dense(line([(0.0, 2.0), (2.0, 2.0), GOAL]))
    flips, off = fm.count_flips([(0, A), (1, far)])
    assert not flips and off == 1


def test_different_goal_not_flip():
    other = dense(line([(0, 0), (0, 1), (2, 1), (2.5, 0)]))
    flips, off = fm.count_flips([(0, A), (1, other)])
    assert not flips and off == 0


def test_spin_runs():
    t = np.arange(0, 4, 0.05)
    cond = (t >= 1.0) & (t < 1.6)  # 0.55 s: 센다
    cond |= (t >= 2.0) & (t < 2.3)  # 0.25 s: 세지 않음
    assert len(fm.count_runs(t, cond, 0.5)) == 1


def truth_spin():
    # 0~2 s 정지, 2~4 s 제자리 회전(1 rad/s), 4~6 s 직진
    t = np.arange(0, 6, 0.05)
    yaw = np.where(t < 2, 0.0, np.where(t < 4, t - 2, 2.0))
    x = np.where(t < 4, 0.5, 0.5 + 0.1 * (t - 4))
    y = np.full_like(t, 0.5)
    return {"pode1": (t, x, y, yaw)}


HALL = [[0.0, 0.0, 1.0, 1.0]]


def test_hall_spin_counts_one_in_place_turn():
    n, total = fm.hall_spins(truth_spin(), None, HALL)
    assert n == 1 and 1.5 < total < 2.5


def test_hall_spin_needs_nav_goal_window():
    assert fm.hall_spins(truth_spin(), {"pode1": [(2.5, 3.5)]}, HALL)[0] == 1
    assert fm.hall_spins(truth_spin(), {"pode1": [(4.5, 5.5)]}, HALL)[0] == 0
    assert fm.hall_spins(truth_spin(), {}, HALL)[0] == 0


def test_hall_spin_outside_hall():
    assert fm.hall_spins(truth_spin(), None, [[5.0, 5.0, 6.0, 6.0]])[0] == 0


def test_dist_jumps():
    s = {("pode1", 1): [1.0, 0.95, 0.90, 1.2, 1.15], ("pode1", 2): [0.5, 0.2]}
    assert fm.dist_jumps(s) == 2  # 늘어난 것 1, 줄어든 것 1


def test_dist_jumps_skips_negative_and_none():
    s = {("pode1", 1): [1.0, -1.0, None, 0.95, 0.9]}
    assert fm.dist_jumps(s) == 0


def test_shim_spins():
    t = [i * 0.05 for i in range(40)]
    cmds = {"pode1": [(ti, 0.0 if 5 <= i < 15 else 0.1, 0.5 if 5 <= i < 15 else 0.0)
                      for i, ti in enumerate(t)]}
    assert fm.shim_spins(cmds) == 1


def test_nav_intervals():
    rows = [
        {"t": 1.0, "out": "MoveOrderCmd", "robot": "pode1", "goal_id": 1,
         "base": [{"seq_id": 1, "mode": "follow_path"}, {"seq_id": 2, "mode": "nav_goal"}]},
        {"t": 2.0, "in": "on_move_feedback", "args": ["pode1", 1, 0, False, "", 0.5]},
        {"t": 3.0, "in": "on_move_feedback", "args": ["pode1", 1, 1, False, "", 0.5]},
        {"t": 5.0, "in": "on_move_result", "args": ["pode1", 1, True, "", 2]},
    ]
    assert fm.nav_intervals(rows) == {"pode1": [(3.0, 5.0)]}
    assert fm.nav_intervals([]) is None


def test_recoveries():
    rows = [{"in": "on_robot_state", "args": ["pode1", 0, 0, 0, s]}
            for s in ("IDLE", "MOVING", "RECOVERING", "MOVING", "BLOCKED", "RECOVERING")]
    assert fm.record_recoveries(rows) == (1, True)
