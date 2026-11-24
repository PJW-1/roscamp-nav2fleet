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


def cmd(t, gid, upd, base):
    return {"t": t, "out": "MoveOrderCmd", "robot": "pode1", "goal_id": gid, "order_id": "pode1-1",
            "update_id": upd, "base": [{"seq_id": s, "mode": m} for s, m in base]}


def fb(t, gid, done):
    return {"t": t, "in": "on_move_feedback", "args": ["pode1", gid, done, False, "", 0.5]}


def res(t, gid, ok=True):
    return {"t": t, "in": "on_move_result", "args": ["pode1", gid, ok, "" if ok else "SUPERSEDED", 0]}


def test_nav_intervals_keep_the_running_nav_goal_across_an_update():
    # 갱신 goal 2의 base는 조각 3뿐이지만 로봇은 조각 2(nav_goal)를 끝까지 한다(11-24 기록)
    rows = [cmd(1.0, 1, 0, [(1, "follow_path"), (2, "nav_goal")]), fb(3.0, 1, 1),
            cmd(4.0, 2, 1, [(3, "follow_path")]), fb(4.01, 2, 1), res(4.02, 1, ok=False),
            fb(6.0, 2, 2), res(8.0, 2)]
    assert fm.nav_intervals(rows) == {"pode1": [(3.0, 6.0)]}


def test_nav_intervals_start_an_added_nav_goal_only_when_reached():
    rows = [cmd(1.0, 1, 0, [(1, "follow_path")]), fb(1.5, 1, 0),
            cmd(2.0, 2, 1, [(2, "nav_goal")]), res(2.01, 1, ok=False), fb(2.2, 2, 0),
            fb(3.0, 2, 1), res(5.0, 2)]
    assert fm.nav_intervals(rows) == {"pode1": [(3.0, 5.0)]}


def test_hall_spin_metric_is_unavailable_without_order_records():
    assert fm.hall_spin_metric(truth_spin(), [], HALL) == (None, None, "none")
    rows = [cmd(1.0, 1, 0, [(1, "nav_goal")]), res(5.0, 1)]
    n, total, src = fm.hall_spin_metric(truth_spin(), rows, HALL)
    assert (n, src) == (1, "fleet_record") and 1.5 < total < 2.5


def test_pick_cmds_chooses_the_topic_per_robot():
    bag = {"cmd": {"pode1": [(0.0, 0.1, 0.0)]}, "cmd_nav": {"pode1": [(0.0, 0.2, 0.0)],
                                                           "pode2": [(0.0, 0.3, 0.0)]}}
    assert fm.pick_cmds(bag) == {"pode1": [(0.0, 0.1, 0.0)], "pode2": [(0.0, 0.3, 0.0)]}
    assert fm.pick_cmds(None) == {}
