"""원인 분류(tools/triage): 원인마다 작은 기록 하나, rec.csv 읽기, 머리 없는 세계 실행 하나."""
import json
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import triage  # noqa: E402
from triage import Obstacle  # noqa: E402

HEAD = {"recorded": True, "robots": {"pinky1": "P-1"}, "map": "",
        "params": {"blocked_mark_s": 10.0, "blocked_timeout_s": 60.0}}


def board(t, *ids):
    return {"t": t, "in": "on_order_board",
            "args": ["RUN", [{"order_id": i, "items": {"A": 1}} for i in ids]]}


def ev(t, code, robot="pinky1", detail=""):
    return {"t": t, "out": "EventOut", "level": "INFO", "code": code, "detail": detail,
            "robot": robot}


def st(t, x, state="MOVING", robot="pinky1", y=0.0):
    return {"t": t, "in": "on_robot_state", "args": [robot, x, y, 0.0, state, 90.0]}


def shipped(t, order="O-1", status="SHIPPED"):
    return {"t": t, "out": "ReportOrderCmd", "order_id": order, "robot": "pinky1",
            "status": status, "placed": {}, "missing": {}, "detail": ""}


def record(*items, head=HEAD, cut=False):
    lines = [json.dumps(head)] + [json.dumps(i) for i in items]
    if cut:
        lines.append('{"t": 99.0, "in": "on_rob')
    return "\n".join(lines)


def walk(t0, t1, x0=0.0, speed=0.1, state="MOVING"):
    """매 초 위치 보고. speed m/s로 간다."""
    return [st(t, x0 + speed * (t - t0), state) for t in range(int(t0), int(t1) + 1)]


def causes(text, **kw):
    run = triage.read_fleet_record(text.splitlines(), **kw.pop("read", {}))
    return [(r.order_id, r.cause, r.event) for r in triage.classify(run, **kw)]


def started(*extra):
    """주문이 묶이고 목표가 가고 움직이기 시작한 기록의 앞부분."""
    return [board(0.1, "O-1"), *walk(0, 9, 0.0, 0.0, "IDLE"),
            ev(10, "ORDER_BOUND", detail="O-1"), ev(10.1, "FOLLOW_SENT"), *extra]


def test_a_finished_order_gets_no_row():
    text = record(*started(), *walk(10, 20, 0.0), shipped(20))
    assert causes(text) == []


def test_an_order_never_bound_is_the_fleets_fault():
    text = record(board(0.1, "O-1"), *walk(0, 30, 0.0, 0.0, "IDLE"))
    assert causes(text) == [("O-1", "관제", "ORDER_BOUND 없음")]


def test_an_unbound_order_behind_an_isolated_robot_is_a_driving_fault():
    text = record(board(0.1, "O-1"), *walk(0, 30, 0.0, 0.0, "IDLE"), ev(12, "ROBOT_ISOLATED"))
    assert causes(text) == [("O-1", "주행", "ROBOT_ISOLATED")]


def test_no_goal_after_the_bind_is_the_fleets_fault():
    text = record(board(0.1, "O-1"), *walk(0, 30, 0.0, 0.0, "IDLE"),
                  ev(10, "ORDER_BOUND", detail="O-1"))
    assert causes(text) == [("O-1", "관제", "ORDER_BOUND")]


def test_a_robot_that_does_not_move_after_the_goal_is_a_driving_fault():
    text = record(*started(), *walk(10, 30, 0.0, 0.0, "IDLE"))
    assert causes(text) == [("O-1", "주행", "FOLLOW_SENT")]


def test_a_no_start_event_is_used_when_present():
    text = record(*started(), ev(16, "NO_START"), *walk(10, 30, 0.0, 0.0, "MOVING"))
    assert causes(text) == [("O-1", "주행", "NO_START")]


def test_a_small_wiggle_is_not_moving():
    text = record(*started(), *walk(10, 30, 0.0, 0.001, "IDLE"))
    assert causes(text) == [("O-1", "주행", "FOLLOW_SENT")]


def _blocked_run(block_s, reactions, obstacle=True):
    """t=20에 BLOCKED, block_s 뒤 풀림(None이면 끝까지)."""
    end = 20 + (block_s or 100)
    after = walk(end, end + 5, 1.0) if block_s else []
    items = [*started(), *walk(10, 19, 0.0), *walk(20, end, 1.0, 0.0, "BLOCKED"),
             *reactions, *after]
    return record(*items)


BOX = [Obstacle(x=1.1, y=0.0, radius=0.05, t0=15.0)]  # 영구


def test_a_block_before_a_standing_obstacle_is_a_real_block():
    text = _blocked_run(None, [ev(20.1, "BLOCKED_WAIT"), ev(30.1, "ZONE_MARKED_BLOCKED")])
    assert causes(text, obstacles=BOX) == [("O-1", "진짜 막힘", "BLOCKED")]


def test_a_block_with_nothing_on_the_floor_is_avoidance():
    text = _blocked_run(None, [ev(20.1, "BLOCKED_WAIT"), ev(30.1, "ZONE_MARKED_BLOCKED")])
    assert causes(text) == [("O-1", "회피", "BLOCKED")]


def test_an_obstacle_that_left_is_not_a_real_block():
    gone = [Obstacle(x=1.1, y=0.0, radius=0.05, t0=15.0, t1=40.0)]
    text = _blocked_run(None, [ev(20.1, "BLOCKED_WAIT"), ev(30.1, "ZONE_MARKED_BLOCKED")])
    assert causes(text, obstacles=gone)[0][1] == "회피"


def test_a_far_obstacle_is_not_the_cause():
    far = [Obstacle(x=5.0, y=3.0, radius=0.05, t0=0.0)]
    text = _blocked_run(None, [ev(20.1, "BLOCKED_WAIT"), ev(30.1, "ZONE_MARKED_BLOCKED")])
    assert causes(text, obstacles=far)[0][1] == "회피"


def test_no_first_reaction_within_a_second_is_the_fleets_fault():
    text = _blocked_run(None, [ev(22.5, "BLOCKED_WAIT"), ev(30.1, "ZONE_MARKED_BLOCKED")])
    assert causes(text, obstacles=BOX) == [("O-1", "관제", "BLOCKED_WAIT 없음(1 s)")]


def test_a_late_next_stage_is_the_fleets_fault():
    text = _blocked_run(None, [ev(20.1, "BLOCKED_WAIT"), ev(36.0, "ZONE_MARKED_BLOCKED")])
    assert causes(text, obstacles=BOX) == [("O-1", "관제", "ZONE_MARKED_BLOCKED 없음(15 s)")]


def test_a_block_that_ended_before_the_next_stage_needs_no_next_stage():
    text = _blocked_run(4, [ev(20.1, "BLOCKED_WAIT")])
    # 4 s 만에 풀렸고 주문은 끝나지 않았다: 막힘이 있었으니 3-1로 간다
    assert causes(text, obstacles=BOX)[0][1] == "진짜 막힘"


def test_a_silent_stall_event_is_the_fleets_fault():
    text = record(*started(), *walk(10, 12, 0.0), *walk(13, 80, 0.2, 0.0, "MOVING"),
                  ev(75, "SILENT_STALL"))
    assert causes(text) == [("O-1", "관제", "SILENT_STALL")]


def test_nav_aborted_counts_as_a_block():
    text = record(*started(), *walk(10, 20, 0.0), ev(20, "NAV_ABORTED"),
                  ev(20.2, "BLOCKED_WAIT"))
    assert causes(text, obstacles=BOX) == [("O-1", "진짜 막힘", "NAV_ABORTED")]


def test_a_record_without_state_reports_is_the_test_frame():
    text = record(board(0.1, "O-1"), ev(10, "ORDER_BOUND", detail="O-1"), ev(10.1, "FOLLOW_SENT"))
    assert causes(text) == [("O-1", "시험 틀", "상태 보고 없음")]


def test_a_cut_record_is_the_test_frame():
    text = record(*started(), *walk(10, 20, 0.0), cut=True)
    assert causes(text) == [("O-1", "시험 틀", "기록이 줄 중간에서 끝남")]


def test_an_error_code_from_the_frame_is_the_test_frame():
    text = record(*started(), *walk(10, 20, 0.0), ev(15, "SIM_DIED", robot=""))
    assert causes(text) == [("O-1", "시험 틀", "SIM_DIED")]


def test_a_held_order_is_unfinished_and_a_partial_one_is_finished():
    text = record(*started(), *walk(10, 12, 0.0), shipped(12, status="HELD"))
    assert [c[0] for c in causes(text)] == ["O-1"]
    text = record(*started(), *walk(10, 12, 0.0), shipped(12, status="PARTIAL"))
    assert causes(text) == []


def test_the_markdown_has_a_header_and_a_line_per_order():
    run = triage.read_fleet_record(record(board(0.1, "O-1", "O-2"), st(1, 0.0)).splitlines())
    text = triage.markdown(triage.classify(run))
    lines = text.splitlines()
    assert lines[0] == "| 주문 id | 원인 | 근거 사건 | 시각 (s) |"
    assert len(lines) == 4 and lines[2].startswith("| O-1 | 관제 |")


# ---- rec.csv ----

CSV = """t,event,robot,level,code,detail
1.0,event,pinky1,INFO,ORDER_BOUND,O-1
1.5,event,pinky1,INFO,NAV_GOAL_SENT,to P-1,extra
2.0,rep,pinky1,0.0,0.0,"0.0 MOVING 90"
2.0,pose,pinky1,0.0,0.0,0.0
4.0,rep,pinky1,0.5,0.0,"0.0 BLOCKED 90"
4.0,pose,pinky1,0.5,0.0,0.0
4.2,event,pinky1,INFO,BLOCKED_WAIT,stands
9.0,rep,pinky1,0.5,0.0,"0.0 BLOCKED 90"
"""


def test_a_rec_csv_is_read_with_or_without_a_header_and_extra_columns(tmp_path):
    path = tmp_path / "rec.csv"
    path.write_text(CSV, encoding="utf-8")
    run = triage.read_rec_csv(path)
    assert [e.code for e in run.events] == ["ORDER_BOUND", "NAV_GOAL_SENT", "BLOCKED_WAIT"]
    assert run.states["pinky1"][1] == (4.0, 0.5, 0.0, "BLOCKED")
    assert run.poses["pinky1"][0] == (2.0, 0.0, 0.0)
    path.write_text(CSV.split("\n", 1)[1], encoding="utf-8")
    assert len(triage.read_rec_csv(path).events) == 3
    rows = triage.classify(run, obstacles=[Obstacle(0.7, 0.0)])
    assert [(r.order_id, r.cause) for r in rows] == [("O-1", "진짜 막힘")]


# ---- 머리 없는 세계 ----


def test_a_headless_run_gives_rows_from_the_cores_own_record():
    # c 배치에서 pinky1이 멈추면 갈림목이 막혀 주문이 끝나지 않는다(situations.py 주석)
    outcome, rows = triage.run_triaged("front_robot_stuck", "c", 1)
    central = outcome.world.central
    assert len(rows) == central.total - central.finished > 0
    assert {r.cause for r in rows} <= set(triage.CAUSES)
    assert ("주행", "ROBOT_ISOLATED") in {(r.cause, r.event) for r in rows}


def test_a_gazebo_obstacle_yaml_becomes_a_circle_at_its_link_fraction():
    path = TOOLS.parent / "sim" / "fleet_gz_sim" / "obstacles" / "d2_express_box.yaml"
    nodes = {"X_W": (0.0, 0.0), "X_E": (2.0, 0.0)}
    assert triage.read_obstacles(path, nodes) == [Obstacle(1.0, 0.0, 0.04, 15.0, 615.0)]
