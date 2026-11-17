"""양보 기록 도구(tools/yield_report.py): 작은 가짜 기록으로 세 가지 집계를 확인한다."""
import gzip
import json
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import yield_report  # noqa: E402


def st(t, robot, state):
    return {"t": t, "in": "on_robot_state", "args": [robot, 0.0, 0.0, 0.0, state, 90.0]}


def ev(t, code, robot, detail):
    return {"t": t, "out": "EventOut", "level": "INFO", "code": code, "detail": detail,
            "robot": robot}


def ly(t, robot, detail):
    return ev(t, "LOCAL_YIELD", robot, detail)


def hy(t, robot, detail):
    return ev(t, "HALL_YIELD", robot, detail)


def write(path, *items, gz=False):
    text = "\n".join(json.dumps(i) for i in [{"recorded": True, "params": {}}, *items]) + "\n"
    if gz:
        with gzip.open(path, "wt", encoding="utf-8") as file:
            file.write(text)
    else:
        path.write_text(text, encoding="utf-8")
    return path


# 한 판: p1은 wait -> backup -> retry -> BLOCKED로 끝나고, 관제는 그 뒤에 p1을 비키게 한다.
# p2는 wait 한 번 만에 풀린다. p3는 로봇 사다리 없이 관제가 먼저 끼어든다(BLOCKED 전).
RUN = [
    st(1, "p1", "MOVING"), st(1, "p2", "MOVING"), st(1, "p3", "MOVING"),
    ly(10.0, "p1", "wait"), st(10.0, "p1", "RECOVERING"),
    ly(12.0, "p1", "backup"), ly(14.0, "p1", "retry"),
    st(15.0, "p1", "BLOCKED"),
    ly(15.5, "p1", "ends BLOCKED after 5 s"),
    hy(16.0, "p1", "stops 5 s for p2"),
    hy(30.0, "p3", "stops 5 s for p2"),
    hy(40.0, "p2", "out of the way of p1, goes to P-3"),
    hy(41.0, "p1", "waits for p2 to clear the way, nothing marked"),
    ly(60.0, "p2", "wait"), ly(60.4, "p2", "ends MOVING after 0 s"),
]


def test_local_yield_steps_are_counted(tmp_path):
    path = write(tmp_path / "fleet_record.jsonl", *RUN)
    total = yield_report.analyse([path]).total()
    assert (total.wait, total.backup, total.retry) == (2, 1, 1)
    assert total.ends == {"BLOCKED": 1, "MOVING": 1}


def test_hall_yield_phrases_and_blocked_guess(tmp_path):
    path = write(tmp_path / "fleet_record.jsonl", *RUN)
    total = yield_report.analyse([path]).total()
    assert total.phrases == {"stops": 2, "out of the way": 1, "waits for": 1}
    # p1이 BLOCKED를 보고한 뒤의 stops, 상대 p1이 BLOCKED인 out of the way, 문구로 정해진 waits for
    assert total.blocked_after == 3
    assert total.blocked_before == 1  # p3의 stops: 직전 상태 MOVING


def test_encounter_order_and_control_first(tmp_path):
    path = write(tmp_path / "fleet_record.jsonl", *RUN)
    report = yield_report.analyse([path])
    rows = {(r.robot, r.wait_t): r for r in report.encounters}
    p1 = rows[("p1", 10.0)]
    assert p1.backup_t == 12.0 and p1.blocked_t == 15.0 and p1.ends == "BLOCKED"
    assert [(h.t, h.phrase) for h in p1.hall] == [(16.0, "stops")]
    assert rows[("p2", 60.0)].hall == []
    # 사다리 밖에서 나온 HALL_YIELD는 t=30, 40, 41이다. t=16은 p1 사다리가 끝난 15.5 s에서
    # 2 s 안이라 사다리 안으로 본다.
    assert report.total().control_first == 3


def test_gz_and_several_files_add_up(tmp_path):
    a = write(tmp_path / "a.jsonl.gz", *RUN, gz=True)
    b = write(tmp_path / "b.jsonl", *RUN)
    report = yield_report.analyse([a, b])
    assert report.total().wait == 4 and len(report.runs) == 2


def test_markdown_has_tables(tmp_path):
    path = write(tmp_path / "fleet_record.jsonl", *RUN)
    text = yield_report.render(yield_report.analyse([path]), encounters=True)
    assert "| wait |" in text and "합계" in text and "BLOCKED 전" in text
