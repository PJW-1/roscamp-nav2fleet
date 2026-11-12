"""tools/situations.py: 결과 표의 "끝나지 않은 주문 원인" 칸을 원인 분류(tools/triage)가 채운다."""
import re
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import situations  # noqa: E402
from fake_robots.world.table import run_situation  # noqa: E402
from triage import CAUSES, recording_factory  # noqa: E402

GROUP = re.compile(r"^(.+) (\d+) \((.+)\)$")  # 원인 건수 (주문 id들)


def cause_cell(out: str, title: str) -> str:
    row = next(line for line in out.splitlines() if line.startswith(f"| {title} "))
    return row.split("|")[-2].strip()


def test_an_unfinished_run_names_one_cause_for_every_open_order(capsys):
    """배치 c의 앞 로봇 고장은 주문을 하나도 못 끝낸다(갈림목이 하나다). 여섯 주문 모두에 원인이
    붙는다."""
    situations.main(["-s", "front_robot_stuck", "-l", "c", "--seeds", "1"])
    cell = cause_cell(capsys.readouterr().out, "앞 로봇 고장")
    orders = []
    for group in cell.split("; "):
        cause, count, ids = GROUP.match(group).groups()
        assert cause in CAUSES
        assert int(count) == len(ids.split())
        orders += ids.split()
    assert sorted(orders) == [f"O-{n:03d}" for n in range(1, 7)], "each open order once"


def test_a_finished_run_leaves_the_cause_empty(capsys):
    situations.main(["-s", "person_passing", "-l", "d2", "--seeds", "1"])
    assert cause_cell(capsys.readouterr().out, "사람이 잠깐 지나감") == ""


def test_recording_a_run_does_not_change_it():
    """원인 칸은 같은 씨드로 다시 돌린 실행에서 나온다. 기록을 붙여도 결과 줄이 같아야 한다."""
    plain = run_situation("front_robot_stuck", "c", 1, avoid_mode="mixed")
    lines: list = []
    recorded = run_situation("front_robot_stuck", "c", 1, avoid_mode="mixed",
                             manager_factory=recording_factory(lines))
    assert lines and recorded.row == plain.row
