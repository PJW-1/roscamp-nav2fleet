"""상황 정의와 결과 표: 관제 없이(대역) 돌려도 모든 상황이 모든 배치에서 서고 표가 찍힌다."""
import importlib.util
from pathlib import Path

import pytest
from fake_robots.world import RecordingManager
from fake_robots.world.situations import LAYOUTS, NAMES, build
from fake_robots.world.table import HEADER, markdown, run_situation

TOOL = Path(__file__).resolve().parents[3] / "tools" / "situations.py"


@pytest.mark.parametrize("layout", LAYOUTS)
@pytest.mark.parametrize("name", NAMES)
def test_every_situation_builds_on_every_layout(name, layout):
    s = build(name, layout)
    assert s.title and s.timeout_s > 0


def test_a_row_has_every_column_and_the_markdown_has_a_line_per_row():
    outcome = run_situation("person_passing", "d2", 1, manager_factory=RecordingManager)
    assert len(outcome.row) == len(HEADER)
    assert outcome.row[:5] == ("사람이 잠깐 지나감", "d2", 1, "perfect", "0/6")
    assert outcome.failures[0].startswith("finished 0/6")
    text = markdown([outcome.row, outcome.row])
    assert text.splitlines()[0].startswith("| 상황 | 배치 | 씨드 | 회피 모드 | 완료 |")
    assert len(text.splitlines()) == 4


def test_the_avoid_mode_reaches_the_world_and_its_column_and_the_cause_column_is_empty():
    outcome = run_situation("person_passing", "d2", 1, manager_factory=RecordingManager,
                            avoid_mode="none")
    assert outcome.world.avoid_mode == "none"
    assert outcome.row[3] == "none"
    assert HEADER[-1] == "끝나지 않은 주문 원인" and outcome.row[-1] == ""


@pytest.mark.skipif(not TOOL.is_file(), reason="only in the source tree")
def test_the_tool_reads_parameter_overrides_as_yaml_and_the_avoid_modes():
    spec = importlib.util.spec_from_file_location("situations_tool", TOOL)
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    assert tool.overrides(["use_order_model=true", "nav_speed_mps=0.15"]) == {
        "use_order_model": True, "nav_speed_mps": 0.15}
    with pytest.raises(SystemExit):
        tool.overrides(["nav_speed_mps"])
    assert tool.avoid_modes(["all"]) == ["none", "perfect", "mixed"]
    assert tool.avoid_modes(["mixed", "none"]) == ["mixed", "none"]
