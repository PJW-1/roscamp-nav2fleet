"""상황 정의와 결과 표: 관제 없이(대역) 돌려도 모든 상황이 모든 배치에서 서고 표가 찍힌다."""
import importlib.util
from pathlib import Path

import pytest
from fake_robots.world import RecordingManager
from fake_robots.world.situations import NAMES, build
from fake_robots.world.table import HEADER, judge, markdown, run_situation
from fleet_core.common.commands import EventOut
from fleet_core.fleet_map import LAYOUTS

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
    assert any(f.startswith("finished 0/6") for f in outcome.failures)
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


def test_perfect_keeps_the_situation_expectations_and_the_others_use_section_three():
    """추가 지시서 2의 3절: none과 mixed는 '늦더라도 다 끝남'과 불변식만 본다."""
    s = build("person_passing", "d2")
    assert s.for_mode("perfect") is s
    rough = s.for_mode("none")
    assert (rough.finish_all, rough.max_retreats, rough.need_events, rough.no_events) == (
        True, None, {}, ())
    stuck_c = build("front_robot_stuck", "c").for_mode("mixed")
    assert stuck_c.finish_all is False and stuck_c.need_events == {}
    assert build("pose_jumps", "d").for_mode("none").allowed == ("I5_STANDING",)


def test_a_detour_long_after_the_person_left_is_not_blamed_on_the_person():
    """'사람이 잠깐 지나감'이 막는 것은 사람 때문의 우회다. 사람이 떠나고 한참 뒤 로봇끼리 서로 막아
    잡은 우회는 그 상황의 기대가 아니다(D2 씨드 3: 96 s 뒤 N_E와 O_Q 곁)."""
    from types import SimpleNamespace

    from fake_robots.world.table import expectations

    s = build("person_passing", "d2")
    [person] = s.injections
    assert s.quiet_until_s == pytest.approx(person.t1 + 30.0)

    def world(reroute_at, retreat_at=()):
        events = [(reroute_at, SimpleNamespace(code="REROUTE"))]
        return SimpleNamespace(
            events=events, t=300.0, central=SimpleNamespace(total=6, finished=6),
            metrics=SimpleNamespace(count=lambda code: sum(e.code == code for _, e in events),
                                    retreats=len(retreat_at), retreat_times=list(retreat_at)))

    assert expectations(s, world(person.t1 + 5.0)) == ["REROUTE x1"]
    assert expectations(s, world(person.t1 + 96.0)) == []
    # 후진도 같은 때까지만 센다: 늦은 우회는 봐주고 같은 때의 후진만 실패로 치지 않는다
    late = person.t1 + 96.0
    assert expectations(s, world(late, retreat_at=[late])) == []
    assert expectations(s, world(late, retreat_at=[person.t1 + 5.0, late])) == ["retreats 1 > 0"]
    whole_run = build("express_both_ends", "d2")
    assert whole_run.quiet_until_s is None


@pytest.mark.parametrize("name", ["person_in_express", "person_at_dock"])
def test_the_store_person_situations_judge_the_same_window(name):
    """매장 상황의 사람 둘도 같다. D2 씨드 3은 사람과 상관없이 118.9 s에 동쪽 구석에서 우회한다."""
    s = build(name, "d2")
    [person] = s.injections
    assert s.quiet_until_s == pytest.approx(person.t1 + 30.0)
    assert run_situation(name, "d2", 3).failures == []


@pytest.mark.parametrize("mode", ["perfect", "none"])
def test_a_box_that_stays_lets_a_robot_wait_in_the_express(mode):
    """상자가 600 s 그대로면 막힌 로봇이 급행 안에서 도크의 로봇이 나가기를 기다린다(정한 동작).
    이 기다림이 불변식 5번으로 실패하면 회귀 명령(avoid_regression.sh)의 1단계가 늘 1로 끝난다."""
    s = build("box_stays", "d2").for_mode(mode)
    assert "I5_STANDING" in s.allowed
    assert run_situation("box_stays", "d2", 1, avoid_mode=mode).failures == []


def test_a_silent_stall_fails_every_mode_as_a_safety_rule():
    for mode in ("none", "perfect"):
        outcome = run_situation("person_passing", "d2", 1, manager_factory=RecordingManager,
                                avoid_mode=mode)
        assert not [f for f in outcome.failures if "SILENT_STALL" in f]
        outcome.world.send(EventOut("ERROR", "SILENT_STALL", "no progress", "pinky1"))
        assert "SILENT_STALL x1" in judge(build("person_passing", "d2").for_mode(mode),
                                          outcome.world)


def test_the_test_layout_runs_by_name_but_is_not_in_the_default_list():
    from fleet_core.fleet_map import KNOWN_LAYOUTS, TEST_LAYOUTS

    assert "d2_shelf" in TEST_LAYOUTS and "d2_shelf" in KNOWN_LAYOUTS
    assert not set(LAYOUTS) & set(TEST_LAYOUTS)
    assert KNOWN_LAYOUTS == (*LAYOUTS, *TEST_LAYOUTS)
    assert build("hall_obstacle", "d2_shelf").title


@pytest.mark.skipif(not TOOL.is_file(), reason="only in the source tree")
def test_the_tool_runs_the_candidates_by_default_and_a_test_layout_when_named(monkeypatch):
    from types import SimpleNamespace

    spec = importlib.util.spec_from_file_location("situations_tool", TOOL)
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    seen = []

    def stub(name, layout, seed, **_):
        seen.append(layout)
        central = SimpleNamespace(finished=1, total=1)
        return SimpleNamespace(world=SimpleNamespace(central=central), failures=[],
                               row=("x", layout, seed, "perfect", "1/1", "", "", "", "", "", "",
                                    "", "", "", ""))

    monkeypatch.setattr(tool, "run_situation", stub)
    tool.main(["-s", "hall_obstacle", "--seeds", "1"])
    assert seen == list(LAYOUTS)
    seen.clear()
    tool.main(["-s", "hall_obstacle", "--seeds", "1", "-l", "d2_shelf"])
    assert seen == ["d2_shelf"]
