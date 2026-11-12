"""회피 회귀 시험의 판정(tools/avoid_regression.py): 한 대 주행 요약과 세 대 경기 결과."""
import json
import re
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import avoid_regression as reg  # noqa: E402
from triage import Row  # noqa: E402


def step(label, result="success", **extra):
    return {"step": label, "result": result, "pos_error_m": 0.02, "yaw_error_deg": 3.0, **extra}


def drive(*results, steps=None):
    return {"layout": "d2", "plan": "stage2", "laps": 3,
            "steps": len(results) if steps is None else steps, "done": len(results),
            "succeeded": sum(1 for r in results if r["result"] == "success"),
            "results": list(results)}


def test_a_drive_where_every_move_ended_well_passes():
    summary = drive(step("work_in", max_cross_track_m=0.01),
                    step("yard", truth_keepout=[], plan_keepout=[], plans=12))
    assert reg.drive_problems(summary) == []


def test_a_failed_move_and_the_moves_left_undone_are_named():
    """drive_check stops at the first failure: the rest of the plan was not driven."""
    problems = reg.drive_problems(drive(step("work_in"), step("work_out", "BLOCKED"), steps=9))
    assert any("work_out" in p and "BLOCKED" in p for p in problems)
    assert any("7" in p for p in problems), "seven moves were never run"


def test_an_empty_drive_does_not_pass():
    assert reg.drive_problems(drive())


def test_a_yard_move_through_a_keepout_zone_fails():
    """The robot's centre inside a keepout zone is deeper than any allowed body overlap."""
    by_truth = reg.drive_problems(drive(step("yard", truth_keepout=["J_W"], plan_keepout=[])))
    by_plan = reg.drive_problems(drive(step("yard", truth_keepout=[], plan_keepout=["EXPRESS"])))
    assert any("J_W" in p for p in by_truth) and any("EXPRESS" in p for p in by_plan)


def test_the_drive_table_has_one_row_per_kind_of_move_with_the_worst_errors():
    summary = drive(step("work_in", pos_error_m=0.02, max_cross_track_m=0.01),
                    step("work_in", pos_error_m=0.05, yaw_error_deg=-8.0, max_cross_track_m=0.03),
                    step("work_in", "BLOCKED"))
    [row] = reg.drive_rows(summary)
    assert row == ("work_in", 3, 2, 0.05, 8.0, 0.03)
    assert "| work_in | 3 | 2 | 0.050 | 8.0 | 0.030 |" in reg.drive_markdown(summary)


CHECK = {"keepout_depth_m": 0.012, "keepout_zone": "J_W", "keepout_robot": "pinky2",
         "keepout_at_s": 41.5}


def test_a_game_without_a_driving_cause_and_inside_the_keepout_limit_passes():
    rows = [Row("O-3", "회피", "BLOCKED 12 s", 80.0), Row("O-4", "진짜 막힘", "상자", 90.0)]
    assert reg.game_problems(rows, CHECK) == []


def test_an_order_left_open_by_driving_fails_the_game():
    problems = reg.game_problems([Row("O-5", "주행", "끝 보고 없음", 120.0)], CHECK)
    assert len(problems) == 1 and "O-5" in problems[0] and "주행" in problems[0]


def test_a_body_deeper_in_a_keepout_zone_than_the_limit_fails_the_game():
    deep = {**CHECK, "keepout_depth_m": 0.031}
    [problem] = reg.game_problems([], deep)
    assert "0.031" in problem and "J_W" in problem and "pinky2" in problem
    assert reg.game_problems([], deep, keepout_max_m=0.04) == []


def test_a_game_that_did_not_measure_the_keepout_depth_does_not_pass():
    assert reg.game_problems([], None)
    assert reg.game_problems([], {"keepout_depth_m": None})


def test_the_script_passes_only_arguments_the_launch_files_declare():
    """tools/avoid_regression.sh starts two launch files: a renamed argument must fail here."""
    script = (TOOLS / "avoid_regression.sh").read_text(encoding="utf-8")
    launch = TOOLS.parent / "sim" / "fleet_gz_sim" / "launch"
    for name in ("one_pinky.launch.py", "gz.launch.py"):
        declared = set(re.findall(r'^    "(\w+)": \(',
                                  (launch / name).read_text(encoding="utf-8"), flags=re.M))
        call = re.search(re.escape(name) + r"((?:.*\\\n)*.*)\n", script).group(1)
        passed = set(re.findall(r"(\w+):=", call))
        assert passed and passed <= declared, (name, sorted(passed - declared))


def test_the_drive_command_ends_with_one_on_a_problem(tmp_path, capsys):
    good, bad = tmp_path / "good.json", tmp_path / "bad.json"
    good.write_text(json.dumps(drive(step("work_in"))), encoding="utf-8")
    bad.write_text(json.dumps(drive(step("work_in", "timeout"))), encoding="utf-8")
    assert reg.main(["drive", str(good)]) == 0
    assert reg.main(["drive", str(bad)]) == 1
    assert "work_in" in capsys.readouterr().out


def test_the_game_command_reads_the_record_and_the_fleet_check(tmp_path, capsys):
    head = {"recorded": True, "robots": {"pinky1": "P-1"}, "map": "", "params": {}}
    (tmp_path / "fleet_record.jsonl").write_text(json.dumps(head) + "\n", encoding="utf-8")
    (tmp_path / "fleet_check.json").write_text(json.dumps(CHECK), encoding="utf-8")
    assert reg.main(["game", str(tmp_path)]) == 0
    (tmp_path / "fleet_check.json").write_text(
        json.dumps({**CHECK, "keepout_depth_m": 0.05}), encoding="utf-8")
    assert reg.main(["game", str(tmp_path)]) == 1
    assert reg.main(["game", str(tmp_path), "--keepout-max", "0.06"]) == 0
    assert reg.main(["game", str(tmp_path / "missing")]) == 1
    assert "0.050" in capsys.readouterr().out
