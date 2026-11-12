"""구역 교통 층 둘레의 배선: 배터리 낮음과 다 참, FleetManager를 거치는 물러나기(Retreat),
재생 도구가 재생을 거절하는 기록."""
import json
import sys
from dataclasses import asdict, replace
from pathlib import Path

import pytest
from fleet_core.common.commands import EventOut, MoveCmd, RetreatCmd
from fleet_core.common.params import DEFAULT_PARAMS, load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.manager import FleetManager
from zone_harness import Harness

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import replay  # noqa: E402

# ---- 배터리 ----

def low_battery_world():
    world = Harness(params=replace(load_params(), battery_low_pct=20.0))
    world.step()
    return world


def battery(world, robot, pct):
    bot = world.bots[robot]
    world.traffic.on_robot_state(robot, bot.x, bot.y, bot.yaw, "IDLE", pct)


def test_a_robot_low_on_battery_takes_no_open_call_until_it_is_charged():
    world = low_battery_world()
    battery(world, "pinky2", 15.0)  # 열린 호출에 보통 오는 로봇이다
    low = [(e.level, e.detail) for e in world.events + world.traffic.take_output()
           if getattr(e, "code", "") == "ROBOT_BATTERY_LOW"]
    assert low == [("WARN", "15%, takes no new call")]
    assert world.view("pinky2")["charging"]
    call = world.call("WORK")
    assert world.run_until(lambda: world.result_of(call) is not None, 120.0)
    assert world.result_of(call).robot_id == "pinky1"
    assert world.phase("pinky2") == "PARKED"
    battery(world, "pinky2", 85.0)
    assert any(isinstance(e, EventOut) and e.code == "ROBOT_CHARGED"
               for e in world.traffic.take_output())
    assert not world.view("pinky2")["charging"]


def test_a_battery_between_the_two_marks_changes_nothing():
    world = low_battery_world()
    battery(world, "pinky1", 15.0)
    battery(world, "pinky1", 50.0)
    assert world.view("pinky1")["charging"], "not charged yet"
    battery(world, "pinky2", 0.0)  # 모르는 값
    assert not world.view("pinky2")["charging"]


def test_the_battery_watch_is_off_at_zero():
    world = Harness()
    world.step()
    battery(world, "pinky1", 5.0)
    assert not world.view("pinky1")["charging"]
    assert "ROBOT_BATTERY_LOW" not in [e.code for e in world.traffic.take_output()]


# ---- FleetManager를 거치는 물러나기 ----

def manager_at_the_work_dock():
    now = [0.0]
    core = FleetManager(load_fleet_map("d2"), load_params(), ["pinky1"], lambda: now[0])
    x, y = core.site.node_pose("W")
    core.on_robot_state("pinky1", x, y, 0.0, "IDLE")
    core.traffic.on_dispatch(77, "OUTBOUND", "pinky1")  # 첫 tick 전에 부른다
    return core, now


def test_the_retreat_out_of_a_dock_goes_out_and_its_result_reaches_the_traffic_layer():
    core, now = manager_at_the_work_dock()
    now[0] += 0.1
    out = core.tick()
    back = [c for c in out if isinstance(c, RetreatCmd)]
    assert len(back) == 1 and back[0].robot == "pinky1"
    x, y = back[0].path[-1][:2]
    core.on_robot_state("pinky1", x, y, 0.0, "IDLE")
    core.on_retreat_result("pinky1", back[0].goal_id, True, "")
    now[0] += 0.1
    out = core.tick()
    assert [c.target_id for c in out if isinstance(c, MoveCmd)] == ["O"], "on along the express"
    assert core.traffic.robot_view("pinky1")["node"] == "N_W"


def test_a_retreat_result_for_an_unknown_robot_is_refused():
    core, now = manager_at_the_work_dock()
    core.on_retreat_result("ghost", 1, True, "")
    out = core.tick()
    refused = [e.detail for e in out if isinstance(e, EventOut) and e.code == "INPUT_REJECTED"]
    assert len(refused) == 1 and refused[0].startswith("retreat result refused")


def test_a_failed_retreat_is_a_failed_attempt():
    core, now = manager_at_the_work_dock()
    now[0] += 0.1
    back = next(c for c in core.tick() if isinstance(c, RetreatCmd))
    core.on_retreat_result("pinky1", back.goal_id, False, "OBSTACLE_BEHIND")
    out = core.tick()
    retry = [e.detail for e in out if isinstance(e, EventOut) and e.code == "GOAL_RETRY"]
    assert retry == ["OBSTACLE_BEHIND, attempt 1"]


# ---- 재생 도구의 거절 ----

def head(**changes):
    params = asdict(load_params())
    text = (DEFAULT_PARAMS.parent / "fleet_map_d2.yaml").read_text(encoding="utf-8")
    line = {"robots": ["pinky1"], "params": params, "map": text, **changes}
    return json.dumps({k: v for k, v in line.items() if v is not None})


TICK = json.dumps({"t": 0.1, "in": "tick", "args": []})
REFUSED = {
    "empty": ([], "no first line"),
    "no inputs": ([head()], "no inputs"),
    "no map": ([head(map=None), TICK], "the first line has no ['map']"),
    "unknown parameter": ([head(params={**asdict(load_params()), "warp_speed": 9}), TICK],
                          "these parameters do not fit this code"),
    "a broken line in the middle": ([head(), "{oops", TICK], "line 2 is not JSON"),
}


@pytest.mark.parametrize("lines, why", REFUSED.values(), ids=REFUSED.keys())
def test_a_record_that_cannot_be_played_back_is_refused_with_its_reason(tmp_path, lines, why):
    path = tmp_path / "run.jsonl"
    path.write_text("".join(line + "\n" for line in lines), encoding="utf-8")
    with pytest.raises(ValueError, match=why.replace("[", r"\[").replace("]", r"\]")):
        replay.replay(path)


def test_a_record_cut_in_the_middle_of_its_last_line_plays_the_rest(tmp_path):
    path = tmp_path / "run.jsonl"
    path.write_text(head() + "\n" + TICK + "\n" + '{"t": 0.2, "in": "ti', encoding="utf-8")
    result = replay.replay(path)
    assert result.cut_line and result.inputs == 1 and not result.compared


def test_the_command_line_says_why_and_exits_with_two(tmp_path, monkeypatch, capsys):
    path = tmp_path / "run.jsonl"
    path.write_text(head() + "\n", encoding="utf-8")
    monkeypatch.setattr(sys, "argv", ["replay.py", str(path)])
    assert replay.main() == 2
    assert capsys.readouterr().out == "cannot play this back: no inputs, so there is nothing to " \
                                      "play back\n"
