"""줄 서는 대기 자리(stations.X.line: true). 켠 스테이션에서만:
- 도크 차례는 줄에 선 자리 순서다(앞자리 먼저). 줄에 아직 안 선 로봇은 줄에 선 로봇 뒤다.
  이미 도크로 떠난 로봇과 다시 도킹하는 로봇은 지금처럼 맨 앞이다.
- 앞자리가 비면 한 칸씩 당긴다.
- 새로 오는 로봇은 맨 뒤에 선 로봇 바로 뒤 빈 자리로 간다(앞자리를 노려 줄 옆을 지나지 않는다).
끄면(키가 없으면) 지금 규칙 그대로다(test_zone_dock_queue).
시험 지도는 v7에 대기 자리 셋(Q1 앞, Q3 꼬리)을 사슬로 이은 것이다: E-3 -> Q3 -> Q2 -> Q1 -> F -> W.
"""
from dataclasses import replace

import pytest
from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import FleetMapError, fleet_map_from_text, load_fleet_map
from zone_harness import DT, Harness

SPOTS = """  Q1: {x: 1.62, y: 0.10, kind: wait, yaw_deg: 0}
  Q2: {x: 1.40, y: 0.10, kind: wait, yaw_deg: 0}
  Q3: {x: 1.18, y: 0.10, kind: wait, yaw_deg: 0}
"""
NAV = """nav:
  - {nodes: [G, F]}
  - {nodes: [N_O, E-3]}
  - {nodes: [O_Q, E-1, E-2]}
  - {nodes: [E-2, E-3]}
  - {nodes: [N_O, N_E]}
  - {nodes: [N_O, O_Q]}
  - {nodes: [N_O, E-1]}
  - {nodes: [N_O, E-2]}
  - {nodes: [N_E, G]}
  - {nodes: [E-3, Q3]}
  - {nodes: [Q3, Q2]}
  - {nodes: [Q2, Q1]}
stations:"""


def line_map(line=True):
    text = load_fleet_map("v7").source_text
    start = text.index("  Q1: {")
    text = text[:start] + SPOTS + text[text.index("\n", start) + 1:]
    text = text[:text.index("nav:")] + NAV + text[text.index("\nstations:") + len("\nstations:"):]
    text = text.replace("  bay_in_1:", "  q_front: {points: [Q1, F]}\n  bay_in_1:")
    key = ", line: true" if line else ""
    text = text.replace("queue: [Q1], server: workcell",
                        f"queue: [Q1, Q2, Q3]{key}, server: workcell")
    return fleet_map_from_text(text)


def targets(world, robot):
    return [c.target_id for c in world.sent(MoveCmd, robot)]


def calls(world, *robots):
    """pinky1이 W에서 일하고(풀어 줄 때까지 도크를 쥔다) robots가 이 순서로 WORK를 바란다.
    첫 걸음 전에 부른다: 부르지 않은 로봇은 대기 자리에서 주차하러 떠난다."""
    world.report()
    busy = world.call("WORK", "pinky1")
    asked = []
    for robot in robots:
        world.now += DT / 2
        asked.append(world.call("WORK", robot))
    assert world.run_until(lambda: world.result_of(busy) is not None, 30.0)
    return asked


def test_the_line_key_is_read_and_is_off_by_default():
    assert line_map().stations["WORK"].line is True
    assert line_map(line=False).stations["WORK"].line is False
    for layout in ("d2", "d", "c", "v7"):
        assert not any(s.line for s in load_fleet_map(layout).stations.values()), layout
    text = line_map().source_text.replace("line: true", "line: 3")
    with pytest.raises(FleetMapError, match="line"):
        fleet_map_from_text(text)


def test_the_front_of_the_line_docks_first_and_the_rest_pull_forward():
    world = Harness(site=line_map(), starts={"pinky1": "W", "pinky2": "Q2", "pinky3": "Q3"})
    back, front = calls(world, "pinky3", "pinky2")  # 꼬리 로봇이 먼저 바랐다
    assert world.run_until(lambda: world.view("pinky2")["node"] == "Q1"
                           and world.view("pinky3")["node"] == "Q2", 60.0), "pulls forward"
    assert targets(world, "pinky2") == ["Q1"] and targets(world, "pinky3") == ["Q2"]
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(front) is not None, 120.0)
    assert world.result_of(front).robot_id == "pinky2", "the front spot docks first"
    assert world.result_of(back) is None
    assert world.run_until(lambda: world.view("pinky3")["node"] == "Q1", 60.0)
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.result_of(back) is not None, 200.0)
    assert "DOCK_TURN_TAKEN" not in world.codes()
    assert not world.problems


def test_without_the_key_the_old_rule_stays():
    world = Harness(site=line_map(line=False),
                    starts={"pinky1": "W", "pinky2": "Q2", "pinky3": "Q3"})
    calls(world, "pinky3", "pinky2")
    world.run(20.0)
    assert targets(world, "pinky2") == [] and targets(world, "pinky3") == [], "no pulling"


def test_a_new_robot_joins_behind_the_last_one_in_line():
    world = Harness(site=line_map(), starts={"pinky1": "W", "pinky2": "Q1", "pinky3": "P-1"})
    calls(world, "pinky2")
    world.call("WORK", "pinky3")
    assert world.run_until(lambda: world.view("pinky3")["node"] == "Q2", 200.0)
    goals = targets(world, "pinky3")
    assert "Q1" not in goals and goals[-1] == "Q2", goals
    assert "Q3" in goals or any(c.target_id == "Q3" for c in world.commands
                                if getattr(c, "robot", "") == "pinky3"), "enters at the tail"
    assert not world.problems


def test_a_robot_low_on_battery_leaves_the_line_and_the_rest_still_dock():
    world = Harness(site=line_map(), params=replace(load_params(), battery_low_pct=20.0),
                    starts={"pinky1": "W", "pinky2": "Q1", "pinky3": "Q2"})
    # 이름 없는 호출이라야 배터리가 낮은 로봇을 호출에서 뺀다(이름 붙은 호출은 끝까지 간다)
    calls(world)
    first, second = world.call("WORK"), world.call("WORK")
    world.run(5.0)
    assert world.phase("pinky2") == world.phase("pinky3") == "TO_WORK"
    bot = world.bots["pinky3"]
    world.traffic.on_robot_state("pinky3", bot.x, bot.y, bot.yaw, "IDLE", 15.0)
    # 줄 반대쪽(꼬리 Q3 -> E-3)으로 빠져 칸에 선다
    assert world.run_until(lambda: world.phase("pinky3") == "PARKED", 300.0), world.log[-20:]
    assert "Q3" in targets(world, "pinky3")
    # 남은 둘(pinky1, pinky2)이 두 호출을 끝낸다: 빠진 로봇이 줄을 막지 않는다
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(first) is not None, 200.0)
    done = world.result_of(first).robot_id
    world.traffic.on_release(done)
    assert world.run_until(lambda: world.result_of(second) is not None, 200.0)
    assert {done, world.result_of(second).robot_id} <= {"pinky1", "pinky2"}
    assert not world.problems
