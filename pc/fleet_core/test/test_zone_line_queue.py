"""줄 서는 대기 자리(stations.X.line: true). 켠 스테이션에서만:
- 도크 차례는 줄에 선 자리 순서다(앞자리 먼저). 줄에 아직 안 선 로봇은 줄에 선 로봇 뒤다.
  이미 도크로 떠난 로봇과 다시 도킹하는 로봇은 지금처럼 맨 앞이다.
- 앞자리가 비면 한 칸씩 당긴다.
- 새로 오는 로봇은 맨 뒤에 선 로봇 바로 뒤 빈 자리로 간다(앞자리를 노려 줄 옆을 지나지 않는다).
끄면(키가 없으면) 지금 규칙 그대로다(test_zone_dock_queue).
규칙 시험의 지도는 자리 셋인 옛 v7(test/data/fleet_map_line3.yaml, 구석 L자, Q1 앞, Q3 꼬리)이다:
G -> Q3 -> Q2 -> Q1 -> W, 빈 줄이면 G -> Q1. 지금 v7은 자리 둘(Q1, Q2)이고 Q2로 들어온다.
"""
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.commands import DockCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import FleetMapError, fleet_map_from_text, load_fleet_map
from zone_harness import DT, Harness

LINE3 = Path(__file__).parent / "data" / "fleet_map_line3.yaml"


def line_map(line=True):
    text = LINE3.read_text(encoding="utf-8")
    old = "queue: [Q1, Q2, Q3], line: true,"
    assert old in text
    return fleet_map_from_text(text if line else text.replace(old, "queue: [Q1, Q2, Q3],"))


def targets(world, robot):
    """보낸 구간(물러나기 말고)의 끝 지점. order 모델이면 base 조각마다 하나다."""
    return [c.target_id for c in world.legs(robot) if c.mode != "retreat"]


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
    for layout in ("d2", "d", "c"):
        assert not any(s.line for s in load_fleet_map(layout).stations.values()), layout
    v7 = load_fleet_map("v7").stations
    assert v7["WORK"].line and not v7["OUTBOUND"].line  # 사용자 결정: WORK에만
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
                    starts={"pinky1": "W", "pinky2": "Q1", "pinky3": "Q3"})
    # 이름 없는 호출이라야 배터리가 낮은 로봇을 호출에서 뺀다(이름 붙은 호출은 끝까지 간다)
    calls(world)
    first, second = world.call("WORK"), world.call("WORK")
    world.run(5.0)
    assert world.phase("pinky2") == world.phase("pinky3") == "TO_WORK"
    bot = world.bots["pinky3"]
    world.traffic.on_robot_state("pinky3", bot.x, bot.y, bot.yaw, "IDLE", 15.0)
    # 꼬리에서 G로 빠져 칸에 선다
    assert world.run_until(lambda: world.phase("pinky3") == "PARKED", 300.0), world.log[-20:]
    assert "G" in targets(world, "pinky3")
    # 남은 둘(pinky1, pinky2)이 두 호출을 끝낸다: 빠진 로봇이 줄을 막지 않는다
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(first) is not None, 200.0)
    done = world.result_of(first).robot_id
    world.traffic.on_release(done)
    assert world.run_until(lambda: world.result_of(second) is not None, 200.0)
    assert {done, world.result_of(second).robot_id} <= {"pinky1", "pinky2"}
    assert not world.problems


def test_a_spot_held_by_a_robot_leaving_the_line_is_not_the_tail():
    """줄을 떠나는 로봇(이 스테이션으로 가지 않는 로봇)이 아직 쥔 자리는 줄 꼬리로 세지 않는다
    (40b9767 리뷰): 앞자리가 비었으면 새 로봇은 앞자리로 간다."""
    world = Harness(site=line_map(), starts={"pinky1": "W", "pinky2": "P-1", "pinky3": "Q3"})
    world.report()
    world.call("WORK", "pinky1")
    world.call("WORK", "pinky2")
    world.step()  # pinky3은 부르지 않았다: Q3을 쥔 채 주차하러 떠난다
    traffic = world.traffic
    station = world.site.stations["WORK"]
    leaving, new = traffic._robots["pinky3"], traffic._robots["pinky2"]
    assert traffic.grants.node_holder("Q3") == "pinky3" and leaving.dest != "WORK"
    assert traffic._open_spots(new, station)[0] == "Q1"


@pytest.mark.parametrize("order_model", [False, True])
def test_a_robot_stepping_forward_ranks_by_the_spot_it_goes_to(order_model):
    """한 칸 당기는 로봇은 마지막으로 선 자리가 아니라 가는 자리로 순위를 받는다(40b9767 리뷰).
    order 모델은 대기 트립 끝에 도크까지 붙여 trip.goal이 도크가 된다(6810b3b 리뷰)."""
    world = Harness(site=line_map(), starts={"pinky1": "W", "pinky2": "Q2", "pinky3": "Q3"},
                    params=replace(load_params(), use_order_model=order_model))
    world.report()
    for robot in ("pinky1", "pinky3", "pinky2"):
        world.call("WORK", robot)
    world.step()
    traffic = world.traffic
    moving = traffic._robots["pinky2"]
    assert moving.node == "Q2" and moving.trip is not None
    assert moving.trip.links[0].b == "Q1"
    assert traffic._line_spot(moving, world.site.stations["WORK"]) == 0


def test_v7_new_robot_stands_behind_the_front_and_the_line_docks_in_order():
    """v7: 앞자리 Q1에 하나가 서 있으면 새 로봇은 칸 입구에서 남쪽 통로로 줄 입구 Q2에 와 서고
    (앞자리를 노리지 않는다), 도크가 비면 Q1, 그다음 Q2 차례로 들어간다(Q2는 Q1으로 당긴 뒤 W로)."""
    world = Harness(layout="v7", starts={"pinky1": "W", "pinky2": "Q1", "pinky3": "P-1"})
    front, back = calls(world, "pinky2", "pinky3")
    assert world.run_until(lambda: world.view("pinky3")["node"] == "Q2", 300.0), world.log[-10:]
    assert targets(world, "pinky3") == ["E-1", "Q2"], "straight along the south aisle"
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(front) is not None, 200.0)
    assert world.result_of(front).robot_id == "pinky2" and world.result_of(back) is None
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.result_of(back) is not None, 300.0)
    assert "Q1" in targets(world, "pinky3"), "steps forward before it docks"
    assert not world.problems


def test_v7_the_front_robot_docks_first_even_if_the_one_behind_asked_first():
    """줄 순서는 지금 선 자리다. order 모델은 트립을 도크까지 이어 두므로 이미 지나온 자리(Q2)로
    순위를 매기면 앞자리 로봇이 먼저 부른 뒷자리 로봇을 기다리고, 뒷자리 로봇은 앞자리가 비기를
    기다려 둘 다 선다(Gazebo v7 2026-11-14: Q1의 로봇이 도크가 빈 채 60 s 넘게 섰다)."""
    world = Harness(layout="v7", starts={"pinky1": "W", "pinky2": "E-3", "pinky3": "P-1"})
    behind, front = calls(world, "pinky3", "pinky2")  # pinky3이 먼저 불렸지만 멀다
    assert world.run_until(lambda: world.view("pinky2")["node"] == "Q1"
                           and world.view("pinky3")["node"] == "Q2", 120.0), world.log[-30:]
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(front) is not None, 60.0), world.log[-10:]
    assert world.result_of(behind) is None
    assert not world.problems


def test_v7_a_full_line_pulls_forward_as_soon_as_the_front_moves():
    """v7 줄이 가득(W, Q1, Q2): 도크가 비어 앞자리 로봇이 들어가면 Q2의 로봇이 바로 Q1으로 당긴다.
    두 자리가 노드 놓는 거리(0.20)보다 멀어 당긴 로봇이 뒷자리를 놓는다."""
    world = Harness(layout="v7", starts={"pinky1": "W", "pinky2": "Q1", "pinky3": "Q2"})
    front, back = calls(world, "pinky2", "pinky3")
    world.run(20.0)
    assert targets(world, "pinky2") == [] and targets(world, "pinky3") == [], "the line is full"
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(front) is not None, 200.0)
    assert world.result_of(front).robot_id == "pinky2" and world.result_of(back) is None
    assert world.run_until(lambda: world.view("pinky3")["node"] == "Q1", 60.0), world.log[-10:]
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.result_of(back) is not None, 300.0)
    assert not world.problems


@pytest.mark.parametrize("parked", [("P-1", "P-2"), ("P-1", "P-3"), ("P-2", "P-3")])
def test_v7_a_robot_leaving_outbound_gets_past_the_doors_parked_robots_hold(parked):
    """칸에 선 로봇은 자기 칸 입구를 쥐고 있다. 출고에서 나온 로봇은 남은 입구로 지나 줄에 선다
    (b5f1e8f 리뷰: 길이 있다는 것만이 아니라 관제가 그 길을 고르는지)."""
    held = ["E-" + spot.removeprefix("P-") for spot in parked]
    world = Harness(layout="v7", starts={"pinky1": held[0], "pinky2": held[1], "pinky3": "O"})
    world.report()
    asked = world.call("WORK", "pinky3")
    assert world.run_until(lambda: world.result_of(asked) is not None, 300.0), world.codes()
    # 할 일 없는 두 로봇은 입구에서 후진으로 칸에 들어가 서고, 입구(0.19 < node_clear_m)를 쥐고 있다
    for robot, spot, door in zip(("pinky1", "pinky2"), parked, held, strict=True):
        assert world.view(robot)["node"] == spot and door in world.view(robot)["nodes"], robot
    held = set(held)
    went = targets(world, "pinky3")
    assert went[0] == "O_OUT" and went[-3:] == ["Q2", "Q1", "W"], went
    assert not held & set(went), went
    assert not world.problems


def test_v7_a_robot_low_on_battery_in_the_open_yard_goes_to_a_door_and_backs_in():
    """마당을 Nav2 goal로 가던 로봇의 배터리가 떨어지면 칸 입구로 보낸 뒤 후진으로 넣는다.
    선 자리에서 칸 안으로 Nav2 goal을 곧장 보내지 않는다(Gazebo 2026-11-13 밤: 'NAV_GOAL_SENT to
    P-3'을 받은 로봇이 칸에 비스듬히 들어가다 입구에 걸려 섰고, 그 앞 입구에 온 로봇까지 막았다)."""
    world = Harness(layout="v7", params=replace(load_params(), battery_low_pct=20.0),
                    starts={"pinky1": "P-1", "pinky2": "W"})
    world.report()
    world.call("WORK", "pinky2")  # 도크를 쥐고 있다
    world.call("WORK")            # 열린 호출: pinky1이 줄로 간다. 배터리가 낮으면 내놓는다
    bot = world.bots["pinky1"]
    assert world.run_until(lambda: bot.x > 0.45, 60.0), (bot.x, bot.y)
    world.traffic.on_robot_state("pinky1", bot.x, bot.y, bot.yaw, "MOVING", 15.0)
    assert world.run_until(lambda: world.view("pinky1")["state"] == "PARKED", 200.0), \
        world.codes("pinky1")[-12:]
    into_bays = [t for t in targets(world, "pinky1") if t.startswith("P-")]
    assert not into_bays, into_bays
    assert [c for c in world.legs("pinky1") if c.mode == "retreat"]
    assert not world.problems


def low_battery_in_line(spot):
    """W에 pinky1, 줄에 둘이 WORK를 기다린다. spot에 선 로봇의 배터리를 낮춘다(누가 어디 서는지는
    부르기 전 주차 출발과 차례에 따라 바뀐다)."""
    world = Harness(layout="v7", params=replace(load_params(), battery_low_pct=20.0),
                    starts={"pinky1": "W", "pinky2": "Q1", "pinky3": "Q2"})
    calls(world)
    asked = world.call("WORK"), world.call("WORK")
    world.run(5.0)
    stand = {world.view(r)["node"]: r for r in ("pinky2", "pinky3")}
    assert set(stand) == {"Q1", "Q2"}, stand
    assert all(world.phase(r) == "TO_WORK" for r in stand.values())
    low = stand[spot]
    bot = world.bots[low]
    world.traffic.on_robot_state(low, bot.x, bot.y, bot.yaw, "IDLE", 15.0)
    return world, asked, low, stand["Q2" if spot == "Q1" else "Q1"]


def finish(world, asked, rest):
    first, second = asked
    assert world.run_until(lambda: world.result_of(first) is not None, 300.0)
    done = world.result_of(first).robot_id
    world.traffic.on_release(done)
    assert world.run_until(lambda: world.result_of(second) is not None, 300.0)
    assert {done, world.result_of(second).robot_id} <= {"pinky1", rest}
    assert not world.problems


def test_v7_the_robot_at_the_entry_low_on_battery_leaves_at_once():
    """v7: 줄 입구 Q2는 마당과 이어져 있다. Q2의 로봇이 배터리가 낮으면 앞자리 로봇을 기다리지 않고
    칸 입구로 나가 칸에 서고, 남은 둘이 두 호출을 끝낸다."""
    world, asked, low, rest = low_battery_in_line("Q2")
    assert world.run_until(lambda: world.phase(low) == "PARKED", 120.0), world.log[-20:]
    assert "Q1" not in targets(world, low) and "W" not in targets(world, low)
    world.traffic.on_release("pinky1")
    finish(world, asked, rest)


def test_v7_the_front_robot_low_on_battery_leaves_through_the_docks():
    """v7: 앞자리 Q1에서 나가는 길은 W뿐이다(사용자 결정 2026-11-13 밤: N_E와 G가 없다). 앞자리
    로봇이 배터리가 낮으면 도크가 빌 때까지 줄을 막고 서 있다가, W와 급행, 출고를 도킹 없이 지나
    칸에 선다.
    늦을 뿐 교착은 없고, 뒤 로봇은 그다음에 도크에 들어간다."""
    world, asked, low, rest = low_battery_in_line("Q1")
    world.run(60.0)
    assert world.view(low)["node"] == "Q1" and world.phase(low) == "TO_PARK", "waits for the dock"
    for _ in range(6):
        if world.run_until(lambda: world.phase(low) == "PARKED", 150.0):
            break
        for robot in ("pinky1", rest):
            if world.phase(robot) == "AT_STATION":
                world.traffic.on_release(robot)
    assert world.phase(low) == "PARKED", world.log[-20:]
    assert targets(world, low)[:3] == ["W", "O", "O_OUT"]
    assert not world.sent(DockCmd, low), "passes the docks without docking"
    assert all(world.result_of(goal) is not None for goal in asked)
    assert not world.problems
