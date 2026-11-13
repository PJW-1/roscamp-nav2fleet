"""줄 서는 대기 자리(stations.X.line: true). 켠 스테이션에서만:
- 도크 차례는 줄에 선 자리 순서다(앞자리 먼저). 줄에 아직 안 선 로봇은 줄에 선 로봇 뒤다.
  이미 도크로 떠난 로봇과 다시 도킹하는 로봇은 지금처럼 맨 앞이다.
- 앞자리가 비면 한 칸씩 당긴다.
- 새로 오는 로봇은 맨 뒤에 선 로봇 바로 뒤 빈 자리로 간다(앞자리를 노려 줄 옆을 지나지 않는다).
끄면(키가 없으면) 지금 규칙 그대로다(test_zone_dock_queue).
시험 지도는 v7 그대로다(구석 L자 셋, Q1 앞, Q3 꼬리): G -> Q3 -> Q2 -> Q1 -> W, 빈 줄이면 G -> Q1.
"""
from dataclasses import replace

import pytest
from fleet_core.common.commands import MoveCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import FleetMapError, fleet_map_from_text, load_fleet_map
from zone_harness import DT, Harness

def line_map(line=True):
    text = load_fleet_map("v7").source_text
    old = "queue: [Q1, Q2, Q3], line: true,"
    assert old in text
    return fleet_map_from_text(text if line else text.replace(old, "queue: [Q1, Q2, Q3],"))


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
    """v7: 앞자리 Q1에 하나가 서 있으면 새 로봇은 G에서 꼬리 Q3으로 들어와 Q2까지 당기고(앞자리를
    노리지 않는다), 도크가 비면 Q1, 그다음 Q2 차례로 들어간다(Q2는 Q1으로 한 칸 당긴 뒤 W로)."""
    world = Harness(layout="v7", starts={"pinky1": "W", "pinky2": "Q1", "pinky3": "P-1"})
    front, back = calls(world, "pinky2", "pinky3")
    assert world.run_until(lambda: world.view("pinky3")["node"] == "Q2", 300.0), world.log[-10:]
    assert "Q1" not in targets(world, "pinky3")
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(front) is not None, 200.0)
    assert world.result_of(front).robot_id == "pinky2" and world.result_of(back) is None
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.result_of(back) is not None, 300.0)
    assert "Q1" in targets(world, "pinky3"), "steps forward before it docks"
    assert not world.problems


def test_v7_a_full_line_pulls_forward_as_soon_as_the_front_moves():
    """v7 줄이 가득(W, Q1, Q2... 로봇 셋이라 W, Q2, Q3): 앞자리가 비면 둘 다 한 칸씩 당긴다. 이웃
    자리가 노드 놓는 거리(0.20)보다 멀어 당긴 로봇이 뒷자리를 놓으므로, 꼬리 로봇이 도크가 빌 때까지
    기다리지 않고 Q2로 온다."""
    world = Harness(layout="v7", starts={"pinky1": "W", "pinky2": "Q2", "pinky3": "Q3"})
    front, back = calls(world, "pinky2", "pinky3")
    assert world.run_until(lambda: world.view("pinky2")["node"] == "Q1"
                           and world.view("pinky3")["node"] == "Q2", 120.0), world.log[-10:]
    assert world.result_of(front) is None  # 도크는 아직 pinky1이 쥐고 있다
    world.traffic.on_release("pinky1")
    assert world.run_until(lambda: world.result_of(front) is not None, 200.0)
    world.traffic.on_release("pinky2")
    assert world.run_until(lambda: world.result_of(back) is not None, 300.0)
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


def test_v7_the_middle_robot_low_on_battery_leaves_once_the_front_moves():
    """v7: 둘째 Q2는 줄 안(Q2 -> Q1)으로만 나간다. 둘째 로봇이 배터리가 낮으면 앞자리 로봇이 도크로
    들어간 뒤 Q1 -> G로 빠진다: 늦을 뿐 교착은 없다."""
    world, asked, low, rest = low_battery_in_line("Q2")
    world.run(30.0)
    assert world.view(low)["node"] == "Q2", "waits for the front robot"
    # 도크에 선 로봇의 일을 끝낼 때마다 줄이 한 칸씩 빠진다(pinky1이 남은 호출을 다시 받을 수 있다)
    for _ in range(5):
        if world.run_until(lambda: world.phase(low) == "PARKED", 120.0):
            break
        for robot in ("pinky1", rest):
            if world.phase(robot) == "AT_STATION":
                world.traffic.on_release(robot)
    assert world.phase(low) == "PARKED", world.log[-20:]
    assert all(world.result_of(goal) is not None for goal in asked)
    assert "Q1" in targets(world, low) and "G" in targets(world, low)
    assert not world.problems


def test_v7_the_front_robot_low_on_battery_leaves_while_the_tail_waits():
    """v7: 줄 앞 Q1의 로봇이 배터리가 낮아 떠난다. 둘째 Q2에 로봇이 서 있으니 Q1 -> G로
    빠져 칸에 서고, 둘째 로봇은 한 칸 당겨 도크에 들어간다(6810b3b 리뷰: 서로 막을 수 있다)."""
    world, asked, low, rest = low_battery_in_line("Q1")
    assert world.run_until(lambda: world.phase(low) == "PARKED", 400.0), world.log[-20:]
    assert "G" in targets(world, low)
    world.traffic.on_release("pinky1")
    finish(world, asked, rest)
