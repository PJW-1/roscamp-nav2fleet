"""줄 서는 대기 자리(stations.X.line: true)의 도크 차례와 한 칸 당기기. 끄면 옛 규칙 그대로다.
지도는 자리 셋인 옛 v7(test/data/fleet_map_line3.yaml)이다."""
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
    """pode1이 W에서 일하고 robots가 이 순서로 WORK를 바란다. 첫 걸음 전에 부른다."""
    world.report()
    busy = world.call("WORK", "pode1")
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
    assert v7["WORK"].line and not v7["OUTBOUND"].line  # WORK에만
    text = line_map().source_text.replace("line: true", "line: 3")
    with pytest.raises(FleetMapError, match="line"):
        fleet_map_from_text(text)


def test_the_front_of_the_line_docks_first_and_the_rest_pull_forward():
    world = Harness(site=line_map(), starts={"pode1": "W", "pode2": "Q2", "pode3": "Q3"})
    back, front = calls(world, "pode3", "pode2")  # 꼬리 로봇이 먼저 바랐다
    assert world.run_until(lambda: world.view("pode2")["node"] == "Q1"
                           and world.view("pode3")["node"] == "Q2", 60.0), "pulls forward"
    assert targets(world, "pode2") == ["Q1"] and targets(world, "pode3") == ["Q2"]
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.result_of(front) is not None, 120.0)
    assert world.result_of(front).robot_id == "pode2", "the front spot docks first"
    assert world.result_of(back) is None
    assert world.run_until(lambda: world.view("pode3")["node"] == "Q1", 60.0)
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(back) is not None, 200.0)
    assert "DOCK_TURN_TAKEN" not in world.codes()
    assert not world.problems


def test_without_the_key_the_old_rule_stays():
    world = Harness(site=line_map(line=False),
                    starts={"pode1": "W", "pode2": "Q2", "pode3": "Q3"})
    calls(world, "pode3", "pode2")
    world.run(20.0)
    assert targets(world, "pode2") == [] and targets(world, "pode3") == [], "no pulling"


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_a_new_robot_joins_behind_the_last_one_in_line(order):
    """order 모델: 줄을 지나는 horizon을 다시 짜면 꼬리(Q3) 대신 앞자리(Q1) 쪽 길로 바뀌었다."""
    world = Harness(site=line_map(), starts={"pode1": "W", "pode2": "Q1", "pode3": "P-1"},
                    params=replace(load_params(), use_order_model=order))
    calls(world, "pode2")
    world.call("WORK", "pode3")
    assert world.run_until(lambda: world.view("pode3")["node"] == "Q2", 200.0)
    goals = targets(world, "pode3")
    assert "Q1" not in goals and goals[-1] == "Q2", goals
    assert "Q3" in goals or any(c.target_id == "Q3" for c in world.commands
                                if getattr(c, "robot", "") == "pode3"), "enters at the tail"
    assert not world.problems


def test_a_robot_low_on_battery_leaves_the_line_and_the_rest_still_dock():
    world = Harness(site=line_map(), params=replace(load_params(), battery_low_pct=20.0),
                    starts={"pode1": "W", "pode2": "Q1", "pode3": "Q3"})
    # 이름 없는 호출이라야 배터리가 낮은 로봇을 호출에서 뺀다(이름 붙은 호출은 끝까지 간다)
    calls(world)
    first, second = world.call("WORK"), world.call("WORK")
    world.run(5.0)
    assert world.phase("pode2") == world.phase("pode3") == "TO_WORK"
    bot = world.bots["pode3"]
    world.traffic.on_robot_state("pode3", bot.x, bot.y, bot.yaw, "IDLE", 15.0)
    # 꼬리에서 G로 빠져 칸에 선다(G는 through 지점이 아니라 합치지 않는다)
    assert world.run_until(lambda: world.phase("pode3") == "PARKED", 300.0), world.log[-20:]
    assert "G" in targets(world, "pode3")
    # 남은 둘(pode1, pode2)이 두 호출을 끝낸다: 빠진 로봇이 줄을 막지 않는다
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.result_of(first) is not None, 200.0)
    done = world.result_of(first).robot_id
    world.traffic.on_release(done)
    assert world.run_until(lambda: world.result_of(second) is not None, 200.0)
    assert {done, world.result_of(second).robot_id} <= {"pode1", "pode2"}
    assert not world.problems


def test_a_spot_held_by_a_robot_leaving_the_line_is_not_the_tail():
    """줄을 떠나는 로봇이 쥔 자리는 줄 꼬리로 세지 않아 새 로봇이 앞자리로 간다."""
    world = Harness(site=line_map(), starts={"pode1": "W", "pode2": "P-1", "pode3": "Q3"})
    world.report()
    world.call("WORK", "pode1")
    world.call("WORK", "pode2")
    world.step()  # pode3은 부르지 않았다: Q3을 쥔 채 주차하러 떠난다
    traffic = world.traffic
    station = world.site.stations["WORK"]
    leaving, new = traffic._robots["pode3"], traffic._robots["pode2"]
    assert traffic.grants.node_holder("Q3") == "pode3" and leaving.dest != "WORK"
    assert traffic._open_spots(new, station)[0] == "Q1"


@pytest.mark.parametrize("order_model", [False, True])
def test_a_robot_stepping_forward_ranks_by_the_spot_it_goes_to(order_model):
    """당기는 로봇은 마지막으로 선 자리가 아니라 가는 자리로 순위를 받는다."""
    world = Harness(site=line_map(), starts={"pode1": "W", "pode2": "Q2", "pode3": "Q3"},
                    params=replace(load_params(), use_order_model=order_model))
    world.report()
    for robot in ("pode1", "pode3", "pode2"):
        world.call("WORK", robot)
    world.step()
    traffic = world.traffic
    moving = traffic._robots["pode2"]
    assert moving.node == "Q2" and moving.route is not None
    assert moving.route.edges[0].b == "Q1"
    assert traffic._line_spot(moving, world.site.stations["WORK"]) == 0


def test_v7_new_robot_stands_behind_the_front_and_the_line_docks_in_order():
    """v7: 앞자리가 차면 새 로봇은 남쪽 통로로 Q2에 와 서고, 도크가 비면 Q1, 그다음 Q2 차례로 든다."""
    world = Harness(layout="v7", starts={"pode1": "W", "pode2": "Q1", "pode3": "P-1"})
    front, back = calls(world, "pode2", "pode3")
    assert world.run_until(lambda: world.view("pode3")["node"] == "Q2", 300.0), world.log[-10:]
    # E-1 -> E-3 -> Q2는 Nav2 goal 하나로 합쳐 보낸다
    assert targets(world, "pode3") == ["E-1", "Q2"], "along the south aisle"
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.result_of(front) is not None, 200.0)
    assert world.result_of(front).robot_id == "pode2" and world.result_of(back) is None
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(back) is not None, 300.0)
    assert "Q1" in targets(world, "pode3"), "steps forward before it docks"
    assert not world.problems


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_v7_the_front_robot_docks_first_even_if_the_one_behind_asked_first(order):
    """줄 순서는 지금 선 자리다(지나온 자리로 매기면 두 로봇이 서로 기다려 선다)."""
    world = Harness(layout="v7", starts={"pode1": "W", "pode2": "E-3", "pode3": "P-1"},
                    params=replace(load_params(), use_order_model=order))
    behind, front = calls(world, "pode3", "pode2")  # pode3이 먼저 불렸지만 멀다
    assert world.run_until(lambda: world.view("pode2")["node"] == "Q1"
                           and world.view("pode3")["node"] == "Q2", 120.0), world.log[-30:]
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.result_of(front) is not None, 60.0), world.log[-10:]
    assert world.result_of(behind) is None
    assert not world.problems


def test_v7_a_full_line_pulls_forward_as_soon_as_the_front_moves():
    """v7 줄이 가득이면 앞자리 로봇이 도크에 들 때 Q2 로봇이 바로 Q1으로 당긴다."""
    world = Harness(layout="v7", starts={"pode1": "W", "pode2": "Q1", "pode3": "Q2"})
    front, back = calls(world, "pode2", "pode3")
    world.run(20.0)
    assert targets(world, "pode2") == [] and targets(world, "pode3") == [], "the line is full"
    world.traffic.on_release("pode1")
    assert world.run_until(lambda: world.result_of(front) is not None, 200.0)
    assert world.result_of(front).robot_id == "pode2" and world.result_of(back) is None
    assert world.run_until(lambda: world.view("pode3")["node"] == "Q1", 60.0), world.log[-10:]
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(back) is not None, 300.0)
    assert not world.problems


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_v7_the_robot_at_the_front_goes_before_a_turn_taken_from_behind(order):
    """한 방향 줄에서는 남의 길 위 자리를 쥔 앞 로봇이 도크 차례를 먼저 잡는다."""
    world = Harness(layout="v7", starts={"pode1": "P-1", "pode2": "Q1"},
                    params=replace(load_params(), use_order_model=order))
    world.report()
    behind = world.call("WORK", "pode1")
    world.step()  # pode2는 아직 WORK로 가지 않는다: pode1이 도크 차례를 가져간다
    route = world.traffic._robots["pode1"].route
    assert route is not None and route.turn and route.goal == "W" and "Q1" in route.nodes_left()
    front = world.call("WORK", "pode2")
    assert world.run_until(lambda: world.result_of(front) is not None, 120.0), world.log[-10:]
    assert world.result_of(front).robot_id == "pode2" and world.result_of(front).success
    assert world.result_of(behind) is None
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: world.result_of(behind) is not None, 300.0), world.log[-10:]
    assert world.result_of(behind).robot_id == "pode1" and world.result_of(behind).success
    assert not world.problems


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_v7_three_robots_all_dock_after_a_turn_taken_from_behind(order):
    """위와 같되 셋째 로봇이 칸(P-3)에서 오고, 세 쌍의 순서가 어긋나도 교착하지 않는다."""
    world = Harness(layout="v7", starts={"pode1": "P-1", "pode2": "Q1", "pode3": "P-3"},
                    params=replace(load_params(), use_order_model=order))
    world.report()
    asked = [world.call("WORK", "pode1")]
    world.step()
    route = world.traffic._robots["pode1"].route
    assert route.turn and route.goal == "W" and "Q1" in route.nodes_left()
    asked += [world.call("WORK", "pode2"), world.call("WORK", "pode3")]
    docked = []

    def next_done():
        return next((world.result_of(g).robot_id for g in asked if world.result_of(g) is not None
                     and world.result_of(g).robot_id not in docked), None)

    for _ in asked:
        assert world.run_until(lambda: next_done() is not None, 300.0), (docked, world.log[-10:])
        docked.append(next_done())
        world.traffic.on_release(docked[-1])
    assert docked[0] == "pode2", "the robot at the front docks first"
    assert all(world.result_of(g).success for g in asked)
    assert not world.problems


# 열린 위험: 빈 칸으로 주차하러 가던 order를 대기 자리로 돌릴 때 E-3 로봇이 입구를 쥐어 M_E로 돈다.
class Detour(AssertionError):
    """M_E로 돌았다(다른 단언은 그냥 실패한다)."""


P3_DETOUR = pytest.mark.xfail(strict=True, raises=Detour,
                              reason="열린 위험: order가 E-3이 놓이기 전에 "
                                     "대기 자리로 돌려 M_E로 돈다")


@pytest.mark.parametrize(("parked", "order"), [
    pytest.param(parked, order, id=f"{parked[0]}-{parked[1]}-order_{'on' if order else 'off'}",
                 marks=[P3_DETOUR] if "P-3" in parked else [])
    for parked in (("P-1", "P-2"), ("P-1", "P-3"), ("P-2", "P-3")) for order in (False, True)])
def test_v7_a_robot_leaving_outbound_passes_the_doors_of_parked_robots(parked, order):
    """칸에 선 로봇은 자기 칸만 쥐고, 출고에서 나온 로봇은 칸 입구를 지나는 Nav2 goal 하나로 줄 입구에 간다."""
    doors = ["E-" + spot.removeprefix("P-") for spot in parked]
    world = Harness(layout="v7", starts={"pode1": doors[0], "pode2": doors[1], "pode3": "O"},
                    params=replace(load_params(), use_order_model=order))
    world.report()
    asked = world.call("WORK", "pode3")
    assert world.run_until(lambda: world.result_of(asked) is not None, 300.0), world.codes()
    # 할 일 없는 두 로봇은 입구에서 후진으로 칸에 들어가 서고, 자기 칸만 쥔다
    for robot, spot in zip(("pode1", "pode2"), parked, strict=True):
        assert world.view(robot)["node"] == spot, robot
        assert set(world.view(robot)["nodes"]) == {spot}, (robot, world.view(robot)["nodes"])
    # 입구 로봇이 후진 중이면 그 입구를 쥐어 다른 입구까지 먼저 가지만 M_E로 돌지는 않는다
    went = targets(world, "pode3")
    assert went[0] == "O_OUT" and went[-3:] == ["Q2", "Q1", "W"], went
    assert not world.problems
    if "M_E" in went:
        raise Detour(went)


def test_v7_a_robot_low_on_battery_in_the_open_yard_goes_to_a_door_and_backs_in():
    """마당을 Nav2 goal로 가던 로봇의 배터리가 떨어지면 칸 입구로 보낸 뒤 후진으로 넣는다."""
    world = Harness(layout="v7", params=replace(load_params(), battery_low_pct=20.0),
                    starts={"pode1": "P-1", "pode2": "W"})
    world.report()
    world.call("WORK", "pode2")  # 도크를 쥐고 있다
    world.call("WORK")            # 열린 호출: pode1이 줄로 간다. 배터리가 낮으면 내놓는다
    bot = world.bots["pode1"]
    assert world.run_until(lambda: bot.x > 0.45, 60.0), (bot.x, bot.y)
    world.traffic.on_robot_state("pode1", bot.x, bot.y, bot.yaw, "MOVING", 15.0)
    assert world.run_until(lambda: world.view("pode1")["state"] == "PARKED", 200.0), \
        world.codes("pode1")[-12:]
    into_bays = [t for t in targets(world, "pode1") if t.startswith("P-")]
    assert not into_bays, into_bays
    assert world.retreats("pode1")
    assert not world.problems


def low_battery_in_line(spot):
    """W에 pode1, 줄에 둘이 WORK를 기다린다. spot에 선 로봇의 배터리를 낮춘다."""
    world = Harness(layout="v7", params=replace(load_params(), battery_low_pct=20.0),
                    starts={"pode1": "W", "pode2": "Q1", "pode3": "Q2"})
    calls(world)
    asked = world.call("WORK"), world.call("WORK")
    world.run(5.0)
    stand = {world.view(r)["node"]: r for r in ("pode2", "pode3")}
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
    assert {done, world.result_of(second).robot_id} <= {"pode1", rest}
    assert not world.problems


def test_v7_the_robot_at_the_entry_low_on_battery_leaves_at_once():
    """v7: Q2 로봇이 배터리가 낮으면 칸 입구로 나가 칸에 서고 남은 둘이 두 호출을 끝낸다."""
    world, asked, low, rest = low_battery_in_line("Q2")
    assert world.run_until(lambda: world.phase(low) == "PARKED", 120.0), world.log[-20:]
    assert "Q1" not in targets(world, low) and "W" not in targets(world, low)
    world.traffic.on_release("pode1")
    finish(world, asked, rest)


def test_v7_the_front_robot_low_on_battery_leaves_through_the_docks():
    """v7: 앞자리 로봇이 배터리가 낮으면 도크가 빌 때까지 줄을 막다가 W를 지나 M_E로 풀려 칸에 선다."""
    world, asked, low, rest = low_battery_in_line("Q1")
    world.run(60.0)
    assert world.view(low)["node"] == "Q1" and world.phase(low) == "TO_PARK", "waits for the dock"
    for _ in range(6):
        if world.run_until(lambda: world.phase(low) == "PARKED", 150.0):
            break
        for robot in ("pode1", rest):
            if world.phase(robot) == "AT_STATION":
                world.traffic.on_release(robot)
    assert world.phase(low) == "PARKED", world.log[-20:]
    went = targets(world, low)
    assert went[:2] == ["W", "M_E"] and not {"O", "O_OUT"} & set(went), went
    assert not world.sent(DockCmd, low), "passes the docks without docking"
    assert all(world.result_of(goal) is not None for goal in asked)
    assert not world.problems
