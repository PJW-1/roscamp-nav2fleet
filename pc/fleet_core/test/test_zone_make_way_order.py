"""order 모델을 켠 채 base 끝에서 goal이 열린 채 기다리는 로봇이 막힌 로봇의 길에 서면, 열린 order를 거두고 막힘 표시 때 주차 자리로 보낸다."""
from dataclasses import replace

from fleet_core.common.commands import CancelCmd
from fleet_core.common.params import load_params
from test_zone_make_way import run_until
from zone_harness import DT, Harness

ON = replace(load_params(), use_order_model=True)


def waiting_at_the_base_end_in_the_way():
    """pode3이 먼저 출고 도크로 불렸다. pode2는 대기 자리까지 가서 base 끝에서 기다린다."""
    world = Harness(starts={"pode1": "P-2", "pode2": "P-3", "pode3": "P-1"}, params=ON)
    world.report()
    first = world.call("OUTBOUND", "pode3")
    world.now += DT / 2
    second = world.call("OUTBOUND", "pode2")
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode2"), 30.0), world.log
    assert world.traffic._robots["pode2"].goal is not None, "its order goal stays open"
    bot = world.bots["pode2"]
    bot.x, bot.y = 1.69, 0.55  # 대기 자리에서 0.14 m 비켜 서서 N_E 앞을 막는다
    return world, first, second


def test_a_robot_waiting_at_its_base_end_in_the_way_is_sent_to_parking():
    world, first, second = waiting_at_the_base_end_in_the_way()
    assert run_until(world, lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode3"), 60.0)
    assert "HALL_YIELD" in world.codes("pode2"), "the robot in the way gives way"
    assert [c.robot for c in world.sent(CancelCmd) if c.robot == "pode2"], "its order is ended"
    assert run_until(world, lambda: world.result_of(first) is not None, 120.0)
    assert world.result_of(first).success, "the blocked robot reaches the dock"
    assert "GOAL_FAILED" not in world.codes()
    world.traffic.on_release("pode3")
    assert run_until(world, lambda: world.result_of(second) is not None, 300.0)
    assert world.result_of(second).success, "the robot that gave way still gets its turn"


def test_a_robot_on_its_way_to_the_dock_keeps_the_turn_when_its_order_is_extended():
    """차례를 받아 도크로 떠난 로봇은 base를 늘릴 때도 차례를 지킨다."""
    world = Harness(layout="d", starts={"pode2": "P-1", "pode3": "P-3"}, params=ON)
    world.report()
    traffic = world.traffic
    for edge in (("P-3", "N_E"), ("P-3", "O_Q")):
        traffic._blocked[edge] = 1e9  # pode3은 남쪽 통로로 돌아간다
    going = world.call("OUTBOUND", "pode3")
    assert world.run_until(lambda: traffic._robots["pode3"].route is not None, 5.0)
    route = traffic._robots["pode3"].route
    assert route.goal == "O" and route.granted < len(route.edges), "the dock zone comes later"
    late = world.call("OUTBOUND", "pode2")
    traffic._robots["pode2"].asked_at = -1.0  # 먼저 불렸다가 앞지름을 당한 로봇인 셈이다
    while world.result_of(going) is None:
        world.step()
        assert world.now < 200.0, world.log[-12:]
    assert world.result_of(going).success and world.result_of(late) is None
