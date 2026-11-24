"""넓은 구역에서 비켜 서는 로봇이 BLOCKED를 보고했어도 그 자리에 서기만 한다."""
import math

from fleet_core.common.commands import RetreatCmd
from zone_harness import Harness


def stuck_pair():
    """P-1의 pode1과 P-2의 pode2가 0.25 m 떨어져 마당에서 Nav2 goal을 받고 서 있다. 배차가 없는 pode2가 비킬 쪽이다."""
    world = Harness(speed=0.0)
    world.step()
    world.call("WORK", "pode1")
    world.open_call = world.call("WORK")  # 열린 호출: pode1은 이미 배차가 있어 pode2가 불린다
    world.step()
    world.step()
    assert world.moves("pode1") and world.moves("pode2")
    world.bots["pode2"].yaw = math.pi
    return world


def reports(world, robot, state):
    """robot은 앞으로 state를 보고한다. 다른 로봇은 그대로다."""
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == robot else plain(bot)


def yields(world):
    return [(e.robot, e.detail) for e in world.events if e.code == "HALL_YIELD"]


def test_with_the_switch_off_a_blocked_robot_only_stops():
    world = stuck_pair()
    reports(world, "pode2", "BLOCKED")
    world.run(world.params.near_stuck_s)
    assert yields(world) == [("pode2", "stops 5 s for pode1")]
    assert not world.sent(RetreatCmd)
