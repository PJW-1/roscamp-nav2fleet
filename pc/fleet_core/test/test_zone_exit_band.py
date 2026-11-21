"""출고 도크를 나오는 고정 경로(out_out) 위에서 호출이 와 route가 바뀐 로봇(2026-11-21 Gazebo).
- HALL은 y 0.765까지라 out_out 끝 무렵의 로봇은 넓은 구역 안이고, O에서 0.15 안이면 노드 위로 친다.
- 하지만 Nav2 keepout은 OUT_DOCK을 0.06 넓혀 y 0.705까지 막는다: 거기서 goal을 끊고 nav_goal을
  새로 주면 Nav2가 길을 찾지 못한다. 고정 경로는 로봇이 가까이 선 노드도 넓은 구역일 때만 끊는다.
"""
from dataclasses import replace

import pytest
from fleet_core.common.commands import CancelCmd, MoveCmd, MoveOrderCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map.geometry import inside
from fleet_core.fleet_map.keepout import keepout_rects
from zone_harness import Harness

BAND_POSE = (0.181, 0.762)  # 기록의 자리: HALL 안, O에서 0.148, keepout 안


def in_keepout(site, x, y) -> bool:
    return any(inside(rect, x, y) for rect in keepout_rects(site))


def called_in_the_band(order_model):
    """pode1이 출고를 끝내고 주차하러 out_out을 가다 띠 안 BAND_POSE에 섰을 때 호출이 온다.
    그 한 걸음만 로봇을 세워 관제가 그 자리를 본다."""
    world = Harness(layout="v7", starts={"pode1": "O"},
                    params=replace(load_params(), use_order_model=order_model))
    world.report()
    out = world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: world.result_of(out), 30.0)
    world.traffic.on_release("pode1")
    bot = world.bots["pode1"]
    assert world.run_until(lambda: bot.y < 0.80, 30.0), world.log[-20:]
    m = world.traffic._robots["pode1"]
    assert m.route.edges[m.route.done].b == "O_OUT"
    bot.x, bot.y = BAND_POSE
    since = len(world.commands)
    world.call("WORK")
    speed, world.speed = world.speed, 0.0
    world.step()
    world.speed = speed
    assert world.traffic._in_yard(m) and world.traffic._on_node(m), "the core sees a yard node"
    assert in_keepout(world.site, *BAND_POSE), "Nav2 sees the keepout"
    return world, since


@pytest.mark.parametrize("order_model", [False, True])
def test_a_call_inside_the_dock_keepout_band_does_not_cut_the_exit_path(order_model):
    """띠 안에서 호출이 와도 out_out을 끊지 않고, 새 nav_goal은 keepout 밖에서 시작한다."""
    world, since = called_in_the_band(order_model)
    bot, end = world.bots["pode1"], world.now + 120.0
    while world.view("pode1")["node"] != "W":
        for c in world.commands[since:]:
            starts_nav = (isinstance(c, MoveCmd) and c.mode == "nav_goal") or (
                isinstance(c, MoveOrderCmd) and c.update_id == 0 and c.base[0].mode == "nav_goal")
            if c.robot == "pode1" and (starts_nav or isinstance(c, CancelCmd)):
                assert not in_keepout(world.site, bot.x, bot.y), (c, bot.x, bot.y, world.log[-20:])
        since = len(world.commands)
        assert world.now < end, world.log[-30:]
        world.step()
    assert not world.problems
