"""급행에서 막힌 로봇은 갈림목이 아니라 마당 지점(N_W, N_E)까지 물러난다(D2 지도). 도크에 로봇이 있으면 그 로봇이 먼저 나간 뒤 물러나고, 길을 못 잡으면 blocked_timeout_s 뒤 운영자에게 넘긴다."""
import math
from dataclasses import replace

import pytest
from fleet_core.common.commands import MoveOrderCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.zone.yard_retreat import yard_retreat
from zone_harness import Harness


def test_a_junction_retreat_goes_down_to_the_yard_node():
    site = Harness().traffic.site
    assert yard_retreat(site, site.edge("X_W", "X_E")) == ("N_W", "X_W")
    assert yard_retreat(site, site.edge("X_E", "X_W")) == ("N_E", "X_E")
    assert yard_retreat(site, site.edge("N_W", "X_W")) == ("N_W", ""), "not from a yard node"
    assert yard_retreat(site, site.edge("X_W", "N_W")) == ("X_W", ""), "already going down"


def blocked_toward_work(docked: bool, **kw):
    """pode1은 N_E에서 급행으로 작업 도크에 가다 막힌다. docked면 pode2가 출고 도크 O에서 일한다."""
    starts = {"pode1": "N_E", "pode2": "O" if docked else "P-3", "pode3": "P-1"}
    world = Harness(starts=starts, **kw)
    world.report()
    if docked:
        world.call("OUTBOUND", "pode2")
    call = world.call("WORK", "pode1")
    world.obstacles.append((1.0, 0.89, 0.15))
    world.run(3.0)
    assert not docked or world.phase("pode2") == "AT_STATION"
    return world, call


def back_path(world):
    return [point[:2] for point in world.retreats("pode1")[0].path]


def test_blocked_in_the_express_it_backs_through_the_junction_down_to_the_yard():
    world, call = blocked_toward_work(docked=False)
    site = world.traffic.site
    assert world.run_until(lambda: world.retreats("pode1"), 200.0)
    assert back_path(world)[-1] == tuple(site.node_pose("N_E"))
    assert tuple(site.node_pose("X_E")) in back_path(world)
    assert world.run_until(lambda: world.result_of(call), 300.0)
    assert world.result_of(call).success and not world.problems


def test_a_robot_working_at_the_dock_beside_the_junction_goes_first_then_it_backs_out():
    world, _ = blocked_toward_work(docked=True)
    assert world.run_until(lambda: "RETREAT_WAIT" in world.codes("pode1"), 200.0)
    assert not world.retreats("pode1")
    assert "J_E" not in world.traffic.grants.held_by("pode1"), "the dock robot can leave"
    world.call("WORK", "pode2")  # 도크의 로봇이 일을 마치고 나간다(갈림목에서 꺾어 N_E로)
    assert world.run_until(lambda: world.retreats("pode1"), 200.0)
    assert back_path(world)[-1] == tuple(world.traffic.site.node_pose("N_E"))
    assert not world.problems


def test_a_robot_that_docks_beside_the_junction_after_the_mark_is_also_waited_for():
    """표시한 뒤에 도크에 들어온 로봇이 있으면 물러날 길(갈림목)을 잡지 않고 기다린다."""
    world, _ = blocked_toward_work(docked=False)
    grants, site = world.traffic.grants, world.traffic.site
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pode1"), 200.0)
    assert grants.force("pode3", zones=("J_E",))  # 표시할 때는 물러날 길을 못 잡는다
    world.silent.add("pode3")
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode1"), 60.0)
    assert "RETREAT_WAIT" not in world.codes("pode1"), "nobody was on the way back yet"
    pode2 = world.bots["pode2"]
    pode2.x, pode2.y = site.node_pose("O")  # 그 사이에 도크에 들어와 섰다
    assert grants.release_zone("pode3", "J_E")
    world.run(3.0)
    assert not world.retreats("pode1"), "it would back into pode2 at the corner"
    assert "J_E" not in grants.held_by("pode1"), "pode2 can leave the dock"
    waits = [e.detail for e in world.events if e.code == "RETREAT_WAIT" and e.robot == "pode1"]
    assert waits == ["pode2 is behind it, backs out after it"]
    pode2.x, pode2.y = site.node_pose("P-3")  # 나갔다
    assert world.run_until(lambda: world.retreats("pode1"), 10.0)


def test_a_retreat_that_cannot_get_its_zones_waits_at_most_blocked_timeout_s():
    params = replace(load_params(), blocked_timeout_s=30.0)
    world, _ = blocked_toward_work(docked=False, params=params)
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pode1"), 200.0)
    # pode1은 J_E를 이미 놓았다. 말이 끊긴 pode3이 J_E를 쥐어 물러날 길을 못 잡는다
    assert "J_E" not in world.traffic.grants.held_by("pode1")
    assert world.traffic.grants.force("pode3", zones=("J_E",))
    world.silent.add("pode3")
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode1"), 60.0)
    marked_at = world.now
    assert world.run_until(lambda: "GOAL_FAILED" in world.codes("pode1"), 200.0)
    assert world.now - marked_at <= params.blocked_timeout_s + 2.0
    assert not world.retreats("pode1")


def test_it_backs_out_to_the_yard_even_with_no_way_on_from_there():
    """물러난 뒤 갈 길이 지금 없어도 물러나기는 한다. 급행이 아니라 마당에서 기다린다."""
    params = replace(load_params(), blocked_timeout_s=30.0)
    world, call = blocked_toward_work(docked=False, params=params)
    t = world.traffic
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pode1"), 200.0)
    for edge in t.site.edges():  # 다른 길도 모두 막혔다
        if {edge.a, edge.b} != {"X_E", "X_W"}:
            t._blocked[(edge.a, edge.b)] = 1e9
    assert world.run_until(lambda: world.retreats("pode1"), 60.0)
    assert back_path(world)[-1] == tuple(t.site.node_pose("N_E"))
    assert world.run_until(lambda: "RETREAT_DONE" in world.codes("pode1"), 60.0)
    world.run(params.blocked_timeout_s + 5.0)
    assert "GOAL_FAILED" not in world.codes("pode1"), "it waits in the yard"
    t._blocked.clear()  # 길이 다시 열렸다
    world.obstacles.clear()
    assert world.run_until(lambda: world.result_of(call), 300.0)
    assert world.result_of(call).success


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_the_junction_is_kept_until_the_robot_says_it_has_backed_down(order):
    """order면 마당 지점까지의 물러나기 뒤 다음 조각이 같은 goal에 붙고, 물러나기는 끝냈다는 답 전에는 보고 위치로 재어 J_E를 일찍 놓지 않는다."""
    params = replace(load_params(), use_order_model=order)
    world, call = blocked_toward_work(docked=False, params=params)
    assert world.run_until(lambda: world.retreats("pode1"), 200.0)
    if order:
        base = world.sent(MoveOrderCmd, "pode1")[-1].base
        assert base[0].mode == "retreat" and len(base) > 1, "the retreat is not the last edge"
    bot, n_e = world.bots["pode1"], world.site.node_pose("N_E")
    assert world.run_until(lambda: math.dist((bot.x, bot.y), n_e) < 0.028, 60.0)
    world._move = lambda b: None  # N_E 앞에서 멈추고 답하지 않는다
    world.run(1.0)
    need = params.footprint_radius_m + params.release_margin_m
    assert world.site.distance_to_zone(bot.x, bot.y, "J_E") < need
    assert "J_E" in world.view("pode1")["held"], world.view("pode1")["held"]
    assert "RETREAT_DONE" not in world.codes("pode1")
    del world._move
    assert world.run_until(lambda: world.result_of(call), 300.0)
    assert world.result_of(call).success and not world.problems


def test_the_retreat_follows_the_bends_of_the_way_down():
    """물러나는 길은 갈림 -> 마당 지점 고정 경로의 꺾는 점(1.70, 0.70)을 따라 몸 반경(0.085) 넘게 떨어진다. 시험 지도는 옛 v7(test/data/fleet_map_line3.yaml)에 W_X -> N_E 길(work_west)을 되살린 것이다."""
    from pathlib import Path

    from fleet_core.fleet_map import fleet_map_from_text
    from fleet_core.fleet_map.geometry import point_segment_distance
    text = (Path(__file__).parent / "data" / "fleet_map_line3.yaml").read_text(encoding="utf-8")
    old = "  work_out: {points: [W, W_X]}"
    assert old in text
    text = text.replace(old, "  work_west: {points: [W_X, [1.70, 0.70], N_E]}\n" + old)
    world = Harness(site=fleet_map_from_text(text),
                    starts={"pode1": "P-1", "pode2": "P-2", "pode3": "P-3"})
    m = world.traffic._robots["pode1"]
    m.x, m.y = 1.80, 0.88  # 급행 들머리에서 막혔다
    m.avoid.retreat_to, m.avoid.retreat_via = "N_E", "W_X"
    back = world.traffic._back_edge(m)
    assert back.points[1:] == ((1.88, 0.80), (1.70, 0.70), world.site.node_pose("N_E"))
    gap = min(point_segment_distance((1.78, 0.63), a, b)
              for a, b in zip(back.points, back.points[1:], strict=False))
    assert gap > world.params.footprint_radius_m, gap
    assert "WORK_DOCK" not in back.zones
