"""급행에서 막힌 로봇은 갈림목 안이 아니라 마당 지점까지 물러난다(추가 지시서 3의 4절, 2-1 지도).

갈림목 지점(X_W, X_E)은 도크에서 0.17 m라서 도크에 로봇이 있으면 갈림목에 설 수 없다
(`retreat_clear_m`, `node_clear_m`). 그래서 급행 → 갈림목 → 마당 지점(N_W, N_E)의 꺾이는 후진으로
물러난다. 도크의 로봇은 몸이 닿을 만큼 가까우므로 그 로봇이 먼저 나간 뒤 물러나고(급행에서
기다리는 로봇은 갈림목을 쥐지 않으니 도크의 로봇이 나갈 수 있다), 물러날 길을 끝내 못 잡으면
blocked_timeout_s 뒤 운영자에게 넘긴다.
"""
from dataclasses import replace

from fleet_core.common.commands import RetreatCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.zone.yard_retreat import yard_retreat
from zone_harness import Harness


def test_a_junction_retreat_goes_down_to_the_yard_node():
    site = Harness().traffic.site
    assert yard_retreat(site, site.link("X_W", "X_E")) == ("N_W", "X_W")
    assert yard_retreat(site, site.link("X_E", "X_W")) == ("N_E", "X_E")
    assert yard_retreat(site, site.link("N_W", "X_W")) == ("N_W", ""), "not from a yard node"
    assert yard_retreat(site, site.link("X_W", "N_W")) == ("X_W", ""), "already going down"


def blocked_toward_work(docked: bool, **kw):
    """pinky1은 N_E에서 급행으로 작업 도크에 가다 막힌다. docked면 pinky2가 출고 도크 O에서
    일한다."""
    starts = {"pinky1": "N_E", "pinky2": "O" if docked else "P-3", "pinky3": "P-1"}
    world = Harness(starts=starts, **kw)
    world.report()
    if docked:
        world.call("OUTBOUND", "pinky2")
    call = world.call("WORK", "pinky1")
    world.obstacles.append((1.0, 0.89, 0.15))
    world.run(3.0)
    assert not docked or world.phase("pinky2") == "AT_STATION"
    return world, call


def back_path(world):
    return [point[:2] for point in world.sent(RetreatCmd, "pinky1")[0].path]


def test_blocked_in_the_express_it_backs_through_the_junction_down_to_the_yard():
    world, call = blocked_toward_work(docked=False)
    site = world.traffic.site
    assert world.run_until(lambda: world.sent(RetreatCmd, "pinky1"), 200.0)
    assert back_path(world)[-1] == tuple(site.node_pose("N_E"))
    assert tuple(site.node_pose("X_E")) in back_path(world)
    assert world.run_until(lambda: world.result_of(call), 300.0)
    assert world.result_of(call).success and not world.problems


def test_a_robot_working_at_the_dock_beside_the_junction_goes_first_then_it_backs_out():
    world, _ = blocked_toward_work(docked=True)
    assert world.run_until(lambda: "RETREAT_WAIT" in world.codes("pinky1"), 200.0)
    assert not world.sent(RetreatCmd, "pinky1")
    assert "J_E" not in world.traffic.grants.held_by("pinky1"), "the dock robot can leave"
    world.call("WORK", "pinky2")  # 도크의 로봇이 일을 마치고 나간다(갈림목에서 꺾어 N_E로)
    assert world.run_until(lambda: world.sent(RetreatCmd, "pinky1"), 200.0)
    assert back_path(world)[-1] == tuple(world.traffic.site.node_pose("N_E"))
    assert not world.problems


def test_a_robot_that_docks_beside_the_junction_after_the_mark_is_also_waited_for():
    """표시한 뒤에 도크에 들어온 로봇이 있으면 물러날 길(갈림목)을 잡지 않고 기다린다. 잡으면 도크의
    로봇이 갈림목을 못 지나 나가지 못하고, 물러나는 로봇은 그 로봇에 닿아 서로 기다린다."""
    world, _ = blocked_toward_work(docked=False)
    grants, site = world.traffic.grants, world.traffic.site
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pinky1"), 200.0)
    assert grants.force("pinky3", zones=("J_E",))  # 표시할 때는 물러날 길을 못 잡는다
    world.silent.add("pinky3")
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky1"), 60.0)
    assert "RETREAT_WAIT" not in world.codes("pinky1"), "nobody was on the way back yet"
    pinky2 = world.bots["pinky2"]
    pinky2.x, pinky2.y = site.node_pose("O")  # 그 사이에 도크에 들어와 섰다
    assert grants.release_zone("pinky3", "J_E")
    world.run(3.0)
    assert not world.sent(RetreatCmd, "pinky1"), "it would back into pinky2 at the corner"
    assert "J_E" not in grants.held_by("pinky1"), "pinky2 can leave the dock"
    waits = [e.detail for e in world.events if e.code == "RETREAT_WAIT" and e.robot == "pinky1"]
    assert waits == ["pinky2 is behind it, backs out after it"]
    pinky2.x, pinky2.y = site.node_pose("P-3")  # 나갔다
    assert world.run_until(lambda: world.sent(RetreatCmd, "pinky1"), 10.0)


def test_a_retreat_that_cannot_get_its_zones_waits_at_most_blocked_timeout_s():
    params = replace(load_params(), blocked_timeout_s=30.0)
    world, _ = blocked_toward_work(docked=False, params=params)
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pinky1"), 200.0)
    # pinky1은 J_E를 이미 놓았다. 말이 끊긴 pinky3이 J_E를 쥐어 물러날 길을 못 잡는다
    assert "J_E" not in world.traffic.grants.held_by("pinky1")
    assert world.traffic.grants.force("pinky3", zones=("J_E",))
    world.silent.add("pinky3")
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky1"), 60.0)
    marked_at = world.now
    assert world.run_until(lambda: "GOAL_FAILED" in world.codes("pinky1"), 200.0)
    assert world.now - marked_at <= params.blocked_timeout_s + 2.0
    assert not world.sent(RetreatCmd, "pinky1")


def test_it_backs_out_to_the_yard_even_with_no_way_on_from_there():
    """물러난 뒤 갈 길이 지금 없어도 물러나기는 한다. 급행이 아니라 마당에서 기다린다."""
    params = replace(load_params(), blocked_timeout_s=30.0)
    world, call = blocked_toward_work(docked=False, params=params)
    t = world.traffic
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pinky1"), 200.0)
    for link in t.site.links():  # 다른 길도 모두 막혔다
        if {link.a, link.b} != {"X_E", "X_W"}:
            t._blocked[(link.a, link.b)] = 1e9
    assert world.run_until(lambda: world.sent(RetreatCmd, "pinky1"), 60.0)
    assert back_path(world)[-1] == tuple(t.site.node_pose("N_E"))
    assert world.run_until(lambda: "RETREAT_DONE" in world.codes("pinky1"), 60.0)
    world.run(params.blocked_timeout_s + 5.0)
    assert "GOAL_FAILED" not in world.codes("pinky1"), "it waits in the yard"
    t._blocked.clear()  # 길이 다시 열렸다
    world.obstacles.clear()
    assert world.run_until(lambda: world.result_of(call), 300.0)
    assert world.result_of(call).success


def test_the_retreat_follows_the_bends_of_the_way_down():
    """물러나는 길은 갈림 -> 마당 지점 고정 경로의 꺾는 점을 따른다(cededa0 리뷰). 시험 지도는 N_E가
    있던 옛 v7(test/data/fleet_map_line3.yaml)에 예전 W_X -> N_E 길(work_west)을 되살린 것이다.
    W_X -> N_E를
    곧게 가면 WORK_DOCK 모서리 (1.78, 0.63)에 0.058까지 붙지만 그 구역은 잡지 않는다. 꺾는 점
    (1.70, 0.70)을 따르면 몸 반경(0.085) 넘게 떨어진다."""
    from pathlib import Path

    from fleet_core.fleet_map import fleet_map_from_text
    from fleet_core.fleet_map.geometry import point_segment_distance
    text = (Path(__file__).parent / "data" / "fleet_map_line3.yaml").read_text(encoding="utf-8")
    old = "  work_out: {points: [W, W_X]}"
    assert old in text
    text = text.replace(old, "  work_west: {points: [W_X, [1.70, 0.70], N_E]}\n" + old)
    world = Harness(site=fleet_map_from_text(text),
                    starts={"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"})
    m = world.traffic._robots["pinky1"]
    m.x, m.y = 1.80, 0.88  # 급행 들머리에서 막혔다
    m.retreat_to, m.retreat_via = "N_E", "W_X"
    back = world.traffic._back_link(m)
    assert back.points[1:] == ((1.88, 0.80), (1.70, 0.70), world.site.node_pose("N_E"))
    gap = min(point_segment_distance((1.78, 0.63), a, b)
              for a, b in zip(back.points, back.points[1:]))
    assert gap > world.params.footprint_radius_m, gap
    assert "WORK_DOCK" not in back.zones
