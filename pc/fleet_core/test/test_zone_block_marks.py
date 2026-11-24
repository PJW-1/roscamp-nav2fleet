"""막힘 표시를 위치 단위로: 장애물 위치가 든 연결만, 로봇이면 비키기."""
import math
from dataclasses import replace

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.traffic.zone.base import HERE
from fleet_core.traffic.zone.block_marks import keys_at
from zone_harness import Harness


class Seeing(Harness):
    """막혀 선 로봇이 앞 obstacle_look_m 안의 가장 가까운 장애물 점을 함께 보고한다(4-1)."""

    def report(self) -> None:
        for bot in self.bots.values():
            if bot.name in self.silent:
                continue
            state = self._state(bot)
            seen = self.seen(bot) if state == "BLOCKED" else None
            self.traffic.on_robot_state(bot.name, bot.x, bot.y, bot.yaw, state,
                                        obstacle=seen and (*seen, "unknown"))

    def seen(self, bot):
        best = None
        for ox, oy, radius in self.obstacles:
            gap = math.hypot(ox - bot.x, oy - bot.y)
            point = (ox - (ox - bot.x) / gap * radius, oy - (oy - bot.y) / gap * radius)
            if gap - radius <= self.params.obstacle_look_m and (best is None or gap < best[0]):
                best = (gap, point)
        return best and best[1]


def from_work_to_outbound(world_type=Seeing, **kw):
    world = world_type(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"}, **kw)
    world.report()
    return world, world.call("OUTBOUND", "pode1")


def marked(world) -> set:
    now = world.now
    return {key for key, until in world.traffic._blocked.items() if until > now}


# ---- 어느 연결을 막나 ----

def test_a_point_on_the_express_marks_both_ways_of_the_express_only():
    site, p = load_fleet_map("d2"), load_params()
    assert set(keys_at(site, p, (1.0, 0.89))) == {("X_W", "X_E"), ("X_E", "X_W")}


def test_a_point_in_the_west_junction_marks_every_line_through_it():
    site, p = load_fleet_map("d2"), load_params()
    keys = set(keys_at(site, p, site.node_pose("X_W")))
    assert {("N_W", "X_W"), ("X_W", "W"), ("W", "N_W"), ("X_W", "N_W"),
            ("X_W", "X_E"), ("X_E", "X_W")} <= keys
    assert not any("O" in key or "N_E" in key for key in keys)


def test_a_point_in_the_yard_marks_only_nav_goals_whose_straight_line_passes_it():
    site, p = load_fleet_map("d2"), load_params()
    keys = set(keys_at(site, p, (0.99, 0.65)))
    assert ("N_W", "N_E") in keys and ("N_E", "N_W") in keys
    assert ("P-1", "W_Q") not in keys and ("P-3", "O_Q") not in keys
    assert not any(a in ("X_W", "X_E") for a, _ in keys), "the fixed lines are not touched"


def test_a_point_in_a_corridor_marks_every_nav_goal_through_that_corridor_and_no_other():
    site, p = load_fleet_map("d"), load_params()
    keys = set(keys_at(site, p, (0.99, 0.60)))
    through = {(edge.a, edge.b) for edge in site.edges() if "CORR_M" in edge.zones}
    assert keys == through
    assert not any("CORR_S" in site.edge(*key).zones for key in keys)


# ---- 표시 ----

def test_blocked_in_the_express_with_a_position_marks_the_express_and_backs_out():
    world, call = from_work_to_outbound()
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode1"), 120.0)
    assert marked(world) == {("X_W", "X_E"), ("X_E", "X_W")}
    detail = next(e.detail for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert "at (0.85, 0.89) in EXPRESS" in detail and "tried again in 60 s" in detail
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).success
    site = world.traffic.site
    back = [point[:2] for point in world.retreats("pode1")[1].path]
    assert back[-1] == tuple(site.node_pose("N_W")), "backs out down to the yard"
    assert tuple(site.node_pose("X_W")) in back, "through the junction"
    assert not world.problems


@pytest.mark.parametrize("express_shut, way", [
    # W -> W_X -> 급행은 이어진 고정 경로라 한 번에 보낸다(급행 끝 대기 자리 O_X에서 서지 않는다)
    (False, ["O"]),
    (True, ["M_E", "O_IN", "O"]),
])
def test_v7_a_robot_at_work_leaves_through_the_hall_while_the_express_is_marked(express_shut,
                                                                               way):
    """v7: 급행이 막혔다고 표시돼 있으면 W의 로봇은 M_E로 풀려 나가 O_IN에서 출고로 들어가고, 비어 있으면 급행이다."""
    world = Harness(layout="v7", starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"})
    if express_shut:
        world.traffic._blocked[("W_X", "O_X")] = 1e9
    world.report()
    call = world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: world.result_of(call), 300.0), world.log[-10:]
    assert world.result_of(call).success
    if world.params.use_order_model and not express_shut:
        # order는 대기 자리 O_X에서 조각을 끊고 도크까지 update로 늘린다. 로봇은 서지 않고 지난다
        way = ["O_X", "O"]
        assert "BASE_END_WAIT" not in world.codes("pode1")
    assert [c.target_id for c in world.legs("pode1") if c.mode != "retreat"] == way
    assert not world.problems


def test_without_a_position_the_driven_edge_is_marked_as_before():
    world, call = from_work_to_outbound(world_type=Harness)
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode1"), 120.0)
    assert marked(world) == {("X_W", "X_E"), ("X_E", "X_W")}
    detail = next(e.detail for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert detail == "X_W->X_E in EXPRESS, tried again in 60 s"
    assert not world.traffic._marks


def test_with_positions_switched_off_the_old_rule_holds():
    params = replace(load_params(), blocked_by_position=False)
    world, _ = from_work_to_outbound(params=params)
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode1"), 120.0)
    detail = next(e.detail for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert detail == "X_W->X_E in EXPRESS, tried again in 60 s"
    assert not world.traffic._marks


def test_blocked_in_the_yard_marks_the_way_from_where_it_stands_too():
    world = Seeing(starts={"pode1": "P-3"})
    world.step()
    call = world.call("WORK")
    world.run(1.0)
    world.obstacles.append((1.1, 0.42, 0.12))
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode1"), 200.0)
    assert (HERE + "pode1", "N_W") in marked(world)
    assert ("P-3", "N_W") in marked(world) or ("P-1", "P-3") not in marked(world)
    assert world.run_until(lambda: world.result_of(call), 200.0)
    assert world.result_of(call).success and not world.problems


# ---- 막은 것이 로봇이면 ----

def test_a_robot_standing_in_the_way_is_sent_off_and_nothing_is_marked():
    world = Seeing(starts={"pode1": "P-1", "pode2": "P-2"})
    world.step()
    call = world.call("OUTBOUND", "pode1")
    # 길 곁에 선 pode2의 몸. 이 세계는 로봇 중심으로 재므로 몸보다 크게 둔다
    world.obstacles.append((*world.site.node_pose("P-2"), 0.15))
    for _ in range(300):
        world.step()
        if "HALL_YIELD" in world.codes():
            break
    world.obstacles.clear()
    assert "HALL_YIELD" in world.codes()
    assert "ZONE_MARKED_BLOCKED" not in world.codes()
    assert not world.traffic._marks
    assert world.run_until(lambda: world.result_of(call), 200.0)


def test_a_robot_parked_in_its_bay_in_the_way_is_not_waited_for():
    """v7: 칸에 주차한 로봇은 비킬 곳이 없어, 그 로봇에 막히면 blocked_mark_s 뒤 바로 그 이음을 표시하고 다른 길을 고른다."""
    world = Seeing(layout="v7", starts={"pode1": "E-1", "pode2": "P-2", "pode3": "P-3"})
    world.step()
    world.call("WORK", "pode1")
    # 길에 닿도록 몸보다 크게 둔 pode2(이 세계는 로봇 중심으로 잰다)
    world.obstacles.append((*world.site.node_pose("P-2"), 0.19))
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pode1"), 30.0)
    world.run(world.params.blocked_mark_s + 0.5)
    codes = world.codes("pode1")
    assert "HALL_YIELD" not in world.codes(), [e.detail for e in world.events
                                               if e.code == "HALL_YIELD"]
    assert "ZONE_MARKED_BLOCKED" in codes and "REROUTE" in codes
    assert world.view("pode2")["node"] == "P-2", "the parked robot stays"


# ---- FleetManager가 받는 칸(계약 0.12-draft) ----

@pytest.mark.parametrize("seen, kept", [
    ((1.0, 0.89, "unknown"), (1.0, 0.89, "unknown")),
    ((1.0, 0.89, "box"), (1.0, 0.89, "unknown")),  # 모르는 종류
    ((math.nan, math.nan, ""), None),  # 모른다
    ((50.0, 0.89, "unknown"), None),  # 지도 밖
    (("x", 0.89, "unknown"), None),  # 숫자가 아니다
])
def test_the_manager_passes_a_usable_obstacle_and_drops_the_rest(seen, kept):
    from fleet_core.manager import FleetManager
    lines = []
    core = FleetManager(load_fleet_map("d2"), load_params(), ["pode1"], lambda: 0.0)
    core.record_to(lines.append)
    x, y = core.site.node_pose("P-1")
    core.on_robot_state("pode1", x, y, 0.0, "BLOCKED", 90.0, "", *seen)
    assert core.traffic._robots["pode1"].report.obstacle == kept
    assert "INPUT_REJECTED" not in [e.code for e in core.tick() if hasattr(e, "code")]


def test_a_report_without_an_obstacle_is_recorded_as_before():
    from fleet_core.manager import FleetManager
    lines = []
    core = FleetManager(load_fleet_map("d2"), load_params(), ["pode1"], lambda: 0.0)
    core.record_to(lines.append)
    core.on_robot_state("pode1", 0.13, 0.15, 0.0, "IDLE", 90.0)
    core.on_robot_state("pode1", 0.13, 0.15, 0.0, "BLOCKED", 90.0, "", 1.0, 0.89, "unknown")
    assert '"args": ["pode1", 0.13, 0.15, 0.0, "IDLE", 90.0]' in lines[1]
    assert '"args": ["pode1", 0.13, 0.15, 0.0, "BLOCKED", 90.0, "", 1.0, 0.89, "unknown"]' in (
        lines[2])


@pytest.mark.parametrize("gap", [0.10, 0.25])
def test_the_robot_check_uses_near_m(gap):
    world = Seeing(starts={"pode1": "P-3", "pode2": "P-2"})
    world.report()
    m = world.traffic._robots["pode1"]
    other = world.traffic._robots["pode2"]
    assert world.traffic._robot_at(m, (other.x + gap, other.y)) is other
    assert world.traffic._robot_at(m, (other.x + 0.31, other.y)) is None


# ---- 우회 종류(4-4) ----

def detours(world, robot="pode1"):
    return [e.detail.split(":")[0] for e in world.events if e.code == "DETOUR"
            and e.robot == robot]


def test_the_express_blocked_detour_goes_beyond_the_keepout():
    world, call = from_work_to_outbound()
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert detours(world) == ["keepout"]


def test_blocked_inside_a_corridor_it_turns_back_and_does_not_drive_past_the_box():
    world = Seeing(layout="d", starts={"pode1": "O_Q", "pode2": "P-1", "pode3": "P-2"})
    world.step()
    call = world.call("WORK", "pode1")
    world.obstacles.append((0.99, 0.58, 0.06))  # CORR_M 가운데
    assert world.run_until(lambda: world.result_of(call), 300.0)
    assert world.result_of(call).success
    assert world.codes("pode1").count("ZONE_MARKED_BLOCKED") == 1
    marked_keys = {key for mark in world.traffic._marks.values() for key in mark.keys}
    assert ("@pode1", "N_W") in marked_keys and ("@pode1", "N_E") not in marked_keys
    assert detours(world) == ["keepout"], "back to HALL_E and round by the express"
    assert not world.problems


def test_detour_kinds():
    from fleet_core.traffic.zone.detours import detour_kind
    from fleet_core.traffic.zone.routes import Route
    site = load_fleet_map("d")
    via = [site.edge("N_W", "N_E")]
    south = [site.edge("N_W", "W_Q"), site.edge("W_Q", "P-2"), site.edge("P-2", "P-3")]
    old = {"HALL_W", "CORR_M", "HALL_E"}
    assert detour_kind(site, "N_E", old, Route("O_Q", via)) == "destination"
    assert detour_kind(site, "P-3", old, Route("P-3", south)) == "corridor"
    inside = [site.edge("N_W", "W_Q"), site.edge("W_Q", "P-2")]
    assert detour_kind(site, "P-2", {"HALL_W"}, Route("P-2", inside)) == "in_zone"
    express = [site.edge("N_W", "X_W"), site.edge("X_W", "X_E"), site.edge("X_E", "N_E")]
    assert detour_kind(site, "N_E", old, Route("N_E", express)) == "keepout"


# ---- 읽기 전용 관측 API(시뮬용) ----

def _marked_world():
    world, _ = from_work_to_outbound()
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: world.traffic._marks, 120.0)
    return world


def _old_live(world, keys, point, radius, now):
    """예전 block_watch의 사설 접근 판정."""
    traffic = world.traffic
    return (any(traffic._blocked.get(key, 0.0) > now for key in keys)
            or any(math.dist((mk.x, mk.y), point) <= radius for mk in traffic._marks.values()))


def test_live_block_gives_the_same_answer_as_the_old_private_reads():
    world = _marked_world()
    traffic, now = world.traffic, world.now
    key = next(iter(traffic._blocked))
    mark = next(iter(traffic._marks.values()))
    cases = [
        ((), (mark.x, mark.y), 0.1),          # 표시 위치 안
        ((), (mark.x + 5, mark.y), 0.1),      # 표시에서 멀다
        ((key,), (99.0, 99.0), 0.1),          # 막힌 연결 키
        (("no", "such"), (99.0, 99.0), 0.1),  # 모르는 키
        ((key,), (99.0, 99.0), 0.1, traffic._blocked[key] + 1),  # 만료 뒤
    ]
    assert any(_old_live(world, c[0], c[1], c[2], now) for c in cases)
    assert not all(_old_live(world, c[0], c[1], c[2], now) for c in cases)
    for keys, point, radius, *later in cases:
        t = later[0] if later else now
        assert traffic.live_block(keys, point, radius, t) == (
            any(traffic._blocked.get(k, 0.0) > t for k in keys)
            or any(math.dist((mk.x, mk.y), point) <= radius for mk in traffic._marks.values()))


def test_live_block_changes_nothing_and_hands_out_no_internal_object():
    world = _marked_world()
    traffic = world.traffic
    before = dict(traffic._blocked), dict(traffic._marks)
    out = traffic.live_block(tuple(traffic._blocked), (0.0, 0.0), 1.0, world.now)
    assert out is True or out is False
    assert (dict(traffic._blocked), dict(traffic._marks)) == before
