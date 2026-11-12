"""막힘 표시를 위치 단위로(추가 지시서 3의 4-1, 4-2): 장애물 위치가 든 연결만, 로봇이면 비키기."""
import math
from dataclasses import replace

import pytest
from fleet_core.common.commands import RetreatCmd
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
    world = world_type(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"}, **kw)
    world.report()
    return world, world.call("OUTBOUND", "pinky1")


def marked(world) -> set:
    now = world.now
    return {key for key, until in world.traffic._blocked.items() if until > now}


# ---- 어느 연결을 막나 ----

def test_a_point_on_the_express_marks_both_ways_of_the_express_only():
    site, p = load_fleet_map("d2"), load_params()
    assert set(keys_at(site, p, (1.0, 0.89))) == {("X_W", "X_E"), ("X_E", "X_W")}


def test_a_point_in_the_west_junction_marks_every_line_through_it():
    site, p = load_fleet_map("d2"), load_params()
    keys = set(keys_at(site, p, (0.40, 0.89)))
    assert {("N_W", "X_W"), ("X_W", "W"), ("W", "X_W"), ("X_W", "N_W"),
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
    through = {(link.a, link.b) for link in site.links() if "CORR_M" in link.zones}
    assert keys == through
    assert not any("CORR_S" in site.link(*key).zones for key in keys)


# ---- 표시 ----

def test_blocked_in_the_express_with_a_position_marks_the_express_and_backs_out():
    world, call = from_work_to_outbound()
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky1"), 120.0)
    assert marked(world) == {("X_W", "X_E"), ("X_E", "X_W")}
    detail = next(e.detail for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert "at (0.85, 0.89) in EXPRESS" in detail and "tried again in 60 s" in detail
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).success
    assert world.sent(RetreatCmd, "pinky1")[1].path[-1][:2] == (0.40, 0.89)
    assert not world.problems


def test_without_a_position_the_driven_link_is_marked_as_before():
    world, call = from_work_to_outbound(world_type=Harness)
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky1"), 120.0)
    assert marked(world) == {("X_W", "X_E"), ("X_E", "X_W")}
    detail = next(e.detail for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert detail == "X_W->X_E in EXPRESS, tried again in 60 s"
    assert not world.traffic._marks


def test_with_positions_switched_off_the_old_rule_holds():
    params = replace(load_params(), blocked_by_position=False)
    world, _ = from_work_to_outbound(params=params)
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky1"), 120.0)
    detail = next(e.detail for e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert detail == "X_W->X_E in EXPRESS, tried again in 60 s"
    assert not world.traffic._marks


def test_blocked_in_the_yard_marks_the_way_from_where_it_stands_too():
    world = Seeing(starts={"pinky1": "P-3"})
    world.step()
    call = world.call("WORK")
    world.run(1.0)
    world.obstacles.append((1.1, 0.42, 0.12))
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky1"), 200.0)
    assert (HERE + "pinky1", "N_W") in marked(world)
    assert ("P-3", "N_W") in marked(world) or ("P-1", "P-3") not in marked(world)
    assert world.run_until(lambda: world.result_of(call), 200.0)
    assert world.result_of(call).success and not world.problems


# ---- 막은 것이 로봇이면 ----

def test_a_robot_standing_in_the_way_is_sent_off_and_nothing_is_marked():
    world = Seeing(starts={"pinky1": "P-1", "pinky2": "P-2"})
    world.step()
    call = world.call("OUTBOUND", "pinky1")
    # 길 곁에 선 pinky2의 몸. 이 세계는 로봇 중심으로 재므로 몸보다 크게 둔다
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
    core = FleetManager(load_fleet_map("d2"), load_params(), ["pinky1"], lambda: 0.0)
    core.record_to(lines.append)
    x, y = core.site.node_pose("P-1")
    core.on_robot_state("pinky1", x, y, 0.0, "BLOCKED", 90.0, "", *seen)
    assert core.traffic._robots["pinky1"].obstacle == kept
    assert "INPUT_REJECTED" not in [e.code for e in core.tick() if hasattr(e, "code")]


def test_a_report_without_an_obstacle_is_recorded_as_before():
    from fleet_core.manager import FleetManager
    lines = []
    core = FleetManager(load_fleet_map("d2"), load_params(), ["pinky1"], lambda: 0.0)
    core.record_to(lines.append)
    core.on_robot_state("pinky1", 0.13, 0.15, 0.0, "IDLE", 90.0)
    core.on_robot_state("pinky1", 0.13, 0.15, 0.0, "BLOCKED", 90.0, "", 1.0, 0.89, "unknown")
    assert '"args": ["pinky1", 0.13, 0.15, 0.0, "IDLE", 90.0]' in lines[1]
    assert '"args": ["pinky1", 0.13, 0.15, 0.0, "BLOCKED", 90.0, "", 1.0, 0.89, "unknown"]' in (
        lines[2])


@pytest.mark.parametrize("gap", [0.10, 0.25])
def test_the_robot_check_uses_near_m(gap):
    world = Seeing(starts={"pinky1": "P-3", "pinky2": "P-2"})
    world.report()
    m = world.traffic._robots["pinky1"]
    other = world.traffic._robots["pinky2"]
    assert world.traffic._robot_at(m, (other.x + gap, other.y)) is other
    assert world.traffic._robot_at(m, (other.x + 0.31, other.y)) is None
