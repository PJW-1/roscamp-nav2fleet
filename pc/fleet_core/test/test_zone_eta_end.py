"""Nav2 goal 조각의 남은 시간에 끝에서 생기는 두 가지를 더한다(2026-11-21 Gazebo 0.12 m/s 판).

- 다음 조각(Nav2 goal이나 고정 경로)이 있으면 로봇은 끝 노드 nav_pass_m 앞에서 넘어간다
  (pode_fsm pass_m). 그만큼 덜 달린다.
- 다음 조각이 후진이거나 Route가 끝나면 로봇은 끝에 서서 끝 방향으로 제자리에서 돈다
  (nav_turn_rps). 아직 오는 중이면 다가오는 방향에서, 끝에 닿았으면 지금 방향에서 잰다.
- 통신이 끊긴 로봇도 이미 받은 base 안에서는 넘어가며 달리고, base 끝에서만 선다(STOP).
- 차선 순서는 입구에 닿는 시각을 견주므로 끝 보정 없이 잰다(ARRIVE).
  로봇은 넘어가도 입구까지는 간다.
- 끝 보정이 0이면 예전 식 그대로 계산한다(옛 기록 재생과 같은 값).
- 옛 기록의 파라미터에는 두 키가 없다. 기본값 0은 꺼짐이라 옛 기록은 예전 판단 그대로 재생된다.
"""
import math
from dataclasses import fields, replace
from itertools import pairwise
from types import SimpleNamespace

import pytest
from fleet_core.common.params import FleetParams, load_params
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, Edge, load_fleet_map
from fleet_core.fleet_map.graph import shortest_path
from fleet_core.traffic.zone.controller import ZoneTraffic
from fleet_core.traffic.zone.routes import (
    ARRIVE,
    STOP,
    Route,
    heading,
    left_s,
    nav_end_s,
    speed,
    travel_s,
)
from zone_harness import Harness

P = load_params()
SITE = load_fleet_map("v7")
E1, E2 = SITE.nodes["E-1"], SITE.nodes["E-2"]


def edge(a, b, mode):
    pa, pb = SITE.nodes[a], SITE.nodes[b]
    points = ((pa.x, pa.y), (pb.x, pb.y))
    return Edge(a, b, mode, "", points, (), math.dist(*points))


TO_E1 = edge("E-2", "E-1", NAV_GOAL)
BACK_P1 = edge("E-1", "P-1", RETREAT)


# ---- 넘어가는 거리 ----

@pytest.mark.parametrize("after", [NAV_GOAL, FOLLOW_PATH])
def test_a_following_piece_takes_nav_pass_m_off(after):
    edges = [TO_E1, edge("E-1", "P-1", after)]
    got = nav_end_s(P, SITE, edges, 1, E2.x, E2.y, math.pi)
    assert got == pytest.approx(-P.nav_pass_m / speed(P, NAV_GOAL))


def test_without_passing_nothing_is_taken_off():
    edges = [TO_E1, edge("E-1", "P-1", FOLLOW_PATH)]
    assert nav_end_s(P, SITE, edges, 1, E2.x, E2.y, math.pi, STOP) >= 0.0
    assert nav_end_s(P, SITE, edges, 1, E2.x, E2.y, math.pi, ARRIVE) == 0.0
    assert nav_end_s(P, SITE, [TO_E1, BACK_P1], 1, E2.x, E2.y, math.pi, ARRIVE) == 0.0


# ---- 끝에서 도는 시간 ----

def test_before_a_retreat_the_robot_turns_from_its_approach_to_the_end_heading():
    edges = [TO_E1, BACK_P1]
    approach = math.atan2(E1.y - E2.y, E1.x - E2.x)
    turn = abs(math.remainder(E1.yaw - approach, math.tau))
    assert turn > 1.0, "E-2 to E-1 runs west, E-1 faces north"
    got = nav_end_s(P, SITE, edges, 1, (E1.x + E2.x) / 2, E1.y, approach)
    assert got == pytest.approx(turn / P.nav_turn_rps)


def test_at_the_end_only_the_turn_still_left_counts():
    edges = [TO_E1, BACK_P1]
    near = P.nav_goal_tolerance_m * 0.5
    got = nav_end_s(P, SITE, edges, 1, E1.x + near, E1.y, E1.yaw - 0.4)
    assert got == pytest.approx(0.4 / P.nav_turn_rps)


def test_the_end_of_the_route_turns_only_to_a_fixed_end_heading():
    free = Edge("", "P-1", NAV_GOAL, "", ((0.40, 0.10), (0.12, 0.10)), (), 0.28)
    assert nav_end_s(P, SITE, [free], 1, 0.40, 0.10, math.pi) == pytest.approx(
        abs(math.remainder(SITE.nodes["P-1"].yaw - math.pi, math.tau)) / P.nav_turn_rps)
    loose = replace(SITE.nodes["P-1"], yaw=None)
    site = SimpleNamespace(nodes={**SITE.nodes, "P-1": loose})
    assert nav_end_s(P, site, [free], 1, 0.40, 0.10, math.pi) == pytest.approx(0.0)


# ---- 끄기와 옛 기록 ----

def test_zero_turns_both_parts_off():
    off = replace(P, nav_pass_m=0.0, nav_turn_rps=0.0)
    assert nav_end_s(off, SITE, [TO_E1, edge("E-1", "P-1", FOLLOW_PATH)], 1, E2.x, E2.y,
                     math.pi) == 0.0
    assert nav_end_s(off, SITE, [TO_E1, BACK_P1], 1, E2.x, E2.y, math.pi) == 0.0


def test_old_records_without_the_keys_replay_with_both_parts_off():
    defaults = {f.name: f.default for f in fields(FleetParams)}
    assert defaults["nav_pass_m"] == 0.0
    assert defaults["nav_turn_rps"] == 0.0


def test_the_running_file_turns_both_parts_on():
    assert P.nav_pass_m == pytest.approx(0.15), "pode_fsm pass_m"
    assert P.nav_turn_rps == pytest.approx(1.0), "RPP rotate_to_heading_angular_vel"
    assert P.nav_goal_tolerance_m == pytest.approx(0.05), "Nav2 general_goal_checker"


# ---- 관제가 쓰는 남은 시간 ----

ON = replace(P, use_order_model=True)


def driving_to_n_w():
    world = Harness(params=ON)
    world.step()
    world.call("WORK")
    world.step()
    m = world.traffic._robots["pode2"]
    route = m.route
    assert route.edges[route.done].mode == NAV_GOAL
    assert route.edges[route.done + 1].mode == FOLLOW_PATH
    return world, m


def test_the_projection_takes_the_hand_over_off_the_nav_part_only():
    world, m = driving_to_n_w()
    route = m.route
    end = route.done + 1
    nav = left_s(ON, route.edges[route.done:end], m.x, m.y)
    rest = travel_s(ON, route.edges[end:route.granted])
    got = world.traffic._left_s(m, route, route.granted)
    assert got == pytest.approx(nav - ON.nav_pass_m / speed(ON, NAV_GOAL) + rest)


def test_inside_the_hand_over_the_nav_part_is_zero_and_the_rest_stays():
    world, m = driving_to_n_w()
    route = m.route
    end = route.done + 1
    node = world.site.node_pose(route.edges[route.done].b)
    world.traffic.on_robot_state("pode2", node[0] - 0.05, node[1], m.yaw, m.report.robot_state)
    rest = travel_s(ON, route.edges[end:route.granted])
    assert world.traffic._left_s(m, route, route.granted) == pytest.approx(rest)


def test_a_cut_off_robot_hands_over_inside_its_base():
    world, m = driving_to_n_w()
    route = m.route
    assert m.move_order.given == len(m.move_order.cuts)
    assert route.edges[route.granted - 1].mode == FOLLOW_PATH
    passing = world.traffic._left_s(m, route, route.granted)
    assert world.traffic._left_s(m, route, route.granted, STOP) == pytest.approx(passing)


def test_a_cut_off_robot_turns_at_the_end_of_a_base_that_ends_on_a_nav_goal():
    world, m = driving_to_n_w()
    first, middle, last = m.route.edges
    end = world.site.nodes[last.b]
    assert end.yaw is not None
    side = end.yaw + math.pi / 2
    start = (end.x - 0.3 * math.cos(side), end.y - 0.3 * math.sin(side))
    sideways = replace(last, mode=NAV_GOAL, points=(start, (end.x, end.y)), length=0.3)
    route = Route(m.route.goal, [first, middle, sideways], granted=3)
    turn = abs(math.remainder(end.yaw - heading(start, (end.x, end.y)), math.tau))
    assert turn == pytest.approx(math.pi / 2)
    passing = world.traffic._left_s(m, route, 3)
    held = world.traffic._left_s(m, route, 3, STOP)
    assert held == pytest.approx(passing + turn / ON.nav_turn_rps)


def test_with_both_parts_off_the_old_sum_comes_out_exactly():
    world, m = driving_to_n_w()
    route = m.route
    world.traffic.params = replace(ON, nav_pass_m=0.0, nav_turn_rps=0.0)
    old = left_s(ON, route.edges[route.done:route.granted], m.x, m.y)
    assert world.traffic._left_s(m, route, route.granted) == old
    world.now += 0.2
    world.traffic.on_move_feedback("pode2", m.goal.id, 0, False, "", 0.8)
    _, _, stop = m.move_order.cuts[0]
    old = 0.8 / speed(ON, NAV_GOAL) + travel_s(ON, route.edges[stop:route.granted])
    assert world.traffic._left_s(m, route, route.granted) == old


def test_the_lane_order_compares_arrival_at_the_entry_without_the_end_parts():
    """d2: pode1이 N_W에서 X_W로 급행에 들어가려 한다. 입구까지의 남은 시간은 끝 보정 없이 잰다."""
    site = load_fleet_map("d2")
    traffic = ZoneTraffic(site, ON, ["pode1"], lambda: 10.0)
    traffic.on_robot_state("pode1", *site.node_pose("P-1"), 0.0, "IDLE")
    m = traffic._robots["pode1"]
    edges = []
    for a, b in pairwise(("P-1", "N_W", "X_W", "X_E", "N_E", "O_Q")):
        edges += shortest_path(site, a, b)
    east = Route("O_Q", edges)
    seen = []
    ask = traffic.lanes.refusal

    def spy(robot, zone, want, now, rivals):
        seen.append((zone, want.eta[zone]))
        return ask(robot, zone, want, now, rivals)

    traffic.lanes.refusal = spy
    traffic._take(m, east)
    [(zone, eta)] = seen
    index = next(i for i, e in enumerate(edges) if e.lane == zone)
    assert edges[0].mode == NAV_GOAL, "the robot drives a Nav2 goal to the entry"
    assert edges[index].mode in (NAV_GOAL, FOLLOW_PATH), "a hand-over would apply at the entry"
    assert eta == left_s(ON, edges[:index], m.x, m.y)
