"""가는 길을 구간(한꺼번에 잡는 단위)과 조각(명령 하나)으로 나누기."""
import math

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, load_fleet_map, shortest_path
from fleet_core.traffic.zone import routes

D2 = load_fleet_map("d2")
PARAMS = load_params()


def way(start, goal):
    return shortest_path(D2, start, goal)


def test_a_stretch_runs_to_the_next_node_where_a_robot_may_wait():
    to_work = way("P-1", "W")
    assert [edge.b for edge in to_work] == ["N_W", "X_W", "W"]
    assert routes.stretch_end(D2, to_work, 0) == 3, "the fork and the junction are no place to wait"
    to_queue = way("P-1", "W_Q")
    assert routes.stretch_end(D2, to_queue, 0) == 1


def test_a_stretch_needs_its_zones_its_nodes_and_the_lane_direction():
    assert routes.needs(way("P-1", "W")) == (["HALL", "J_W", "WORK_DOCK"], ["N_W", "X_W", "W"], {})
    zones, nodes, lanes = routes.needs(way("W", "O"))
    assert zones == ["WORK_DOCK", "J_W", "HALL", "EXPRESS", "J_E", "OUT_DOCK"]
    assert nodes == ["N_W", "X_W", "X_E", "O"], "it backs into the hall before the express"
    assert lanes == {"EXPRESS": "X_W"}, "the lane runs from the node the robot enters by"


def test_one_command_per_nav_goal_edge_and_one_per_run_of_fixed_path():
    to_work = way("P-1", "W")
    assert [edge.mode for edge in to_work] == [NAV_GOAL, FOLLOW_PATH, FOLLOW_PATH]
    assert routes.segment_end(D2, to_work, 0, 3) == 1
    assert routes.segment_end(D2, to_work, 1, 3) == 3
    assert routes.segment_end(D2, to_work, 1, 2) == 2, "never past what is granted"
    to_out = way("W", "O")
    assert [edge.mode for edge in to_out] == [RETREAT, FOLLOW_PATH, FOLLOW_PATH, FOLLOW_PATH]
    assert routes.segment_end(D2, to_out, 0, 4) == 1
    assert routes.segment_end(D2, to_out, 1, 4) == 4


def test_a_nav_goal_command_faces_the_way_the_fixed_path_continues():
    to_work = way("P-1", "W")
    cmd = routes.move_command(D2, "pode1", 7, to_work[:1], 0.13, 0.15, after=to_work[1])
    assert (cmd.mode, cmd.target_id, cmd.path, cmd.goal_id) == ("nav_goal", "N_W", (), 7)
    assert (cmd.x, cmd.y) == (0.35, 0.65)
    assert cmd.yaw == pytest.approx(math.pi / 2), "work_in starts northward"


def test_a_nav_goal_command_uses_the_heading_written_on_the_node():
    cmd = routes.move_command(D2, "pode1", 1, way("P-1", "W_Q"), 0.13, 0.15)
    assert cmd.yaw == pytest.approx(math.radians(90))
    cmd = routes.move_command(D2, "pode1", 1, way("W_Q", "N_E"), 0.20, 0.55)
    assert cmd.yaw == pytest.approx(math.atan2(0.65 - 0.55, 1.63 - 0.20)), "the way it drives"


def test_a_follow_path_command_is_one_polyline_through_the_nodes():
    to_work = way("P-1", "W")
    cmd = routes.move_command(D2, "pode1", 8, to_work[1:], 0.35, 0.65)
    assert (cmd.mode, cmd.target_id, cmd.final_goal) == ("follow_path", "W", True)
    assert [(x, y) for x, y, _ in cmd.path] == [(0.35, 0.65), (0.35, 0.89), (0.18, 0.89)]
    assert [yaw for _, _, yaw in cmd.path] == pytest.approx([math.pi / 2, math.pi, math.pi])
    assert cmd.yaw == pytest.approx(math.pi)


def test_a_follow_path_command_sent_again_starts_where_the_robot_is_on_the_line():
    express = way("X_W", "X_E")
    cmd = routes.move_command(D2, "pode1", 9, express, 0.90, 0.91)
    assert [(x, y) for x, y, _ in cmd.path] == [pytest.approx((0.90, 0.89)), (1.63, 0.89)]


# D2 후진 꺾는 점은 (0.30, 0.89)다(2-1: 진열대 끝에서 멀리 돌고, 마지막 조각은 N_W로 비스듬히
# 내려간다)
SLANT = math.hypot(0.05, 0.24)


def test_a_retreat_command_backs_round_the_bend_into_the_hall_facing_the_dock():
    out = way("W", "O")[:1]
    cmd = routes.retreat_command(PARAMS, "pode1", 3, out, 0.18, 0.89)
    assert cmd.distance_m == pytest.approx(0.12 + SLANT)
    assert cmd.speed_mps == PARAMS.retreat_speed_mps
    assert [(x, y) for x, y, _ in cmd.path] == [(0.18, 0.89), (0.30, 0.89), (0.35, 0.65)]
    assert [yaw for _, _, yaw in cmd.path] == pytest.approx(
        [math.pi, math.pi, math.atan2(0.24, -0.05)])


def test_a_retreat_sent_again_past_the_start_keeps_the_bend_ahead():
    out = way("W", "O")[:1]
    cmd = routes.retreat_command(PARAMS, "pode1", 3, out, 0.25, 0.90)
    assert [(x, y) for x, y, _ in cmd.path] == [
        pytest.approx((0.25, 0.89)), (0.30, 0.89), (0.35, 0.65)]
    # 비스듬한 조각 위 y 0.75: 남은 길은 그 조각의 0.10 m / 0.24 m
    cmd = routes.retreat_command(PARAMS, "pode1", 3, out, 0.33, 0.75)
    assert [(x, y) for x, y, _ in cmd.path][-1] == (0.35, 0.65)
    assert cmd.distance_m == pytest.approx(SLANT * 0.10 / 0.24, abs=0.002)


def test_where_a_robot_is_along_a_bent_edge():
    out = way("W", "O")[0]
    assert out.length == pytest.approx(0.12 + SLANT)
    assert routes.along(out, 0.25, 0.90) == pytest.approx(0.07)
    assert routes.along(out, 0.33, 0.75) == pytest.approx(0.12 + SLANT * 0.14 / 0.24, abs=0.002)
    assert not routes.passed(out, 0.35, 0.70)
    assert routes.passed(out, 0.35, 0.66)


def test_where_a_robot_is_along_an_edge():
    express = way("X_W", "X_E")[0]
    assert routes.along(express, 0.90, 0.95) == pytest.approx(0.55)
    assert routes.along(express, 0.10, 0.89) == 0.0
    assert not routes.passed(express, 1.55, 0.89)
    assert routes.passed(express, 1.61, 0.89)


def test_edges_that_are_not_on_the_map():
    free = routes.free_edge(D2, 1.00, 0.30, "N_E")
    assert (free.a, free.b, free.mode, free.zones) == ("", "N_E", NAV_GOAL, ("HALL",))
    assert free.length == pytest.approx(math.dist((1.00, 0.30), (1.63, 0.65)))
    back = routes.back_edge(D2, 0.90, 0.89, "X_W", ("EXPRESS", "J_W"))
    assert (back.b, back.mode, back.zones, back.lane) == ("X_W", RETREAT, ("EXPRESS", "J_W"), "")
    assert back.length == pytest.approx(0.55)


def test_time_left_on_a_run_of_edges():
    to_out = way("W", "O")
    whole = ((0.12 + SLANT) / PARAMS.retreat_speed_mps
             + (0.24 + 1.28 + 0.17) / PARAMS.follow_speed_mps)
    assert routes.travel_s(PARAMS, to_out) == pytest.approx(whole)
    assert routes.left_s(PARAMS, to_out[2:], 0.99, 0.89) == pytest.approx(
        (0.64 + 0.17) / PARAMS.follow_speed_mps)
    assert routes.left_s(PARAMS, [], 0.0, 0.0) == 0.0


def test_a_route_lists_the_nodes_still_ahead():
    route = routes.Route("W", way("P-1", "W"))
    assert route.nodes_left() == ["N_W", "X_W", "W"] and not route.finished
    route.done = 1
    assert route.nodes_left() == ["X_W", "W"]
    route.done = 3
    assert route.finished
