"""가는 길을 구간(한꺼번에 잡는 단위)과 조각(명령 하나)으로 나누기."""
import math

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, load_fleet_map, route
from fleet_core.traffic.zone import trips

D2 = load_fleet_map("d2")
PARAMS = load_params()


def way(start, goal):
    return route(D2, start, goal)


def test_a_stretch_runs_to_the_next_node_where_a_robot_may_wait():
    to_work = way("P-1", "W")
    assert [link.b for link in to_work] == ["N_W", "X_W", "W"]
    assert trips.stretch_end(D2, to_work, 0) == 3, "the fork and the junction are no place to wait"
    to_queue = way("P-1", "W_Q")
    assert trips.stretch_end(D2, to_queue, 0) == 1


def test_a_stretch_needs_its_zones_its_nodes_and_the_lane_direction():
    assert trips.needs(way("P-1", "W")) == (["HALL", "J_W", "WORK_DOCK"], ["N_W", "X_W", "W"], {})
    zones, nodes, lanes = trips.needs(way("W", "O"))
    assert zones == ["WORK_DOCK", "J_W", "EXPRESS", "J_E", "OUT_DOCK"]
    assert nodes == ["X_W", "X_E", "O"]
    assert lanes == {"EXPRESS": "X_W"}, "the lane runs from the node the robot enters by"


def test_one_command_per_nav_goal_link_and_one_per_run_of_fixed_path():
    to_work = way("P-1", "W")
    assert [link.via for link in to_work] == [NAV_GOAL, FOLLOW_PATH, FOLLOW_PATH]
    assert trips.segment_end(to_work, 0, 3) == 1
    assert trips.segment_end(to_work, 1, 3) == 3
    assert trips.segment_end(to_work, 1, 2) == 2, "never past what is granted"
    to_out = way("W", "O")
    assert [link.via for link in to_out] == [RETREAT, FOLLOW_PATH, FOLLOW_PATH]
    assert trips.segment_end(to_out, 0, 3) == 1
    assert trips.segment_end(to_out, 1, 3) == 3


def test_a_nav_goal_command_faces_the_way_the_fixed_path_continues():
    to_work = way("P-1", "W")
    cmd = trips.move_command(D2, "pinky1", 7, to_work[:1], 0.13, 0.15, after=to_work[1])
    assert (cmd.mode, cmd.target_id, cmd.path, cmd.goal_id) == ("nav_goal", "N_W", (), 7)
    assert (cmd.x, cmd.y) == (0.40, 0.65)
    assert cmd.yaw == pytest.approx(math.pi / 2), "work_in starts northward"


def test_a_nav_goal_command_uses_the_heading_written_on_the_node():
    cmd = trips.move_command(D2, "pinky1", 1, way("P-1", "W_Q"), 0.13, 0.15)
    assert cmd.yaw == pytest.approx(math.radians(90))
    cmd = trips.move_command(D2, "pinky1", 1, way("W_Q", "N_E"), 0.20, 0.55)
    assert cmd.yaw == pytest.approx(math.atan2(0.65 - 0.55, 1.58 - 0.20)), "the way it drives"


def test_a_follow_path_command_is_one_polyline_through_the_nodes():
    to_work = way("P-1", "W")
    cmd = trips.move_command(D2, "pinky1", 8, to_work[1:], 0.40, 0.65)
    assert (cmd.mode, cmd.target_id, cmd.final_goal) == ("follow_path", "W", True)
    assert [(x, y) for x, y, _ in cmd.path] == [(0.40, 0.65), (0.40, 0.89), (0.18, 0.89)]
    assert [yaw for _, _, yaw in cmd.path] == pytest.approx([math.pi / 2, math.pi, math.pi])
    assert cmd.yaw == pytest.approx(math.pi)


def test_a_follow_path_command_sent_again_starts_where_the_robot_is_on_the_line():
    express = way("X_W", "X_E")
    cmd = trips.move_command(D2, "pinky1", 9, express, 0.90, 0.91)
    assert [(x, y) for x, y, _ in cmd.path] == [pytest.approx((0.90, 0.89)), (1.58, 0.89)]


def test_a_retreat_command_backs_along_the_line_facing_the_dock():
    out = way("W", "O")[:1]
    cmd = trips.retreat_command(PARAMS, "pinky1", 3, out, 0.18, 0.89)
    assert cmd.distance_m == pytest.approx(0.22)
    assert cmd.speed_mps == PARAMS.retreat_speed_mps
    assert [(x, y) for x, y, _ in cmd.path] == [(0.18, 0.89), (0.40, 0.89)]
    assert [yaw for _, _, yaw in cmd.path] == pytest.approx([math.pi, math.pi])


def test_where_a_robot_is_along_a_link():
    express = way("X_W", "X_E")[0]
    assert trips.along(express, 0.90, 0.95) == pytest.approx(0.50)
    assert trips.along(express, 0.10, 0.89) == 0.0
    assert not trips.passed(express, 1.50, 0.89)
    assert trips.passed(express, 1.56, 0.89)


def test_links_that_are_not_on_the_map():
    free = trips.free_link(D2, 1.00, 0.30, "N_E")
    assert (free.a, free.b, free.via, free.zones) == ("", "N_E", NAV_GOAL, ("HALL",))
    assert free.length == pytest.approx(math.dist((1.00, 0.30), (1.58, 0.65)))
    back = trips.back_link(D2, 0.90, 0.89, "X_W", ("EXPRESS", "J_W"))
    assert (back.b, back.via, back.zones, back.lane) == ("X_W", RETREAT, ("EXPRESS", "J_W"), "")
    assert back.length == pytest.approx(0.50)


def test_time_left_on_a_run_of_links():
    to_out = way("W", "O")
    whole = 0.22 / PARAMS.retreat_speed_mps + (1.18 + 0.22) / PARAMS.follow_speed_mps
    assert trips.travel_s(PARAMS, to_out) == pytest.approx(whole)
    assert trips.left_s(PARAMS, to_out[1:], 0.99, 0.89) == pytest.approx(
        (0.59 + 0.22) / PARAMS.follow_speed_mps)
    assert trips.left_s(PARAMS, [], 0.0, 0.0) == 0.0


def test_a_trip_lists_the_nodes_still_ahead():
    trip = trips.Trip("W", way("P-1", "W"))
    assert trip.route() == ["N_W", "X_W", "W"] and not trip.finished
    trip.done = 1
    assert trip.route() == ["X_W", "W"]
    trip.done = 3
    assert trip.finished
