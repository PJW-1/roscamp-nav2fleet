"""RobotSelector: 누가 채움 도크로 불리는가. 순수 함수이고 단순 로봇 기록으로 구동.

D2 배치에서 W(채우기 도크)까지 길을 따라 잰 거리(m): X_W 0.17, N_W 0.41, P-2 0.91, P-1 0.96,
N_E 1.69, P-3 1.99, O 2.10(N_E까지 후진한 뒤). 곧은 거리로는 P-1이 P-2보다 가깝다.
"""
from dataclasses import replace

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import FleetMap, load_fleet_map
from fleet_core.traffic.selector import RobotInfo, RobotSelector
from zone_harness import Harness

SITE = load_fleet_map("d2")


@pytest.fixture(scope="module")
def selector():
    return RobotSelector(SITE, load_params())


def at(name, node, **kw):
    return RobotInfo(name, node, **kw)


def test_no_orders_no_calls(selector):
    assert selector.pick(0, [at("a", "P-1")]) == set()


def test_calls_follow_the_number_of_unbound_orders(selector):
    robots = [at("a", "P-2"), at("b", "P-1"), at("c", "P-3")]
    assert selector.pick(1, robots) == {"a"}
    assert selector.pick(2, robots) == {"a", "b"}
    assert selector.pick(5, robots) == {"a", "b", "c"}, "no more calls than robots"


@pytest.mark.parametrize("near_in_a_line, near_along_the_way", [("P-1", "P-2")])
def test_shortest_way_along_the_graph_wins_not_the_straight_line(selector, near_in_a_line,
                                                                 near_along_the_way):
    robots = [at("in_a_line", near_in_a_line), at("along", near_along_the_way)]
    assert selector.pick(1, robots) == {"along"}


def test_only_eligible_robots_are_called(selector):
    robots = [at("busy", "X_W", eligible=False), at("free", "P-3")]
    assert selector.pick(1, robots) == {"free"}


def test_a_called_robot_keeps_the_call_across_the_whole_d2_floor(selector):
    """score_same_call -4.0은 0.91/m로 4.4 m 값이다. D2의 어느 길보다 길다."""
    far = at("far", "P-3", called=True)
    assert selector.pick(1, [far, at("next_to_the_dock", "X_W")]) == {"far"}


def test_a_called_robot_keeps_the_call_unless_another_is_clearly_closer():
    selector = RobotSelector(SITE, replace(load_params(), score_same_call=-1.0))
    far = at("far", "P-3", called=True)  # 1.99 m, 1.81 - 1.0의 가치
    assert selector.pick(1, [far, at("closer", "P-2")]) == {"far"}, "0.96 m, worth 0.87"
    assert selector.pick(1, [far, at("much_closer", "X_W")]) == {"much_closer"}, "worth 0.20"


def test_name_breaks_a_tie_so_a_run_can_be_replayed(selector):
    assert selector.pick(1, [at("b", "P-1"), at("a", "P-1")]) == {"a"}


# ---- 값(price_m)과 실제 길이가 다른 길 ----
# pc/fleet_core/test/test_fleet_map.py의 SMALL에 지점 S를 더한 지도. S -> N_W 곧은 이음(0.25 m)에
# extra_m 1.0을 붙여, 값으로는 W_Q를 거치는 길(0.627 m)이 싸다. W까지 실제로 가장 짧은 길은 곧은
# 이음으로 0.75 m, 값으로 고른 길의 길이는 1.127 m다. P-1에서 W까지는 0.992 m로 그 사이다.
PRICED = {
    "name": "priced",
    "bounds": [0.0, 0.0, 2.0, 1.0],
    "shelves": [[0.5, 0.75, 1.5, 0.8]],
    "zones": {
        "HALL": {"kind": "shared", "capacity": 2,
                 "rects": [[0.0, 0.0, 2.0, 0.75], [0.0, 0.75, 0.5, 0.8], [1.5, 0.75, 2.0, 0.8]]},
        "DOCK_W": {"kind": "dock", "rects": [[0.0, 0.8, 0.3, 1.0]]},
        "J_W": {"kind": "dock", "rects": [[0.3, 0.8, 0.5, 1.0]]},
        "EXPRESS": {"kind": "lane", "rects": [[0.5, 0.8, 1.5, 1.0]]},
        "J_E": {"kind": "dock", "rects": [[1.5, 0.8, 1.7, 1.0]]},
        "DOCK_E": {"kind": "dock", "rects": [[1.7, 0.8, 2.0, 1.0]]},
    },
    "nodes": {
        "W": {"x": 0.2, "y": 0.9, "kind": "dock", "yaw_deg": 180},
        "O": {"x": 1.8, "y": 0.9, "kind": "dock", "yaw_deg": 0},
        "X_W": {"x": 0.4, "y": 0.9, "kind": "pass"},
        "X_E": {"x": 1.6, "y": 0.9, "kind": "pass"},
        "N_W": {"x": 0.4, "y": 0.6, "kind": "fork"},
        "N_E": {"x": 1.6, "y": 0.6, "kind": "fork"},
        "W_Q": {"x": 0.2, "y": 0.5, "kind": "wait"},
        "O_Q": {"x": 1.8, "y": 0.5, "kind": "wait"},
        "S": {"x": 0.6, "y": 0.45, "kind": "fork"},
        "P-1": {"x": 0.2, "y": 0.15, "kind": "parking", "yaw_deg": 90},
        "P-2": {"x": 1.8, "y": 0.15, "kind": "parking"},
    },
    "paths": {
        "work_in": {"points": ["N_W", "X_W", "W"]},
        "work_out": {"points": ["W", "X_W"], "reverse": True},
        "out_in": {"points": ["N_E", "X_E", "O"]},
        "out_out": {"points": ["O", "X_E"], "reverse": True},
        "express": {"points": ["X_W", "X_E"], "two_way": True},
        "j_w_down": {"points": ["X_W", "N_W"]},
        "j_e_down": {"points": ["X_E", "N_E"]},
    },
    "nav": [{"nodes": ["N_W", "N_E", "W_Q", "O_Q", "P-1", "P-2"]},
            {"nodes": ["S", "N_W"], "extra_m": 1.0},
            {"nodes": ["S", "W_Q"]}],
    "stations": {
        "WORK": {"dock": "W", "queue": ["W_Q"], "server": "workcell", "arms": ["arm1", "arm2"],
                 "action": "fill"},
        "OUTBOUND": {"dock": "O", "queue": ["O_Q"], "server": "arm3", "arms": ["arm3"],
                     "action": "swap"},
    },
    "parking": ["P-1", "P-2"],
}


def test_the_robot_nearest_by_real_length_is_called_even_when_its_cheapest_way_is_longer():
    selector = RobotSelector(FleetMap(PRICED), load_params())
    assert selector.pick(1, [at("priced", "S"), at("plain", "P-1")]) == {"priced"}


def test_the_dock_queue_measures_the_way_to_the_dock_by_real_length():
    """asked_gap은 같은 때 부른 로봇의 도크 줄 순서를 정한다(turns.py)."""
    h = Harness(site=FleetMap(PRICED), starts={"pinky1": "S"})
    h.report()
    h.call("WORK", "pinky1")
    assert h.traffic._robots["pinky1"].asked_gap == pytest.approx(0.75, abs=1e-3)
