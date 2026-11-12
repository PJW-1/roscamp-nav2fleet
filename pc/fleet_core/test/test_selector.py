"""RobotSelector: 누가 채움 도크로 불리는가. 순수 함수이고 단순 로봇 기록으로 구동.

D2 배치에서 W(채우기 도크)까지 길을 따라 잰 거리(m): X_W 0.17, N_W 0.41, P-2 0.91, P-1 0.96,
N_E 1.69, P-3 1.99, O 2.10(N_E까지 후진한 뒤). 곧은 거리로는 P-1이 P-2보다 가깝다.
"""
from dataclasses import replace

import pytest
from fleet_core.common.params import load_params
from fleet_core.fleet_map import load_fleet_map
from fleet_core.traffic.selector import RobotInfo, RobotSelector

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
