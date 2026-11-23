"""ETA 순항 속도 비율 nav_eta_speed_ratio(사용자 결정 2026-11-23: 틀만, 값은 실측 뒤).

- Nav2 goal 조각의 남은 시간(ETA)만 nav_speed_mps x 비율로 잰다. 비율이 r이면 그 부분이
  정확히 1/r배 길어진다.
- 고정 경로(follow_speed_mps)와 후진 속도, 길 비용(route 고르기)은 비율을 쓰지 않는다.
- 기본값 1.0이면 모든 값이 예전과 같다(옛 기록 재생도 같다).
"""
import math
from dataclasses import replace

import pytest
import yaml
from fleet_core.common.params import DEFAULT_PARAMS, FleetParams, load_params
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, load_fleet_map, shortest_path
from fleet_core.traffic.zone import routes
from fleet_core.traffic.zone.routes import ARRIVE, PASS, STOP
from test_zone_distance_left import ON, driving_to_n_w, feed
from zone_harness import Harness

P = load_params()
V7 = load_fleet_map("v7")
D2 = load_fleet_map("d2")
R = 0.85
SLOW = replace(P, nav_eta_speed_ratio=R)


# ---- 파라미터 ----

def test_the_default_is_one_in_the_code_and_in_the_file():
    assert FleetParams.__dataclass_fields__["nav_eta_speed_ratio"].default == 1.0
    assert yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))["nav_eta_speed_ratio"] == 1.0
    assert P.nav_eta_speed_ratio == 1.0


@pytest.mark.parametrize("ratio", [1.0, 0.5, 0.01, 1])
def test_a_ratio_in_zero_to_one_is_taken(ratio):
    assert replace(P, nav_eta_speed_ratio=ratio).nav_eta_speed_ratio == ratio


@pytest.mark.parametrize("ratio", [0.0, -0.1, 1.01, 2.0, math.nan, math.inf, True, False,
                                   "0.9", None])
def test_a_ratio_outside_zero_to_one_is_refused(ratio):
    with pytest.raises(ValueError, match="nav_eta_speed_ratio"):
        replace(P, nav_eta_speed_ratio=ratio)


def test_a_ratio_so_small_the_eta_speed_is_zero_is_refused():
    assert P.nav_speed_mps * 5e-324 == 0.0
    with pytest.raises(ValueError, match="nav_eta_speed_ratio"):
        replace(P, nav_eta_speed_ratio=5e-324)


@pytest.mark.parametrize("ratio", [0.0, 1.5, True])
def test_the_file_check_refuses_a_bad_ratio(tmp_path, ratio):
    data = yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
    data["nav_eta_speed_ratio"] = ratio
    path = tmp_path / "params.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(ValueError, match="nav_eta_speed_ratio"):
        load_params(path)


# ---- 속도 ----

@pytest.mark.parametrize("mode", [NAV_GOAL, FOLLOW_PATH, RETREAT])
def test_with_ratio_one_the_eta_speed_is_the_route_cost_speed(mode):
    assert routes.eta_speed(P, mode) == routes.speed(P, mode)


def test_only_the_nav_goal_eta_speed_takes_the_ratio():
    assert routes.eta_speed(SLOW, NAV_GOAL) == P.nav_speed_mps * R
    assert routes.eta_speed(SLOW, FOLLOW_PATH) == P.follow_speed_mps
    assert routes.eta_speed(SLOW, RETREAT) == P.retreat_speed_mps
    for mode in (NAV_GOAL, FOLLOW_PATH, RETREAT):
        assert routes.speed(SLOW, mode) == routes.speed(P, mode), "길 비용 속도는 그대로"


# ---- left_s, travel_s, nav_end_s ----

@pytest.mark.parametrize(("x", "y"), [(0.15, 0.24), (0.60, 0.24), (1.15, 0.20), (1.60, 0.165)])
def test_time_left_on_nav_goal_edges_grows_by_one_over_the_ratio(x, y):
    run = shortest_path(V7, "E-1", "Q2")
    assert {edge.mode for edge in run} == {NAV_GOAL}
    assert routes.left_s(SLOW, run, x, y) == pytest.approx(routes.left_s(P, run, x, y) / R)
    assert routes.travel_s(SLOW, run) == pytest.approx(routes.travel_s(P, run) / R)


def test_on_a_mixed_run_only_the_nav_goal_part_grows():
    edges = shortest_path(V7, "E-1", "Q1")
    assert [edge.mode for edge in edges] == [NAV_GOAL, NAV_GOAL, FOLLOW_PATH]
    lane = edges[2].length / P.follow_speed_mps
    nav = routes.left_s(P, edges, 1.15, 0.20) - lane
    assert routes.left_s(SLOW, edges, 1.15, 0.20) == pytest.approx(nav / R + lane)


def test_fixed_path_and_retreat_time_left_does_not_change():
    fixed = shortest_path(V7, "Q2", "W")
    assert {edge.mode for edge in fixed} == {FOLLOW_PATH}
    assert routes.left_s(SLOW, fixed, 1.75, 0.16) == routes.left_s(P, fixed, 1.75, 0.16)
    back = [V7.edge("E-3", "P-3")]
    assert routes.left_s(SLOW, back, 0.64, 0.18) == routes.left_s(P, back, 0.64, 0.18)
    to_out = shortest_path(D2, "W", "O")
    assert routes.travel_s(SLOW, to_out) == routes.travel_s(P, to_out)


def test_the_pass_distance_grows_with_the_ratio_and_the_end_turn_does_not():
    run = shortest_path(V7, "E-1", "Q1")
    passing = routes.nav_end_s(P, V7, run, 2, 0.6, 0.24, 0.0, PASS)
    assert passing < 0
    assert routes.nav_end_s(SLOW, V7, run, 2, 0.6, 0.24, 0.0, PASS) == pytest.approx(passing / R)
    for ending in (STOP, ARRIVE):
        assert (routes.nav_end_s(SLOW, V7, run, 2, 0.6, 0.24, 0.0, ending)
                == routes.nav_end_s(P, V7, run, 2, 0.6, 0.24, 0.0, ending))


# ---- 관제의 남은 시간(_left_s): 답, 대기 앞보기, 차선 ETA, 통신 끊김 대기 ----

def nav_and_rest(world, m, ending):
    """지금 Nav2 goal 이어짐(끝 보정 포함)과 그 뒤 고정 경로 부분으로 나눈다."""
    route = m.route
    chain = route.done + routes.nav_stretch(route.edges[route.done:route.granted])
    rest = routes.travel_s(world.traffic.params, route.edges[chain:route.granted])
    return world.traffic._left_s(m, route, route.granted, ending) - rest, rest


@pytest.mark.parametrize("ending", [PASS, ARRIVE])
@pytest.mark.parametrize("distance", [None, 0.80])
def test_the_fleet_eta_grows_by_one_over_the_ratio_only_on_the_nav_goal_part(ending, distance):
    world, goal_id, m = driving_to_n_w()
    if distance is not None:
        feed(world, goal_id, distance)
    nav, rest = nav_and_rest(world, m, ending)
    world.traffic.params = replace(ON, nav_eta_speed_ratio=R)
    slow_nav, slow_rest = nav_and_rest(world, m, ending)
    assert rest > 0 and slow_rest == rest
    assert slow_nav == pytest.approx(nav / R)


def test_the_comm_loss_hold_uses_the_longer_eta():
    world, _, m = driving_to_n_w()
    extra = ON.comm_base_hold_extra_s
    world.traffic._hold_base(m, world.now)
    before = m.base_until - world.now - extra
    world.traffic.params = replace(ON, nav_eta_speed_ratio=R)
    world.traffic._hold_base(m, world.now)
    after = m.base_until - world.now - extra
    assert after == pytest.approx(world.traffic._left_s(m, m.route, m.route.granted, STOP))
    assert after > before


# ---- 길 비용과 route 고르기는 비율을 쓰지 않는다 ----

def costs(params):
    world = Harness(params=params)
    world.step()
    world.call("WORK")
    world.step()
    m = world.traffic._robots["pode2"]
    cost = world.traffic._cost(m)
    return [cost(edge) for edge in world.site.edges()], [
        (e.a, e.b, e.mode) for e in m.route.edges]


def test_route_costs_and_the_chosen_route_do_not_change():
    assert costs(ON) == costs(replace(ON, nav_eta_speed_ratio=0.5))
