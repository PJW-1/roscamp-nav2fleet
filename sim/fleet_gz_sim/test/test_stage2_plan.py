"""2단계 주행 계획: 구역 지도의 연결로 만들어지는지와 지표 계산. ROS 없이 돈다."""
import math

import pytest

pytest.importorskip("fleet_core")
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, load_fleet_map  # noqa: E402
from fleet_gz_sim import stage2_plan as sp  # noqa: E402

SITE = load_fleet_map("d2")


@pytest.mark.parametrize("plan", sorted(sp.PLANS))
def test_every_plan_is_made_of_d2_links_and_ends_where_it_started(plan):
    steps = sp.build(SITE, plan, laps=2)
    assert len(steps) == 2 * len(sp.PLANS[plan])
    assert steps[0].nodes[0] == sp.start_node(plan) == steps[-1].nodes[-1]
    for one, two in zip(steps, steps[1:], strict=False):
        assert one.nodes[-1] == two.nodes[0]


def test_stage2_covers_every_segment_type_of_the_acceptance():
    """추가 지시서 3의 2-5: 마당 goal, 도크 진입 둘, Retreat 둘, 급행 양방향, 갈림목 내려오기 둘."""
    kinds = {(s.kind, s.nodes) for s in sp.build(SITE, "stage2")}
    assert (FOLLOW_PATH, ("N_W", "X_W", "W")) in kinds
    assert (FOLLOW_PATH, ("N_E", "X_E", "O")) in kinds
    assert (RETREAT, ("W", "N_W")) in kinds and (RETREAT, ("O", "N_E")) in kinds
    assert (FOLLOW_PATH, ("N_W", "X_W", "X_E", "N_E")) in kinds
    assert (FOLLOW_PATH, ("N_E", "X_E", "X_W", "N_W")) in kinds
    assert sum(k == NAV_GOAL for k, _ in kinds) == 2


def test_a_nav_goal_ends_on_the_node_yaw_or_the_fixed_path_it_joins():
    to_park = sp.make_step(SITE, NAV_GOAL, ("N_W", "P-3"))
    assert to_park.target == pytest.approx((1.85, 0.15, math.pi / 2))
    to_n_w = sp.make_step(SITE, NAV_GOAL, ("P-1", "N_W"))
    assert to_n_w.target == pytest.approx((0.35, 0.65, math.pi / 2))  # N_W -> X_W points north


def test_a_retreat_path_keeps_its_bend_and_faces_the_way_the_robot_looks():
    step = sp.make_step(SITE, RETREAT, ("W", "N_W"))
    assert [p[:2] for p in step.poses] == [pytest.approx((0.18, 0.89)), pytest.approx((0.35, 0.89)),
                                           pytest.approx((0.35, 0.65))]
    assert abs(abs(step.poses[0][2]) - math.pi) < 1e-9  # facing west, driving east
    assert step.poses[-1][2] == pytest.approx(math.pi / 2)  # facing north, driving south


@pytest.mark.parametrize("kind, nodes", [
    (FOLLOW_PATH, ("W", "X_W")),     # a Retreat link, not a fixed path
    (RETREAT, ("X_W", "W")),
    (NAV_GOAL, ("X_W", "X_E")),
    (NAV_GOAL, ("P-1", "nowhere")),
])
def test_a_step_against_the_map_is_refused(kind, nodes):
    with pytest.raises((sp.PlanError, KeyError)):
        sp.make_step(SITE, kind, nodes)


def test_an_unknown_plan_is_refused():
    with pytest.raises(sp.PlanError, match="stage2"):
        sp.build(SITE, "warehouse")


def test_metrics():
    assert sp.yaw_error(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)
    line = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
    assert sp.cross_track(line, 0.5, 0.03) == pytest.approx(0.03)
    assert sp.cross_track(line, 1.2, 0.5) == pytest.approx(0.2)
    assert sp.cross_track(line, -0.3, 0.4) == pytest.approx(0.5)


def test_keepout_zones_lists_each_non_shared_zone_once_in_order():
    points = [(0.4, 0.5), (0.99, 0.89), (1.2, 0.89), (0.40, 0.89), (5.0, 5.0)]
    assert sp.keepout_zones(SITE, points) == ["EXPRESS", "J_W"]
    assert sp.keepout_zones(SITE, [(0.4, 0.5), (1.0, 0.3)]) == []
