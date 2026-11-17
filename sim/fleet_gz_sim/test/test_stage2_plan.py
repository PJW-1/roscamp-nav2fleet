"""2단계 주행 계획: 구역 지도의 연결로 만들어지는지와 지표 계산. ROS 없이 돈다."""
import math

import pytest

pytest.importorskip("fleet_core")
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, load_fleet_map  # noqa: E402
from fleet_gz_sim import stage2_plan as sp  # noqa: E402

SITE = load_fleet_map("d2")


@pytest.mark.parametrize("plan", sorted(sp.PLANS))
def test_every_plan_is_made_of_its_layout_links_and_ends_where_it_started(plan):
    steps = sp.build(load_fleet_map(sp.plan_layout(plan)), plan, laps=2)
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
    # D2 후진 꺾는 점 (0.30, 0.89): 마지막 조각은 N_W로 비스듬히 내려간다(2-1)
    assert [p[:2] for p in step.poses] == [pytest.approx((0.18, 0.89)), pytest.approx((0.30, 0.89)),
                                           pytest.approx((0.35, 0.65))]
    assert abs(abs(step.poses[0][2]) - math.pi) < 1e-9  # facing west, driving east
    # facing back up the slant (north-west), driving down it
    assert step.poses[-1][2] == pytest.approx(math.atan2(0.24, -0.05))


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


# ---- 가운데 통로 재현(v7, 2026-11-17) ----
V7 = load_fleet_map("v7")


def test_corridor_plan_goes_o_in_to_m_e_and_back_by_nav_goal_on_v7():
    assert sp.plan_layout("corridor") == "v7" and sp.plan_layout("stage2") == "d2"
    assert sp.start_node("corridor") == "O_IN"
    steps = sp.build(V7, "corridor", laps=3)
    assert [s.label for s in steps] == ["nav_goal O_IN>M_E", "nav_goal M_E>O_IN"] * 3
    # M_E는 yaw도 이어지는 고정 경로도 없어 시작→끝 방향, O_IN은 지점 yaw 108도
    assert steps[0].target == pytest.approx((1.645, 0.55, math.atan2(-0.075, 1.155)))
    assert steps[1].target == pytest.approx((0.49, 0.625, math.radians(108)))


def test_corridor_plan_is_refused_on_another_layout():
    with pytest.raises(sp.PlanError, match="v7"):
        sp.build(SITE, "corridor")


def test_the_corridor_link_and_the_robot_body_along_it_are_outside_the_keepout_mask():
    """keepout 마스크가 가운데 통로를 막으면 재현이 안 된다. 통로 안 keepout은 북쪽 끝 띠(y 0.75~)와
    남쪽 경계선뿐."""
    from fleet_gz_sim import arena_maps as am
    mask = am.keepout(am.FINE_M, V7)

    def inside(x, y):
        i, j = am.cell_of(x, y, am.FINE_M, V7)
        return mask[j][i]
    (ax, ay), (bx, by) = V7.node_pose("O_IN"), V7.node_pose("M_E")
    for n in range(201):
        x, y = ax + n / 200 * (bx - ax), ay + n / 200 * (by - ay)
        assert not any(inside(x, y + d) for d in (-0.06, 0.0, 0.06)), (x, y)
    x0, y0, x1, y1 = sp.CORRIDOR_GATES[V7.name]
    blocked = {round(y0 + n * 0.005, 3) for n in range(round((y1 - y0) / 0.005) + 1)
               for x in (x0, (x0 + x1) / 2, x1 - 1e-6) if inside(x, y0 + n * 0.005)}
    # 남쪽 끝 0.405는 x 0.99~ 건넘 틈 keepout_extra의 경계(진열대 면과 같은 높이)다
    assert blocked and all(y >= 0.75 or y <= y0 + 1e-9 for y in blocked), sorted(blocked)


def test_through_gate_decimate_and_min_distance():
    gate = (0.90, 0.405, 1.00, 0.77)
    assert sp.through_gate([(0.5, 0.6), (0.95, 0.5), (1.6, 0.55)], gate)
    assert not sp.through_gate([(0.5, 0.6), (0.95, 0.2), (1.6, 0.55)], gate)  # 남쪽 통로
    assert not sp.through_gate([], gate)
    line = [(0.01 * i, 0.0) for i in range(31)]
    assert sp.decimate(line, 0.05) == [line[0], line[5], line[10], line[15], line[20],
                                       line[25], line[30]]
    assert sp.decimate([(0.0, 0.0), (0.01, 0.0)], 0.05) == [(0.0, 0.0), (0.01, 0.0)]
    assert sp.decimate([], 0.05) == []
    assert sp.min_distance([(0.0, 0.0), (1.0, 0.0)], (1.0, 0.3)) == pytest.approx(0.3)
    assert sp.min_distance([], (0.0, 0.0)) is None
