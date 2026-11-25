"""2단계 주행 계획: 구역 지도의 연결로 만들어지는지와 지표 계산. ROS 없이 돈다."""
import math

import pytest

pytest.importorskip("fleet_core")
from fleet_core.fleet_map import FOLLOW_PATH, NAV_GOAL, RETREAT, load_fleet_map  # noqa: E402
from fleet_gz_sim import stage2_plan as sp  # noqa: E402

SITE = load_fleet_map("d2")


@pytest.mark.parametrize("plan", sorted(sp.PLANS))
def test_every_plan_is_made_of_its_layout_edges_and_ends_where_it_started(plan):
    steps = sp.build(load_fleet_map(sp.plan_layout(plan)), plan, laps=2)
    assert len(steps) == 2 * len(sp.PLANS[plan])
    assert steps[0].nodes[0] == sp.start_node(plan) == steps[-1].nodes[-1]
    for one, two in zip(steps, steps[1:], strict=False):
        assert one.nodes[-1] == two.nodes[0]


def test_stage2_covers_every_segment_type_of_the_acceptance():
    """완료 기준 구간: 마당 goal, 도크 진입 둘, Retreat 둘, 급행 양방향, 갈림목 내려오기 둘."""
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
    assert to_n_w.target == pytest.approx((0.35, 0.65, math.pi / 2))  # N_W -> X_W는 북쪽


def test_a_retreat_path_keeps_its_bend_and_faces_the_way_the_robot_looks():
    step = sp.make_step(SITE, RETREAT, ("W", "N_W"))
    # D2 후진 꺾는 점 (0.30, 0.89): 마지막 조각은 N_W로 비스듬히 내려간다
    assert [p[:2] for p in step.poses] == [pytest.approx((0.18, 0.89)), pytest.approx((0.30, 0.89)),
                                           pytest.approx((0.35, 0.65))]
    assert abs(abs(step.poses[0][2]) - math.pi) < 1e-9  # 서쪽을 보며 동쪽으로 달림
    # 비스듬한 길을 올려다보며(북서) 내려간다
    assert step.poses[-1][2] == pytest.approx(math.atan2(0.24, -0.05))


@pytest.mark.parametrize("kind, nodes", [
    (FOLLOW_PATH, ("W", "X_W")),  # Retreat 이음이지 고정 경로가 아님
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


# ---- 가운데 통로 재현(v7) ----
V7 = load_fleet_map("v7")


def test_without_layout_and_plan_the_check_drives_v7_by_its_corridor_plan():
    assert sp.pick_layout_plan(None, None) == ("v7", "corridor")
    # 계획만 주면 그 계획의 배치, 배치만 주면 그 배치의 첫 계획(없으면 stage2)
    assert sp.pick_layout_plan(None, "stage2") == ("d2", "stage2")
    assert sp.pick_layout_plan(None, "cross") == ("d2", "cross")
    assert sp.pick_layout_plan(None, "gap") == ("v7", "gap")
    assert sp.pick_layout_plan("d2", None) == ("d2", "stage2")
    assert sp.pick_layout_plan("v7", None) == ("v7", "corridor")
    assert sp.pick_layout_plan("c", "stage2") == ("c", "stage2")
    layout, plan = sp.pick_layout_plan(None, None)
    assert sp.build(load_fleet_map(layout), plan)


def test_corridor_plan_goes_o_in_to_m_e_and_back_by_nav_goal_on_v7():
    assert sp.plan_layout("corridor") == "v7" and sp.plan_layout("stage2") == "d2"
    assert sp.start_node("corridor") == "O_IN"
    steps = sp.build(V7, "corridor", laps=3)
    assert [s.label for s in steps] == ["nav_goal O_IN>M_E", "nav_goal M_E>O_IN"] * 3
    # M_E는 yaw도 이어지는 고정 경로도 없어 시작→끝 방향, O_IN은 지점 yaw 110도
    assert steps[0].target == pytest.approx((1.645, 0.55, math.atan2(-0.075, 1.155)))
    assert steps[1].target == pytest.approx((0.49, 0.625, math.radians(110)))


def test_corridor_plan_is_refused_on_another_layout():
    with pytest.raises(sp.PlanError, match="v7"):
        sp.build(SITE, "corridor")


def test_the_corridor_edge_and_the_robot_body_along_it_are_outside_the_keepout_mask():
    """keepout 마스크가 가운데 통로를 막으면 재현이 안 된다. 통로 안 keepout은 북쪽 끝 띠(y 0.75~)뿐이다."""
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
    # 남쪽 끝 0.404는 틈 keepout_extra가 있는 지도에서만 막힌다. 북쪽 띠는 급행 구역 아래 끝에서 몸 반폭 0.06 아래부터다(1 cm 칸)
    north = V7.zones["EXPRESS"].rects[0][1] - 0.06
    assert blocked and all(y >= north - 0.01 or y <= y0 + 1e-9 for y in blocked), sorted(blocked)


def test_through_gate_decimate_and_min_distance():
    gate = (0.90, 0.3875, 1.00, 0.735)
    assert sp.through_gate([(0.5, 0.6), (0.95, 0.5), (1.6, 0.55)], gate)
    assert sp.through_gate([(1.6, 0.55), (0.95, 0.5), (0.93, 0.6), (0.5, 0.6)], gate)
    assert not sp.through_gate([(0.5, 0.6), (0.95, 0.2), (1.6, 0.55)], gate)  # 남쪽 통로
    assert not sp.through_gate([(0.5, 0.6), (0.95, 0.5), (0.6, 0.6)], gate)  # 들어갔다 되돌아옴
    assert not sp.through_gate([(0.5, 0.6), (0.95, 0.5)], gate)  # 안에서 멈춤
    assert not sp.through_gate([], gate)
    # axis=1: 남북으로 지나기(진열대 사이 틈)
    gap = (0.99, 0.3475, 1.25, 0.3875)
    assert sp.through_gate([(1.12, 0.55), (1.12, 0.385), (1.12, 0.24)], gap, axis=1)
    assert not sp.through_gate([(0.6, 0.55), (0.6, 0.385), (0.6, 0.24)], gap, axis=1)
    line = [(0.01 * i, 0.0) for i in range(31)]
    assert sp.decimate(line, 0.05) == [line[0], line[5], line[10], line[15], line[20],
                                       line[25], line[30]]
    assert sp.decimate([(0.0, 0.0), (0.01, 0.0)], 0.05) == [(0.0, 0.0), (0.01, 0.0)]
    assert sp.decimate([], 0.05) == []
    assert sp.min_distance([(0.0, 0.0), (1.0, 0.0)], (1.0, 0.3)) == pytest.approx(0.3)
    assert sp.min_distance([], (0.0, 0.0)) is None


# ---- 남쪽 진열대 사이 건넘 틈(v7) ----
def test_gap_plan_crosses_the_gap_both_ways_between_two_plan_points():
    assert sp.plan_layout("gap") == "v7" and sp.start_node("gap") == "M_E"
    steps = sp.build(V7, "gap", laps=2)
    assert [s.label for s in steps] == ["nav_goal M_E>GAP_N", "nav_goal GAP_N>GAP_S",
                                        "nav_goal GAP_S>GAP_N", "nav_goal GAP_N>M_E"] * 2
    north, south = sp.PLAN_POINTS["gap"]["GAP_N"], sp.PLAN_POINTS["gap"]["GAP_S"]
    assert [s.target for s in steps[:4]] == [
        pytest.approx((*north, math.pi)), pytest.approx((*south, -math.pi / 2)),
        pytest.approx((*north, math.pi / 2)), pytest.approx((1.645, 0.55, 0.0))]
    gate = sp.GAP_GATES[V7.name]
    assert sp.through_gate([north, south], gate, axis=1)
    assert sp.through_gate([south, north], gate, axis=1)
    with pytest.raises(sp.PlanError, match="v7"):
        sp.build(SITE, "gap")


def test_gap_points_can_be_moved_and_a_point_needs_a_nav_goal():
    steps = sp.build(V7, "gap", points={"GAP_N": (1.10, 0.56)})
    assert steps[0].target[:2] == pytest.approx((1.10, 0.56))
    with pytest.raises(sp.PlanError, match="nav_goal"):
        sp.make_step(V7, FOLLOW_PATH, ("M_E", "GAP_N"), sp.PLAN_POINTS["gap"])
    # 계획 전용 점만 바꾼다: 지도 지점, 오타, 다른 계획, 유한하지 않은 좌표는 거부
    for plan, points in (("gap", {"M_E": (1.6, 0.5)}), ("gap", {"GAP_X": (1.1, 0.5)}),
                         ("corridor", {"GAP_N": (1.1, 0.5)}),
                         ("gap", {"GAP_N": (math.nan, 0.55)}), ("gap", {"GAP_N": 1.1}),
                         ("gap", {"GAP_N": (1.1, "x")}), ("gap", {"GAP_N": {1.1: 0, 0.5: 1}}),
                         ("gap", {"GAP_N": (True, 0.5)}), ("gap", {"GAP_N": (10 ** 400, 0.5)}),
                         ("gap", {"GAP_N": (10 ** 5000, 0.5)})):  # repr도 못 하는 큰 수
        with pytest.raises(sp.PlanError, match="point"):
            sp.build(V7, plan, points=points)
    # 정수와 numpy 수도 받고, 목표는 float로 보낸다(Pose2D는 float만 받는다)
    np = pytest.importorskip("numpy")
    for xy in ((1, 0), (np.float32(1.1), np.int64(0))):
        target = sp.build(V7, "gap", points={"GAP_N": xy})[0].target
        assert all(type(v) is float for v in target), target


def test_gap_points_sit_clear_of_shelves_walls_and_keepout():
    """끝점이 진열대·벽에서 내접 0.09보다 멀고 keepout 마스크 밖이다(goal이 거절되지 않게)."""
    from fleet_gz_sim import arena_maps as am
    mask = am.keepout(am.FINE_M, V7)
    for x, y in sp.PLAN_POINTS["gap"].values():
        i, j = am.cell_of(x, y, am.FINE_M, V7)
        assert not mask[j][i], (x, y)
        for b in am.obstacles(V7):
            dx, dy = max(b.x0 - x, 0, x - b.x1), max(b.y0 - y, 0, y - b.y1)
            assert math.hypot(dx, dy) > 0.09, (x, y, b)
        assert min(x, y, 1.98 - x, V7.bounds()[3] - y) > 0.09


def test_body_distance_and_pose_error():
    square = [(0.06, 0.06), (0.06, -0.06), (-0.06, -0.06), (-0.06, 0.06)]
    assert sp.body_distance((0.0, 0.0, 0.0), square, (0.16, 0.0)) == pytest.approx(0.10)
    assert sp.body_distance((0.0, 0.0, 0.0), square, (0.03, 0.02)) == 0.0  # 몸 안
    # 45도 돌면 모서리가 0.0849까지 나온다
    assert sp.body_distance((0.0, 0.0, math.pi / 4), square, (0.16, 0.0)) == \
        pytest.approx(0.16 - 0.06 * math.sqrt(2))
    truth = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0)]
    # 시각이 가장 가까운 실제 위치와 비교한다
    assert sp.max_pose_error([(0.9, 1.0, 0.1), (2.1, 2.0, 0.0)], truth) == pytest.approx(0.1)
    assert sp.max_pose_error([], truth) is None
    assert sp.max_pose_error([(0.0, 0.0, 0.0)], []) is None
    # 실제 위치와 시각이 max_dt_s보다 먼 추정은 버린다(끊긴 기록, 늦게 온 메시지)
    assert sp.max_pose_error([(5.0, 9.0, 9.0), (1.0, 1.0, 0.05)], truth, max_dt_s=0.2) == \
        pytest.approx(0.05)
    assert sp.max_pose_error([(5.0, 9.0, 9.0)], truth, max_dt_s=0.2) is None
    assert sp.parse_footprint("[[0.06, 0.06], [0.06, -0.06], [-0.06, -0.06]]") == \
        [(0.06, 0.06), (0.06, -0.06), (-0.06, -0.06)]
