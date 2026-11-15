"""관제 화면: 구역(종류별), 고정 경로, 지점, 로봇, 쥔 구역을 마커 명세로 바꾸는지와 RViz 설정."""
from dataclasses import dataclass, field
from pathlib import Path

import pytest
import yaml

pytest.importorskip("fleet_core")
from fleet_core.fleet_map import DOCK, LANE, SHARED, load_fleet_map  # noqa: E402
from fleet_gz_sim import fleet_view as fv  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
SITE = load_fleet_map("d2")


@dataclass
class Status:
    robot_id: str
    fsm_state: str = "IDLE"
    task_type: str = ""
    order_id: str = ""
    fault: str = "NONE"
    fault_reason: str = ""
    x: float = 0.0
    y: float = 0.0
    base: list = field(default_factory=list)
    horizon: list = field(default_factory=list)


@dataclass
class Zone:
    zone_id: str
    holder: str = ""
    queue: list = field(default_factory=list)


def by_ns(specs, ns):
    return [s for s in specs if s.ns == ns]


# ---- 지도 ----------------------------------------------------------------------------------


@pytest.mark.parametrize("layout", ["d2", "d", "c"])
def test_zones_are_drawn_by_kind(layout):
    site = load_fleet_map(layout)
    specs = fv.map_specs(site)
    floors = by_ns(specs, "zones")
    assert len(floors) == sum(len(z.rects) for z in site.zones.values())
    colors = {s.color for s in floors}
    assert colors <= {fv.ZONE_COLORS[k] for k in (SHARED, LANE, DOCK)}
    assert len({fv.ZONE_COLORS[k] for k in (SHARED, LANE, DOCK)}) == 3
    labels = {s.text for s in by_ns(specs, "zone_labels")}
    assert labels == set(site.zones)
    ids = [(s.ns, s.id) for s in specs]
    assert len(ids) == len(set(ids))


def test_the_express_floor_is_lane_blue_and_the_docks_amber():
    floors = {(round(s.pose[0], 3), round(s.pose[1], 3)): s
              for s in by_ns(fv.map_specs(SITE), "zones")}
    express = floors[(0.99, 0.89)]
    assert express.color == fv.ZONE_COLORS[LANE]
    assert express.scale[:2] == (pytest.approx(0.98), pytest.approx(0.20))
    work = floors[(0.15, 0.89)]
    assert work.color == fv.ZONE_COLORS[DOCK]


def test_fixed_paths_are_thin_lines_and_nodes_get_dots_and_labels():
    specs = fv.map_specs(SITE)
    [paths] = by_ns(specs, "paths")
    assert paths.kind == "LINE_LIST" and paths.scale[0] <= 0.02
    points = {(round(x, 3), round(y, 3)) for x, y in paths.points}
    assert {(0.35, 0.65), (0.35, 0.89), (0.18, 0.89), (1.63, 0.89)} <= points
    assert (0.13, 0.15) not in points  # parking is reached by Nav2 goals, no fixed path
    [dots] = by_ns(specs, "nodes")
    assert len(dots.points) == len(SITE.nodes)
    assert {s.text for s in by_ns(specs, "node_labels")} == set(SITE.nodes)


# ---- 쥔 구역 -------------------------------------------------------------------------------


def test_a_zone_held_by_one_robot_is_covered_in_its_color():
    view = fv.FleetView(SITE)
    view.set_zones([Zone("J_W", "pinky2"), Zone("EXPRESS", "")])
    specs = view.specs(now=0.0)
    [held] = by_ns(specs, "zone_held")
    assert held.color[:3] == fv.ROBOT_COLORS["pinky2"][:3]
    assert held.pose[:2] == (pytest.approx(0.40), pytest.approx(0.89))
    [text] = by_ns(specs, "zone_holder")
    assert text.text == "J_W: pinky2"


def test_a_zone_held_by_several_robots_gets_an_outline_and_their_names():
    view = fv.FleetView(SITE)
    view.set_zones([Zone("EXPRESS", "pinky1,pinky3")])
    specs = view.specs(now=0.0)
    assert not by_ns(specs, "zone_held")
    [outline] = by_ns(specs, "zone_shared_by")
    assert outline.kind == "LINE_LIST"
    assert len(outline.points) == 8 * len(SITE.zones["EXPRESS"].rects)
    [text] = by_ns(specs, "zone_holder")
    assert text.text == "EXPRESS: pinky1, pinky3"
    assert fv.holders_of("pinky1,pinky3") == ["pinky1", "pinky3"] and fv.holders_of("") == []


@pytest.mark.parametrize("holder, names",
                         [("pinky1", "pinky1"), ("pinky1,pinky3", "pinky1, pinky3")])
def test_the_shared_floor_is_only_named_never_covered_or_outlined(holder, names):
    """2026-11-16: 한 대가 HALL을 쥐면 바닥 전부가 그 로봇 색으로 덮여 다 잠긴 것처럼 보였다.
    HALL은 세 대가 함께 쓰는 구역이라 덮지 않고 쥔 로봇의 이름만 적는다."""
    assert SITE.zones["HALL"].kind == SHARED
    view = fv.FleetView(SITE)
    view.set_zones([Zone("HALL", holder)])
    specs = view.specs(now=0.0)
    assert not by_ns(specs, "zone_held") and not by_ns(specs, "zone_shared_by")
    [text] = by_ns(specs, "zone_holder")
    assert text.text == f"HALL: {names}"


def test_a_waiting_queue_is_listed_and_a_free_zone_draws_nothing():
    view = fv.FleetView(SITE)
    view.set_zones([Zone("OUT_DOCK", "", ["pinky1"]), Zone("J_E")])
    specs = view.specs(now=0.0)
    [queue] = by_ns(specs, "zone_queue")
    assert queue.text == "OUT_DOCK wait: pinky1"
    assert not by_ns(specs, "zone_held") and not by_ns(specs, "zone_holder")


def test_an_unknown_zone_id_is_skipped():
    view = fv.FleetView(SITE)
    view.set_zones([Zone("c12_17", "pinky1")])
    assert not by_ns(view.specs(now=0.0), "zone_held")


def test_each_robot_shows_its_goal_node_or_station_dock_with_a_line():
    view = fv.FleetView(SITE)
    view.set_pose("pinky1", 0.13, 0.15, 0.0, now=0.0)
    view.set_goal("pinky1", "N_W")
    view.set_goal("pinky2", "OUTBOUND")
    view.set_goal("pinky3", "")
    specs = view.specs(now=0.1)
    goals = {s.text: s.pose[:2] for s in by_ns(specs, "goal_text")}
    assert goals == {"pinky1 -> N_W": (0.35, 0.65), "pinky2 -> OUTBOUND": (1.80, 0.89)}
    [line] = by_ns(specs, "goal_line")
    assert line.points == ((0.13, 0.15), (0.35, 0.65))


# ---- base와 horizon -------------------------------------------------------------------------


def _pairs(points):
    return [(points[i], points[i + 1]) for i in range(0, len(points), 2)]


def test_base_is_a_solid_line_from_the_robot_and_horizon_a_lighter_dashed_one():
    """RobotStatus.base/horizon(order-model 95b5215): 쥔 길은 실선, 아직 안 받은 길은 옅은 점선."""
    view = fv.FleetView(SITE)
    view.set_statuses([Status("pinky1", "MOVING", x=0.35, y=0.65, base=["X_W", "W"],
                              horizon=["X_E"])])
    specs = view.specs(now=0.0)
    [base] = by_ns(specs, "way_base")
    [horizon] = by_ns(specs, "way_horizon")
    assert base.kind == horizon.kind == "LINE_LIST"
    assert _pairs(base.points) == [((0.35, 0.65), (0.35, 0.89)), ((0.35, 0.89), (0.18, 0.89))]
    assert base.color[:3] == fv.robot_color("pinky1")[:3] and base.color[3] == 1.0
    assert horizon.color[3] < base.color[3] and horizon.scale[0] < base.scale[0]
    # the horizon starts where the base ends and is cut into dashes along W -> X_E
    dashes = _pairs(horizon.points)
    assert dashes[0][0] == pytest.approx((0.18, 0.89))
    assert len(dashes) > 3
    assert all(abs(a[1] - 0.89) < 1e-9 and abs(b[1] - 0.89) < 1e-9 for a, b in dashes)
    covered = sum(abs(b[0] - a[0]) for a, b in dashes)
    assert covered < 0.8 * (1.63 - 0.18)  # gaps between the dashes


def test_no_way_ahead_draws_nothing_and_a_horizon_alone_starts_at_the_robot():
    view = fv.FleetView(SITE)
    view.set_statuses([Status("pinky1"), Status("pinky2", x=0.13, y=0.15, horizon=["N_W"])])
    specs = view.specs(now=0.0)
    assert not by_ns(specs, "way_base")
    [horizon] = by_ns(specs, "way_horizon")
    assert horizon.points[0] == pytest.approx((0.13, 0.15))


def test_the_way_starts_at_the_drawn_robot_when_a_pose_is_known():
    """로봇 몸은 set_pose(기본 Gazebo 실제 위치)로 그리므로 앞길도 거기서 시작한다."""
    view = fv.FleetView(SITE)
    view.set_statuses([Status("pinky1", x=0.40, y=0.60, base=["X_W"])])
    view.set_pose("pinky1", 0.41, 0.66, 1.57, now=0.0)
    [base] = by_ns(view.specs(now=0.0), "way_base")
    assert base.points[0] == (0.41, 0.66)


V7 = load_fleet_map("v7")


def _corners(view, ns="way_base"):
    [line] = by_ns(view.specs(now=0.0), ns)
    return [a for a, _ in _pairs(line.points)] + [line.points[-1]]


def test_a_node_passed_inside_one_nav_goal_does_not_bend_the_way():
    """2026-11-16: 관제의 길 E-1 -> E-3 -> Q2는 Nav2 goal 하나(Q2)로 합쳐 나가는데, 선이 E-3에서
    꺾여 로봇이 그 지점을 꼭 지나는 것처럼 보였다. 로봇이 받은 goal의 끝까지 곧게 잇는다."""
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", "MOVING", x=0.12, y=0.24, base=["E-3", "Q2"],
                              horizon=["Q1", "W"])])
    view.set_goal("pinky1", "Q2")
    assert _corners(view) == [(0.12, 0.24), V7.node_pose("Q2")]
    [horizon] = by_ns(view.specs(now=0.0), "way_horizon")
    assert horizon.points[0] == pytest.approx(V7.node_pose("Q2"))


@pytest.mark.parametrize("goal", ["E-3", "Q2", ""])
@pytest.mark.parametrize("y", [0.10, 0.17, 0.23])
def test_a_bay_door_reached_on_a_fixed_path_stays_on_the_way(goal, y):
    """Codex 검수(2026-11-16): 상태와 goal은 다른 토픽이라 어긋날 수 있다. 칸을 나오는 고정 경로
    위에 있는 로봇의 칸 입구는 보고된 goal이 무엇이든 남긴다."""
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", "MOVING", x=0.64, y=y, base=["E-3", "Q2"])])
    view.set_goal("pinky1", goal)
    assert _corners(view) == [(0.64, y), V7.node_pose("E-3"), V7.node_pose("Q2")]


def test_a_node_the_robot_is_told_to_stop_at_stays_on_the_way():
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", "MOVING", x=0.15, y=0.62, base=["E-3", "Q2"])])
    view.set_goal("pinky1", "E-3")
    assert _corners(view) == [(0.15, 0.62), V7.node_pose("E-3"), V7.node_pose("Q2")]


def test_the_end_of_the_granted_way_is_never_merged_into_the_horizon():
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", "MOVING", x=0.15, y=0.62, base=["E-3"],
                              horizon=["Q2"])])
    view.set_goal("pinky1", "Q2")
    assert _corners(view) == [(0.15, 0.62), V7.node_pose("E-3")]
    view.set_statuses([Status("pinky1", "MOVING", x=0.15, y=0.70, base=["O_OUT", "E-3"],
                              horizon=["Q2"])])
    assert _corners(view) == [(0.15, 0.70), V7.node_pose("O_OUT"), V7.node_pose("E-3")]


def test_an_unknown_link_further_on_does_not_undo_a_merge_before_it():
    assert V7.link("Q2", "P-1") is None
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", "MOVING", x=0.15, y=0.70,
                              base=["O_OUT", "E-3", "Q2", "P-1"])])
    assert _corners(view) == [(0.15, 0.70), V7.node_pose("O_OUT"), V7.node_pose("Q2"),
                              V7.node_pose("P-1")]


def test_a_later_nav_goal_is_drawn_straight_to_its_end():
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", "MOVING", x=0.15, y=0.70,
                              base=["O_OUT", "E-3", "Q2"])])
    view.set_goal("pinky1", "O_OUT")
    assert _corners(view) == [(0.15, 0.70), V7.node_pose("O_OUT"), V7.node_pose("Q2")]


def test_a_bay_door_before_backing_in_stays_on_the_way():
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", "MOVING", x=0.15, y=0.62, base=["E-3", "P-3"])])
    view.set_goal("pinky1", "E-3")
    assert _corners(view) == [(0.15, 0.62), V7.node_pose("E-3"), V7.node_pose("P-3")]


def test_a_goal_already_sent_stays_on_the_way_when_the_granted_way_grows():
    """Codex 검수(2026-11-16): 관제는 goal을 보낼 때의 허가 끝까지만 합친다. 그 뒤에 허가가 늘어도
    로봇은 받은 goal의 끝에 서므로, 보고된 goal 지점은 선에 남는다."""
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", "MOVING", x=0.15, y=0.62, base=["E-1", "E-2", "E-3"])])
    assert _corners(view) == [(0.15, 0.62), V7.node_pose("E-3")]
    view.set_goal("pinky1", "E-2")
    assert _corners(view) == [(0.15, 0.62), V7.node_pose("E-2"), V7.node_pose("E-3")]
    view.set_statuses([Status("pinky1", "MOVING", x=0.15, y=0.62, base=["E-1", "E-2"],
                              horizon=["E-3"])])
    assert _corners(view) == [(0.15, 0.62), V7.node_pose("E-2")]


def test_a_waiting_robot_without_a_goal_shows_the_merged_way_from_its_node():
    view = fv.FleetView(V7)
    view.set_statuses([Status("pinky1", x=0.12, y=0.24, horizon=["E-3", "Q2"])])
    [horizon] = by_ns(view.specs(now=0.0), "way_horizon")
    slope = (0.16 - 0.24) / (1.66 - 0.12)
    assert horizon.points[0] == pytest.approx((0.12, 0.24))
    assert all(y == pytest.approx(0.24 + (x - 0.12) * slope) for x, y in horizon.points)


def test_the_plan_nav2_made_is_drawn_while_it_is_fresh():
    """Nav2가 실제로 계획한 경로(로봇의 plan 토픽). 넓은 구역 goal 동안 1초마다 새로 오고,
    끊기면 지운다: 고정 경로를 달리는 동안 옛 계획이 남아 보이지 않게."""
    view = fv.FleetView(V7)
    view.set_plan("pinky2", [(0.2, 0.3), (0.9, 0.3), (1.66, 0.16)], now=10.0)
    [plan] = by_ns(view.specs(now=11.0), "nav_plan")
    assert plan.kind == "LINE_STRIP" and plan.points == ((0.2, 0.3), (0.9, 0.3), (1.66, 0.16))
    assert plan.color[:3] == fv.robot_color("pinky2")[:3]
    assert not by_ns(view.specs(now=10.0 + fv.PLAN_FRESH_S + 0.1), "nav_plan")
    view.set_plan("pinky2", [(0.2, 0.3)], now=20.0)
    assert not by_ns(view.specs(now=20.0), "nav_plan")


def test_a_plan_that_went_stale_does_not_come_back_when_the_clock_rewinds():
    view = fv.FleetView(V7)
    view.set_plan("pinky2", [(0.2, 0.3), (0.9, 0.3)], now=10.0)
    assert not by_ns(view.specs(now=13.0), "nav_plan")
    assert not by_ns(view.specs(now=11.0), "nav_plan")
    view.set_plan("pinky2", [(0.2, 0.3), (0.9, 0.3)], now=10.0)
    assert not by_ns(view.specs(now=9.0), "nav_plan")
    assert not by_ns(view.specs(now=10.5), "nav_plan")


def test_a_status_without_base_fields_and_an_unknown_node_are_skipped():
    @dataclass
    class Old:
        robot_id: str = "pinky1"
        fsm_state: str = "IDLE"
    view = fv.FleetView(SITE)
    view.set_statuses([Old(), Status("pinky2", base=["nowhere", "W"])])
    [base] = by_ns(view.specs(now=0.0), "way_base")
    assert _pairs(base.points) == [((0.0, 0.0), (0.18, 0.89))]


# ---- 로봇 ----------------------------------------------------------------------------------


def test_robot_label_reads_id_state_task_and_order():
    text, color = fv.robot_label(Status("pinky1", "MOVING", "TO_WORK", "R-003"), stale=False)
    assert text == "pinky1  MOVING  TO_WORK R-003"
    assert color == fv.ROBOT_COLORS["pinky1"]


def test_blocked_or_faulted_is_red_and_recovering_amber():
    assert fv.robot_label(Status("pinky2", "BLOCKED"), stale=False)[1] == fv.RED
    text, color = fv.robot_label(Status("pinky3", fault="ERROR", fault_reason="BLOCKED"), False)
    assert text == "pinky3  IDLE  ERROR:BLOCKED" and color == fv.RED
    assert fv.robot_label(Status("pinky1", "RECOVERING"), stale=False)[1] == fv.AMBER


def test_robot_body_arrow_and_label_sit_at_the_pose_and_go_gray_when_stale():
    view = fv.FleetView(SITE)
    view.set_statuses([Status("pinky1", "MOVING")])
    view.set_pose("pinky1", 1.2, 0.3, 1.57, now=10.0)
    [body] = by_ns(view.specs(now=10.5), "robot_body")
    [arrow] = by_ns(view.specs(now=10.5), "robot_heading")
    assert body.pose[:2] == (1.2, 0.3) and arrow.pose[3] == 1.57
    assert body.color == fv.ROBOT_COLORS["pinky1"]
    [old] = by_ns(view.specs(now=13.1), "robot_body")
    [label] = by_ns(view.specs(now=13.1), "robot_label")
    assert old.color == fv.GRAY and "STALE" in label.text


def test_robot_without_a_pose_is_not_drawn():
    view = fv.FleetView(SITE)
    view.set_statuses([Status("pinky1")])
    assert not by_ns(view.specs(now=1.0), "robot_body")


# ---- RViz 설정 -----------------------------------------------------------------------------


def test_rviz_config_parses_and_shows_the_marker_topics():
    config = yaml.safe_load((PKG / "rviz" / "fleet_view.rviz").read_text(encoding="utf-8"))
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "map"
    marker_topics = {d["Topic"]["Value"] for d in manager["Displays"]
                     if d["Class"].endswith("MarkerArray")}
    assert marker_topics == {"/fleet_view/markers", "/sim/obstacle_markers"}


def test_setup_installs_the_rviz_config_and_the_node():
    setup = (PKG / "setup.py").read_text(encoding="utf-8")
    assert "rviz/*.rviz" in setup
    assert "fleet_view = fleet_gz_sim.fleet_view_node:main" in setup
