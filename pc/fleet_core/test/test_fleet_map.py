"""구역 지도: 지점, 구역, 고정 경로, 연결, 스테이션을 읽고 검사한다."""
import copy
import math

import pytest
from fleet_core.fleet_map import FleetMap, FleetMapError, load_fleet_map, route

# 가로 2 m, 세로 1 m. 북쪽 띠가 도크(서) - 갈림목 - 급행 - 갈림목 - 도크(동), 나머지는 마당이다.
SMALL = {
    "name": "small",
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
    "nav": [{"nodes": ["N_W", "N_E", "W_Q", "O_Q", "P-1", "P-2"]}],
    "stations": {
        "WORK": {"dock": "W", "queue": ["W_Q"], "server": "workcell", "arms": ["arm1", "arm2"],
                 "action": "fill"},
        "OUTBOUND": {"dock": "O", "queue": ["O_Q"], "server": "arm3", "arms": ["arm3"],
                     "action": "swap"},
    },
    "parking": ["P-1", "P-2"],
}


def build(**changes) -> FleetMap:
    data = copy.deepcopy(SMALL)
    for key, value in changes.items():
        if value is None:
            data.pop(key)
        else:
            data[key] = value
    return FleetMap(data)


def edited(section: str, name: str, **fields):
    """SMALL의 한 항목만 바꾼 사본의 그 구획."""
    part = copy.deepcopy(SMALL[section])
    part[name] = {**part[name], **fields}
    return part


# ---- 읽은 것 --------------------------------------------------------------------------------


def test_a_node_knows_its_zone_kind_and_heading():
    site = build()
    dock = site.nodes["W"]
    assert (dock.x, dock.y, dock.zone, dock.kind) == (0.2, 0.9, "DOCK_W", "dock")
    assert dock.yaw == pytest.approx(math.pi)
    assert site.nodes["N_W"].zone == "HALL" and site.nodes["N_W"].yaw is None
    assert site.nodes["X_W"].zone == "J_W"


def test_a_zone_keeps_its_kind_and_how_many_robots_it_takes():
    site = build()
    assert site.zones["HALL"].kind == "shared" and site.zones["HALL"].capacity == 2
    assert site.zones["J_W"].kind == "dock" and site.zones["J_W"].capacity == 1
    assert site.zones["EXPRESS"].kind == "lane" and site.zones["EXPRESS"].capacity == 2


def test_only_docks_wait_spots_and_parking_are_places_to_wait():
    site = build()
    waiting = {name for name in site.nodes if site.can_wait(name)}
    assert waiting == {"W", "O", "W_Q", "O_Q", "P-1", "P-2"}


def test_a_fixed_path_becomes_one_follow_link_per_stretch():
    site = build()
    first, second = site.link("N_W", "X_W"), site.link("X_W", "W")
    assert (first.via, first.path, first.zones) == ("follow_path", "work_in", ("HALL", "J_W"))
    assert (second.via, second.zones) == ("follow_path", ("J_W", "DOCK_W"))
    assert first.points == ((0.4, 0.6), (0.4, 0.9))
    assert first.length == pytest.approx(0.3)
    assert site.link("W", "N_W") is None


def test_a_reverse_path_is_driven_as_a_retreat():
    site = build()
    out = site.link("W", "X_W")
    assert (out.via, out.path, out.zones) == ("retreat", "work_out", ("DOCK_W", "J_W"))


def test_a_path_may_bend_at_a_point_that_is_not_a_node():
    paths = edited("paths", "work_out", points=["W", [0.4, 0.9], "N_W"])
    site = build(paths=paths)
    out = site.link("W", "N_W")
    assert (out.via, out.zones) == ("retreat", ("DOCK_W", "J_W", "HALL"))
    assert out.points == ((0.2, 0.9), (0.4, 0.9), (0.4, 0.6))
    assert out.length == pytest.approx(0.5)
    assert site.link("W", "X_W") is None


def test_a_bend_is_a_pair_of_numbers_between_two_nodes():
    refused("bend", paths=edited("paths", "work_out", points=["W", [0.4], "N_W"]))
    refused("node", paths=edited("paths", "work_out", points=[[0.3, 0.9], "W", "N_W"]))


def test_room_beside_the_shelves_is_checked_on_every_piece_of_a_bent_path():
    near_shelf = ["W", [0.47, 0.9], [0.47, 0.6], "N_W"]
    refused("shelf", paths=edited("paths", "work_out", points=near_shelf))


def test_a_line_that_goes_round_a_shelf_end_too_close_is_a_warning():
    site = build(turn_radius_m=0.0)  # 곧은 선만 본다. 호는 아래 시험
    near = [w for w in site.warnings if "end of the shelf" in w]
    assert any("work_in from N_W to X_W" in w for w in near)
    assert any("j_e_down from X_E to N_E" in w for w in near)
    assert not any("express" in w for w in near), "the express runs beside the shelf, not round it"
    assert build(shelf_end_clear_m=0.10, turn_radius_m=0.0).warnings == site.warnings[len(near):]


def test_a_retreat_that_ends_inside_a_narrow_strip_is_a_warning():
    site = build()
    assert any("work_out ends at X_W" in w and "turn" in w for w in site.warnings)
    paths = edited("paths", "work_out", points=["W", [0.4, 0.9], "N_W"])
    assert not any("work_out ends" in w for w in build(paths=paths).warnings)


def test_a_fixed_line_that_leaves_a_zone_and_comes_back_into_it_is_refused():
    """구역 쥐기는 한 연결이 구역을 한 번 지난다고 본다(Codex 검토 1): 다시 들어오면 받지 않는다."""
    refused("leaves DOCK_W and comes back into it",
            paths=edited("paths", "work_out", points=["W", [0.4, 0.9], [0.25, 0.85], "X_W"]))


def test_a_fixed_line_that_turns_back_or_crosses_itself_is_refused():
    """진행 자리는 가장 가까운 조각에 내린다(Codex 검토 2): 되돌아가거나 스스로 건너는 선은
    받지 않는다."""
    refused("turns back", paths=edited("paths", "work_out",
                                       points=["W", [0.45, 0.9], [0.35, 0.92], "N_W"]))
    refused("crosses itself", paths=edited("paths", "work_in",
                                           points=["N_W", [0.4, 0.95], [0.45, 0.95], [0.45, 0.85],
                                                   [0.35, 0.85], [0.35, 0.9], "X_W", "W"]))
    assert build(paths=edited("paths", "work_out", points=["W", [0.4, 0.9], "N_W"]))


def test_the_shelf_end_check_measures_the_arc_the_robot_drives_at_a_corner():
    """핑키는 꺾이는 곳을 turn_radius_m 호로 돈다(Codex 검토 4). 진열대 끝 밖에 놓인 호의 점도
    잰다."""
    site = build()
    corner = [w for w in site.warnings if "corner" in w]
    # 급행에서 갈림목으로 내려가는 호의 가운데는 진열대 모서리 (0.5, 0.8)이고 반지름은 0.10이다
    assert any("express from X_E to X_W, then j_w_down at X_W" in w and "0.100 m" in w
               for w in corner), corner
    assert not any("corner" in w for w in build(turn_radius_m=0.0).warnings)


def test_corner_arc_on_a_first_or_last_piece_may_use_the_whole_piece():
    """핑키(geometry.rounded)는 선의 첫 선분과 마지막 선분에서는 호가 그 선분을 다 쓸 수 있다.
    D2 work_out W -> (0.35, 0.89) -> N_W: 첫 선분이 0.17 m라 반으로 줄인 호는 접점 거리 0.085 m,
    다 쓰는 호는 0.10 m다(Codex 검토 2-1 둘째 차례). 90° 호의 가운데는 꼭짓점에서 접점 거리 x
    (sqrt 2 - 1)."""
    from fleet_core.fleet_map.checks import corner_arc
    a, b, c = (0.18, 0.89), (0.35, 0.89), (0.35, 0.65)
    halved = corner_arc(a, b, c, 0.10)
    whole = corner_arc(a, b, c, 0.10, halve=False)
    assert math.dist(halved[9], b) == pytest.approx(0.085 * (math.sqrt(2) - 1))
    assert math.dist(whole[9], b) == pytest.approx(0.10 * (math.sqrt(2) - 1))


def test_the_shelf_end_check_warns_with_the_arc_closer_to_the_shelf():
    """이어 달릴 때는 어느 선분이 처음과 끝인지가 명령마다 다르니 두 호를 다 재고 진열대에 더
    가까운 쪽으로 경고한다. 짧은 첫 선분(0.17 m) 뒤 진열대 끝 쪽으로 꺾는 선: 반 호는 경고 밖,
    다 쓰는 호는 경고 안."""
    from fleet_core.fleet_map.checks import corner_arc, shelf_end_warnings
    from fleet_core.fleet_map.geometry import point_rect_distance

    class Link:
        via, path, a, b = "follow_path", "probe", "A", "C"
        points = ((0.18, 0.89), (0.35, 0.89), (0.35, 0.65))

    class Site:
        nodes = {}
        shelves = ((0.20, 0.775, 0.30, 0.79),)

        def links(self):
            return [Link]

        def links_from(self, _node):
            return []

    shelf = Site.shelves[0]
    halved = min(point_rect_distance(shelf, *p) for p in corner_arc(*Link.points, 0.10))
    whole = min(point_rect_distance(shelf, *p)
                for p in corner_arc(*Link.points, 0.10, halve=False))
    assert whole < halved
    clear = (whole + halved) / 2
    warnings = [w for w in shelf_end_warnings(Site(), clear, turn_radius=0.10) if "corner" in w]
    assert len(warnings) == 1 and f"{whole:.3f} m" in warnings[0]


def test_a_two_way_path_gives_a_link_each_way_and_names_the_lane_it_crosses():
    site = build()
    east, west = site.link("X_W", "X_E"), site.link("X_E", "X_W")
    assert east.zones == ("J_W", "EXPRESS", "J_E") and west.zones == ("J_E", "EXPRESS", "J_W")
    assert east.lane == west.lane == "EXPRESS"
    assert site.link("N_W", "X_W").lane == ""


def test_nodes_of_one_nav_group_reach_each_other_by_nav_goal():
    site = build()
    hop = site.link("P-1", "O_Q")
    assert (hop.via, hop.path, hop.zones) == ("nav_goal", "", ("HALL",))
    assert hop.length == pytest.approx(math.hypot(1.6, 0.35))
    assert site.link("O_Q", "P-1").via == "nav_goal"
    assert {link.b for link in site.links_from("N_W")} == {"X_W", "N_E", "W_Q", "O_Q", "P-1", "P-2"}


def test_a_nav_group_may_name_the_zones_it_crosses():
    zones = copy.deepcopy(SMALL["zones"])
    zones["HALL"]["rects"] = [[0.0, 0.0, 0.9, 0.75], [0.0, 0.75, 0.5, 0.8]]
    zones["GAP"] = {"kind": "shared", "capacity": 1, "rects": [[0.9, 0.0, 1.1, 0.75]]}
    zones["HALL_E"] = {"kind": "shared", "capacity": 2,
                       "rects": [[1.1, 0.0, 2.0, 0.75], [1.5, 0.75, 2.0, 0.8]]}
    nav = [{"nodes": ["N_W", "W_Q", "P-1"]}, {"nodes": ["N_E", "O_Q", "P-2"]},
           {"nodes": ["N_W", "N_E"], "zones": ["HALL", "GAP", "HALL_E"]}]
    site = build(zones=zones, nav=nav)
    assert site.link("N_W", "N_E").zones == ("HALL", "GAP", "HALL_E")
    assert site.link("N_E", "N_W").zones == ("HALL_E", "GAP", "HALL")
    assert site.link("W_Q", "O_Q") is None


def test_stations_keep_the_fields_orders_and_arms_read():
    site = build()
    work = site.stations["WORK"]
    assert (work.id, work.dock, work.queue, work.server, work.arms, work.action) == (
        "WORK", "W", ("W_Q",), "workcell", ("arm1", "arm2"), "fill")
    assert site.parking == ("P-1", "P-2")
    assert site.docks == {"W", "O"}


def test_bounds_and_shelves_are_kept_for_the_world_builders():
    site = build()
    assert site.bounds() == (0.0, 0.0, 2.0, 1.0)
    assert site.shelves == ((0.5, 0.75, 1.5, 0.8),)
    assert site.name == "small"


# ---- 자리 묻기 ------------------------------------------------------------------------------


def test_zone_at_names_the_zone_under_a_point():
    site = build()
    assert site.zone_at(1.0, 0.3) == "HALL"
    assert site.zone_at(0.1, 0.78) == "HALL"
    assert site.zone_at(1.0, 0.9) == "EXPRESS"
    assert site.zone_at(1.0, 0.78) is None  # 진열대 자리는 구역이 아니다
    assert site.zone_at(3.0, 0.5) is None


def test_nodes_in_lists_the_nodes_of_one_zone_in_map_order():
    site = build()
    assert site.nodes_in("HALL") == ["N_W", "N_E", "W_Q", "O_Q", "P-1", "P-2"]
    assert site.nodes_in("EXPRESS") == []


def test_distance_to_zone_is_zero_inside_and_grows_outside():
    site = build()
    assert site.distance_to_zone(1.0, 0.9, "EXPRESS") == 0.0
    assert site.distance_to_zone(1.0, 0.7, "EXPRESS") == pytest.approx(0.1)
    assert site.distance_to_zone(0.3, 0.7, "EXPRESS") == pytest.approx(math.hypot(0.2, 0.1))
    with pytest.raises(KeyError):
        site.distance_to_zone(0.0, 0.0, "NOWHERE")


def test_nearest_node_can_be_limited_to_some_nodes_and_a_radius():
    site = build()
    assert site.nearest_node(0.22, 0.52) == "W_Q"
    assert site.nearest_node(0.22, 0.52, nodes=site.parking) == "P-1"
    assert site.nearest_node(1.0, 0.3, radius=0.1) is None


def test_node_pose_gives_the_point():
    assert build().node_pose("O_Q") == (1.8, 0.5)


# ---- 길 -------------------------------------------------------------------------------------


def test_route_is_the_shortest_chain_of_links():
    site = build()
    way = route(site, "P-1", "W")
    assert [link.b for link in way] == ["N_W", "X_W", "W"]
    assert [link.via for link in way] == ["nav_goal", "follow_path", "follow_path"]


def test_route_takes_the_express_from_dock_to_dock():
    way = route(build(), "W", "O")
    assert [(link.via, link.b) for link in way] == [
        ("retreat", "X_W"), ("follow_path", "X_E"), ("follow_path", "O")]


def test_route_follows_the_cost_it_is_given():
    site = build()

    def no_express(link):
        return math.inf if link.lane else link.length

    way = route(site, "W", "O", cost=no_express)
    assert [link.b for link in way] == ["X_W", "N_W", "N_E", "X_E", "O"]


def test_route_to_the_same_node_is_empty_and_an_unreachable_one_is_none():
    site = build()
    assert route(site, "W", "W") == []
    assert route(site, "W", "O", cost=lambda link: math.inf) is None


# ---- 잘못 적은 지도 -------------------------------------------------------------------------


def refused(match: str, **changes):
    with pytest.raises(FleetMapError, match=match):
        build(**changes)


def test_a_zone_needs_a_known_kind_and_a_shared_zone_its_capacity():
    refused("kind", zones=edited("zones", "HALL", kind="yard"))
    hall = copy.deepcopy(SMALL["zones"])
    del hall["HALL"]["capacity"]
    refused("capacity", zones=hall)
    refused("one robot", zones=edited("zones", "J_W", capacity=2))


def test_zones_may_not_overlap_or_leave_the_floor():
    refused("overlap", zones=edited("zones", "J_W", rects=[[0.2, 0.8, 0.5, 1.0]]))
    refused("outside", zones=edited("zones", "DOCK_E", rects=[[1.7, 0.8, 2.1, 1.0]]))


def test_a_node_must_stand_in_a_zone_of_the_right_kind():
    refused("no zone", nodes=edited("nodes", "N_W", x=1.0, y=0.78))
    refused("shared", nodes=edited("nodes", "W_Q", x=1.0, y=0.9))
    refused("dock zone", nodes=edited("nodes", "W", y=0.6))
    refused("kind", nodes=edited("nodes", "W_Q", kind="queue"))


def test_a_node_may_not_sit_on_a_shelf_or_a_wall():
    refused("shelf", nodes=edited("nodes", "N_W", x=0.47, y=0.72))
    refused("wall", nodes=edited("nodes", "P-1", x=0.03))


def test_a_fixed_path_needs_known_nodes_and_room_beside_the_shelves():
    refused("NOPE", paths=edited("paths", "work_in", points=["N_W", "NOPE", "W"]))
    refused("two", paths=edited("paths", "work_in", points=["N_W"]))
    nodes = copy.deepcopy(SMALL["nodes"])
    nodes["N_W"]["x"] = nodes["X_W"]["x"] = 0.46
    refused("shelf", nodes=nodes)


def test_a_link_is_written_once():
    paths = copy.deepcopy(SMALL["paths"])
    paths["again"] = {"points": ["X_W", "N_W"]}
    refused("twice", paths=paths)


def test_a_nav_group_stays_in_shared_zones():
    refused("shared", nav=[{"nodes": ["N_W", "N_E", "W_Q", "O_Q", "P-1", "P-2", "X_W"]}])
    refused("zones", nav=[{"nodes": ["N_W", "N_E", "W_Q", "O_Q", "P-1", "P-2"],
                           "zones": ["HALL", "EXPRESS"]}])


def test_there_is_one_fill_station_and_one_swap_station():
    stations = copy.deepcopy(SMALL["stations"])
    stations["OUTBOUND"]["action"] = "fill"
    refused("fill", stations=stations)
    refused("dock", stations=edited("stations", "WORK", dock="W_Q"))
    refused("queue", stations=edited("stations", "WORK", queue=["N_W"]))


def test_every_parking_spot_reaches_both_docks_and_back():
    paths = copy.deepcopy(SMALL["paths"])
    del paths["out_in"]
    refused("no way", paths=paths)


def test_parking_lists_every_parking_node():
    refused("parking", parking=["P-1"])
    refused("parking", parking=["P-1", "P-2", "W_Q"])


# ---- 파일 -----------------------------------------------------------------------------------


@pytest.mark.parametrize("layout", ["d2", "c", "d"])
def test_shipped_maps_load(layout):
    site = load_fleet_map(layout)
    assert site.name == f"arena_{layout}"
    assert site.bounds() == (0.0, 0.0, 1.98, 0.99)
    assert len(site.parking) == 3
    assert site.source_text


def test_d2_is_the_default_and_matches_the_instruction():
    site = load_fleet_map()
    assert site.name == "arena_d2"
    assert site.shelves == ((0.50, 0.775, 1.48, 0.79),)
    assert site.node_pose("W") == (0.18, 0.89) and site.node_pose("O") == (1.80, 0.89)
    assert site.zones["HALL"].capacity == 3
    assert [link.b for link in route(site, "W", "O")] == ["N_W", "X_W", "X_E", "O"]
    # 곧은 선은 진열대 끝에서 0.15 m다. 급행과 갈림목 사이의 호(반지름 0.10)는 0.115 m까지 다가간다.
    # 갈림목 선을 더 옮기면 도크 W, O와 너무 가까워지므로 남겨 둔 경고다(2-1, 사용자 결정 거리)
    assert len(site.warnings) == 4
    assert all("corner" in w and "0.115 m" in w for w in site.warnings), site.warnings
    assert not any("ends at" in w for w in site.warnings), "D2 backs into the hall"


def test_load_accepts_a_path(tmp_path):
    import yaml

    target = tmp_path / "mine.yaml"
    target.write_text(yaml.safe_dump(SMALL), encoding="utf-8")
    assert load_fleet_map(target).name == "small"
