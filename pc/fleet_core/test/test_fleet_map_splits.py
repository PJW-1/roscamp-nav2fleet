"""나눈 goal(splits, 해결안 B): 출발 자리에 따라 한 Nav2 goal을 중간 지점을 거쳐 둘로 보낸다.

지도는 어디서 어디로 갈 때 어느 지점을 거치는지만 적는다. 켜고 끄기는 관제 파라미터 plan_split이다.
"""
import copy

import pytest
from fleet_core.fleet_map import FleetMap, FleetMapError
from test_fleet_map import SMALL

# 마당 가운데 아래의 중간 지점 V. V -> P-2만 잇는다(자유 이음의 목적지가 아니다)
VIA = {"x": 1.0, "y": 0.3, "kind": "fork", "via": True}
SPLIT = {"to": "P-2", "via": "V", "from": [[0.6, 0.4, 1.4, 0.75]]}


def data_with(split=None, via=None, nav=None, more=()):
    data = copy.deepcopy(SMALL)
    data["nodes"]["V"] = dict(VIA if via is None else via)
    data["nav"].append({"nodes": ["V", "P-2"]} if nav is None else nav)
    data["splits"] = [dict(SPLIT if split is None else split), *more]
    return data


def test_a_split_names_the_goal_the_via_node_and_where_the_robot_starts():
    site = FleetMap(data_with())
    assert site.nodes["V"].via
    assert [(s.to, s.via, s.rects) for s in site.splits] == [("P-2", "V", ((0.6, 0.4, 1.4, 0.75),))]
    assert site.split_via("P-2", 1.0, 0.6) == "V"
    assert site.split_via("P-2", 1.0, 0.2) == ""   # 출발 자리가 사각형 밖
    assert site.split_via("O_Q", 1.0, 0.6) == ""   # 다른 도착 지점


def test_a_map_without_splits_has_none():
    site = FleetMap(copy.deepcopy(SMALL))
    assert site.splits == () and site.split_via("P-2", 1.0, 0.6) == ""
    assert not any(node.via for node in site.nodes.values())


def test_the_first_split_whose_area_holds_the_start_wins():
    other = {"to": "P-2", "via": "N_E", "from": [[0.0, 0.62, 0.6, 0.75]]}
    site = FleetMap(data_with(more=[other]))
    assert site.split_via("P-2", 0.3, 0.7) == "N_E"
    assert site.split_via("P-2", 1.0, 0.6) == "V"


@pytest.mark.parametrize("split, match", [
    ({**SPLIT, "to": "NOPE"}, "not a node"),
    ({**SPLIT, "via": "NOPE"}, "not a node"),
    ({**SPLIT, "via": "P-2"}, "itself"),
    ({**SPLIT, "via": "X_W"}, "same zone"),              # X_W는 J_W 안의 지나가는 점
    ({**SPLIT, "from": []}, "area"),
    ({**SPLIT, "from": [[0.6, 0.4, 2.4, 0.75]]}, "outside the floor"),
])
def test_a_split_must_name_nodes_joined_by_a_nav_goal_edge_and_an_area(split, match):
    with pytest.raises(FleetMapError, match=match):
        FleetMap(data_with(split=split))


def test_a_via_node_needs_a_nav_goal_edge_to_the_goal():
    with pytest.raises(FleetMapError, match="no nav goal edge"):
        FleetMap(data_with(nav={"nodes": ["V", "N_W"]}))


def test_a_via_node_must_be_a_fork():
    """설 수 있는 지점(wait 등)이면 첫 조각까지만 잡아 두 goal이 한 base에 들지 않는다."""
    with pytest.raises(FleetMapError, match="fork"):
        FleetMap(data_with(split={**SPLIT, "via": "W_Q"}))


def test_a_through_node_cannot_be_a_via_node():
    """through 지점은 이어진 goal을 하나로 합친다. 나누는 지점으로 쓰면 다시 합쳐진다."""
    data = data_with(via={"x": 1.0, "y": 0.3, "kind": "fork", "through": True})
    with pytest.raises(FleetMapError, match="through"):
        FleetMap(data)


def test_only_a_fork_can_be_marked_via_and_it_must_be_used_by_a_split():
    with pytest.raises(FleetMapError, match="only a fork"):
        FleetMap(data_with(via={**VIA, "kind": "wait"}))
    data = data_with()
    data["splits"] = [{**SPLIT, "via": "N_E", "from": [[0.0, 0.62, 0.6, 0.75]]}]
    with pytest.raises(FleetMapError, match="no split uses"):
        FleetMap(data)


def test_two_splits_to_one_goal_may_not_share_an_area():
    other = {"to": "P-2", "via": "N_E", "from": [[1.2, 0.4, 1.8, 0.75]]}
    with pytest.raises(FleetMapError, match="overlap"):
        FleetMap(data_with(more=[other]))


def test_a_graph_edge_to_the_goal_from_inside_the_area_is_refused():
    """관제는 노드 밖에서 다시 계획한 첫 이음(자유 이음)만 나눈다. 사각형 안의 노드에서 가는 지도
    이음은 나누지 않으므로, 그런 이음이 있으면 지도를 거절한다(N_W -> P-2가 사각형 안에서 출발)."""
    split = {**SPLIT, "from": [[0.3, 0.4, 1.4, 0.75]]}
    with pytest.raises(FleetMapError, match="N_W"):
        FleetMap(data_with(split=split))
