"""drive_check: 상자 표시(/sim/obstacle_markers)에서 놓인 상자 중심을 읽는 부분."""
import pytest

pytest.importorskip("rclpy")
visualization_msgs = pytest.importorskip("visualization_msgs.msg")
pytest.importorskip("fleet_interfaces")
from fleet_gz_sim import drive_check  # noqa: E402
from fleet_gz_sim.obstacles import marker_specs  # noqa: E402

Marker, MarkerArray = visualization_msgs.Marker, visualization_msgs.MarkerArray


def _array(specs) -> MarkerArray:
    """obstacle_node._publish_markers와 같은 방식으로 만든다."""
    out = MarkerArray()
    for spec in specs:
        marker = Marker()
        if spec["action"] == "deleteall":
            marker.action = Marker.DELETEALL
        else:
            marker.ns, marker.id = spec["ns"], spec["id"]
            marker.type = Marker.CUBE if spec["type"] == "cube" else Marker.TEXT_VIEW_FACING
            marker.pose.position.x, marker.pose.position.y = spec["x"], spec["y"]
        out.markers.append(marker)
    return out


class _Occ:
    def __init__(self, number, x, y):
        self.number, self.name = number, f"obstacle_{number}"
        self.obstacle = type("O", (), {"x": x, "y": y, "yaw": 0.0})()


def test_box_centres_reads_the_placed_cubes_and_skips_the_labels():
    msg = _array(marker_specs([_Occ(1, 0.952, 0.595), _Occ(3, 0.3, 0.4)], (0.08, 0.08, 0.25)))
    assert drive_check.box_centres(msg) == {"obstacle_1": pytest.approx((0.952, 0.595)),
                                            "obstacle_3": pytest.approx((0.3, 0.4))}


def test_box_centres_is_empty_when_nothing_is_placed():
    assert drive_check.box_centres(_array(marker_specs([], (0.08, 0.08, 0.25)))) == {}


def test_the_body_footprint_comes_from_the_local_costmap_of_pode_nav_config():
    """FollowPath 충돌 검사와 같은 몸 다각형(base_footprint 틀). 없으면 None(중심 - 0.056으로 잰다)."""
    polygon = drive_check.load_footprint()
    if polygon is None:
        pytest.skip("pode_nav_config is not installed")
    assert len(polygon) == 14
    assert max(abs(y) for _, y in polygon) == pytest.approx(0.056)
    assert min(x for x, _ in polygon) == pytest.approx(-0.076)
    assert drive_check.load_footprint("/nowhere/nav2_params.yaml") is None
