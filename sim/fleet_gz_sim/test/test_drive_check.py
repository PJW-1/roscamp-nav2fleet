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
    """FollowPath 충돌 검사와 같은 몸 다각형(base_footprint 틀).
    없으면 None(중심 - 0.056으로 잰다)."""
    polygon = drive_check.load_footprint()
    if polygon is None:
        pytest.skip("pode_nav_config is not installed")
    assert len(polygon) == 14
    assert max(abs(y) for _, y in polygon) == pytest.approx(0.056)
    assert min(x for x, _ in polygon) == pytest.approx(-0.076)
    assert drive_check.load_footprint("/nowhere/nav2_params.yaml") is None


class _FakeNode:
    """run_step이 쓰는 것만: 결과는 바로 성공, 실제 위치 기록은 없음."""
    def __init__(self):
        import threading
        self.lock = threading.Lock()
        self.truth, self.states, self.plans, self.box_min, self.amcl = [], [], [], {}, []
        self.footprint = None

    def send(self, step):
        return {"done": True, "success": True, "handle": None}

    def last_truth(self):
        return None


def test_amcl_error_key_is_always_written_null_when_not_measurable():
    from fleet_core.fleet_map import load_fleet_map
    from fleet_gz_sim import stage2_plan as sp
    site = load_fleet_map("v7")
    step = sp.build(site, "corridor")[0]
    out = drive_check.run_step(_FakeNode(), site, step, timeout=1.0)
    assert out["result"] == "success"
    assert "amcl_max_error_m" in out and out["amcl_max_error_m"] is None


def test_without_arguments_the_check_drives_the_real_arena_v7():
    args = drive_check.parse_args(["--contract", "c.yaml"])
    assert (args.layout, args.plan) == ("v7", "corridor")
    args = drive_check.parse_args(["--contract", "c.yaml", "--layout", "d2"])
    assert (args.layout, args.plan) == ("d2", "stage2")
    args = drive_check.parse_args(["--contract", "c.yaml", "--plan", "cross"])
    assert (args.layout, args.plan) == ("d2", "cross")
