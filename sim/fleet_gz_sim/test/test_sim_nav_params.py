"""Sim-only Nav2 parameters: AMCL takes sim values, every controller checks collisions, the
speed cap reaches every forward controller, and the real robot's file stays as it is."""
from pathlib import Path

import pytest
import yaml

launch = pytest.importorskip("launch")
pytest.importorskip("launch.actions")  # ROS가 없으면 이 패키지의 launch 폴더가 빈 패키지로 잡힌다
from fleet_gz_sim import launch_parts  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
MAPS = {"nav_map": "/m/arena_d2.yaml", "nav_map_fine": "/m/arena_d2_fine.yaml"}
NAV = REPO / "pinky" / "pinky_nav_config" / "params" / "nav2_params.yaml"


def _amcl(data: dict) -> dict:
    return data["amcl"]["ros__parameters"]


def test_sim_params_change_amcl_only_and_leave_the_real_file(tmp_path):
    before = NAV.read_bytes()
    out = launch_parts.sim_nav_params(NAV, tmp_path / "nav2_sim.yaml")
    assert NAV.read_bytes() == before
    base, sim = yaml.safe_load(before), yaml.safe_load(out.read_text(encoding="utf-8"))
    assert {k: v for k, v in sim.items() if k != "amcl"} == {
        k: v for k, v in base.items() if k != "amcl"}
    for key, value in launch_parts.SIM_AMCL.items():
        assert _amcl(sim)[key] == value, key
    untouched = set(_amcl(base)) - set(launch_parts.SIM_AMCL)
    assert {k: _amcl(sim)[k] for k in untouched} == {k: _amcl(base)[k] for k in untouched}
    # nav2.launch.py fills these in per robot
    assert "<robot_namespace>" in out.read_text(encoding="utf-8")


def test_sim_amcl_trusts_the_scan_more_than_the_real_config():
    """In the warehouse A4->A3 corridor the real values let the AMCL pose sit 0.13-0.29 m to the
    side of the true pose (2026-11-08). The sim values weigh the map match more, use more beams
    and update the filter more often."""
    base, sim = _amcl(yaml.safe_load(NAV.read_text(encoding="utf-8"))), launch_parts.SIM_AMCL
    assert sim["z_rand"] < base["z_rand"] and sim["z_hit"] > base["z_hit"]
    assert sim["z_hit"] + sim["z_rand"] == pytest.approx(1.0)
    assert sim["max_beams"] > base["max_beams"]
    assert sim["update_min_d"] < base["update_min_d"]
    assert sim["update_min_a"] < base["update_min_a"]
    assert sim["laser_max_range"] == pytest.approx(12.0)  # the sim lidar's <max>


def _need(*packages):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")


def test_each_pinky_starts_nav2_with_the_sim_params(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pinky_nav_config")
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    for ns in ("pinky1", "pinky2"):
        includes = [a for a in launch_parts.pinky(ns, 0.5, 0.5, 0.0, fsm=False, **MAPS)
                    if isinstance(a, IncludeLaunchDescription)]
        args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
        nav = next(a for a in args if "map_fine" in a)
        path = Path(nav["params_file"])
        assert path.parent == tmp_path
        # one file per launch process: two sim runs with the same robot names do not share it
        # (Codex review of 5a0587b)
        assert ns in path.name and str(launch_parts.os.getpid()) in path.name
        amcl = _amcl(yaml.safe_load(path.read_text(encoding="utf-8")))
        assert {k: amcl[k] for k in launch_parts.SIM_AMCL} == launch_parts.SIM_AMCL


def test_one_pinky_sends_lock_boundary_paths_to_the_through_rpp_by_default(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pinky_nav_config", "pinky_fsm")
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def fsm_args(**kw):
        includes = [a for a in launch_parts.pinky("pinky1", 0.5, 0.5, 0.0, **MAPS, **kw)
                    if isinstance(a, IncludeLaunchDescription)]
        args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
        return next(a for a in args if "heartbeat_timeout_s" in a)

    assert fsm_args()["through_controller_id"] == "FollowPathThrough"
    assert fsm_args(through_controller_id="")["through_controller_id"] == ""
    # A speed cap also caps the FSM's lock-boundary speed limit: RPP takes that limit as its
    # desired_linear_vel, so a limit above the cap would speed the robot up again.
    assert fsm_args()["horizon_cap_mps"] == str(launch_parts.DEFAULT_SPEED_MPS)
    assert fsm_args(nav_max_speed=0.14)["horizon_cap_mps"] == "0.14"


def _rpp(data: dict, name: str) -> dict:
    return data["controller_server"]["ros__parameters"][name]


def test_sim_params_cap_every_forward_controller_and_change_nothing_else(tmp_path):
    """The arena runs at 0.12 to 0.20 m/s; the reverse controller keeps its slower 0.05 m/s."""
    capped = yaml.safe_load(launch_parts.sim_nav_params(
        NAV, tmp_path / "capped.yaml", max_speed=0.12).read_text(encoding="utf-8"))
    plain = yaml.safe_load(launch_parts.sim_nav_params(
        NAV, tmp_path / "plain.yaml").read_text(encoding="utf-8"))
    for name in launch_parts.CONTROLLERS:
        if name == launch_parts.REVERSE_CONTROLLER:
            assert _rpp(capped, name)["desired_linear_vel"] == 0.05
            continue
        assert _rpp(capped, name)["desired_linear_vel"] == 0.12
        assert _rpp(plain, name)["desired_linear_vel"] == 0.25
        _rpp(plain, name)["desired_linear_vel"] = 0.12
    assert capped == plain


@pytest.mark.parametrize("bad", [0.0, -0.1, float("nan"), float("inf"), 0.3, True, "0.1"])
def test_a_speed_cap_must_be_a_number_between_zero_and_the_robot_speed(tmp_path, bad):
    with pytest.raises(ValueError, match="max_speed"):
        launch_parts.sim_nav_params(NAV, tmp_path / "a.yaml", max_speed=bad)


def test_every_controller_checks_collisions_even_if_the_base_file_turned_it_off(tmp_path):
    """Instruction 3: the collision check is on in every run (it was off on the last arena)."""
    base = yaml.safe_load(NAV.read_text(encoding="utf-8"))
    for name in launch_parts.CONTROLLERS:
        _rpp(base, name)["use_collision_detection"] = False
    off = tmp_path / "off.yaml"
    off.write_text(yaml.safe_dump(base), encoding="utf-8")
    sim = yaml.safe_load(launch_parts.sim_nav_params(off, tmp_path / "sim.yaml")
                         .read_text(encoding="utf-8"))
    assert launch_parts.CONTROLLERS == ("FollowPath", "FollowPathThrough", "FollowPathReverse",
                                        "FollowPathGoal")
    for name in launch_parts.CONTROLLERS:
        assert _rpp(sim, name)["use_collision_detection"] is True, name


def test_overrides_go_deep_and_cannot_turn_the_collision_check_off(tmp_path):
    local = {"resolution": 0.01, "obstacle_layer": {"scan": {"obstacle_min_range": 0.1}}}
    overrides = {"local_costmap": {"local_costmap": {"ros__parameters": local}},
                 "controller_server": {"ros__parameters": {
                     "FollowPath": {"use_collision_detection": False}}}}
    sim = yaml.safe_load(launch_parts.sim_nav_params(NAV, tmp_path / "o.yaml", overrides=overrides)
                         .read_text(encoding="utf-8"))
    params = sim["local_costmap"]["local_costmap"]["ros__parameters"]
    assert params["resolution"] == 0.01
    assert params["obstacle_layer"]["scan"]["obstacle_min_range"] == 0.1
    assert params["obstacle_layer"]["scan"]["topic"]  # the rest of the layer stays
    assert _rpp(sim, "FollowPath")["use_collision_detection"] is True


def test_overrides_win_over_the_sim_amcl_values(tmp_path):
    """nav_overrides로 AMCL 값도 바꿀 수 있다(위치 추정을 조사할 때 쓴다)."""
    overrides = {"amcl": {"ros__parameters": {"z_hit": 0.5}}}
    sim = yaml.safe_load(launch_parts.sim_nav_params(NAV, tmp_path / "a.yaml", overrides=overrides)
                         .read_text(encoding="utf-8"))
    assert _amcl(sim)["z_hit"] == 0.5
    assert _amcl(sim)["max_beams"] == launch_parts.SIM_AMCL["max_beams"]


def test_the_fixed_line_check_can_be_turned_off_and_the_yard_check_stays_on(tmp_path):
    """Lead decision 2026-11-12 (G1 not achieved): fixed lines without the RPP check, yard goals
    with it. The default still equals the real settings."""
    sim = yaml.safe_load(launch_parts.sim_nav_params(NAV, tmp_path / "f.yaml",
                                                     fixed_line_check=False)
                         .read_text(encoding="utf-8"))
    assert launch_parts.FIXED_LINE_CONTROLLERS == ("FollowPath", "FollowPathThrough",
                                                   "FollowPathReverse")
    for name in launch_parts.FIXED_LINE_CONTROLLERS:
        assert _rpp(sim, name)["use_collision_detection"] is False, name
    assert _rpp(sim, "FollowPathGoal")["use_collision_detection"] is True
    default = yaml.safe_load(launch_parts.sim_nav_params(NAV, tmp_path / "d.yaml")
                             .read_text(encoding="utf-8"))
    assert all(_rpp(default, n)["use_collision_detection"] is True
               for n in launch_parts.CONTROLLERS)


def test_pinky_passes_the_fixed_line_switch_to_its_nav2_params(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pinky_nav_config")
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    includes = [a for a in launch_parts.pinky("pinky1", 0.5, 0.5, 0.0, fsm=False,
                                              fixed_line_check=False, **MAPS)
                if isinstance(a, IncludeLaunchDescription)]
    args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
    nav = next(a for a in args if "map_fine" in a)
    sim = yaml.safe_load(Path(nav["params_file"]).read_text(encoding="utf-8"))
    assert _rpp(sim, "FollowPath")["use_collision_detection"] is False
    assert _rpp(sim, "FollowPathGoal")["use_collision_detection"] is True


def test_a_base_file_without_one_of_the_controllers_is_refused(tmp_path):
    base = yaml.safe_load(NAV.read_text(encoding="utf-8"))
    del base["controller_server"]["ros__parameters"]["FollowPathGoal"]
    path = tmp_path / "old.yaml"
    path.write_text(yaml.safe_dump(base), encoding="utf-8")
    with pytest.raises(ValueError, match="FollowPathGoal"):
        launch_parts.sim_nav_params(path, tmp_path / "sim.yaml")


@pytest.mark.parametrize("text, value", [("", 0.12), ("0.12", 0.12), ("0.15", 0.15), ("0.2", 0.2)])
def test_the_launch_speed_is_the_arena_range(text, value):
    assert launch_parts.speed_of(text) == value


@pytest.mark.parametrize("bad", ["0.11", "0.21", "0.25", "fast", "nan", "inf"])
def test_a_launch_speed_outside_the_arena_range_is_refused(bad):
    with pytest.raises(ValueError, match="nav_max_speed"):
        launch_parts.speed_of(bad)
