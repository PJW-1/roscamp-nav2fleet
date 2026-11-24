"""시뮬 전용 Nav2 파라미터: AMCL은 시뮬 값, 모든 controller가 충돌 검사, 속도 상한은 앞으로 가는 controller 전부, 실물 파일은 그대로."""
from pathlib import Path

import pytest
import yaml

launch = pytest.importorskip("launch")
pytest.importorskip("launch.actions")  # ROS가 없으면 launch 폴더가 빈 패키지로 잡힌다
from fleet_gz_sim import launch_parts  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
MAPS = {"nav_map": "/m/arena_d2.yaml", "nav_map_fine": "/m/arena_d2_fine.yaml"}
NAV = REPO / "pode" / "pode_nav_config" / "params" / "nav2_params.yaml"
SPEED_FILE = REPO / "pode" / "pode_nav_config" / "params" / "nav_speed.yaml"
# 회피 담당의 파일(기본 회피 패키지)
AVOIDANCE = REPO / "pode" / "pode_avoidance" / "params" / "avoidance.yaml"


def _sim(base, out, **kw):
    return launch_parts.sim_nav_params(base, out, avoidance=AVOIDANCE, **kw)


def _base() -> dict:
    """실물 설정: 주행 파일에 회피 담당의 goal controller를 얹은 것."""
    data = yaml.safe_load(NAV.read_text(encoding="utf-8"))
    theirs = yaml.safe_load(AVOIDANCE.read_text(encoding="utf-8"))
    data["controller_server"]["ros__parameters"]["FollowPathGoal"] = theirs["follow_path_goal"]
    return data


def _amcl(data: dict) -> dict:
    return data["amcl"]["ros__parameters"]


def test_sim_params_change_amcl_and_the_mppi_entry_only_and_leave_the_real_file(tmp_path):
    before = NAV.read_bytes(), AVOIDANCE.read_bytes()
    out = _sim(NAV, tmp_path / "nav2_sim.yaml")
    assert (NAV.read_bytes(), AVOIDANCE.read_bytes()) == before
    base, sim = _base(), yaml.safe_load(out.read_text(encoding="utf-8"))
    base["controller_server"]["ros__parameters"]["controller_plugins"].append("FollowPathMPPI")
    assert {k: v for k, v in sim.items() if k != "amcl"} == {
        k: v for k, v in base.items() if k != "amcl"}
    for key, value in launch_parts.SIM_AMCL.items():
        assert _amcl(sim)[key] == value, key
    untouched = set(_amcl(base)) - set(launch_parts.SIM_AMCL)
    assert {k: _amcl(sim)[k] for k in untouched} == {k: _amcl(base)[k] for k in untouched}
    # nav2.launch.py가 로봇마다 채운다
    assert "<robot_namespace>" in out.read_text(encoding="utf-8")


def test_sim_amcl_trusts_the_scan_more_than_the_real_config():
    """시뮬 AMCL 값은 지도 맞춤을 더 믿고 빔을 늘리고 갱신을 자주 한다."""
    base, sim = _amcl(yaml.safe_load(NAV.read_text(encoding="utf-8"))), launch_parts.SIM_AMCL
    assert sim["z_rand"] < base["z_rand"] and sim["z_hit"] > base["z_hit"]
    assert sim["z_hit"] + sim["z_rand"] == pytest.approx(1.0)
    assert sim["max_beams"] > base["max_beams"]
    assert sim["update_min_d"] < base["update_min_d"]
    assert sim["update_min_a"] < base["update_min_a"]
    assert sim["laser_max_range"] == pytest.approx(12.0)  # 시뮬 라이다의 <max>


def _need(*packages):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")


def test_each_pode_starts_nav2_with_the_sim_params(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pode_nav_config")
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    for ns in ("pode1", "pode2"):
        includes = [a for a in launch_parts.pode(ns, 0.5, 0.5, 0.0, fsm=False, **MAPS)
                    if isinstance(a, IncludeLaunchDescription)]
        args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
        nav = next(a for a in args if "map_fine" in a)
        path = Path(nav["params_file"])
        assert path.parent == tmp_path
        # launch 프로세스마다 파일 하나: 같은 로봇 이름의 시뮬 둘이 나누지 않는다
        assert ns in path.name and str(launch_parts.os.getpid()) in path.name
        amcl = _amcl(yaml.safe_load(path.read_text(encoding="utf-8")))
        assert {k: amcl[k] for k in launch_parts.SIM_AMCL} == launch_parts.SIM_AMCL


def test_one_pode_sends_lock_boundary_paths_to_the_through_rpp_by_default(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pode_nav_config", "pode_fsm")
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def fsm_args(**kw):
        includes = [a for a in launch_parts.pode("pode1", 0.5, 0.5, 0.0, **MAPS, **kw)
                    if isinstance(a, IncludeLaunchDescription)]
        args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
        return next(a for a in args if "heartbeat_timeout_s" in a)

    assert fsm_args()["through_controller_id"] == "FollowPathThrough"
    assert fsm_args(through_controller_id="")["through_controller_id"] == ""
    # 속도 상한은 FSM의 잠금 경계 속도 제한도 자른다: RPP는 그 제한을 desired_linear_vel로 받는다.
    assert fsm_args()["horizon_cap_mps"] == str(launch_parts.nav_speed_mps())
    assert fsm_args(nav_max_speed=0.10)["horizon_cap_mps"] == "0.1"


def _rpp(data: dict, name: str) -> dict:
    return data["controller_server"]["ros__parameters"][name]


def test_sim_params_cap_every_forward_controller_and_change_nothing_else(tmp_path):
    """단일 출처보다 낮은 상한은 앞으로 가는 controller 전부를 늦추고 후진은 0.05 m/s를 유지한다."""
    top = launch_parts.nav_speed_mps(SPEED_FILE)
    capped = yaml.safe_load(_sim(
        NAV, tmp_path / "capped.yaml", max_speed=0.10).read_text(encoding="utf-8"))
    plain = yaml.safe_load(_sim(
        NAV, tmp_path / "plain.yaml").read_text(encoding="utf-8"))
    for name in launch_parts.CONTROLLERS:
        if name == launch_parts.REVERSE_CONTROLLER:
            assert _rpp(capped, name)["desired_linear_vel"] == 0.05
            continue
        assert _rpp(capped, name)["desired_linear_vel"] == 0.10
        assert _rpp(plain, name)["desired_linear_vel"] == top
        _rpp(plain, name)["desired_linear_vel"] = 0.10
    assert _rpp(capped, launch_parts.MPPI_CONTROLLER)["vx_max"] == 0.10
    assert _rpp(plain, launch_parts.MPPI_CONTROLLER)["vx_max"] == top
    _rpp(plain, launch_parts.MPPI_CONTROLLER)["vx_max"] = 0.10
    steps = _rpp(plain, launch_parts.MPPI_CONTROLLER)["time_steps"]
    assert _rpp(capped, launch_parts.MPPI_CONTROLLER)["time_steps"] == round(steps * top / 0.10)
    _rpp(plain, launch_parts.MPPI_CONTROLLER)["time_steps"] = round(steps * top / 0.10)
    assert capped == plain


def test_a_speed_cap_keeps_the_mppi_horizon_distance(tmp_path):
    """더 낮은 상한은 MPPI의 time_steps를 늘려 같은 거리를 내다보게 한다."""
    def mppi(out, **kw):
        data = yaml.safe_load(_sim(NAV, tmp_path / out, **kw).read_text(encoding="utf-8"))
        return _rpp(data, launch_parts.MPPI_CONTROLLER)

    def reach(m):
        return m["time_steps"] * m["model_dt"] * m["vx_max"]
    plain = mppi("plain.yaml")
    for speed in (0.05, 0.08, 0.10, launch_parts.nav_speed_mps(SPEED_FILE)):
        capped = mppi(f"c{speed}.yaml", max_speed=speed)
        assert abs(reach(capped) - reach(plain)) <= capped["model_dt"] * speed / 2 + 1e-9
    assert mppi("same.yaml", max_speed=plain["vx_max"])["time_steps"] == plain["time_steps"]


def test_an_mppi_from_the_overrides_is_capped_without_changing_the_callers_dict(tmp_path):
    """overrides로 통째로 넘긴 MPPI 매핑은 출력에서만 상한을 받고, time_steps를 Nav2 기본에 맡긴 것은 비워 둔다."""
    base = _base()
    del base["controller_server"]["ros__parameters"][launch_parts.MPPI_CONTROLLER]
    path = tmp_path / "base.yaml"
    path.write_text(yaml.safe_dump(base), encoding="utf-8")
    for extra in ({}, {"time_steps": 40}):
        mppi = {"plugin": "nav2_mppi_controller::MPPIController", "vx_max": 0.12,
                "model_dt": 0.05, **extra}
        overrides = {"controller_server": {"ros__parameters": {
            launch_parts.MPPI_CONTROLLER: mppi}}}
        out = launch_parts.sim_nav_params(path, tmp_path / "sim.yaml", avoidance=AVOIDANCE,
                                          overrides=overrides, max_speed=0.06)
        sim = _rpp(yaml.safe_load(out.read_text(encoding="utf-8")), launch_parts.MPPI_CONTROLLER)
        assert mppi == {"plugin": "nav2_mppi_controller::MPPIController", "vx_max": 0.12,
                        "model_dt": 0.05, **extra}
        assert sim["vx_max"] == 0.06
        assert sim.get("time_steps") == (80 if extra else None)


def test_a_goal_controller_from_the_overrides_is_capped_without_changing_the_callers_dict(
        tmp_path):
    goal = yaml.safe_load(AVOIDANCE.read_text(encoding="utf-8"))["follow_path_goal"]
    overrides = {"controller_server": {"ros__parameters": {"FollowPathGoal": goal}}}
    before = yaml.safe_dump(overrides)
    out = launch_parts.sim_nav_params(NAV, tmp_path / "sim.yaml", avoidance=None,
                                      overrides=overrides, max_speed=0.06)
    assert yaml.safe_dump(overrides) == before
    sim = _rpp(yaml.safe_load(out.read_text(encoding="utf-8")), "FollowPathGoal")
    assert sim["desired_linear_vel"] == 0.06 and sim["use_collision_detection"] is True


@pytest.mark.parametrize("bad", [0.0, -0.1, float("nan"), float("inf"), 0.3, True, "0.1"])
def test_a_speed_cap_must_be_a_number_between_zero_and_the_robot_speed(tmp_path, bad):
    with pytest.raises(ValueError, match="max_speed"):
        _sim(NAV, tmp_path / "a.yaml", max_speed=bad)


def test_every_controller_checks_collisions_even_if_the_base_file_turned_it_off(tmp_path):
    """충돌 검사는 모든 실행에서 켜 있다."""
    base = _base()
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
    sim = yaml.safe_load(_sim(NAV, tmp_path / "o.yaml", overrides=overrides)
                         .read_text(encoding="utf-8"))
    params = sim["local_costmap"]["local_costmap"]["ros__parameters"]
    assert params["resolution"] == 0.01
    assert params["obstacle_layer"]["scan"]["obstacle_min_range"] == 0.1
    assert params["obstacle_layer"]["scan"]["topic"]  # 층의 나머지는 그대로
    assert _rpp(sim, "FollowPath")["use_collision_detection"] is True


def test_the_rl_controllers_come_only_from_their_overrides_file_and_keep_the_base_ones(tmp_path):
    """RL controller(shim으로 감싼 FollowPathRL, pode_rl_controller 단독 FollowPathRLAlone)는
    기본으로 띄우지 않고 nav_overrides 파일로만 더 띄운다. nav2_params.yaml에 controller가 늘면
    그 파일의 목록에도 더해야 한다."""
    base = _base()["controller_server"]["ros__parameters"]["controller_plugins"]
    assert "FollowPathRL" not in base
    path = REPO / "sim" / "fleet_gz_sim" / "nav_overrides" / "follow_path_rl.yaml"
    overrides = yaml.safe_load(path.read_text(encoding="utf-8"))
    sim = yaml.safe_load(_sim(NAV, tmp_path / "rl.yaml", overrides=overrides)
                         .read_text(encoding="utf-8"))
    server = sim["controller_server"]["ros__parameters"]
    assert server["controller_plugins"] == [*base, "FollowPathMPPI", "FollowPathRL",
                                            "FollowPathRLAlone"]
    for name in ("FollowPathRL", "FollowPathRLAlone"):
        assert server[name]["policy"] == "store_arena_4b"
        assert "goal_rule" not in server[name]  # 정책 명세가 정한다
        assert "lookahead_dist" not in server[name]
        assert server[name]["rear_stop_range"] == 0.0  # 후진 막기는 기본 끔
    assert not (path.parent / "follow_path_rl_shim.yaml").exists()


def test_the_default_rl_option_is_the_shim_wrapped_policy_and_rl_alone_stays(tmp_path):
    """시뮬의 RL 기본(FollowPathRL)은 RotationShim으로 감싼 정책이다.
    감싸지 않은 같은 정책은 비교용 FollowPathRLAlone으로 남는다."""
    path = REPO / "sim" / "fleet_gz_sim" / "nav_overrides" / "follow_path_rl.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    server = data["controller_server"]["ros__parameters"]
    rl, wrapped = server["FollowPathRLAlone"], dict(server["FollowPathRL"])
    assert rl["plugin"] == "pode_rl_controller::FollowPathRL"
    assert wrapped.pop("plugin") == "nav2_rotation_shim_controller::RotationShimController"
    assert wrapped.pop("primary_controller") == rl["plugin"]
    shim_keys = {"angular_dist_threshold", "angular_disengage_threshold",
                 "forward_sampling_distance", "rotate_to_heading_angular_vel",
                 "max_angular_accel", "simulate_ahead_time", "rotate_to_goal_heading"}
    assert set(wrapped) - set(rl) == shim_keys
    assert {k: v for k, v in wrapped.items() if k not in shim_keys} == \
        {k: v for k, v in rl.items() if k != "plugin"}
    assert wrapped["angular_dist_threshold"] == 0.785
    assert wrapped["angular_disengage_threshold"] == 0.39
    assert wrapped["rotate_to_goal_heading"] is True
    sim = yaml.safe_load(_sim(NAV, tmp_path / "rl.yaml", overrides=data, max_speed=0.10)
                         .read_text(encoding="utf-8"))
    out = sim["controller_server"]["ros__parameters"]
    assert out["FollowPathRL"]["primary_controller"] == "pode_rl_controller::FollowPathRL"
    assert out["FollowPathRL"]["max_linear_vel"] == 0.10
    assert out["FollowPathRLAlone"]["max_linear_vel"] == 0.10


def test_mppi_is_loaded_only_in_the_sim(tmp_path):
    """실물 Nav2는 nav2_mppi_controller 없이 떠서 nav2_params.yaml 목록에 FollowPathMPPI가 없다. 시뮬은 블록이 있으면 목록 끝에 한 번 더한다."""
    base = _base()["controller_server"]["ros__parameters"]
    assert "FollowPathMPPI" not in base["controller_plugins"] and "FollowPathMPPI" in base
    sim = yaml.safe_load(_sim(NAV, tmp_path / "s.yaml").read_text(encoding="utf-8"))
    assert sim["controller_server"]["ros__parameters"]["controller_plugins"] == [
        *base["controller_plugins"], "FollowPathMPPI"]


def test_the_launch_speed_default_is_the_one_source():
    assert launch_parts.speed_of("") == launch_parts.nav_speed_mps(SPEED_FILE)


def test_the_rl_controller_gets_the_same_speed_cap_as_rpp_and_mppi(tmp_path):
    """세 controller 비교가 공정하도록 FollowPathRL의 max_linear_vel도 max_speed로 자른다."""
    path = REPO / "sim" / "fleet_gz_sim" / "nav_overrides" / "follow_path_rl.yaml"
    overrides = yaml.safe_load(path.read_text(encoding="utf-8"))
    for speed in (0.12, 0.10):
        sim = yaml.safe_load(_sim(NAV, tmp_path / "rl.yaml", overrides=overrides, max_speed=speed)
                             .read_text(encoding="utf-8"))
        server = sim["controller_server"]["ros__parameters"]
        assert server["FollowPathRL"]["max_linear_vel"] == speed
        assert server["FollowPathMPPI"]["vx_max"] == speed
        assert server["FollowPathGoal"]["desired_linear_vel"] == speed
    sim = yaml.safe_load(_sim(NAV, tmp_path / "rl.yaml", overrides=overrides)
                         .read_text(encoding="utf-8"))
    assert "max_linear_vel" not in sim["controller_server"]["ros__parameters"]["FollowPathRL"]
    overrides["controller_server"]["ros__parameters"]["FollowPathRL"]["max_linear_vel"] = 0.0
    sim = yaml.safe_load(_sim(NAV, tmp_path / "rl.yaml", overrides=overrides, max_speed=0.10)
                         .read_text(encoding="utf-8"))
    assert sim["controller_server"]["ros__parameters"]["FollowPathRL"]["max_linear_vel"] == 0.10


def test_overrides_win_over_the_sim_amcl_values(tmp_path):
    """nav_overrides로 AMCL 값도 바꿀 수 있다(위치 추정을 조사할 때 쓴다)."""
    overrides = {"amcl": {"ros__parameters": {"z_hit": 0.5}}}
    sim = yaml.safe_load(_sim(NAV, tmp_path / "a.yaml", overrides=overrides)
                         .read_text(encoding="utf-8"))
    assert _amcl(sim)["z_hit"] == 0.5
    assert _amcl(sim)["max_beams"] == launch_parts.SIM_AMCL["max_beams"]


def test_the_fixed_line_check_can_be_turned_off_and_the_yard_check_stays_on(tmp_path):
    """고정 경로는 RPP 검사 없이, 마당 goal은 검사와 함께. 기본은 실물 설정과 같다."""
    sim = yaml.safe_load(_sim(NAV, tmp_path / "f.yaml",
                                                     fixed_line_check=False)
                         .read_text(encoding="utf-8"))
    assert launch_parts.FIXED_LINE_CONTROLLERS == ("FollowPath", "FollowPathThrough",
                                                   "FollowPathReverse")
    for name in launch_parts.FIXED_LINE_CONTROLLERS:
        assert _rpp(sim, name)["use_collision_detection"] is False, name
    assert _rpp(sim, "FollowPathGoal")["use_collision_detection"] is True
    default = yaml.safe_load(_sim(NAV, tmp_path / "d.yaml")
                             .read_text(encoding="utf-8"))
    assert all(_rpp(default, n)["use_collision_detection"] is True
               for n in launch_parts.CONTROLLERS)


def test_pode_passes_the_fixed_line_switch_to_its_nav2_params(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pode_nav_config")
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    includes = [a for a in launch_parts.pode("pode1", 0.5, 0.5, 0.0, fsm=False,
                                              fixed_line_check=False, **MAPS)
                if isinstance(a, IncludeLaunchDescription)]
    args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
    nav = next(a for a in args if "map_fine" in a)
    sim = yaml.safe_load(Path(nav["params_file"]).read_text(encoding="utf-8"))
    assert _rpp(sim, "FollowPath")["use_collision_detection"] is False
    assert _rpp(sim, "FollowPathGoal")["use_collision_detection"] is True


def test_the_goal_controller_comes_from_the_avoidance_file(tmp_path):
    """goal controller 값은 회피 담당의 것이다. 시뮬이 자기 변경보다 먼저 넣어 속도 상한과 충돌 검사가 닿게 한다."""
    sim = yaml.safe_load(_sim(NAV, tmp_path / "s.yaml", max_speed=0.12)
                         .read_text(encoding="utf-8"))
    theirs = yaml.safe_load(AVOIDANCE.read_text(encoding="utf-8"))["follow_path_goal"]
    assert _rpp(sim, "FollowPathGoal") == {**theirs, "desired_linear_vel": 0.12,
                                           "use_collision_detection": True}


def test_a_goal_controller_that_is_no_rpp_is_left_as_the_avoidance_teammate_wrote_it(tmp_path):
    """담당이 plugin을 바꿀 수 있다. 시뮬의 속도 상한과 충돌 스위치는 RPP 키라 다른 plugin에는 넣지 않고, 고정 경로 RPP 셋은 그대로 갖는다."""
    base = _base()
    mppi = {"plugin": "nav2_mppi_controller::MPPIController", "vx_max": 0.12}
    base["controller_server"]["ros__parameters"]["FollowPathGoal"] = dict(mppi)
    path = tmp_path / "mppi.yaml"
    path.write_text(yaml.safe_dump(base), encoding="utf-8")
    sim = yaml.safe_load(launch_parts.sim_nav_params(path, tmp_path / "sim.yaml", max_speed=0.12)
                         .read_text(encoding="utf-8"))
    assert _rpp(sim, "FollowPathGoal") == mppi
    assert _rpp(sim, "FollowPath")["desired_linear_vel"] == 0.12
    assert _rpp(sim, "FollowPath")["use_collision_detection"] is True


def test_a_base_file_without_one_of_the_controllers_is_refused(tmp_path):
    base = yaml.safe_load(NAV.read_text(encoding="utf-8"))  # 사본 옆에 회피 파일 없음
    assert "FollowPathGoal" not in base["controller_server"]["ros__parameters"]
    path = tmp_path / "old.yaml"
    path.write_text(yaml.safe_dump(base), encoding="utf-8")
    with pytest.raises(ValueError, match="FollowPathGoal"):
        launch_parts.sim_nav_params(path, tmp_path / "sim.yaml")


@pytest.mark.parametrize("text, value", [("", 0.12), ("0.12", 0.12), ("0.05", 0.05), ("0.1", 0.1)])
def test_the_launch_speed_is_the_arena_range(text, value):
    assert launch_parts.speed_of(text) == value


@pytest.mark.parametrize("bad", ["0.04", "0.13", "0.25", "fast", "nan", "inf"])
def test_a_launch_speed_outside_the_arena_range_is_refused(bad):
    with pytest.raises(ValueError, match="nav_max_speed"):
        launch_parts.speed_of(bad)


def test_the_goal_controller_comes_from_the_chosen_avoidance_package_file(tmp_path):
    """avoidance_package: 담당 패키지가 제 파일을 내놓는다. base 옆에는 아무것도 없다."""
    theirs = yaml.safe_load(AVOIDANCE.read_text(encoding="utf-8"))
    theirs["follow_path_goal"] = {**theirs["follow_path_goal"], "lookahead_time": 0.9}
    path = tmp_path / "team" / "avoidance.yaml"
    path.parent.mkdir()
    path.write_text(yaml.safe_dump(theirs), encoding="utf-8")
    sim = yaml.safe_load(launch_parts.sim_nav_params(NAV, tmp_path / "s.yaml", avoidance=path)
                         .read_text(encoding="utf-8"))
    assert _rpp(sim, "FollowPathGoal")["lookahead_time"] == 0.9


def test_a_file_beside_the_base_is_not_read_any_more(tmp_path):
    (tmp_path / "nav2_params.yaml").write_bytes(NAV.read_bytes())
    (tmp_path / "avoidance.yaml").write_bytes(AVOIDANCE.read_bytes())
    with pytest.raises(ValueError, match="FollowPathGoal"):
        launch_parts.sim_nav_params(tmp_path / "nav2_params.yaml", tmp_path / "sim.yaml")


def _nav_args(tmp_path, monkeypatch, **kw):
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    context = launch.LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    includes = [a for a in launch_parts.pode("pode1", 0.5, 0.5, 0.0, fsm=False, **MAPS, **kw)
                if isinstance(a, IncludeLaunchDescription)]
    args = [{text(k): text(v) for k, v in a.launch_arguments} for a in includes]
    return next(a for a in args if "map_fine" in a)


def test_pode_reads_the_goal_controller_from_the_avoidance_package(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pode_nav_config", "pode_avoidance")
    nav = _nav_args(tmp_path, monkeypatch)
    assert nav["avoidance_package"] == "pode_avoidance"
    assert nav["vision_stand_in"] == "false"
    sim = yaml.safe_load(Path(nav["params_file"]).read_text(encoding="utf-8"))
    installed = launch_parts.share("pode_avoidance") / "params" / "avoidance.yaml"
    theirs = yaml.safe_load(installed.read_text(encoding="utf-8"))["follow_path_goal"]
    assert _rpp(sim, "FollowPathGoal")["plugin"] == theirs["plugin"]
    assert _rpp(sim, "FollowPathGoal")["lookahead_dist"] == theirs["lookahead_dist"]


def test_pode_passes_the_vision_stand_in_switch(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pode_nav_config", "pode_avoidance")
    assert _nav_args(tmp_path, monkeypatch, vision_stand_in=True)["vision_stand_in"] == "true"


def test_pode_with_an_unknown_avoidance_package_stops(tmp_path, monkeypatch):
    _need("fleet_gz_sim", "pode_nav_config")
    from ament_index_python.packages import PackageNotFoundError
    with pytest.raises(PackageNotFoundError):
        _nav_args(tmp_path, monkeypatch, avoidance_package="no_such_avoidance_package")
