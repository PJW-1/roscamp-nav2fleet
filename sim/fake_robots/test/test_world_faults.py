"""세계의 흠과 주입: 같은 씨드는 같은 기록, 통신 끊김, 치우기와 다시 넣기, 멈춤, 튀는 보고,
배터리."""
import math
from dataclasses import replace

from fake_robots.world import (
    CommLoss,
    Imperfections,
    LowBattery,
    NoisyPose,
    RecordingManager,
    Reinstate,
    Removal,
    Stuck,
    ZoneWorld,
)
from fleet_core.common.commands import RetreatCmd
from world_helpers import follow, nav, quiet_world, results, states


def _trace(seed):
    world = ZoneWorld("d2", seed=seed, orders=[], manager_factory=RecordingManager)
    world.send(nav(world, "pode1", 1, "N_E"))
    world.send(nav(world, "pode3", 2, "O_Q"))
    world.run(20.0)
    return world.manager.calls


def test_the_manager_comes_from_the_factory_with_the_layout_and_the_overrides():
    world = ZoneWorld("c", seed=1, orders=[], manager_factory=RecordingManager,
                      overrides={"nav_speed_mps": 0.15})
    m = world.manager
    assert m.site.name == "arena_c" and m.robots == ["pode1", "pode2", "pode3"]
    assert (m.params.traffic_mode, m.params.nav_speed_mps) == ("zones", 0.15)
    assert [world.robots[n].pose for n in m.robots] == [
        world.site.node_pose(p) for p in world.site.parking]


def test_the_same_seed_gives_the_same_record_and_another_seed_does_not():
    assert _trace(4) == _trace(4)
    assert _trace(4) != _trace(5)


def test_imperfections_slow_the_robot_and_delay_reports():
    world = ZoneWorld("d2", seed=3, orders=[], manager_factory=RecordingManager,
                      imperfect=Imperfections(speed_spread=0.2, report_delay_max_s=0.3))
    r = world.robots["pode1"]
    assert 0.8 <= r.profile.speed_factor <= 1.0
    delay = r.profile.report_delay_s
    world.run(1.0)
    first = world.manager.of("on_robot_state", "pode1")[0][0]
    assert math.isclose(first, 0.1 + delay)


def test_comm_loss_stops_reports_and_drops_commands_and_results():
    world = quiet_world(injections=[CommLoss("pode1", 1.0, 4.0)])
    world.run(2.0)
    world.send(nav(world, "pode1", 1, "W_Q"))
    world.run(3.0)
    seen = [t for t, _ in states(world)]
    assert not [t for t in seen if 1.0 <= t < 4.0]
    assert max(seen) >= 4.0
    assert world.robots["pode1"].run is None  # 끊긴 동안의 명령은 닿지 않았다


def test_removal_clears_its_zones_and_reinstate_puts_it_back():
    world = quiet_world(injections=[Removal("pode1", 1.0), Reinstate("pode1", 3.0, "P-2")])
    world.manager.traffic.grants.give("pode1", "HALL")
    world.run(2.0)
    assert world.manager.of("confirm_clear") == [(1.0, ("pode1", ["HALL"],
                                                       "removed from the floor"))]
    last = max(t for t, _ in states(world))
    assert last < 1.0
    world.run(2.0)
    assert world.manager.of("reinstate")[0][1][:2] == ("pode1", "EMPTY")
    assert world.robots["pode1"].pose == world.site.node_pose("P-2")
    assert max(t for t, _ in states(world)) >= 3.0


def test_a_reinstate_onto_a_taken_spot_puts_the_robot_on_the_nearest_free_one():
    """운영자는 다른 로봇이 선 자리에 로봇을 겹쳐 놓지 않는다."""
    world = quiet_world(robots={"pode1": "P-1", "pode2": "P-2"},
                        injections=[Removal("pode1", 1.0), Reinstate("pode1", 3.0, "P-2")])
    world.run(4.0)
    spot = world.robots["pode1"].pose
    assert spot != world.site.node_pose("P-2")
    assert any(spot == world.site.node_pose(n) for n in world.site.nodes
               if world.site.can_wait(n))
    assert not [v for v in world.violations if v.code == "I6_TOO_CLOSE"]


def test_a_reinstate_never_lands_on_a_dock_or_a_spot_another_robot_reserved():
    """운영자는 도크에 놓지 않는다. 관제가 다른 로봇에 잡아 준 지점에도 놓지 않는다(I8)."""
    world = quiet_world("c", robots={"pode2": "P-1", "pode3": "P-2"})
    assert world._free_spot("P-2", "pode2") == "P-1"  # W가 더 가깝지만 도크다
    world = quiet_world(robots={"pode1": "N_E", "pode2": "P-1", "pode3": "P-2"})
    world.manager.traffic.grants.robot_nodes = {"pode1": ("P-1",)}
    world.robots["pode2"].x = 1.0  # P-1은 비었지만 pode1이 잡았다
    assert world._free_spot("P-2", "pode2") == "W_Q"


def test_a_retreat_out_of_a_dock_is_counted_apart_from_other_retreats():
    world = quiet_world(robots={"pode1": "W"})
    world.robots["pode1"].docked_at = "WORK"
    x, y = world.site.node_pose("W")
    world.send(RetreatCmd("pode1", 1, 0.2, 0.05, ((x, y, 0.0), (x + 0.2, y, 0.0))))
    world.send(RetreatCmd("pode1", 2, 0.1, 0.05, ()))
    assert (world.metrics.dock_exits, world.metrics.retreats) == (1, 1)


def test_a_stuck_robot_standing_in_a_junction_is_not_an_invariant_violation():
    """계약 7절 5번은 교통 층이 로봇을 세워 둔 경우다. 고장 주입으로 선 로봇은 빼고 본다."""
    world = quiet_world(robots={"pode1": "X_W"}, injections=[Stuck("pode1", 0.0)])
    world.send(follow(world, "pode1", 1, ["X_W", "W"]))
    world.run(8.0)
    assert not [v for v in world.violations if v.code == "I5_STANDING"]


def test_a_stuck_robot_does_not_move_and_its_move_times_out():
    world = quiet_world(injections=[Stuck("pode1", 0.0)])
    world.send(nav(world, "pode1", 1, "N_E"))
    world.run(11.0)
    assert results(world, "on_move_result")[0][2:] == (False, "TIMEOUT")
    assert world.robots["pode1"].pose == world.site.node_pose("P-1")


def test_noisy_reports_jump_but_the_true_pose_stays():
    world = quiet_world(injections=[NoisyPose("pode1", 1.0, 2.0, sigma_m=0.0, jump_m=0.3)])
    world.run(3.0)
    xs = {round(args[1], 3) for _, args in world.manager.of("on_robot_state", "pode1")}
    x0 = world.site.node_pose("P-1")[0]
    assert xs == {round(x0, 3), round(x0 + 0.3, 3)}


def test_low_battery_is_reported_and_charges_on_a_parking_spot():
    world = quiet_world(injections=[LowBattery("pode1", 1.0, 5.0)])
    world.run(8.0)
    levels = [args[5] for _, args in world.manager.of("on_robot_state", "pode1")]
    assert levels[0] == 90.0 and min(levels) == 10.0 and levels[-1] == 90.0


# ---- 믿는 자리로 달리기(drive_by_estimate) ----

def _drive_trace(injections, seconds=12.0):
    """N_W에서 N_E로 따라가며 걸음마다 (시각, 실제 x, 실제 y, 보고 x, 보고 y)."""
    world = quiet_world(robots={"pode1": "N_W"}, injections=injections)
    world.send(follow(world, "pode1", 1, ["N_W", "N_E"]))
    out = []
    for _ in range(round(seconds / world.dt)):
        world.step()
        r = world.robots["pode1"]
        _, args = world.manager.of("on_robot_state", "pode1")[-1]
        out.append((world.t, r.x, r.y, args[1], args[2]))
    return world, out


def test_driving_by_estimate_offsets_the_body_by_the_bias_and_the_report_stays_on_the_path():
    bias = 0.2
    world, trace = _drive_trace([NoisyPose("pode1", 1.0, 30.0, jump_m=bias,
                                           drive_by_estimate=True)])
    y0 = world.site.node_pose("N_W")[1]
    end_x = world.site.node_pose("N_E")[0]
    inside = [s for s in trace if 1.0 + 1e-9 < s[0] < 30.0]
    assert inside
    for _, x, y, rx, ry in inside:
        assert abs(rx - x - bias) < 1e-9  # 보고 = 믿는 자리 = 실제 + 치우침
        assert abs(ry - y) < 1e-9 and abs(y - y0) < 1e-9
    assert results(world, "on_move_result")[0][2] is True
    _, x, _, rx, _ = trace[-1]
    assert abs(rx - end_x) < 1e-6  # 믿는 자리로는 도착
    assert abs(x - (end_x - bias)) < 1e-6  # 몸은 치우침만큼 못 미쳤다


def test_after_the_window_the_belief_snaps_back_and_the_robot_drives_from_its_true_pose():
    world, trace = _drive_trace([NoisyPose("pode1", 1.0, 2.0, jump_m=0.2,
                                           drive_by_estimate=True)], seconds=15.0)
    end = world.site.node_pose("N_E")
    after = [s for s in trace if s[0] >= 2.0]
    for _, x, y, rx, ry in after:
        assert (rx, ry) == (x, y)  # 창이 끝나면 보고가 실제로 돌아온다
    _, x, y, _, _ = trace[-1]
    assert results(world, "on_move_result")[0][2] is True
    assert math.dist((x, y), end) < 1e-6  # 실제 자리에서 이어 가 끝에 선다


def test_without_drive_by_estimate_the_body_drives_as_if_there_were_no_pose_error():
    clean = _drive_trace([])[1]
    _, noisy = _drive_trace([NoisyPose("pode1", 1.0, 30.0, jump_m=0.2)])
    assert [s[:3] for s in noisy] == [s[:3] for s in clean]
    assert NoisyPose("pode1", 1.0, 2.0) == NoisyPose("pode1", 1.0, 2.0, drive_by_estimate=False)


def test_a_window_closing_while_turning_at_the_end_drives_the_body_back_to_the_goal():
    # 믿는 자리로 9.7 s쯤 끝에 닿고 3 s 넘게 돈다. 도는 동안 창이 닫힌다
    world = quiet_world(robots={"pode1": "N_W"}, injections=[
        NoisyPose("pode1", 1.0, 11.0, jump_m=0.2, drive_by_estimate=True)])
    world.send(follow(world, "pode1", 1, ["N_W", "N_E"], yaw=math.pi))
    world.run(20.0)
    assert results(world, "on_move_result")[0][2] is True
    assert math.dist(world.robots["pode1"].pose, world.site.node_pose("N_E")) < 1e-6


def test_rebase_moves_the_first_point_and_the_progress_mark_and_keeps_a_reached_end():
    from types import SimpleNamespace

    from fake_robots.world.world import _rebase
    run = SimpleNamespace(way=[(0.0, 0.0), (1.0, 0.0)], mark=(0.0, 0.0))
    _rebase(run, (0.2, 0.0), 0.2)
    assert run.way == [(0.2, 0.0), (1.0, 0.0)] and run.mark == (0.2, 0.0)
    done = SimpleNamespace(way=[(1.0, 0.0)])  # 끝에 닿아 도는 중
    _rebase(done, (0.8, 0.0), -0.2)
    assert done.way == [(0.8, 0.0), (1.0, 0.0)]


def test_pose_jumps_drive_is_pose_jumps_driven_by_the_estimate_and_stays_out_of_the_table():
    from fake_robots.world.situations import NAMES, build
    plain, drive = build("pose_jumps", "d2"), build("pose_jumps_drive", "d2")
    assert [replace(n, drive_by_estimate=True) for n in plain.injections] == drive.injections
    assert not plain.injections[0].drive_by_estimate
    assert "pose_jumps_drive" not in NAMES
