"""세계의 흠과 주입: 같은 씨드는 같은 기록, 통신 끊김, 치우기와 다시 넣기, 멈춤, 튀는 보고,
배터리."""
import math

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
    world.send(nav(world, "pinky1", 1, "N_E"))
    world.send(nav(world, "pinky3", 2, "O_Q"))
    world.run(20.0)
    return world.manager.calls


def test_the_manager_comes_from_the_factory_with_the_layout_and_the_overrides():
    world = ZoneWorld("c", seed=1, orders=[], manager_factory=RecordingManager,
                      overrides={"nav_speed_mps": 0.15})
    m = world.manager
    assert m.site.name == "arena_c" and m.robots == ["pinky1", "pinky2", "pinky3"]
    assert (m.params.traffic_mode, m.params.nav_speed_mps) == ("zones", 0.15)
    assert [world.robots[n].pose for n in m.robots] == [
        world.site.node_pose(p) for p in world.site.parking]


def test_the_same_seed_gives_the_same_record_and_another_seed_does_not():
    assert _trace(4) == _trace(4)
    assert _trace(4) != _trace(5)


def test_imperfections_slow_the_robot_and_delay_reports():
    world = ZoneWorld("d2", seed=3, orders=[], manager_factory=RecordingManager,
                      imperfect=Imperfections(speed_spread=0.2, report_delay_max_s=0.3))
    r = world.robots["pinky1"]
    assert 0.8 <= r.profile.speed_factor <= 1.0
    delay = r.profile.report_delay_s
    world.run(1.0)
    first = world.manager.of("on_robot_state", "pinky1")[0][0]
    assert math.isclose(first, 0.1 + delay)


def test_comm_loss_stops_reports_and_drops_commands_and_results():
    world = quiet_world(injections=[CommLoss("pinky1", 1.0, 4.0)])
    world.run(2.0)
    world.send(nav(world, "pinky1", 1, "W_Q"))
    world.run(3.0)
    seen = [t for t, _ in states(world)]
    assert not [t for t in seen if 1.0 <= t < 4.0]
    assert max(seen) >= 4.0
    assert world.robots["pinky1"].run is None  # 끊긴 동안의 명령은 닿지 않았다


def test_removal_clears_its_zones_and_reinstate_puts_it_back():
    world = quiet_world(injections=[Removal("pinky1", 1.0), Reinstate("pinky1", 3.0, "P-2")])
    world.manager.traffic.grants.give("pinky1", "HALL")
    world.run(2.0)
    assert world.manager.of("confirm_clear") == [(1.0, ("pinky1", ["HALL"],
                                                       "removed from the floor"))]
    last = max(t for t, _ in states(world))
    assert last < 1.0
    world.run(2.0)
    assert world.manager.of("reinstate")[0][1][:2] == ("pinky1", "EMPTY")
    assert world.robots["pinky1"].pose == world.site.node_pose("P-2")
    assert max(t for t, _ in states(world)) >= 3.0


def test_a_reinstate_onto_a_taken_spot_puts_the_robot_on_the_nearest_free_one():
    """운영자는 다른 로봇이 선 자리에 로봇을 겹쳐 놓지 않는다."""
    world = quiet_world(robots={"pinky1": "P-1", "pinky2": "P-2"},
                        injections=[Removal("pinky1", 1.0), Reinstate("pinky1", 3.0, "P-2")])
    world.run(4.0)
    spot = world.robots["pinky1"].pose
    assert spot != world.site.node_pose("P-2")
    assert any(spot == world.site.node_pose(n) for n in world.site.nodes
               if world.site.can_wait(n))
    assert not [v for v in world.violations if v.code == "I6_TOO_CLOSE"]


def test_a_reinstate_never_lands_on_a_dock_or_a_spot_another_robot_reserved():
    """운영자는 도크에 놓지 않는다. 관제가 다른 로봇에 잡아 준 지점에도 놓지 않는다(I8)."""
    world = quiet_world("c", robots={"pinky2": "P-1", "pinky3": "P-2"})
    assert world._free_spot("P-2", "pinky2") == "P-1"  # W가 더 가깝지만 도크다
    world = quiet_world(robots={"pinky1": "N_E", "pinky2": "P-1", "pinky3": "P-2"})
    world.manager.traffic.grants.robot_nodes = {"pinky1": ("P-1",)}
    world.robots["pinky2"].x = 1.0  # P-1은 비었지만 pinky1이 잡았다
    assert world._free_spot("P-2", "pinky2") == "W_Q"


def test_a_retreat_out_of_a_dock_is_counted_apart_from_other_retreats():
    world = quiet_world(robots={"pinky1": "W"})
    world.robots["pinky1"].docked_at = "WORK"
    x, y = world.site.node_pose("W")
    world.send(RetreatCmd("pinky1", 1, 0.2, 0.05, ((x, y, 0.0), (x + 0.2, y, 0.0))))
    world.send(RetreatCmd("pinky1", 2, 0.1, 0.05, ()))
    assert (world.metrics.dock_exits, world.metrics.retreats) == (1, 1)


def test_a_stuck_robot_standing_in_a_junction_is_not_an_invariant_violation():
    """계약 7절 5번은 교통 층이 로봇을 세워 둔 경우다. 고장 주입으로 선 로봇은 빼고 본다."""
    world = quiet_world(robots={"pinky1": "X_W"}, injections=[Stuck("pinky1", 0.0)])
    world.send(follow(world, "pinky1", 1, ["X_W", "W"]))
    world.run(8.0)
    assert not [v for v in world.violations if v.code == "I5_STANDING"]


def test_a_stuck_robot_does_not_move_and_its_move_times_out():
    world = quiet_world(injections=[Stuck("pinky1", 0.0)])
    world.send(nav(world, "pinky1", 1, "N_E"))
    world.run(11.0)
    assert results(world, "on_move_result")[0][2:] == (False, "TIMEOUT")
    assert world.robots["pinky1"].pose == world.site.node_pose("P-1")


def test_noisy_reports_jump_but_the_true_pose_stays():
    world = quiet_world(injections=[NoisyPose("pinky1", 1.0, 2.0, sigma_m=0.0, jump_m=0.3)])
    world.run(3.0)
    xs = {round(args[1], 3) for _, args in world.manager.of("on_robot_state", "pinky1")}
    x0 = world.site.node_pose("P-1")[0]
    assert xs == {round(x0, 3), round(x0 + 0.3, 3)}


def test_low_battery_is_reported_and_charges_on_a_parking_spot():
    world = quiet_world(injections=[LowBattery("pinky1", 1.0, 5.0)])
    world.run(8.0)
    levels = [args[5] for _, args in world.manager.of("on_robot_state", "pinky1")]
    assert levels[0] == 90.0 and min(levels) == 10.0 and levels[-1] == 90.0
