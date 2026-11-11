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
from world_helpers import nav, quiet_world, results, states


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
