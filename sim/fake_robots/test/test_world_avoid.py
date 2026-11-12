"""회피 흉내 세 모드(추가 지시서 2의 1절): none은 정적 지도 길만, perfect는 돌아감, mixed는 섞음."""
import math

import pytest
from fake_robots.world import RecordingManager, ZoneWorld
from fake_robots.world.avoid import AVOID_MODES, MixedSettings
from world_helpers import nav, place, quiet_world, results, states


def _blocked_by_a_standing_robot(mode, **kw):
    """pinky1이 N_W에서 N_E로 간다. pinky2가 그 곧은 길 한가운데(y 0.65)에 서 있다."""
    world = quiet_world(robots={"pinky1": "N_W", "pinky2": "P-2"}, avoid_mode=mode, **kw)
    place(world, "pinky2", 1.0, 0.65)
    world.send(nav(world, "pinky1", 1, "N_E"))
    return world


def test_perfect_is_the_default_and_an_unknown_mode_is_refused():
    assert AVOID_MODES == ("none", "perfect", "mixed")
    assert quiet_world().avoid_mode == "perfect"
    with pytest.raises(ValueError, match="avoid_mode"):
        quiet_world(avoid_mode="clever")


def test_none_never_goes_around_it_stands_and_reports_blocked():
    world = _blocked_by_a_standing_robot("none")
    ys = []
    for _ in range(300):
        world.step()
        ys.append(world.robots["pinky1"].y)
    r = world.robots["pinky1"]
    assert max(abs(y - 0.65) for y in ys) < 1e-9  # 길을 떠나지 않는다
    assert r.x < 1.0 - 2 * world.radius
    assert states(world)[-1][1] == "BLOCKED"
    assert not results(world, "on_move_result")
    assert world.guard_stops == 0


def test_none_goes_on_when_the_way_opens():
    world = _blocked_by_a_standing_robot("none")
    world.run(15.0)
    place(world, "pinky2", 1.0, 0.30)
    assert world.run_until(lambda: results(world, "on_move_result"), timeout_s=30)
    assert results(world, "on_move_result")[0][2:] == (True, "")


def test_none_two_robots_that_meet_both_stop():
    world = quiet_world(robots={"pinky1": "N_W", "pinky2": "N_E"}, avoid_mode="none")
    world.send(nav(world, "pinky1", 1, "N_E"))
    world.send(nav(world, "pinky2", 2, "N_W"))
    world.run(30.0)
    a, b = world.robots["pinky1"], world.robots["pinky2"]
    assert a.gap(b) >= 2 * world.radius
    assert states(world, "pinky1")[-1][1] == states(world, "pinky2")[-1][1] == "BLOCKED"
    assert not results(world, "on_move_result") and not results(world, "on_move_result", "pinky2")


def test_perfect_goes_around_the_same_robot():
    world = _blocked_by_a_standing_robot("perfect")
    assert world.run_until(lambda: results(world, "on_move_result"), timeout_s=60)
    assert results(world, "on_move_result")[0][2:] == (True, "")


def test_mixed_that_always_detours_goes_around_after_a_delay_within_the_limit():
    settings = MixedSettings(p_detour=1.0, delay_max_s=3.0, stray_p=0.0)
    world = _blocked_by_a_standing_robot("mixed", avoid_settings=settings)
    stood, left, last = None, None, world.robots["pinky1"].pose
    while not results(world, "on_move_result") and world.t < 60:
        world.step()
        r = world.robots["pinky1"]
        if stood is None and world.t > 1.0 and r.pose == last:
            stood = world.t  # 곧은 길로 가다 pinky2 앞에서 섰다
        if left is None and abs(r.y - 0.65) > 1e-6:
            left = world.t  # 비켜 가기 시작했다
        last = r.pose
    assert results(world, "on_move_result")[0][2:] == (True, "")
    assert stood is not None and left is not None
    assert 0.0 <= left - stood <= 3.0 + 0.2


def test_mixed_that_never_detours_behaves_like_none():
    settings = MixedSettings(p_detour=0.0, stray_p=0.0)
    world = _blocked_by_a_standing_robot("mixed", avoid_settings=settings)
    world.run(30.0)
    assert abs(world.robots["pinky1"].y - 0.65) < 1e-9
    assert states(world)[-1][1] == "BLOCKED"


def test_mixed_is_the_same_for_the_same_seed():
    def trace(seed):
        w = ZoneWorld("d2", seed=seed, robots={"pinky1": "N_W", "pinky2": "P-2"}, orders=[],
                      avoid_mode="mixed", manager_factory=RecordingManager)
        place(w, "pinky2", 1.0, 0.65)
        w.send(nav(w, "pinky1", 1, "N_E"))
        out = []
        for _ in range(400):
            w.step()
            out.append(w.robots["pinky1"].pose)
        return out
    assert trace(4) == trace(4)


def test_a_mixed_run_can_stray_sideways_but_no_more_than_the_limit():
    settings = MixedSettings(stray_p=1.0, stray_max_m=0.05)
    world = quiet_world(robots={"pinky1": "N_W"}, avoid_mode="mixed", avoid_settings=settings)
    world.send(nav(world, "pinky1", 1, "N_E"))
    off = 0.0
    while not results(world, "on_move_result") and world.t < 60:
        world.step()
        off = max(off, abs(world.robots["pinky1"].y - 0.65))
    assert results(world, "on_move_result")[0][2:] == (True, "")
    assert 0.0 < off <= 0.05 + 1e-9
    assert math.dist(world.robots["pinky1"].pose, world.site.node_pose("N_E")) < 1e-6
