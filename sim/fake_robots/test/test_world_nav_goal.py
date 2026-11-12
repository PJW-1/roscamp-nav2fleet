"""nav_goal 흉내(지시서 6-1): 스스로 길을 짜 가고, 돌아가고, 서고, BLOCKED, NAV_ABORTED."""
import math

from fake_robots.world import Obstacle
from world_helpers import nav, place, quiet_world, results, states


def test_drives_to_the_goal_turns_to_its_heading_and_reports_success():
    world = quiet_world()
    world.send(nav(world, "pinky1", 1, "N_E", yaw=math.pi / 2))
    assert world.run_until(lambda: results(world, "on_move_result"), timeout_s=60)
    (t, goal, ok, reason), = results(world, "on_move_result")
    r = world.robots["pinky1"]
    assert (goal, ok, reason) == (1, True, "")
    assert math.dist(r.pose, world.site.node_pose("N_E")) < 1e-6
    assert abs(r.yaw - math.pi / 2) < 1e-6
    # 약 1.53 m를 0.12 m/s로, 끝에서 조금 돈다
    distance = math.dist(world.site.node_pose("P-1"), world.site.node_pose("N_E"))
    assert distance / 0.12 < t < distance / 0.12 + 3.0
    assert "MOVING" in {s for _, s in states(world)}


def test_goes_around_a_standing_robot_without_touching_it():
    world = quiet_world(robots={"pinky1": "N_W", "pinky2": "P-2"})
    place(world, "pinky2", 1.0, 0.60)
    world.send(nav(world, "pinky1", 1, "N_E"))
    gaps = []
    while not results(world, "on_move_result") and world.t < 60:
        world.step()
        gaps.append(world.robots["pinky1"].gap(world.robots["pinky2"]))
    assert results(world, "on_move_result")[0][2:] == (True, "")
    assert min(gaps) >= 2 * world.radius
    assert world.guard_stops == 0


def test_goes_around_a_box_in_the_hall():
    box = Obstacle(1.0, 0.65, 0.05)
    world = quiet_world(robots={"pinky1": "N_W"}, injections=[box])
    world.send(nav(world, "pinky1", 1, "N_E"))
    least = math.inf
    while not results(world, "on_move_result") and world.t < 60:
        world.step()
        least = min(least, math.dist(world.robots["pinky1"].pose, (1.0, 0.65)))
    assert results(world, "on_move_result")[0][2:] == (True, "")
    assert least >= world.radius + 0.05


def _wall(t0=0.0, t1=math.inf):
    return [Obstacle(1.0, y / 100, 0.05, t0, t1) for y in range(0, 80, 4)]


def test_stops_reports_blocked_after_ten_seconds_and_goes_on_when_the_way_opens():
    world = quiet_world(robots={"pinky1": "N_W"}, injections=_wall(0.0, 30.0))
    world.send(nav(world, "pinky1", 1, "N_E"))
    world.run(29.0)
    seen = states(world)
    first_blocked = next(t for t, s in seen if s == "BLOCKED")
    stood = next(t for t, s in seen if s == "MOVING")
    # 서고 나서 10 s(계약 0.13-draft의 blocked_after_s. 벽까지 가는 시간 안쪽)
    assert first_blocked >= stood + 10.0 - 1.0
    assert seen[-1][1] == "BLOCKED"
    assert not results(world, "on_move_result")
    assert world.robots["pinky1"].x < 1.0 - 0.05 - world.radius
    assert world.run_until(lambda: results(world, "on_move_result"), timeout_s=30)
    assert results(world, "on_move_result")[0][2:] == (True, "")


def test_no_way_on_the_map_ends_with_nav_aborted():
    world = quiet_world(robots={"pinky1": "N_W"})
    world.send(nav(world, "pinky1", 1, "X_E"))  # 갈림목은 keepout
    world.run(1.0)
    assert results(world, "on_move_result") == [(0.6, 1, False, "NAV_ABORTED")]  # 0.1 + 0.5
    assert world.robots["pinky1"].pose == world.site.node_pose("N_W")


def test_a_way_closed_for_good_ends_with_nav_aborted_after_the_give_up_time():
    world = quiet_world(robots={"pinky1": "N_W"}, injections=_wall())
    world.send(nav(world, "pinky1", 1, "N_E"))
    assert world.run_until(lambda: results(world, "on_move_result"), timeout_s=120)
    assert results(world, "on_move_result")[0][2:] == (False, "NAV_ABORTED")


def test_a_new_goal_replaces_the_running_one():
    world = quiet_world()
    world.send(nav(world, "pinky1", 1, "N_E"))
    world.run(2.0)
    world.send(nav(world, "pinky1", 2, "W_Q"))
    assert world.run_until(lambda: len(results(world, "on_move_result")) == 2, timeout_s=60)
    (_, g1, ok1, why1), (_, g2, ok2, _) = results(world, "on_move_result")
    assert (g1, ok1, why1) == (1, False, "CANCELED")
    assert (g2, ok2) == (2, True)
    assert math.dist(world.robots["pinky1"].pose, world.site.node_pose("W_Q")) < 1e-6


def test_two_robots_swapping_places_pass_each_other_without_touching():
    world = quiet_world(robots={"pinky1": "N_W", "pinky2": "N_E"})
    world.send(nav(world, "pinky1", 1, "N_E"))
    world.send(nav(world, "pinky2", 2, "N_W"))
    world.run(60.0)
    for robot in ("pinky1", "pinky2"):
        assert results(world, "on_move_result", robot)[0][2:] == (True, "")
    assert world.metrics.min_gap >= 2 * world.radius
    assert not [v for v in world.violations if v.code == "I6_TOO_CLOSE"]
