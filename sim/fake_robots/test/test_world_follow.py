"""follow_path, Retreat, Dock, Cancel 실행: 선을 떠나지 않고, 막히면 서고,
뒤가 막히면 OBSTACLE_BEHIND."""
import math

from fake_robots.world import Imperfections, Obstacle
from fake_robots.world.geom import path_gap
from fleet_core.common.commands import CancelCmd, DockCmd, RetreatCmd
from world_helpers import follow, place, quiet_world, results, states


def _line(world, *nodes):
    return [world.site.node_pose(n) for n in nodes]


def test_follows_the_line_into_the_dock_and_turns_at_the_end():
    world = quiet_world(robots={"pinky1": "N_W"})
    world.send(follow(world, "pinky1", 1, ["N_W", "X_W", "W"], yaw=math.pi))
    line = _line(world, "N_W", "X_W", "W")
    off = 0.0
    while not results(world, "on_move_result") and world.t < 30:
        world.step()
        off = max(off, path_gap(world.robots["pinky1"].pose, line))
    assert results(world, "on_move_result")[0][1:] == (1, True, "")
    assert off < 1e-9
    r = world.robots["pinky1"]
    assert math.dist(r.pose, world.site.node_pose("W")) < 1e-9
    assert abs(abs(r.yaw) - math.pi) < 1e-9


def test_an_obstacle_on_the_line_stops_it_on_the_line_and_it_reports_blocked():
    box = Obstacle(1.0, 0.89, 0.05, 0.0, 15.0)
    world = quiet_world(robots={"pinky1": "N_W"}, injections=[box])
    place(world, "pinky1", *world.site.node_pose("X_W"))
    world.send(follow(world, "pinky1", 1, ["X_W", "X_E"]))
    world.run(14.0)
    r = world.robots["pinky1"]
    assert abs(r.y - 0.89) < 1e-9 and r.x < 1.0 - 0.05 - world.radius
    assert states(world)[-1][1] == "BLOCKED"
    assert not results(world, "on_move_result")
    assert world.run_until(lambda: results(world, "on_move_result"), timeout_s=30)
    assert results(world, "on_move_result")[0][2] is True


def test_a_path_far_from_the_robot_is_refused():
    world = quiet_world(robots={"pinky1": "P-1"})
    world.send(follow(world, "pinky1", 1, ["N_W", "X_W"]))
    world.run(0.5)
    assert results(world, "on_move_result") == [(0.1, 1, False, "PATH_NOT_CONNECTED")]


def test_not_final_goal_does_not_turn_at_the_end():
    world = quiet_world(robots={"pinky1": "N_W"})
    world.send(follow(world, "pinky1", 1, ["N_W", "X_W"], yaw=math.pi, final=False))
    assert world.run_until(lambda: results(world, "on_move_result"), timeout_s=10)
    assert abs(world.robots["pinky1"].yaw - math.pi / 2) < 1e-9  # 북쪽을 보고 달려 왔다


def test_a_robot_starting_beside_the_line_comes_back_to_it_and_ends_on_it():
    world = quiet_world(robots={"pinky1": "N_W"},
                        imperfect=Imperfections(0.0, 0.0, 0.01, 0.0, 0.0, 1.0))
    place(world, "pinky1", 0.36, 0.65)
    world.send(follow(world, "pinky1", 1, ["N_W", "X_W", "W"], yaw=math.pi))
    line = _line(world, "N_W", "X_W", "W")
    gaps = []
    while not results(world, "on_move_result") and world.t < 30:
        world.step()
        gaps.append(path_gap(world.robots["pinky1"].pose, line))
    assert results(world, "on_move_result")[0][2] is True
    assert max(gaps) <= 0.01 + 1e-9
    assert abs(world.robots["pinky1"].y - 0.89) < 1e-9  # 마지막 선분 위(지나치면 그 연장선 위)
    assert math.dist(world.robots["pinky1"].pose, world.site.node_pose("W")) <= 0.01 + 1e-9


def test_the_world_refuses_a_step_that_would_cross_another_body():
    world = quiet_world(robots={"pinky1": "N_W", "pinky2": "N_E"})
    place(world, "pinky1", 1.0, 0.4)
    place(world, "pinky2", 1.1, 0.4)
    r = world.robots["pinky1"]
    assert not world.clear_to(r, (1.2, 0.4))  # 몸을 지나 멀어지는 걸음도 막는다
    assert world.clear_to(r, (0.9, 0.4))
    assert world.guard_stops == 1


def _retreat(world, goal_id, nodes):
    points = tuple((*world.site.node_pose(n), 0.0) for n in nodes)
    distance = sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:], strict=False))
    return RetreatCmd("pinky1", goal_id, distance, 0.05, points)


def test_retreat_backs_along_the_path_facing_the_same_way():
    world = quiet_world(robots={"pinky1": "N_W"})
    place(world, "pinky1", *world.site.node_pose("W"), yaw=math.pi)
    world.send(_retreat(world, 1, ["W", "X_W"]))
    assert world.run_until(lambda: results(world, "on_retreat_result"), timeout_s=10)
    (t, goal, ok, _), = results(world, "on_retreat_result")
    assert (goal, ok) == (1, True)
    assert 0.17 / 0.05 - 0.2 <= t <= 0.17 / 0.05 + 0.2  # W에서 X_W까지 0.17 m
    r = world.robots["pinky1"]
    assert math.dist(r.pose, world.site.node_pose("X_W")) < 1e-9 and r.yaw == math.pi


def test_retreat_stops_with_obstacle_behind():
    box = Obstacle(0.42, 0.89, 0.04)
    world = quiet_world(robots={"pinky1": "N_W"}, injections=[box])
    place(world, "pinky1", *world.site.node_pose("W"), yaw=math.pi)
    world.send(_retreat(world, 1, ["W", "X_W"]))
    assert world.run_until(lambda: results(world, "on_retreat_result"), timeout_s=10)
    assert results(world, "on_retreat_result")[0][2:] == (False, "OBSTACLE_BEHIND")
    assert math.dist(world.robots["pinky1"].pose, (0.42, 0.89)) >= world.radius + 0.04


def test_dock_takes_its_time_and_fails_away_from_the_dock():
    world = quiet_world(robots={"pinky1": "N_W"}, dock_s=2.0)
    place(world, "pinky1", *world.site.node_pose("W"))
    world.send(DockCmd("pinky1", 1, "WORK"))
    world.run(1.0)
    assert states(world)[-1][1] == "DOCKING"
    world.run(1.5)
    assert results(world, "on_dock_result") == [(2.0, 1, True, 0.4)]
    assert world.robots["pinky1"].docked_at == "WORK"
    place(world, "pinky1", *world.site.node_pose("N_W"))
    world.send(DockCmd("pinky1", 2, "WORK"))
    world.run(2.5)
    assert results(world, "on_dock_result")[-1][1:3] == (2, False)
    assert "I4_COMMAND" in [v.code for v in world.violations]


def test_cancel_stops_the_robot_and_answers_canceled_next_step():
    world = quiet_world(robots={"pinky1": "N_W"})
    world.send(follow(world, "pinky1", 7, ["N_W", "X_W"]))
    world.run(1.0)
    here = world.robots["pinky1"].pose
    world.send(CancelCmd("pinky1", 7))
    world.send(CancelCmd("pinky1", 99))  # 모르는 goal은 무시
    world.run(1.0)
    assert results(world, "on_move_result") == [(1.1, 7, False, "CANCELED")]
    assert world.robots["pinky1"].pose == here
