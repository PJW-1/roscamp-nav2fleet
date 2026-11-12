"""머리 없는 세계의 local costmap 흉내: BLOCKED의 장애물 위치(4-1), CheckClear의 답(4-3)."""
import math

from fake_robots.world import ZoneWorld
from fake_robots.world.injections import Obstacle
from fake_robots.world.obstacle_view import clear_answer, seen_ahead
from world_helpers import follow, place, quiet_world

SHELF = (0.50, 0.775, 1.48, 0.79)


def test_the_nearest_rim_ahead_is_seen():
    seen = seen_ahead(0.5, 0.89, 0.0, [(0.8, 0.89, 0.05), (0.7, 0.89, 0.05)], 0.30)
    assert seen == (0.65, 0.89) or math.dist(seen, (0.65, 0.89)) < 1e-9


def test_rims_behind_aside_or_beyond_the_look_are_not_seen():
    discs = [(0.2, 0.89, 0.05), (0.5, 1.2, 0.05), (1.0, 0.89, 0.05)]
    assert seen_ahead(0.5, 0.89, 0.0, discs, 0.30) is None


def test_a_check_answers_from_the_objects_within_range_and_line_of_sight():
    objects = [(1.0, 0.89, 0.05)]
    assert clear_answer((0.6, 0.89), (0.95, 0.89), 0.10, objects, [SHELF], 0.6) == "blocked"
    assert clear_answer((0.6, 0.89), (0.95, 0.89), 0.10, [], [SHELF], 0.6) == "clear"
    assert clear_answer((0.2, 0.89), (0.95, 0.89), 0.10, objects, [SHELF], 0.6) == "unknown"
    assert clear_answer((1.0, 0.65), (1.0, 0.89), 0.10, objects, [SHELF], 0.6) == "unknown", (
        "behind the shelf")


def test_a_blocked_robot_reports_where_the_box_is():
    world = quiet_world(robots={"pinky1": "X_W"}, injections=[Obstacle(1.0, 0.89, 0.05)])
    place(world, "pinky1", 0.40, 0.89, 0.0)
    world.send(follow(world, "pinky1", 7, ["X_W", "X_E"]))
    world.run(4.0)
    reports = [args for _, args in world.manager.of("on_robot_state", "pinky1")
               if args[4] == "BLOCKED"]
    assert reports, "it stood and reported BLOCKED"
    *_, x, y, kind = reports[-1]
    assert math.isclose(x, 0.95, abs_tol=1e-6) and math.isclose(y, 0.89, abs_tol=1e-6)
    assert kind == "unknown"


def test_a_robot_blocked_before_it_turns_looks_along_its_way_not_its_yaw():
    """선 채로 막힌 로봇은 아직 돌지 않았다(yaw는 옛 방향). costmap은 가려던 쪽을 본다."""
    world = quiet_world(robots={"pinky1": "X_W"}, injections=[Obstacle(0.62, 0.89, 0.05)])
    place(world, "pinky1", 0.40, 0.89, math.pi / 2)  # 길은 동쪽, 몸은 북쪽을 본다
    world.send(follow(world, "pinky1", 7, ["X_W", "X_E"]))
    world.run(4.0)
    reports = [args for _, args in world.manager.of("on_robot_state", "pinky1")
               if args[4] == "BLOCKED"]
    assert reports and len(reports[-1]) == 10, "it reported what blocks it"
    *_, x, y, kind = reports[-1]
    assert math.isclose(x, 0.57, abs_tol=1e-6) and math.isclose(y, 0.89, abs_tol=1e-6)


def test_the_fleet_marks_the_express_by_the_reported_position():
    world = ZoneWorld("d2", seed=1, injections=[Obstacle(1.0, 0.89, 0.04, 20.0, 140.0)])
    world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.event_codes(), 300.0)
    detail = next(e.detail for _, e in world.events if e.code == "ZONE_MARKED_BLOCKED")
    assert "X_W->X_E, X_E->X_W at (" in detail and "in EXPRESS" in detail
