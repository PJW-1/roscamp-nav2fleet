"""로봇단 양보 사다리 흉내: 켜면 가짜 로봇이 pode_fsm과 같은 순서로
1초 서고, local_wait_s 기다리고, 낮은 쪽만 물러나고, 다시 가고, 그래도 막히면 BLOCKED다.
끄면(기본) 지금과 같다."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "pode" / "pode_fsm"))

from fake_robots.world.table import run_situation  # noqa: E402
from fleet_core.common.commands import MoveCmd, MoveOrderCmd, SegmentOut  # noqa: E402
from pode_fsm.params import FsmParams  # noqa: E402
from world_helpers import nav, place, quiet_world  # noqa: E402

P = FsmParams()
LANE_Y = 0.89  # d2 급행 X_W(0.35) - X_E(1.63)


def reports(world, robot):
    """(시각, 상태, local_yield 단계). 단계가 없는 보고는 빈 문자열."""
    out = []
    for t, args in world.manager.of("on_robot_state", robot):
        out.append((t, args[4], args[10] if len(args) > 10 else ""))
    return out


def steps(world, robot):
    """단계가 바뀐 순서(같은 단계가 이어지면 하나로)."""
    out = []
    for _, state, step in reports(world, robot):
        label = step or state
        if not out or out[-1] != label:
            out.append(label)
    return out


def east(world, robot, goal_id, x0, x1):
    return MoveCmd(robot, goal_id, "", x1, LANE_Y, 0.0,
                   path=((x0, LANE_Y, 0.0), (x1, LANE_Y, 0.0)))


def west(world, robot, goal_id, x0, x1):
    return MoveCmd(robot, goal_id, "", x1, LANE_Y, math.pi,
                   path=((x0, LANE_Y, math.pi), (x1, LANE_Y, math.pi)))


def lane_pair(**kw):
    world = quiet_world(robots={"pode1": "X_W", "pode2": "X_E"}, local_yield=True, **kw)
    place(world, "pode1", 0.35, LANE_Y, 0.0)
    place(world, "pode2", 1.15, LANE_Y, math.pi)
    return world


def test_off_by_default_reports_blocked_without_recovering():
    world = quiet_world(robots={"pode1": "X_W", "pode2": "X_E"})
    place(world, "pode1", 0.35, LANE_Y, 0.0)
    place(world, "pode2", 1.15, LANE_Y, math.pi)
    world.send(east(world, "pode1", 1, 0.35, 1.63))
    world.run(8.0)
    assert "RECOVERING" not in [s for _, s, _ in reports(world, "pode1")]
    assert reports(world, "pode1")[-1][1] == "BLOCKED"
    assert all(len(args) <= 10 for _, args in world.manager.of("on_robot_state"))


def test_the_lower_robot_waits_backs_up_goes_again_and_then_reports_blocked():
    world = lane_pair()
    world.send(east(world, "pode1", 1, 0.35, 1.63))  # pode2는 서서 pode1 쪽을 본다
    xs = []
    for _ in range(150):
        world.step()
        xs.append((world.t, world.robots["pode1"].x))
    assert steps(world, "pode1") == ["MOVING", "wait", "backup", "retry", "BLOCKED"]
    rows = reports(world, "pode1")
    stop_x = max(x for _, x in xs)
    backup_from = next(t for t, _, step in rows if step == "backup")
    retry_from = next(t for t, _, step in rows if step == "retry")
    low = min(x for t, x in xs if backup_from <= t <= retry_from)
    assert stop_x - low == pytest.approx(P.local_backup_m, abs=0.01)
    after = [x for t, x in xs if t > retry_from]
    assert max(after) > low + 0.05  # 다시 앞으로 간다
    wait_from = next(t for t, _, step in rows if step == "wait")
    assert backup_from - wait_from == pytest.approx(P.local_wait_s, abs=0.15)
    blocked_at = next(t for t, s, _ in rows if s == "BLOCKED")
    assert blocked_at - wait_from < P.blocked_after_s


def test_the_higher_robot_waits_and_reports_blocked_without_backing_up():
    world = lane_pair()
    world.send(west(world, "pode2", 1, 1.15, 0.35))  # pode1이 서서 pode2 쪽을 본다
    world.robots["pode1"].yaw = 0.0
    xs = []
    for _ in range(100):
        world.step()
        xs.append(world.robots["pode2"].x)
    assert steps(world, "pode2") == ["MOVING", "wait", "BLOCKED"]
    rows = reports(world, "pode2")
    wait_from = next(t for t, _, step in rows if step == "wait")
    blocked_at = next(t for t, s, _ in rows if s == "BLOCKED")
    assert blocked_at - wait_from == pytest.approx(P.local_wait_s, abs=0.15)
    stop = xs.index(min(xs))
    assert max(xs[stop:]) - min(xs) < 1e-9  # 물러나지 않는다


def test_face_to_face_only_the_lower_name_backs_up():
    world = lane_pair()
    world.send(east(world, "pode1", 1, 0.35, 1.63))
    world.send(west(world, "pode2", 2, 1.15, 0.35))
    world.run(15.0)
    assert "backup" in steps(world, "pode1")
    assert "backup" not in steps(world, "pode2")
    assert steps(world, "pode2")[:3] == ["MOVING", "wait", "BLOCKED"]


def test_priority_from_the_fleet_view_decides_who_backs_up():
    world = lane_pair()
    world.traffic.views = {"pode1": {"priority": 5}, "pode2": {"priority": 1}}
    world.send(east(world, "pode1", 1, 0.35, 1.63))
    world.send(west(world, "pode2", 2, 1.15, 0.35))
    world.run(15.0)
    assert "backup" in steps(world, "pode2")
    assert "backup" not in steps(world, "pode1")


def test_a_box_is_not_a_peer_so_nobody_backs_up():
    from fake_robots.world import Obstacle
    box = Obstacle(1.0, LANE_Y, 0.05, 0.0, 60.0)
    world = quiet_world(robots={"pode1": "X_W"}, injections=[box], local_yield=True)
    place(world, "pode1", 0.35, LANE_Y, 0.0)
    world.send(east(world, "pode1", 1, 0.35, 1.63))
    world.run(10.0)
    assert steps(world, "pode1") == ["MOVING", "wait", "BLOCKED"]


def test_once_per_spot_then_plain_blocked():
    world = lane_pair()
    world.send(east(world, "pode1", 1, 0.35, 1.63))
    world.run(15.0)
    assert steps(world, "pode1").count("wait") == 1
    assert reports(world, "pode1")[-1][1] == "BLOCKED"


def test_a_short_drive_has_no_room_to_back_up():
    world = quiet_world(robots={"pode1": "X_W", "pode2": "X_E"}, local_yield=True)
    place(world, "pode1", 0.80, LANE_Y, 0.0)
    place(world, "pode2", 1.15, LANE_Y, math.pi)
    world.send(east(world, "pode1", 1, 0.80, 1.63))
    world.run(8.0)
    assert steps(world, "pode1") == ["MOVING", "wait", "BLOCKED"]


def test_a_nav_goal_robot_runs_the_same_ladder():
    world = quiet_world(robots={"pode1": "P-1", "pode2": "P-3"}, local_yield=True,
                        avoid_mode="none")
    place(world, "pode1", 0.40, 0.40, 0.0)
    place(world, "pode2", 1.10, 0.40, math.pi)
    world.send(nav(world, "pode1", 1, xy=(1.60, 0.40)))
    world.run(15.0)
    assert steps(world, "pode1")[:4] == ["MOVING", "wait", "backup", "retry"]


def test_a_nav_goal_segment_counts_only_its_own_drive():
    """order의 nav_goal 조각은 그 조각에서 지나온 길로만 물러난다(pode_fsm의 NavGoal 궤적)."""
    world = quiet_world(robots={"pode1": "P-1", "pode2": "P-3"}, local_yield=True)
    place(world, "pode1", 0.40, 0.40, 0.0)
    place(world, "pode2", 1.15, 0.40, math.pi)
    lane = SegmentOut(1, "follow_path", "", path=((0.40, 0.40, 0.0), (0.90, 0.40, 0.0)),
                      x=0.90, y=0.40)
    yard = SegmentOut(2, "nav_goal", "", x=1.15, y=0.40)
    world.send(MoveOrderCmd("pode1", 1, "o1", 0, "", base=(lane, yard)))
    world.run(15.0)
    assert steps(world, "pode1")[:3] == ["MOVING", "wait", "BLOCKED"]


def test_the_wait_ends_when_the_way_clears():
    world = lane_pair()
    world.send(east(world, "pode1", 1, 0.35, 1.63))
    assert world.run_until(lambda: reports(world, "pode1")[-1][2] == "wait", timeout_s=10)
    world.robots["pode2"].removed = True
    world.run(2.0)
    assert steps(world, "pode1") == ["MOVING", "wait", "MOVING"]


def test_with_the_real_fleet_the_steps_become_local_yield_events_in_order():
    """hall_obstacle v7 씨드 11: 두 대가 마주쳐 낮은 pode1이 물러나고 높은 pode3은 BLOCKED.

    전에는 d2 씨드 2였다. 가짜 로봇 배율을 0.67..1로 넓힌 뒤(2026-11-22) d2 씨드 1~40에는 이렇게
    마주치는 판이 없어 판을 옮겼다. 2026-11-23 진열대 끝 실측(v7)으로 씨드 6에서는 마주치지 않아
    씨드 1~10에 없는 이 판을 처음 나오는 씨드 11로 옮겼다."""
    outcome = run_situation("hall_obstacle", "v7", 11, local_yield=True)
    by_robot: dict = {}
    for _, e in outcome.world.events:
        if e.code == "LOCAL_YIELD":
            by_robot.setdefault(e.robot, []).append(e.detail)
    assert by_robot["pode1"][:4] == ["wait", "backup", "retry", by_robot["pode1"][3]]
    assert by_robot["pode1"][3].startswith("ends BLOCKED")
    assert by_robot["pode3"][0] == "wait"
    assert by_robot["pode3"][1].startswith("ends BLOCKED")
    for details in by_robot.values():  # 단계마다 wait로 시작해 ends로 끝난다
        first = [d.split()[0] for d in details]
        assert first[0] == "wait" and first[-1] == "ends"
        assert all(a == "ends" or b != "wait" for a, b in zip(first, first[1:], strict=False))
