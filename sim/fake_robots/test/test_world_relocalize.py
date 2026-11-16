"""세계의 위치 다시 찾기(Relocalize): 선 로봇이 약 2 s 뒤 답한다.

성공하면 drive_by_estimate 창의 믿는 치우침이 끝난다(그 뒤로 믿는 자리 = 실제 자리). 보고만 미는
NoisyPose 창은 고치지 못한다. RelocalizeFails 주입이 있으면 실패로 답한다(격리 길 시험용).
움직이는 로봇은 받자마자 BUSY로 끝낸다(계약: 일과 상태는 그대로).
"""
import math

import pytest
from fake_robots.world import NoisyPose, RelocalizeFails
from fake_robots.world.relocalize import RELOCALIZE_S
from fleet_core.common.commands import CancelCmd, RelocalizeCmd
from world_helpers import follow, quiet_world, states

BIAS = 0.2


def answers(world, robot="pode1"):
    """(시각, goal_id, 성공, 까닭, 끝 자세, 시작 자세)."""
    return [(t, *args[1:]) for t, args in world.manager.of("on_relocalize_result", robot)]


def last_report(world, robot="pode1"):
    return world.manager.of("on_robot_state", robot)[-1][1]


def test_relocalize_ends_the_belief_offset_and_reports_the_true_pose_after_about_2_s():
    world = quiet_world(robots={"pode1": "N_W"}, injections=[
        NoisyPose("pode1", 1.0, 60.0, jump_m=BIAS, drive_by_estimate=True)])
    world.run(2.0)
    r = world.robots["pode1"]
    assert last_report(world)[1] == pytest.approx(r.x + BIAS)
    world.send(RelocalizeCmd("pode1", 5))
    sent = world.t
    world.run(1.0)
    assert states(world)[-1][1] == "LOCALIZING" and not answers(world)
    world.run(RELOCALIZE_S)
    ((t, goal, ok, reason, pose, start),) = answers(world)
    assert (goal, ok, reason) == (5, True, "")
    assert t - sent == pytest.approx(RELOCALIZE_S, abs=0.11)
    assert pose[0] == pytest.approx(r.x) and start[0] == pytest.approx(r.x + BIAS)
    world.run(1.0)
    assert last_report(world)[1] == pytest.approx(r.x), "belief = truth from then on"
    world.send(follow(world, "pode1", 6, ["N_W", "N_E"]))
    world.run(15.0)
    assert math.dist(r.pose, world.site.node_pose("N_E")) < 1e-6, "drives from its true pose"


def test_a_report_only_window_is_not_fixed_by_relocalize():
    world = quiet_world(robots={"pode1": "N_W"}, injections=[
        NoisyPose("pode1", 1.0, 60.0, jump_m=BIAS)])
    world.run(2.0)
    world.send(RelocalizeCmd("pode1", 5))
    world.run(RELOCALIZE_S + 1.0)
    ((_, _, ok, _, pose, _),) = answers(world)
    r = world.robots["pode1"]
    assert ok and pose[0] == pytest.approx(r.x + BIAS)
    assert last_report(world)[1] == pytest.approx(r.x + BIAS)


def test_a_drive_window_that_starts_after_the_relocalize_still_applies():
    world = quiet_world(robots={"pode1": "N_W"}, injections=[
        NoisyPose("pode1", 6.0, 60.0, jump_m=BIAS, drive_by_estimate=True)])
    world.run(1.0)
    world.send(RelocalizeCmd("pode1", 5))
    world.run(7.0)
    assert last_report(world)[1] == pytest.approx(world.robots["pode1"].x + BIAS)


def test_relocalize_fails_while_the_injection_says_so():
    world = quiet_world(robots={"pode1": "N_W"}, injections=[
        NoisyPose("pode1", 1.0, 60.0, jump_m=BIAS, drive_by_estimate=True),
        RelocalizeFails("pode1", 0.0, 10.0, reason="SCAN_MISMATCH")])
    world.run(2.0)
    world.send(RelocalizeCmd("pode1", 5))
    world.run(RELOCALIZE_S + 0.5)
    ((_, _, ok, reason, pose, _),) = answers(world)
    r = world.robots["pode1"]
    assert (ok, reason) == (False, "SCAN_MISMATCH")
    assert pose[0] == pytest.approx(r.x + BIAS), "the estimate stays wrong"
    world.run(1.0)
    assert last_report(world)[1] == pytest.approx(r.x + BIAS)
    world.run(8.0)
    world.send(RelocalizeCmd("pode1", 6))
    world.run(RELOCALIZE_S + 0.5)
    assert answers(world)[-1][2] is True, "after the window it works"


def test_a_moving_robot_ends_the_goal_at_once_with_busy_and_keeps_driving():
    world = quiet_world(robots={"pode1": "N_W"})
    world.send(follow(world, "pode1", 1, ["N_W", "N_E"]))
    world.run(1.0)
    world.send(RelocalizeCmd("pode1", 5))
    world.run(0.2)
    ((_, goal, ok, reason, pose, start),) = answers(world)
    assert (goal, ok, reason, pose, start) == (5, False, "BUSY", None, None)
    assert world.robots["pode1"].run is not None and states(world)[-1][1] == "MOVING"


def test_a_cancelled_relocalize_answers_canceled_and_stops_localizing():
    world = quiet_world(robots={"pode1": "N_W"})
    world.run(1.0)
    world.send(RelocalizeCmd("pode1", 5))
    world.run(0.5)
    world.send(CancelCmd("pode1", 5))
    world.run(0.5)
    ((_, goal, ok, reason, _, _),) = answers(world)
    assert (goal, ok, reason) == (5, False, "CANCELED")
    assert states(world)[-1][1] == "IDLE"
