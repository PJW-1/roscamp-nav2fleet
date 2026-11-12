"""위치 추정 감시(LOCALIZATION_SUSPECT): 보고 자세가 로봇이 갈 수 있는 것보다 크게 튄다."""
import math
from dataclasses import replace

from fleet_core.common.params import load_params
from fleet_core.traffic.zone.localization import pose_jump
from zone_harness import Harness

P = load_params()
STILL = (0.0, 0.0, 0.0)


def count(world, code, robot=""):
    return world.codes(robot).count(code)


def reports(world, robot, state):
    """robot은 앞으로 state를 보고한다. 다른 로봇은 그대로다."""
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == robot else plain(bot)


# ---- 순수 함수 ----

def test_a_pose_that_moves_as_fast_as_the_robot_can_is_no_jump():
    assert pose_jump(STILL, (0.05, 0.0, 0.2), 0.2, P) == ""


def test_a_pose_that_moves_farther_than_the_robot_can_is_a_jump():
    assert "moved 0.40 m" in pose_jump(STILL, (0.40, 0.0, 0.0), 0.2, P)


def test_a_flipped_pose_tells_how_far_it_turned():
    why = pose_jump((0.4, 0.3, 3.1), (1.5, 0.7, -0.1), 0.2, P)
    assert "moved 1.17 m and turned 177 deg" in why


def test_a_heading_that_changes_alone_is_no_jump():
    """The test worlds turn a robot at once, and a flip away from the centre moves it too."""
    assert pose_jump(STILL, (0.0, 0.0, math.pi), 0.2, P) == ""


def test_reports_far_apart_in_time_are_not_compared():
    late = P.pose_jump_window_s + 0.1
    assert pose_jump(STILL, (2.0, 0.0, 0.0), late, P) == ""
    assert pose_jump(STILL, (2.0, 0.0, 0.0), 0.0, P) == ""


def test_the_slack_grows_with_the_time_between_reports():
    step = 0.30  # 0.2 s에는 튄 것이지만 1 s에는 로봇이 갈 수 있는 거리다
    assert pose_jump(STILL, (step, 0.0, 0.0), 0.2, P) != ""
    assert pose_jump(STILL, (step, 0.0, 0.0), 1.0, P) == ""


# ---- 관제 ----

def test_a_jumping_report_is_recorded_once_as_localization_suspect():
    world = Harness()
    world.run(1.0)
    world.bots["pinky1"].x += 0.5  # 로봇은 서 있었는데 보고 위치가 0.5 m 옮겨졌다
    world.run(1.0)
    assert count(world, "LOCALIZATION_SUSPECT", "pinky1") == 1
    assert count(world, "LOCALIZATION_SUSPECT") == 1
    event = next(e for e in world.events if e.code == "LOCALIZATION_SUSPECT")
    assert event.level == "WARN" and "moved 0.50 m" in event.detail


def test_a_robot_that_drives_its_way_is_not_suspect():
    world = Harness()
    world.step()
    world.call("WORK", "pinky2")
    world.run(60.0)
    assert count(world, "LOCALIZATION_SUSPECT") == 0


def test_a_new_pose_after_localizing_is_not_a_jump():
    """위치를 다시 잡은 로봇(LOCALIZING)의 다음 자세는 앞 자세와 견주지 않는다."""
    world = Harness()
    world.run(1.0)
    reports(world, "pinky1", "LOCALIZING")
    world.run(0.5)
    world.bots["pinky1"].x += 0.5
    del world._state
    world.run(1.0)
    assert count(world, "LOCALIZATION_SUSPECT") == 0


def test_jumps_in_a_row_are_one_event_until_the_quiet_time_is_over():
    world = Harness()
    world.run(1.0)
    bot = world.bots["pinky1"]
    for _ in range(3):
        bot.x += 0.4
        world.step()
        bot.x -= 0.4
        world.step()
    assert count(world, "LOCALIZATION_SUSPECT", "pinky1") == 1
    world.run(P.pose_jump_quiet_s)
    bot.x += 0.4
    world.run(0.5)
    assert count(world, "LOCALIZATION_SUSPECT", "pinky1") == 2


def test_the_first_pose_after_lost_contact_is_not_a_jump():
    """The pose of a robot that went silent may have been set again in the meantime."""
    world = Harness(params=replace(P, comm_timeout_s=1.0))
    world.run(1.0)
    world.silent.add("pinky1")
    assert world.run_until(lambda: "COMM_LOST" in world.codes("pinky1"), 5.0)
    assert world.now < 1.0 + P.pose_jump_window_s, "still inside the compared time"
    world.bots["pinky1"].x += 1.5
    world.silent.clear()
    world.run(1.0)
    assert count(world, "LOCALIZATION_SUSPECT") == 0
