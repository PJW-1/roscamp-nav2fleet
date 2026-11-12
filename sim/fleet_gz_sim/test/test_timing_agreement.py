"""관제, 핑키, 머리 없는 세계가 BLOCKED까지의 시간을 같게 아는지(계약 0.13-draft). ROS 없이 돈다.

세 패키지를 함께 보는 곳이 여기뿐이라 이 패키지의 시험에 둔다. Gazebo 세 대 경기(2026-11-13)에서
이 시간들이 어긋나 있었다: 핑키는 선 때부터 10 s 뒤에 BLOCKED를 보고하는데 관제는 6 s 만에 goal을
거뒀고(GOAL_STALLED), 세 번 거둔 뒤 로봇을 격리했다. 머리 없는 세계의 로봇은 5 s로 남아 있어서
이 차이가 시험에 드러나지 않았다.
"""
from fake_robots import nav_goal_drive
from fake_robots.world import world
from fleet_core.common.params import load_params
from pinky_fsm.params import FsmParams

REPORT_MARGIN_S = 2.0  # 상태 보고 주기 0.5 s, 두 쪽이 진전을 재기 시작한 때의 차이, 여유


def test_the_fleet_calls_a_goal_stalled_only_after_the_robot_could_report_blocked():
    assert load_params().stall_s >= FsmParams().blocked_after_s + REPORT_MARGIN_S


def test_the_headless_robots_report_blocked_when_the_real_ones_do():
    real = FsmParams().blocked_after_s
    assert world.BLOCKED_AFTER_S == real
    assert nav_goal_drive.BLOCKED_AFTER_S == real
