"""구역 세계가 찾은 교통 층 결함(B1, B3)의 회귀 시험. nav2-base 0e9d454에서 고쳐졌다.

설명과 사건 기록은 교통 층 담당에게 보낸 traffic_bugs.md에 있다.
"""
from fake_robots.world import NoisyPose, ZoneWorld
from fake_robots.world.table import run_situation


def test_b1_no_robot_is_ever_outside_the_zones_it_holds():
    world = ZoneWorld("d", seed=2, injections=[
        NoisyPose("pinky1", 20.0, 40.0, sigma_m=0.03, jump_m=0.10)])
    world.run(65.0)
    # 결함은 주입이 없는 pinky2에서 났다(주입은 때만 바꾼다)
    assert not [v for v in world.violations if v.code.startswith("I3") and v.robot == "pinky2"]


def test_b3_a_robot_stuck_in_the_yard_does_not_stop_the_others():
    outcome = run_situation("front_robot_stuck", "d2", 1)
    # pinky1은 HALL에 서 있다. 가지 못할 길의 J_W와 WORK_DOCK은 놓아야 한다
    held = set(outcome.world.grants().held_by("pinky1"))
    assert not held & {"J_W", "WORK_DOCK"}, held
    assert outcome.world.central.finished >= 5  # 6건 중 멈춘 로봇의 것 하나는 운영자를 기다린다
