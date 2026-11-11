"""교통 층 결함의 재현(구역 세계에서). 고쳐지면 strict xfail이 빨개지므로 그때 지운다.

설명과 사건 기록은 교통 층 담당에게 보낸 traffic_bugs.md에 있다.
"""
import pytest
from fake_robots.world import NoisyPose, ZoneWorld


@pytest.mark.xfail(strict=True, raises=AssertionError,
                   reason="B1: a nav_goal robot drives back into a shared zone it has released "
                          "(invariant 3)")
def test_b1_no_robot_is_ever_outside_the_zones_it_holds():
    world = ZoneWorld("d", seed=2, injections=[
        NoisyPose("pinky1", 20.0, 40.0, sigma_m=0.03, jump_m=0.10)])
    world.run(65.0)
    assert not [v for v in world.violations if v.code == "I3_OUTSIDE_HELD"]
