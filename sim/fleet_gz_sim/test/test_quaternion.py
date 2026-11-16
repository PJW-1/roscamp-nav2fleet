"""쿼터니언에서 yaw를 뽑는 식. ROS 없이 돈다."""
import math
from types import SimpleNamespace

import pytest
from fleet_gz_sim.quaternion import yaw_of


@pytest.mark.parametrize("yaw", [0.0, 0.5, -1.2, math.pi / 2, 3.0, -3.0])
def test_yaw_of_a_pure_z_rotation_returns_the_angle(yaw):
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    assert yaw_of(q) == pytest.approx(yaw)
