"""머리 없는 구역 세계(지시서 6-1): 가짜 핑키 세 대가 관제의 명령을 해 보인다. ROS를 쓰지 않는다."""
from .injections import (
    CommLoss,
    LowBattery,
    NoisyPose,
    Obstacle,
    Reinstate,
    Removal,
    SlowStation,
    Stuck,
)
from .recording import FakeGrants, RecordingManager
from .robot import Imperfections
from .stations import six_orders
from .world import DT, ZoneWorld, default_manager, default_robots

__all__ = [
    "DT", "CommLoss", "FakeGrants", "Imperfections", "LowBattery", "NoisyPose", "Obstacle",
    "RecordingManager", "Reinstate", "Removal", "SlowStation", "Stuck", "ZoneWorld",
    "default_manager", "default_robots", "six_orders",
]
