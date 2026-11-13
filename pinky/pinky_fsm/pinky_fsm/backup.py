from __future__ import annotations

import math
from dataclasses import dataclass

OBSTACLE_BEHIND = "OBSTACLE_BEHIND"
TIMEOUT = "TIMEOUT"
CANCELED = "CANCELED"
FAILED = "FAILED"

NAV2_TIMEOUT = 711
NAV2_COLLISION_AHEAD = 714
NAV2_NO_VALID_CONTROL = 106


@dataclass(frozen=True)
class BackUp:
    seq: int
    distance_m: float
    speed_mps: float
    time_allowance_s: float


@dataclass(frozen=True)
class CancelBackUp:
    pass


def backup_distance(travelled_m: float, max_m: float, keep_m: float) -> float:
    return max(0.0, min(max_m, travelled_m - keep_m))


def keep_lateral(distance_m: float, lateral_m: float, heading_err: float, limit_m: float) -> float:
    k = math.sin(heading_err)
    if abs(k) < 1e-9 or distance_m <= 0.0:
        return max(0.0, distance_m)
    bound = max(limit_m, abs(lateral_m))
    reach = (lateral_m + bound) / k if k > 0 else (bound - lateral_m) / -k
    return max(0.0, min(distance_m, reach))


def reason_of(outcome: str, error_code: int) -> str:
    if outcome == "succeeded":
        return ""
    if outcome == "canceled":
        return CANCELED
    if outcome == "aborted" and error_code in (NAV2_COLLISION_AHEAD, NAV2_NO_VALID_CONTROL):
        return OBSTACLE_BEHIND
    if outcome == "aborted" and error_code == NAV2_TIMEOUT:
        return TIMEOUT
    return FAILED
