# ---- 교통 모드 선택과 교통 계층 생성 ----
from __future__ import annotations

import importlib

# ---- 모드 이름과 zone 컨트롤러 모듈 경로 ----
NEEDS_ZONE_TRAFFIC = "needs the zone traffic layer (stage 3)"
ZONES = "zones"
MODES = (ZONES,)
ZONE_CONTROLLER = "fleet_core.traffic.zone.controller"


class NoZoneTraffic(RuntimeError):
    pass


# ---- 모드 확인 후 ZoneTraffic 불러와 만들기 ----
def build_traffic(site, params, robots, clock, next_goal_id):
    mode = params.traffic_mode
    if mode != ZONES:
        raise ValueError(f"traffic_mode must be one of {', '.join(MODES)}, not {mode!r}")
    try:
        module = importlib.import_module(ZONE_CONTROLLER)
    except ModuleNotFoundError as error:
        missing = error.name or ""
        if missing != ZONE_CONTROLLER and not ZONE_CONTROLLER.startswith(missing + "."):
            raise
        raise NoZoneTraffic(f"{NEEDS_ZONE_TRAFFIC}: {ZONE_CONTROLLER} is not on this branch"
                            ) from None
    return module.ZoneTraffic(site, params, robots, clock, next_goal_id)
