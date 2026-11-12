"""traffic_mode로 교통 층을 고른다. 주문 코어와 FleetManager의 바깥 API는 그대로다.

모드는 zones 하나다(지점과 구역 그래프 위의 구역 허락, 넓은 구역은 Nav2 goal). 그 교통 층은
fleet_core/traffic/zone/controller.py의 ZoneTraffic이고 바깥 API는 api.py다. 이름으로 읽어
들이므로 시험은 그 자리에 가짜를 둘 수 있다. 모듈을 찾지 못하면 NoZoneTraffic을 낸다.
"""
from __future__ import annotations

import importlib

NEEDS_ZONE_TRAFFIC = "needs the zone traffic layer (stage 3)"
ZONES = "zones"
MODES = (ZONES,)
ZONE_CONTROLLER = "fleet_core.traffic.zone.controller"


class NoZoneTraffic(RuntimeError):
    """교통 층 모듈이 아직 없다."""


def build_traffic(site, params, robots, clock, next_goal_id):
    """교통 층(api.TrafficApi)을 만든다. 인자는 지운 레인 제어기와 같다."""
    mode = params.traffic_mode
    if mode != ZONES:
        raise ValueError(f"traffic_mode must be one of {', '.join(MODES)}, not {mode!r}")
    try:
        module = importlib.import_module(ZONE_CONTROLLER)
    except ModuleNotFoundError as error:
        missing = error.name or ""
        if missing != ZONE_CONTROLLER and not ZONE_CONTROLLER.startswith(missing + "."):
            raise  # 교통 층 안의 임포트가 틀렸다. 감추지 않는다
        raise NoZoneTraffic(f"{NEEDS_ZONE_TRAFFIC}: {ZONE_CONTROLLER} is not on this branch"
                            ) from None
    return module.ZoneTraffic(site, params, robots, clock, next_goal_id)
