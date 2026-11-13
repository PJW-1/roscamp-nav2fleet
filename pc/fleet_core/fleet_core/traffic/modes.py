from __future__ import annotations

import importlib

NEEDS_ZONE_TRAFFIC = "needs the zone traffic layer (stage 3)"
ZONES = "zones"
MODES = (ZONES,)
ZONE_CONTROLLER = "fleet_core.traffic.zone.controller"


class NoZoneTraffic(RuntimeError):
    pass


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
