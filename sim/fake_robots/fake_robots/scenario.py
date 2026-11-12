"""가짜 로봇용 시나리오 파일: 읽기, 기본값, 검증. ROS를 쓰지 않는다."""
from __future__ import annotations

from pathlib import Path

import yaml
from fleet_core.fleet_map import KNOWN_LAYOUTS

from fake_robots.scenario_cfg import (  # noqa: F401  (다른 모듈이 여기서 가져간다)
    ARM_FAULTS,
    EXPECT_DEFAULTS,
    FAULT_TYPES,
    GONE_FAULTS,
    LOW_BATTERY_KEYS,
    OBJECT_KEYS,
    OBJECT_MAP_MARGIN_M,
    OBJECT_RADIUS_M,
    ORDER_RESULTS,
    RANDOM_KEYS,
    REINSTATE_BASKETS,
    ROBOT_FAULTS,
    TOP_LEVEL_KEYS,
    WORLD_FAULTS,
    ArmCfg,
    DockCfg,
    Expect,
    Fault,
    OrderCfg,
    RobotCfg,
    Scenario,
    ScenarioError,
)
from fake_robots.scenario_parts import (  # noqa: F401
    _arms_with,
    _check_reinstates,
    _counts,
    _not_negative,
    _object_place,
    _parse_expect,
    _parse_fault,
    _parse_inventory,
    _parse_orders,
    _parse_stock,
    _positive,
    _resolve_start,
    _span,
    expand_random,
)


def scenario_map_file(scenario_path) -> str:
    """시나리오의 `map`: 배치 이름(d2, d, c, 시험용 d2_shelf). 없으면 빈 값(기본 배치 d2).

    load_site와 load_fleet_map이 이 이름을 그대로 받는다.
    """
    path = Path(scenario_path)
    name = (yaml.safe_load(path.read_text(encoding="utf-8")) or {}).get("map")
    if not name:
        return ""
    if name not in KNOWN_LAYOUTS:
        raise ScenarioError(f"map {name!r} is not one of {list(KNOWN_LAYOUTS)}")
    return str(name)


def parse_scenario(data: dict, site, seed: int | None = None) -> Scenario:
    """seed: random 블록을 풀 seed. None이면 파일의 seed."""
    data = expand_random(data, seed)
    unknown = set(data) - TOP_LEVEL_KEYS
    if unknown:
        raise ScenarioError(f"unknown scenario keys: {sorted(unknown)}")
    robots = {}
    for name, raw in (data.get("robots") or {}).items():
        raw = raw or {}
        if "start" not in raw:
            raise ScenarioError(f"robot {name} has no start")
        robots[name] = RobotCfg(
            start=_resolve_start(raw["start"], site),
            speed_mps=_positive(raw, "speed_mps", 0.2),
            pose_noise_m=float(raw.get("pose_noise_m", 0.0)),
            report_delay_s=_not_negative(raw, "report_delay_s"),
            stop_error_m=_not_negative(raw, "stop_error_m"),
            localizing_s=_not_negative(raw, "localizing_s"),
        )
    if not robots:
        raise ScenarioError("scenario has no robots")

    stock = _parse_stock(data, site)
    faults = [_parse_fault(raw, robots, site) for raw in data.get("faults") or []]
    _check_reinstates(faults)
    arm = data.get("arm") or {}
    dock = data.get("dock") or {}
    return Scenario(
        seed=int(data.get("seed", 0)),
        speed_factor=_positive(data, "speed_factor", 1.0),
        timeout_s=_positive(data, "timeout_s", 900),
        stock=stock,
        inventory=_parse_inventory(data, stock),
        orders=_parse_orders(data),
        robots=robots,
        arm=ArmCfg(float(arm.get("seconds_per_item", 4.0)), float(arm.get("swap_seconds", 8.0)),
                   float(arm.get("fail_rate", 0.0)), float(arm.get("cancel_seconds", 0.0))),
        dock=DockCfg(float(dock.get("duration_s", 3.0)), float(dock.get("error_cm", 1.0))),
        faults=faults,
        expect=_parse_expect(data, faults),
    )


def load_scenario(path, site=None, seed: int | None = None) -> Scenario:
    if site is None:
        from fleet_core.traffic.site import load_site
        site = load_site(scenario_map_file(path) or None)
    return parse_scenario(yaml.safe_load(Path(path).read_text(encoding="utf-8")), site,
                          seed)
