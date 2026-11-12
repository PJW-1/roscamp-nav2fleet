"""시나리오 조각의 검사와 파싱: 고장, 물체, 재고, 주문, expect, random 풀기."""
from __future__ import annotations

import copy
import math
import random

from fake_robots.scenario_cfg import (
    EXPECT_DEFAULTS,
    FAULT_TYPES,
    LOW_BATTERY_KEYS,
    OBJECT_KEYS,
    OBJECT_MAP_MARGIN_M,
    OBJECT_RADIUS_M,
    ORDER_RESULTS,
    RANDOM_KEYS,
    REINSTATE_BASKETS,
    ROBOT_FAULTS,
    Expect,
    Fault,
    OrderCfg,
    ScenarioError,
)


def _positive(data: dict, key: str, default=None) -> float:
    value = data.get(key, default)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise ScenarioError(f"{key} must be a positive number, got {value!r}")
    return float(value)


def _not_negative(data: dict, key: str) -> float:
    value = data.get(key, 0.0)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value < 0:
        raise ScenarioError(f"{key} must not be negative, got {value!r}")
    return float(value)


def _resolve_start(raw, site) -> str:
    """출발 자리는 로봇이 서서 기다려도 되는 지점(도크, 대기 자리, 주차)이다."""
    if not isinstance(raw, str) or raw not in site.nodes:
        raise ScenarioError(f"start must be a node of the map, got {raw!r}")
    if not site.can_wait(raw):
        raise ScenarioError(f"start {raw!r} is a {site.nodes[raw].kind} node; a robot waits "
                            "only on a dock, wait or parking node")
    return raw


def _arms_with(site, action: str) -> list[str]:
    return [arm for s in site.stations.values() if s.action == action for arm in s.arms]


def _counts(raw, what: str, minimum: int) -> dict:
    if not isinstance(raw, dict):
        raise ScenarioError(f"{what} must map item kinds to counts, got {raw!r}")
    out = {}
    for kind, count in raw.items():
        if not isinstance(count, int) or isinstance(count, bool) or count < minimum:
            raise ScenarioError(f"{what}: {kind} must be an integer of at least {minimum}, "
                                f"got {count!r}")
        out[str(kind)] = count
    return out


def _parse_stock(data: dict, site) -> dict:
    fill_arms = _arms_with(site, "fill")
    stock = {arm: {} for arm in fill_arms}
    for arm, items in (data.get("stock") or {}).items():
        if arm not in fill_arms:
            raise ScenarioError(f"stock for {arm}, which is not one of {fill_arms}")
        stock[arm] = _counts(items or {}, f"stock of {arm}", 0)
    return stock


def _parse_inventory(data: dict, stock: dict) -> dict:
    if data.get("inventory") is not None:
        return _counts(data["inventory"], "inventory", 0)
    total: dict[str, int] = {}
    for items in stock.values():
        for kind, count in items.items():
            total[kind] = total.get(kind, 0) + count
    return total


def _parse_orders(data: dict) -> list[OrderCfg]:
    orders = []
    for raw in data.get("orders") or []:
        if not raw.get("id") or not raw.get("items"):
            raise ScenarioError(f"order needs an id and items: {raw}")
        at_s = raw.get("at_s", 0)
        if not isinstance(at_s, (int, float)) or at_s < 0:
            raise ScenarioError(f"bad at_s in order {raw}")
        orders.append(OrderCfg(float(at_s), str(raw["id"]),
                               _counts(raw["items"], f"order {raw['id']}", 1)))
    ids = [o.id for o in orders]
    if len(ids) != len(set(ids)):
        raise ScenarioError(f"duplicate order ids in {ids}")
    return sorted(orders, key=lambda o: o.at_s)  # 안정 정렬: at_s가 같으면 파일 순서 유지


def _parse_fault(raw: dict, robots: dict, site) -> Fault:
    kind = raw.get("type")
    if kind not in FAULT_TYPES:
        raise ScenarioError(f"unknown fault type in {raw}")
    if kind in ROBOT_FAULTS and raw.get("robot") not in robots:
        raise ScenarioError(f"fault for unknown robot in {raw}")
    if kind == "arm_fail" and raw.get("arm") not in _arms_with(site, "fill"):
        raise ScenarioError(f"arm_fail needs a workspace arm in {raw}")
    if "at_s" not in raw:
        raise ScenarioError(f"fault has no at_s: {raw}")
    if kind == "blocked":
        _positive(raw, "duration_s")  # 물체가 있는 시간
    if kind == "low_battery":
        _positive(raw, "duration_s")  # 충전에 걸리는 시간
    if kind == "obstacle_behind":
        _positive(raw, "duration_s")
        _positive(raw, "distance_m")
    if kind == "object":
        x, y, radius = _object_place(raw, site)
    else:
        if any(key in raw for key in OBJECT_KEYS):
            raise ScenarioError(f"{', '.join(OBJECT_KEYS)} belong to an object fault: {raw}")
        x, y, radius = 0.0, 0.0, OBJECT_RADIUS_M
    start, basket = None, str(raw.get("basket", "EMPTY"))
    if kind == "operator_reinstate":
        if "start" not in raw:
            raise ScenarioError(f"operator_reinstate needs a start: {raw}")
        start = _resolve_start(raw["start"], site)
        if basket not in REINSTATE_BASKETS:
            raise ScenarioError(f"operator_reinstate basket must be one of {REINSTATE_BASKETS}")
    if kind == "drift" and not (raw.get("duration_s", 0) > 0 and raw.get("offset_m", 0) != 0):
        raise ScenarioError(f"drift needs a positive duration_s and a nonzero offset_m: {raw}")
    target = str(raw.get("target", ""))
    if target and (kind != "nav_abort" or target not in site.nodes):
        raise ScenarioError(f"target needs nav_abort and a node of the map in {raw}")
    return Fault(float(raw["at_s"]), raw.get("robot", ""), kind,
                 float(raw.get("duration_s", 0.0)), int(raw.get("count", 1)),
                 raw.get("arm", ""), int(raw.get("after_items", 0)),
                 start=start, basket=basket, target=target,
                 offset_m=float(raw.get("offset_m", 0.0)),
                 distance_m=float(raw.get("distance_m", 0.0)), x=x, y=y, radius_m=radius)


def _object_place(raw: dict, site) -> tuple[float, float, float]:
    """object 고장의 (x, y, 반지름). 로봇에 묶이지 않고, 지도 범위 가까이에 있어야 한다."""
    if raw.get("robot"):
        raise ScenarioError(f"an object is not tied to a robot: {raw}")
    _positive(raw, "duration_s")  # 물체가 있는 시간
    radius = _positive(raw, "radius_m", OBJECT_RADIUS_M)
    numbers = {}
    for key in ("at_s", "duration_s", "radius_m", "x", "y"):
        value = raw.get(key, OBJECT_RADIUS_M if key == "radius_m" else None)
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value)):
            raise ScenarioError(f"object needs a finite number {key}, got {value!r}: {raw}")
        numbers[key] = float(value)
    if numbers["at_s"] < 0:
        raise ScenarioError(f"object at_s must not be negative: {raw}")
    place = [numbers["x"], numbers["y"]]
    x0, y0, x1, y1 = site.bounds()
    m = OBJECT_MAP_MARGIN_M
    if not (x0 - m <= place[0] <= x1 + m and y0 - m <= place[1] <= y1 + m):
        raise ScenarioError(f"object at {tuple(place)} is far off the map: {raw}")
    return place[0], place[1], radius


def _check_reinstates(faults: list) -> None:
    """다시 투입은 그 로봇을 치운 뒤에만 할 수 있다."""
    removed: set[str] = set()
    for f in sorted(faults, key=lambda f: f.at_s):
        if f.type == "operator_remove":
            removed.add(f.robot)
        elif f.type == "operator_reinstate":
            if f.robot not in removed:
                raise ScenarioError(f"operator_reinstate of {f.robot} at {f.at_s} s comes "
                                    "before any operator_remove of it")
            removed.discard(f.robot)


def _parse_expect(data: dict, faults: list) -> Expect:
    raw = data.get("expect") or {}
    unknown = set(raw) - set(EXPECT_DEFAULTS)
    if unknown:
        raise ScenarioError(f"unknown expect keys: {sorted(unknown)}")
    ids = {str((o or {}).get("id")) for o in data.get("orders") or []}
    for order, result in (raw.get("order_results") or {}).items():
        if order not in ids or result not in ORDER_RESULTS:
            raise ScenarioError(f"order_results: {order}: {result} needs a scenario order and "
                                f"one of {ORDER_RESULTS}")
    expect = Expect(**{**EXPECT_DEFAULTS, **raw})
    if faults and max(expect.orders_shipped or 0, expect.orders_shipped_min) < 1:
        raise ScenarioError("a scenario with faults must expect at least one shipped order "
                            "(orders_shipped or orders_shipped_min), so that it cannot pass "
                            "with nothing shipped")
    return expect


def _span(lo, hi, what: str) -> tuple[float, float]:
    for value in (lo, hi):
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value < 0:
            raise ScenarioError(f"{what} needs non-negative numbers, got {lo!r}, {hi!r}")
    if lo > hi:
        raise ScenarioError(f"{what}: {lo} is above {hi}")
    return float(lo), float(hi)


def expand_random(data: dict, seed: int | None = None) -> dict:
    """`random` 블록을 정해진 시각으로 푼 새 dict. 입력은 바꾸지 않는다.

    seed가 None이면 파일의 seed. 주문을 파일 순서로 먼저 뽑고 로봇은 이름 순서로 뽑는다.
    random이 없으면 내용이 같은 사본을 돌려준다.
    """
    out = copy.deepcopy(data)
    spec = out.pop("random", None)
    if spec is None:
        return out
    if not isinstance(spec, dict):
        raise ScenarioError(f"random must be a mapping, got {spec!r}")
    unknown = set(spec) - RANDOM_KEYS
    if unknown:
        raise ScenarioError(f"unknown random keys: {sorted(unknown)}")
    gap = spec.get("order_gap_s")
    if gap is not None and not (isinstance(gap, (list, tuple)) and len(gap) == 2):
        raise ScenarioError(f"random.order_gap_s must be [low, high], got {gap!r}")
    gap = _span(*gap, "random.order_gap_s") if gap is not None else None
    battery = spec.get("low_battery")
    if battery is not None:
        if not isinstance(battery, dict) or set(battery) != LOW_BATTERY_KEYS:
            raise ScenarioError(f"random.low_battery needs exactly {sorted(LOW_BATTERY_KEYS)}, "
                                f"got {battery!r}")
        span = _span(battery["from_s"], battery["to_s"], "random.low_battery from_s/to_s")
        charge_s = _positive(battery, "charge_s")
    seed = int(out.get("seed", 0) if seed is None else seed)
    rng = random.Random(seed)
    out["seed"] = seed
    if gap is not None:
        t = 0.0
        for order in out.get("orders") or []:
            t += rng.uniform(*gap)
            order["at_s"] = round(t, 1)
    if battery is not None:
        added = [{"type": "low_battery", "robot": robot, "at_s": round(rng.uniform(*span), 1),
                  "duration_s": charge_s}
                 for robot in sorted(out.get("robots") or {})]
        out["faults"] = list(out.get("faults") or []) + added
    return out
