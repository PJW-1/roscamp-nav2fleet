"""상황을 돌리고 결과 표(지시서 6-2)의 한 줄과 못 지킨 기대를 만든다.

판정은 둘이다(추가 지시서 2의 2절): 정확성(안전 불변식, 관제 반응 시간, SILENT_STALL 0)은 모든
모드에서 같고, 상황의 기대는 모드마다 다르다(Situation.for_mode). 나머지 칸은 성능이다.

회피 흉내 모드(avoid.py)마다 돌릴 수 있다. "끝나지 않은 주문 원인" 칸은 비워 둔다:
tools/situations.py가 원인 분류(tools/triage)로 채운다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .avoid import PERFECT
from .situations import build
from .world import ZoneWorld, default_manager

HEADER = ("상황", "배치", "씨드", "회피 모드", "완료", "종료 시각", "대기", "로봇 사이 최소 거리",
          "교착",
          "구역 밖(OUTSIDE_ZONE / 공유 구역 들어감)", "Retreat(+도크 나오기)",
          "급행 사용과 방향 전환", "위반", "판정", "끝나지 않은 주문 원인")


NEVER_EVENTS = ("SILENT_STALL",)  # 추가 지시서 2의 3절: 나오면 실패


@dataclass
class Outcome:
    world: ZoneWorld
    row: tuple
    failures: list[str]


def run_situation(name, layout="d2", seed=1, overrides=None,
                  manager_factory=default_manager, avoid_mode=PERFECT) -> Outcome:
    s = build(name, layout).for_mode(avoid_mode)
    world = ZoneWorld(layout, seed=seed, overrides=overrides, robots=s.robots, orders=s.orders,
                      injections=s.injections, manager_factory=manager_factory,
                      avoid_mode=avoid_mode)
    world.run_until(world.orders_done, timeout_s=s.timeout_s)
    failures = judge(s, world)
    return Outcome(world, row(s, layout, seed, world, failures), failures)


def judge(s, world) -> list[str]:
    """s: 그 모드의 기대(Situation.for_mode)."""
    return correctness(s, world) + expectations(s, world)


def correctness(s, world) -> list[str]:
    """안전 불변식과 관제 반응 시간(invariants.py, reaction.py), 조용히 멈춤 0. 모든 모드."""
    out = [f"{v.code} at {v.t:.1f}: {v.detail}" for v in world.violations
           if v.code not in s.allowed]
    out += [f"{code} x{world.metrics.count(code)}" for code in NEVER_EVENTS
            if world.metrics.count(code)]
    return out


def expectations(s, world) -> list[str]:
    m, out = world.metrics, []
    total = world.central.total
    if s.finish_all and world.central.finished < total:
        out.append(f"finished {world.central.finished}/{total} by {world.t:.0f} s")
    for code in s.no_events:
        if m.count(code):
            out.append(f"{code} x{m.count(code)}")
    for code, least in s.need_events.items():
        if m.count(code) < least:
            out.append(f"{code} x{m.count(code)}, want >= {least}")
    if s.max_retreats is not None and m.retreats > s.max_retreats:
        out.append(f"retreats {m.retreats} > {s.max_retreats}")
    return out


def row(s, layout, seed, world, failures) -> tuple:
    m = world.metrics
    end = f"{m.end_t:.1f}" if m.end_t is not None else f"> {world.t:.0f}"
    waits = "/".join(f"{m.wait_s[n]:.0f}" for n in sorted(m.wait_s))
    gap = f"{m.min_gap:.3f}" if math.isfinite(m.min_gap) else "-"
    return (s.title, layout, seed, world.avoid_mode,
            f"{world.central.finished}/{world.central.total}", end,
            waits, gap, m.count("DEADLOCK_SEEN"),
            f"{m.count('OUTSIDE_ZONE')} / {world.invariants.shared_outside}",
            f"{m.retreats} (+{m.dock_exits})",
            f"{m.lane_uses} / {m.direction_changes}", len(world.violations),
            "통과" if not failures else "실패", "")


def markdown(rows) -> str:
    lines = ["| " + " | ".join(HEADER) + " |", "|" + "---|" * len(HEADER)]
    lines += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(lines)
