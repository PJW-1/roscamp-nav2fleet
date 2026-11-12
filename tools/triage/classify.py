"""끝나지 않은 주문마다 원인 하나(지시서 6절). 아래 순서로 처음 걸리는 것이 원인이다.

0. 시험 틀: 기록이 잘렸거나 상태 보고가 없거나 틀의 오류 사건이 있다.
1. 관제가 목표를 보냈나(없으면 관제).  2. 목표 뒤 로봇이 움직였나(아니면 주행).
3. 움직이다 막혔나.  4. 막힌 뒤 관제가 시간 상한 안에 반응했나(아니면 관제).
3-1. 그 막힘이 장애물 때문이고 끝까지 안 풀렸나(예면 진짜 막힘, 아니면 회피).
막혔을 때는 4를 3-1보다 먼저 본다(가정: 관제가 늦었으면 막힘의 이유와 상관없이 관제 탓).
"""
from __future__ import annotations

import math

from .model import (
    BLOCK_CODES,
    FIRST_REACTION,
    FRAME_CODES,
    GOAL_CODES,
    NEXT_STAGE,
    Row,
    Run,
)

DONE = ("SHIPPED", "PARTIAL")


def _pos_at(moves, t):
    """t 이전 마지막 위치. 없으면 첫 위치."""
    before = [m for m in moves if m[0] <= t]
    return (before or moves)[-1][1:3] if moves else None


def _episodes(run: Run, robot, t0, t1):
    """(시작, 끝, 근거 이름): 이어진 BLOCKED 보고 한 덩이, 그 안에 없는 NAV_ABORTED 한 점."""
    out, start = [], None
    reports = [s for s in run.states.get(robot, ()) if t0 <= s[0] <= t1]
    for t, _x, _y, state in reports:
        if state == "BLOCKED" and start is None:
            start = t
        elif state != "BLOCKED" and start is not None:
            out.append((start, t, "BLOCKED"))
            start = None
    if start is not None:
        out.append((start, t1, "BLOCKED"))
    for e in run.events:
        if (e.code in BLOCK_CODES and e.robot == robot and t0 <= e.t <= t1
                and not any(a <= e.t <= b for a, b, _ in out)):
            out.append((e.t, e.t, e.code))
    return sorted(out)


def _late(run, robot, episode, cfg):
    """4번: 늦은 반응이 있으면 (근거 이름, 시각)."""
    start, end, _ = episode
    seen = [e for e in run.events if e.robot == robot and e.t >= start]
    if end - start > cfg["first_reaction_s"] or end == start:
        if not any(e.code in FIRST_REACTION and e.t <= start + cfg["first_reaction_s"] + 1e-6
                   for e in seen):
            return f"BLOCKED_WAIT 없음({cfg['first_reaction_s']:g} s)", start
    retreating = _retreating(run, robot, start)
    limit = cfg["stage_slack_s"] + (cfg["blocked_timeout_s"] if retreating
                                    else cfg["blocked_mark_s"])
    if end - start > limit and not any(
            e.code in NEXT_STAGE and e.t <= start + limit + 1e-6 for e in seen):
        name = "GOAL_FAILED" if retreating else "ZONE_MARKED_BLOCKED"
        return f"{name} 없음({limit:g} s)", start
    return None


def _retreating(run, robot, t) -> bool:
    """t에 물러나는 goal 중이었나: 마지막 RETREAT_START 뒤 끝 사건이 아직 없다."""
    mine = [e for e in run.events if e.robot == robot and e.t <= t]
    starts = [e.t for e in mine if e.code == "RETREAT_START"]
    return bool(starts) and not any(
        e.code in ("RETREAT_DONE", "RETREAT_FAILED", "GOAL_FAILED") and e.t > starts[-1]
        for e in mine)


def _really_blocked(run, robot, episode, end_t, obstacles, cfg) -> bool:
    """3-1: 막힌 자리 가까이에 장애물이 있었고 창이 끝날 때까지 그대로다."""
    here = _pos_at(run.moves_of(robot), episode[0])
    return here is not None and any(
        o.t0 <= episode[0] < o.t1 and o.t1 > end_t
        and math.dist(here, (o.x, o.y)) <= o.radius + cfg["reach_m"] for o in obstacles)


def _frame(run: Run, robot, t0, t1):
    if run.cut:
        return "기록이 줄 중간에서 끝남", run.end_t
    for e in run.events:
        if e.code in FRAME_CODES and e.robot in ("", robot):
            return e.code, e.t
    if robot:
        silent = not any(t0 <= s[0] <= t1 for s in run.states.get(robot, ()))
    else:
        silent = not run.states
    return ("상태 보고 없음", t0) if silent else None


def _started(run, robot, goal, cfg) -> tuple | None:
    """2번: 시작하지 못했으면 (근거 이름, 시각)."""
    limit = goal.t + cfg["start_limit_s"]
    for e in run.events:
        if e.code == "NO_START" and e.robot == robot and e.t >= goal.t:
            return e.code, e.t
    moves = run.moves_of(robot)
    home = _pos_at(moves, goal.t)
    tried = any(s[0] <= limit and s[0] >= goal.t and s[3] == "BLOCKED"
                for s in run.states.get(robot, ())) or any(
        e.code in BLOCK_CODES and e.robot == robot and goal.t <= e.t <= limit for e in run.events)
    moved = home is not None and any(
        goal.t < t <= limit and math.dist(home, (x, y)) > cfg["move_m"] for t, x, y in moves)
    return None if moved or tried else (goal.code, goal.t)


def _cause(run, order, obstacles, cfg) -> Row:
    binds = [e for e in run.events if e.code == "ORDER_BOUND" and e.detail.split()[:1] == [order]]
    report = run.reports.get(order)
    end = report[0] if report else run.end_t
    robot, t0 = (binds[-1].robot, binds[-1].t) if binds else ("", end)
    frame = _frame(run, robot, t0, end)
    if frame:
        return Row(order, "시험 틀", *frame)
    if not binds:
        isolated = next((e for e in run.events if e.code == "ROBOT_ISOLATED"), None)
        if isolated:  # 가정: 격리된 로봇이 구역을 쥐고 있어 줄이 안 풀렸다
            return Row(order, "주행", isolated.code, isolated.t)
        return Row(order, "관제", "ORDER_BOUND 없음", end)
    goals = [e for e in run.events if e.code in GOAL_CODES and e.robot == robot
             and t0 <= e.t <= end]
    if not goals:
        return Row(order, "관제", "ORDER_BOUND", t0)
    stuck = _started(run, robot, goals[-1], cfg)
    if stuck:
        return Row(order, "주행", *stuck)
    episodes = _episodes(run, robot, goals[0].t, end)
    for episode in episodes:
        late = _late(run, robot, episode, cfg)
        if late:
            return Row(order, "관제", *late)
    if episodes:
        last = episodes[-1]
        real = _really_blocked(run, robot, last, end, obstacles, cfg)
        return Row(order, "진짜 막힘" if real else "회피", last[2], last[0])
    for e in run.events:
        if e.code == "SILENT_STALL" and e.robot == robot and t0 <= e.t <= end:
            return Row(order, "관제", e.code, e.t)
    return Row(order, "주행", "끝 보고 없음", goals[-1].t)  # 막히지 않았고 끝나지도 않았다


def classify(run: Run, obstacles=(), **override) -> list[Row]:
    """끝나지 않은 주문마다 한 줄. 끝난 것은 SHIPPED나 PARTIAL로 보고된 주문이다.

    override: start_limit_s, move_m, first_reaction_s, stage_slack_s, reach_m,
    blocked_mark_s, blocked_timeout_s. 뒤 둘의 기본은 기록 머리의 파라미터다.
    """
    cfg = {"start_limit_s": 5.0, "move_m": 0.02, "first_reaction_s": 1.0, "stage_slack_s": 5.0,
           "reach_m": 0.35,
           "blocked_mark_s": run.params.get("blocked_mark_s", 10.0),
           "blocked_timeout_s": run.params.get("blocked_timeout_s", 60.0)}
    cfg.update(override)
    return [_cause(run, order, obstacles, cfg) for order in run.orders
            if run.reports.get(order, (0, ""))[1] not in DONE]


def markdown(rows) -> str:
    lines = ["| 주문 id | 원인 | 근거 사건 | 시각 (s) |", "|---|---|---|---|"]
    lines += [f"| {r.order_id} | {r.cause} | {r.event} | {r.t:.1f} |" for r in rows]
    return "\n".join(lines)
