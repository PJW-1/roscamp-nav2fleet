"""양보 기록: 관제 기록(fleet_record.jsonl, .jsonl.gz)에서 로봇단 양보(LOCAL_YIELD)와
관제 양보(HALL_YIELD)를 세고, 마주침마다 누가 먼저 나섰는지 시간 순서로 보인다.

    python tools/yield_report.py 기록1.jsonl.gz 기록2.jsonl.gz [--encounters]

읽기만 한다. 기록 형식은 tools/replay.py 머리에 있다.

추정 규칙(틀릴 수 있다. 자세한 한계는 docs/worklog_yield_report_2026-11-18.md):
- HALL_YIELD가 BLOCKED 뒤인지: 이벤트를 낸 로봇, 또는 `out of the way of <로봇>`이 가리키는 로봇이
  그 줄 앞에서 마지막으로 보고한 상태가 BLOCKED면 뒤(`_yield_to_robot`), 아니면 앞(`_watch_near`).
  `waits for`와 `backs out to ... for`는 `_yield_to_robot`만 내므로 문구로 뒤로 정한다.
- 마주침 하나 = 한 로봇의 LOCAL_YIELD 사다리(첫 단계 ~ `ends`). 그 로봇이 낸 또는 이름이 나온
  HALL_YIELD를 사다리 시작 10 s 전부터 끝난 뒤 10 s(`--grace`)까지 묶는다.
- 관제가 먼저 끼어든 HALL_YIELD: 관계된 로봇(낸 로봇과 이름이 나온 로봇) 누구의 사다리도 그때
  (끝난 뒤 10 s까지) 진행 중이 아닌 것.
"""
from __future__ import annotations

import argparse
import gzip
import json
import re
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

LEAD_S = 10.0   # 사다리 시작보다 이만큼 앞선 HALL_YIELD까지 그 마주침으로 묶는다
GRACE_S = 10.0  # 사다리가 끝난 뒤 이만큼까지는 그 마주침으로 본다(BLOCKED 뒤 관제 처리를 담는다)
STEPS = ("wait", "backup", "retry")
# (이름, 문구에 들어 있는 글). 위에서부터 처음 맞는 것
PHRASES = (("backs then stops", re.compile(r"^backs [\d.]+ m, then stops")),
           ("stops", re.compile(r"^stops \d+ s for")),
           ("out of the way", re.compile(r"^out of the way of")),
           ("waits for", re.compile(r"^waits for")),
           ("backs out", re.compile(r"^backs out to")))
NAMED = re.compile(r"\b(?:for|of) ([\w-]+)")
AFTER_BY_PHRASE = ("waits for", "backs out")


@dataclass
class Hall:
    t: float
    robot: str
    phrase: str
    detail: str
    involved: tuple[str, ...]
    after_blocked: bool
    control_first: bool = False


@dataclass
class Encounter:
    robot: str
    wait_t: float
    backup_t: float | None = None
    retry_t: float | None = None
    end_t: float | None = None
    ends: str = ""
    blocked_t: float | None = None
    hall: list[Hall] = field(default_factory=list)

    def active(self, t: float, grace: float = GRACE_S) -> bool:
        return self.wait_t <= t <= (float("inf") if self.end_t is None else self.end_t + grace)


@dataclass
class Stats:
    wait: int = 0
    backup: int = 0
    retry: int = 0
    ends: Counter = field(default_factory=Counter)
    phrases: Counter = field(default_factory=Counter)
    blocked_before: int = 0
    blocked_after: int = 0
    control_first: int = 0
    ladders_hall_first: int = 0

    def add(self, other: "Stats") -> None:
        for name in ("wait", "backup", "retry", "blocked_before", "blocked_after",
                     "control_first", "ladders_hall_first"):
            setattr(self, name, getattr(self, name) + getattr(other, name))
        self.ends.update(other.ends)
        self.phrases.update(other.phrases)


@dataclass
class RunReport:
    name: str
    stats: Stats = field(default_factory=Stats)
    encounters: list[Encounter] = field(default_factory=list)


@dataclass
class Report:
    runs: list[RunReport] = field(default_factory=list)

    @property
    def encounters(self) -> list[Encounter]:
        return [e for run in self.runs for e in run.encounters]

    def total(self) -> Stats:
        total = Stats()
        for run in self.runs:
            total.add(run.stats)
        return total


def _lines(path: Path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as file:
        yield from file


def _phrase(detail: str) -> str:
    return next((name for name, pattern in PHRASES if pattern.match(detail)), "other")


def analyse_one(path: Path, name: str = "", grace: float = GRACE_S) -> RunReport:
    report = RunReport(name or path.parent.name)
    last: dict[str, str] = {}                 # 로봇 -> 지금까지 마지막 보고 상태
    blocked_at: dict[str, list[float]] = {}   # 로봇 -> BLOCKED 보고 시각
    open_ladder: dict[str, Encounter] = {}
    halls: list[Hall] = []
    for line in _lines(path):
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            break  # 줄 중간에서 끊긴 마지막 줄
        if "t" not in entry:
            continue
        t = float(entry["t"])
        if entry.get("in") == "on_robot_state":
            robot, state = entry["args"][0], entry["args"][4]
            last[robot] = state
            if state == "BLOCKED":
                blocked_at.setdefault(robot, []).append(t)
        elif entry.get("out") != "EventOut":
            continue
        elif entry["code"] == "LOCAL_YIELD":
            _local_yield(report, open_ladder, entry["robot"], entry["detail"], t)
        elif entry["code"] == "HALL_YIELD":
            halls.append(_hall(entry, t, last))
    _link(report, halls, blocked_at, grace)
    return report


def _local_yield(report, open_ladder, robot, detail, t) -> None:
    stats = report.stats
    word = detail.split()[0] if detail.split() else ""
    if word in STEPS:
        setattr(stats, word, getattr(stats, word) + 1)
        if robot not in open_ladder:
            open_ladder[robot] = Encounter(robot, t)
            report.encounters.append(open_ladder[robot])
        ladder = open_ladder[robot]
        if word == "backup" and ladder.backup_t is None:
            ladder.backup_t = t
        elif word == "retry" and ladder.retry_t is None:
            ladder.retry_t = t
    elif word == "ends":
        state = detail.split()[1] if len(detail.split()) > 1 else "?"
        stats.ends[state] += 1
        ladder = open_ladder.pop(robot, None)
        if ladder is not None:
            ladder.end_t, ladder.ends = t, state


def _hall(entry, t, last) -> Hall:
    robot, detail = entry["robot"], entry["detail"]
    phrase = _phrase(detail)
    named = tuple(NAMED.findall(detail))
    watched = (robot, *(named if phrase == "out of the way" else ()))
    after = phrase in AFTER_BY_PHRASE or any(last.get(r) == "BLOCKED" for r in watched)
    return Hall(t, robot, phrase, detail, tuple(dict.fromkeys((robot, *named))), after)


def _link(report: RunReport, halls: list[Hall], blocked_at, grace: float) -> None:
    stats = report.stats
    for ladder in report.encounters:
        end = float("inf") if ladder.end_t is None else ladder.end_t
        ladder.blocked_t = next((b for b in blocked_at.get(ladder.robot, ())
                                 if ladder.wait_t <= b <= end), None)
    for hall in halls:
        stats.phrases[hall.phrase] += 1
        if hall.after_blocked:
            stats.blocked_after += 1
        else:
            stats.blocked_before += 1
        hall.control_first = not any(ladder.active(hall.t, grace) for ladder in report.encounters
                                     if ladder.robot in hall.involved)
        stats.control_first += hall.control_first
        for ladder in report.encounters:
            if (ladder.robot in hall.involved
                    and ladder.wait_t - LEAD_S <= hall.t
                    and (ladder.active(hall.t, grace) or hall.t < ladder.wait_t)):
                ladder.hall.append(hall)
    stats.ladders_hall_first = sum(1 for ladder in report.encounters
                                   if any(h.t < ladder.wait_t for h in ladder.hall))


def analyse(paths, grace: float = GRACE_S) -> Report:
    report = Report()
    for path in map(Path, paths):
        # 같은 이름이 겹치면(판 이름이 game1 같은 것뿐일 때) 윗 폴더까지 붙인다
        name = "/".join(path.parts[-3:-1]) if len(path.parts) >= 3 else path.stem
        report.runs.append(analyse_one(path, name, grace))
    return report


def _ends(counter: Counter) -> str:
    return ", ".join(f"{state} {n}" for state, n in sorted(counter.items())) or "-"


def _fmt(t) -> str:
    return "-" if t is None else f"{t:.1f}"


def render(report: Report, encounters: bool = False) -> str:
    rows = [(run.name, run.stats) for run in report.runs]
    rows.append(("합계", report.total()))
    out = ["## LOCAL_YIELD 단계별 건수", "",
           "| 판 | wait | backup | retry | ends |", "|---|---|---|---|---|"]
    out += [f"| {n} | {s.wait} | {s.backup} | {s.retry} | {_ends(s.ends)} |" for n, s in rows]
    names = [name for name, _ in PHRASES] + ["other"]
    out += ["", "## HALL_YIELD 문구별 건수와 추정", "",
            "| 판 | 전체 | " + " | ".join(names) + " | BLOCKED 전 | BLOCKED 뒤 | 관제가 먼저 |",
            "|---|---|" + "---|" * (len(names) + 3)]
    for n, s in rows:
        counts = " | ".join(str(s.phrases[name]) for name in names)
        out.append(f"| {n} | {sum(s.phrases.values())} | {counts} | {s.blocked_before} | "
                   f"{s.blocked_after} | {s.control_first} |")
    total = report.total()
    out += ["", f"사다리(마주침) {len(report.encounters)}번 가운데 사다리 시작보다 앞선 "
            f"HALL_YIELD가 묶인 것은 {total.ladders_hall_first}번이다."]
    if encounters:
        out += ["", "## 마주침별 시간 순서", "",
                "시각은 모두 판 시작부터의 초. HALL_YIELD는 wait 시작을 기준으로 한 차이(초)와 "
                "문구, 추정을 적는다.", "",
                "| 판 | 로봇 | wait | HALL_YIELD | backup | retry | BLOCKED | ends |",
                "|---|---|---|---|---|---|---|---|"]
        for run in report.runs:
            for e in run.encounters:
                halls = "; ".join(f"{h.t - e.wait_t:+.1f} {h.phrase} "
                                  f"({'뒤' if h.after_blocked else '전'}"
                                  f"{', 먼저' if h.control_first else ''})" for h in e.hall)
                out.append(f"| {run.name} | {e.robot} | {_fmt(e.wait_t)} | {halls or '-'} | "
                           f"{_fmt(e.backup_t)} | {_fmt(e.retry_t)} | {_fmt(e.blocked_t)} | "
                           f"{e.ends or '(끝 없음)'} |")
    return "\n".join(out) + "\n"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("records", nargs="+", help="fleet_record.jsonl 또는 .jsonl.gz")
    parser.add_argument("--grace", type=float, default=GRACE_S,
                        help="사다리가 끝난 뒤 이 초까지의 HALL_YIELD는 그 마주침으로 본다")
    parser.add_argument("--encounters", action="store_true", help="마주침마다 한 줄씩 보인다")
    args = parser.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")
    print(render(analyse(args.records, args.grace), args.encounters), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
