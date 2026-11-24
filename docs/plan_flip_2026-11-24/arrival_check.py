#!/usr/bin/env python3
"""Nav2 goal 도착 판정 지연을 잰다(해결안 C 확인용, 11-18 keeper 작업 기록 2절의 문제).

조각마다 truth로 목적지 NEAR_M 안에 처음 들어온 시각부터 관제가 그 조각 끝(last_done_seq)을 받은
시각까지를 잰다. 먼저 넘겨받은 조각(pass_m 앞에서 끝남)은 들어오기 전에 끝나므로 뺀다.

사용: python3 arrival_check.py 판폴더... [--csv out.csv]
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

NEAR_M = 0.06  # Nav2 xy_goal_tolerance 0.05 + 1 cm
SLOW_S = 5.0  # 닿은 뒤 방향 맞추기가 기준 판에서 1.4~3.3 s 걸린다


def segment_ends(rows):
    """nav_goal 조각마다 (로봇, x, y, 시작, 끝). 주문 갱신은 같은 order의 진행으로 잇는다."""
    cur, out = {}, []

    def advance(r, t, done):
        segs, old, start, order, chain = cur[r]
        for s in segs:
            if old < s["seq_id"] <= done:
                if s["mode"] == "nav_goal":
                    out.append((r, s["x"], s["y"], start, t))
                start = t
        cur[r] = (segs, max(old, done), start, order, chain)

    for d in rows:
        t = d.get("t")
        if d.get("out") == "MoveOrderCmd":
            r = d["robot"]
            old = cur.get(r)
            if d.get("update_id", 0) > 0 and old is not None and old[3] == d.get("order_id"):
                segs = [s for s in old[0] if s["seq_id"] > old[1]] + list(d["base"])
                cur[r] = (segs, old[1], old[2], old[3], old[4] | {d["goal_id"]})
            else:
                cur[r] = (list(d["base"]), 0, t, d.get("order_id"), {d["goal_id"]})
        elif d.get("in") == "on_move_feedback":
            r, gid, done = d["args"][0], d["args"][1], d["args"][2]
            if r in cur and gid in cur[r][4]:
                advance(r, t, done)
        elif d.get("in") == "on_move_result":
            r, gid = d["args"][0], d["args"][1]
            if r in cur and gid in cur[r][4] and len(d["args"]) > 4:
                advance(r, t, d["args"][4])
    return out


def lags(segments, truth):
    """truth: {로봇: [(t, x, y)]} 시간순. 반환: 목적지에 닿은 조각마다 지연(초)."""
    res = []
    for r, gx, gy, t0, t1 in segments:
        first = next((t for t, x, y in truth.get(r, ()) if t0 <= t <= t1
                      and math.hypot(x - gx, y - gy) <= NEAR_M), None)
        if first is not None:
            res.append(t1 - first)
    return res


def read_truth(path):
    truth = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            truth.setdefault(row["robot"], []).append(
                (float(row["sim_t"]), float(row["x"]), float(row["y"])))
    return truth


def analyze(run):
    rows = [json.loads(line) for line in (run / "result/fleet_record.jsonl").read_text().splitlines()
            if line.strip()]
    v = sorted(lags(segment_ends(rows), read_truth(run / "truth.csv")))
    return {"run": run.name, "arrived": len(v), "slow": sum(x > SLOW_S for x in v),
            "lag_median_s": round(v[len(v) // 2], 2) if v else None,
            "lag_max_s": round(v[-1], 2) if v else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--csv")
    a = ap.parse_args()
    out = [analyze(Path(r)) for r in a.runs]
    w = csv.DictWriter(open(a.csv, "w", newline="") if a.csv else sys.stdout, fieldnames=list(out[0]))
    w.writeheader()
    w.writerows(out)
    if a.csv:
        for o in out:
            print(o)


if __name__ == "__main__":
    main()
