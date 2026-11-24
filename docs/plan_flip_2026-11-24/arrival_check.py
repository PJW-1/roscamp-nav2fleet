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
# 기준: 1단계 기준 판 4판(s1_a1_r1, s1_a1_v1, s1_a_r1, s1_a_v1)에서 닿은 조각 32개의 지연은 7.2 s 한 건 말고
# 모두 3.33 s 이하였다(닿은 뒤 방향 맞추기, yaw 허용치 0.12 rad). 그 위로 여유를 두어 5 s
SLOW_S = 5.0
LOOK_BACK_S = 0.5  # 관제 feedback 주기(0.2 s)보다 넉넉히: 앞 조각 끝 보고가 늦은 만큼


def segment_ends(rows):
    """nav_goal 조각마다 (로봇, x, y, 창 시작, 끝, 성공). 창은 조각이 시작한 때(앞 조각 끝이 보고된 때)보다
    LOOK_BACK_S 앞에서 연다. 주문 갱신은 같은 order의 진행으로 잇는다. 실패·취소로 끝난 nav_goal 조각도
    내며, 갱신 뒤에 온 옛 goal의 실패와 새 order 뒤에 온 이전 order의 취소도 센다."""
    cur, retired, out = {}, {}, []

    def advance(c, t, done):
        for s in c["segs"]:
            if c["done"] < s["seq_id"] <= done:
                if s["mode"] == "nav_goal":
                    out.append((c["robot"], s["x"], s["y"], c["start"] - LOOK_BACK_S, t, True))
                c["start"] = t
        c["done"] = max(c["done"], done)

    def state_of(r, gid):
        for table in (cur, retired):
            if r in table and gid in table[r]["chain"]:
                return table
        return None

    for d in rows:
        t = d.get("t")
        if d.get("out") == "MoveOrderCmd":
            r = d["robot"]
            old = cur.get(r)
            if d.get("update_id", 0) > 0 and old is not None and old["order"] == d.get("order_id"):
                old["segs"] = [s for s in old["segs"] if s["seq_id"] > old["done"]] + list(d["base"])
                old["chain"].add(d["goal_id"])
                old["goal"] = d["goal_id"]
            else:
                if old is not None:
                    retired[r] = old
                cur[r] = {"robot": r, "segs": list(d["base"]), "done": 0, "start": t,
                          "order": d.get("order_id"), "chain": {d["goal_id"]}, "goal": d["goal_id"]}
        elif d.get("in") == "on_move_feedback":
            r, gid, done = d["args"][0], d["args"][1], d["args"][2]
            table = state_of(r, gid)
            if table is not None:
                advance(table[r], t, done)
        elif d.get("in") == "on_move_result":
            r, gid, ok = d["args"][0], d["args"][1], d["args"][2]
            table = state_of(r, gid)
            if table is None:
                continue
            c = table[r]
            if len(d["args"]) > 4:
                advance(c, t, d["args"][4])
            reason = d["args"][3] if len(d["args"]) > 3 else ""
            if not ok and reason != "SUPERSEDED":
                s = next((s for s in c["segs"] if s["seq_id"] > c["done"]), None)
                if s is not None and s["mode"] == "nav_goal":
                    out.append((r, s["x"], s["y"], c["start"] - LOOK_BACK_S, t, False))
                del table[r]
            elif ok and gid == c["goal"]:
                del table[r]
    return out


def lags(segments, truth):
    """truth: {로봇: [(t, x, y)]} 시간순. 반환: (끝난 조각의 지연 목록, 닿은 뒤 실패한 조각 수)."""
    done, near_fail = [], 0
    for r, gx, gy, t0, t1, ok in segments:
        first = next((t for t, x, y in truth.get(r, ()) if t0 <= t <= t1
                      and math.hypot(x - gx, y - gy) <= NEAR_M), None)
        if first is None:
            continue
        if ok:
            done.append(t1 - first)
        else:
            near_fail += 1
    return done, near_fail


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
    v, near_fail = lags(segment_ends(rows), read_truth(run / "truth.csv"))
    v = sorted(v)
    return {"run": run.name, "arrived": len(v), "slow": sum(x > SLOW_S for x in v),
            "near_fail": near_fail,
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
