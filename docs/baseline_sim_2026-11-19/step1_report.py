"""1번 기준 시뮬 여러 판 합계: Q2를 지나는 Route마다 구간 모양, 갱신 시점, 선 시간.

사용: python3 step1_report.py NAME=RESULT_DIR [NAME=RESULT_DIR ...] [--routes]
Route = 주문 하나(order_id), ORDER_SENT부터 그 주문의 마지막 SEGMENT_DONE까지.
"""
import csv
import json
import math
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

Q2 = (1.66, 0.15)
MOVING = 0.03  # m/s, 이보다 빠르면 다시 움직인 것으로 본다


def load(d):
    d = Path(d)
    ev = [dict(r, t=float(r["time_s"])) for r in csv.DictReader(open(d / "events.csv"))]
    rec = [json.loads(line) for line in open(d / "fleet_record.jsonl")] \
        if (d / "fleet_record.jsonl").exists() else []
    pos = defaultdict(list)
    for r in csv.DictReader(open(d / "robots.csv")):
        pos[r["robot"]].append((float(r["time_s"]), float(r["x"]), float(r["y"])))
    summ = json.loads((d / "summary.json").read_text())
    check = json.loads((d / "fleet_check.json").read_text()) if (d / "fleet_check.json").exists() else {}
    safety = json.loads((d / "safety.json").read_text()) if (d / "safety.json").exists() else {}
    return ev, rec, pos, summ, check, safety


def segs(cmd):
    return [f'{s["mode"]}:{s["end_node"]}' for s in cmd.get("base", [])], \
           [f'{s["mode"]}:{s["end_node"]}' for s in cmd.get("horizon", [])]


def resume_after(rows, t):
    for (t0, x0, y0), (t1, x1, y1) in zip(rows, rows[1:]):
        if t1 <= t or t1 <= t0:
            continue
        if math.hypot(x1 - x0, y1 - y0) / (t1 - t0) > MOVING:
            return round((t0 + t1) / 2 - t, 2)
    return None


def q2_routes(ev, rec, pos):
    cmds = defaultdict(list)
    for r in rec:
        if r.get("out") == "MoveOrderCmd":
            cmds[r["order_id"]].append(r)
    done = defaultdict(list)
    sent = {}
    for e in ev:
        oid = e["detail"].split(" ")[0].rstrip(":")
        if e["code"] == "SEGMENT_DONE":
            done[oid].append((e["t"], e["detail"].rsplit(" to ", 1)[1], e["detail"]))
        elif e["code"] == "ORDER_SENT":
            sent[oid] = e["t"]
    waits = [(e["t"], e["source"]) for e in ev
             if e["code"] == "BASE_END_WAIT" and e["detail"].startswith("stands at Q2,")]
    zone_waits = [(e["t"], e["source"], e["detail"]) for e in ev if e["code"] == "ZONE_WAIT"]
    out = []
    for oid, ds in done.items():
        arr = [t for t, node, _ in ds if node == "Q2"]
        if not arr or oid not in sent:
            continue
        t_arr = arr[0]
        first = cmds[oid][0] if cmds[oid] else {}
        base, horizon = segs(first)
        robot = first.get("robot") or oid.split("-")[0]
        # Q2에서 이어 붙인 갱신(stitch_node == Q2)
        ups = [c for c in cmds[oid] if c.get("stitch_node") == "Q2"]
        lead = round(t_arr - ups[0]["t"], 2) if ups else None
        stood = any(abs(t - t_arr) < 0.2 and src == robot for t, src in waits)
        zw = [z for z in zone_waits if z[1] == robot and t_arr - 1 <= z[0] <= t_arr + 5]
        prev_end = None
        rows = pos.get(robot, [])
        at = [p for p in rows if p[0] <= sent[oid]]
        if at:
            prev_end = (round(at[-1][1], 2), round(at[-1][2], 2))
        out.append({"route": oid, "robot": robot, "sent": round(sent[oid], 2),
                    "start_xy": prev_end, "base": " ".join(base), "horizon": " ".join(horizon),
                    "arrive_Q2": round(t_arr, 2), "update_lead_s": lead,
                    "base_end_wait": stood, "zone_wait_near": len(zw),
                    "route_s": round(ds[-1][0] - sent[oid], 2), "end": ds[-1][1]})
    return sorted(out, key=lambda r: r["sent"])


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    show = "--routes" in sys.argv
    table = []
    for a in args:
        name, d = a.split("=", 1)
        ev, rec, pos, summ, check, safety = load(d)
        rs = q2_routes(ev, rec, pos)
        all_routes = {}
        sent = {}
        for e in ev:
            oid = e["detail"].split(" ")[0].rstrip(":")
            if e["code"] == "ORDER_SENT":
                sent[oid] = e["t"]
            elif e["code"] == "SEGMENT_DONE" and oid in sent:
                all_routes[oid] = e["t"] - sent[oid]
        codes = defaultdict(int)
        for e in ev:
            codes[e["code"]] += 1
        leads = [r["update_lead_s"] for r in rs if r["update_lead_s"] is not None]
        table.append({
            "run": name, "pass": summ.get("pass"),
            "shipped": f'{summ.get("orders_shipped")}/{summ.get("orders_total")}',
            "duration_s": round(summ.get("duration_s", 0), 1),
            "violations": len(summ.get("violations", [])),
            "min_dist_m": check.get("min_distance_m"), "min_gap_m": safety.get("min_gap_m"),
            "routes": len(all_routes),
            "route_mean_s": round(st.mean(all_routes.values()), 2) if all_routes else None,
            "q2_routes": len(rs),
            "q2_route_mean_s": round(st.mean(r["route_s"] for r in rs), 2) if rs else None,
            "q2_stood": sum(r["base_end_wait"] for r in rs),
            # 기록 시각은 0.1 s 틱이라 ±0.1 s 안은 도착과 같은 때로 본다
            "update_before": sum(1 for x in leads if x > 0.1),
            "update_at_arrival": sum(1 for x in leads if -0.1 <= x <= 0.1),
            "update_after": sum(1 for x in leads if x < -0.1),
            "no_q2_update": sum(1 for r in rs if r["update_lead_s"] is None),
            "lead_median_s": round(st.median(leads), 2) if leads else None,
            "base_end_wait_all": codes["BASE_END_WAIT"], "zone_wait": codes["ZONE_WAIT"],
            "local_yield": codes["LOCAL_YIELD"], "blocked_wait": codes["BLOCKED_WAIT"],
            "self_recovery": codes["SELF_RECOVERY"], "comm_lost": codes["COMM_LOST"],
        })
        if show:
            print(f"\n### {name}")
            keys = list(rs[0].keys()) if rs else []
            print("| " + " | ".join(keys) + " |")
            print("|" + "---|" * len(keys))
            for r in rs:
                print("| " + " | ".join(str(r[k]) for k in keys) + " |")
    keys = list(table[0].keys())
    print("\n| " + " | ".join(keys) + " |")
    print("|" + "---|" * len(keys))
    for row in table:
        print("| " + " | ".join(str(row[k]) for k in keys) + " |")


if __name__ == "__main__":
    main()
