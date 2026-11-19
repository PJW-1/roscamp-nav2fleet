"""1번 기준 시뮬 여러 판 합계: Q2를 지나는 Route마다 구간 모양, 갱신 시점, 선 시간.

사용: python3 step1_report.py NAME=RESULT_DIR [NAME=RESULT_DIR ...] [--routes]
Route = 주문 하나(order_id), ORDER_SENT부터 그 주문의 마지막 SEGMENT_DONE까지.

합친 nav_goal Q2 Route(2026-11-20 정의): Q2로 가는 nav_goal Segment의 시작 node가 Q2와 바로 이어진
node(E-3, M_E)가 아닌 Route. E-1, E-2, O_OUT 출발이 들어간다(열 q2_from, joined).
Q2에서 선 Route의 원인(열 stop_cause): 도착한 때 다른 로봇이 Q1을 쥐고 있으면 q1_busy(Q1에서
0.20 m 안: 노드를 놓는 거리. W가 차 있으면 앞 로봇이 Q1에서 기다리므로 이 안에 들어간다), 아니고
도착 앞뒤로 ZONE_WAIT가 있으면 zone_wait, 둘 다 아니면 other.
"""
import csv
import json
import math
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

Q2 = (1.66, 0.15)
Q1 = (1.8491, 0.26)
MOVING = 0.03  # m/s, 이보다 빠르면 다시 움직인 것으로 본다
NEXT_TO_Q2 = ("E-3", "M_E")  # Q2와 nav 이음으로 바로 이어진 node
YARD = {"E-1": (0.12, 0.24), "E-2": (0.38, 0.24), "E-3": (0.64, 0.24), "O_OUT": (0.18, 0.62),
        "O_IN": (0.49, 0.625), "M_E": (1.645, 0.55)}  # v7에서 nav_goal이 시작할 수 있는 node
ON_NODE = 0.10   # m, Route를 받은 자리가 node에서 이 안이면 그 node에서 출발한 것으로 본다
HOLDS_Q1 = 0.20  # m, 노드를 놓는 거리


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


def q2_from(cmds, start_xy):
    """Q2로 가는 nav_goal Segment의 시작 node: 앞 Segment의 끝 node, 첫 Segment면 받은 자리의 node.

    목적이 바뀌어 다시 받은 Route는 seq_id가 겹칠 수 있어, 명령을 받은 순서대로 보고 최신 것을 쓴다.
    """
    seen = {}
    for c in cmds:
        segs_ = [*c.get("base", []), *c.get("horizon", [])]
        for i, s in enumerate(segs_):
            if s["mode"] == "nav_goal" and s["end_node"] == "Q2":
                before = segs_[i - 1] if i else seen.get(s["seq_id"] - 1)
                if before is not None:
                    return before["end_node"]
                if start_xy is None:
                    return "?"
                node = min(YARD, key=lambda n: math.dist(YARD[n], start_xy))
                return node if math.dist(YARD[node], start_xy) <= ON_NODE else "?"
        seen.update({s["seq_id"]: s for s in segs_})
    return None


def others_at(pos, robot, t):
    out = []
    for name, rows in pos.items():
        at = [p for p in rows if p[0] <= t]
        if name != robot and at:
            out.append((at[-1][1], at[-1][2]))
    return out


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
        start = q2_from(cmds[oid], prev_end)
        q1_busy = any(math.dist(p, Q1) < HOLDS_Q1 for p in others_at(pos, robot, t_arr))
        cause = "" if not stood else "q1_busy" if q1_busy else "zone_wait" if zw else "other"
        out.append({"route": oid, "robot": robot, "sent": round(sent[oid], 2),
                    "start_xy": prev_end, "base": " ".join(base), "horizon": " ".join(horizon),
                    "q2_from": start, "joined": start not in (None, "?", *NEXT_TO_Q2),
                    "arrive_Q2": round(t_arr, 2), "update_lead_s": lead,
                    "base_end_wait": stood, "stop_cause": cause, "zone_wait_near": len(zw),
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
            "joined_q2": sum(r["joined"] for r in rs),
            "joined_before": sum(1 for r in rs if r["joined"] and (r["update_lead_s"] or 0) > 0.1),
            "joined_stood": sum(1 for r in rs if r["joined"] and r["base_end_wait"]),
            "q2_from_unknown": sum(1 for r in rs if r["q2_from"] in (None, "?")),
            "stop_q1_busy": sum(r["stop_cause"] == "q1_busy" for r in rs),
            "stop_zone_wait": sum(r["stop_cause"] == "zone_wait" for r in rs),
            "stop_other": sum(r["stop_cause"] == "other" for r in rs),
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
