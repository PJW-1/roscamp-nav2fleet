import csv
import json
import math
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "nav_bench"))
import metrics as mt  # noqa: E402
import movers as mv  # noqa: E402

HERE = Path(__file__).resolve().parent
ORDER = ["FollowPathGoal", "FollowPathMPPI", "FollowPathRLShim", "FollowPathRLShimC",
         "FollowPathRLShimACD"]
NAMES = {"FollowPathGoal": "RPP", "FollowPathMPPI": "MPPI", "FollowPathRLShim": "RL+shim 4b",
         "FollowPathRLShimC": "RL+shim c", "FollowPathRLShimACD": "RL+shim acd"}
BATCHES = (("q2e3", ROOT / "results/2026-11-22/compare_012/q2e3.yaml"),
           ("main", ROOT / "tools/nav_bench/scenarios.yaml"))
COLUMNS = ["controller", "name", "scenario", "runs", "successes", "wall_obstacle_contacts",
           "robot_contacts", "timeouts", "mean_time_s", "p90_time_s", "min_clearance_m"]


def p90(values):
    v = sorted(values)
    return v[min(len(v) - 1, math.ceil(0.9 * len(v)) - 1)] if v else None


def leg_row(rec, cfg, body, statics):
    m = mt.leg_metrics(rec, cfg, body, statics)
    names = rec.get("movers_names", [])
    t0, t1 = rec["t_start"], rec["t_end"]
    truth = mt.within(rec["truth"], t0, t1)
    others = [cfg["movers"][n] for n in names if n != "robot"]
    robot = [cfg["movers"][n] for n in names if n == "robot"]
    idx = [i for i, n in enumerate(names) if n != "robot"]
    track_o = [[t[0]] + [t[1 + i] for i in idx] for t in rec["movers"]]
    track_r = [[t[0]] + [t[1 + i] for i, n in enumerate(names) if n == "robot"]
               for t in rec["movers"]]
    walls = mt.body_clearance(truth, body, statics, others, track_o)
    bot = mt.body_clearance(truth, body, [], robot, track_r) if robot else []
    return {"ok": m["ok"], "time_s": m["time_s"], "status": rec["status"],
            "walls": mt.events_below(walls, cfg["collision_m"]),
            "robot": mt.events_below(bot, cfg["collision_m"]) if bot else 0,
            "body_min": m["body_min_m"]}


def rows_for(out):
    body, statics = mt.footprint(), mt.arena_rects()
    cells = {}
    for batch, cfg_path in BATCHES:
        cfg = mv.load(cfg_path)
        for p in sorted((out / batch / "legs").glob("*.json")):
            rec = json.loads(p.read_text())
            scenario = "q2e3" if batch == "q2e3" else rec["scenario"]
            cells.setdefault((rec["controller"], scenario), []).append(
                leg_row(rec, cfg, body, statics))
    rows = []
    key = lambda k: (ORDER.index(k[0]) if k[0] in ORDER else 99, k[1])  # noqa: E731
    for ctl, scenario in sorted(cells, key=key):
        legs = cells[(ctl, scenario)]
        times = [g["time_s"] for g in legs if g["ok"]]
        mins = [g["body_min"] for g in legs if g["body_min"] is not None]
        rows.append({
            "controller": ctl, "name": NAMES.get(ctl, ctl), "scenario": scenario,
            "runs": len(legs), "successes": sum(g["ok"] for g in legs),
            "wall_obstacle_contacts": sum(g["walls"] for g in legs),
            "robot_contacts": sum(g["robot"] for g in legs),
            "timeouts": sum(g["status"] == "TIMEOUT" for g in legs),
            "mean_time_s": round(sum(times) / len(times), 1) if times else None,
            "p90_time_s": round(p90(times), 1) if times else None,
            "min_clearance_m": round(min(mins), 3) if mins else None})
    return rows


def controller_lines():
    try:
        from fleet_gz_sim import launch_parts as lp
    except ImportError:
        return ["(source the ROS install to list the merged controller settings)"]
    ov = yaml.safe_load((HERE / "item8_overrides.yaml").read_text(encoding="utf-8"))
    merged = lp.sim_nav_params(ROOT / "pode/pode_nav_config/params/nav2_params.yaml",
                               Path(tempfile.gettempdir()) / "item8_merged.yaml",
                               max_speed=lp.nav_speed_mps(), overrides=ov,
                               avoidance=lp.avoidance_file(lp.AVOIDANCE_PACKAGE))
    cs = yaml.safe_load(merged.read_text())["controller_server"]["ros__parameters"]
    lines = []
    for ctl in ORDER:
        c = cs[ctl]
        shim = ""
        if "primary_controller" in c:
            shim = (f"`{c['plugin']}` wrapping `{c['primary_controller']}` (shim "
                    f"angular_dist_threshold {c['angular_dist_threshold']}, "
                    f"angular_disengage_threshold "
                    f"{c.get('angular_disengage_threshold', 'default 0.3925')}, "
                    f"rotate {c['rotate_to_heading_angular_vel']} rad/s, "
                    f"accel {c['max_angular_accel']}, "
                    f"simulate_ahead_time {c['simulate_ahead_time']} s, "
                    f"rotate_to_goal_heading {c['rotate_to_goal_heading']})")
        if "policy" in c:
            spec = json.loads((ROOT / "pode/pode_rl_controller/policies" / c["policy"] /
                               "policy_spec.json").read_text())
            cap = spec.get("speed_cap")
            speed = (f"speed cap obs = min(max_linear_vel {c['max_linear_vel']}, nav_speed), "
                     f"trained range {cap['min']}-{cap['max']}" if cap else
                     f"max_linear_vel {c['max_linear_vel']} m/s")
            src = spec["source"]
            lines.append(
                f"- **{NAMES[ctl]}** (`{ctl}`): {shim}; policy `{c['policy']}` "
                f"(format {spec['format']}, {spec['obs_dim']} inputs, "
                f"blocks {spec.get('obs_blocks', [])}, {src['checkpoint']} from {src['branch']} "
                f"{src['commit']}); {speed}")
        elif ctl == "FollowPathMPPI":
            lines.append(f"- **MPPI** (`{ctl}`): {shim or c['plugin']}; vx_max {c['vx_max']} "
                         f"m/s, time_steps {c.get('time_steps')}, model_dt {c.get('model_dt')}")
        else:
            lines.append(f"- **RPP** (`{ctl}`): `{c['plugin']}`, desired_linear_vel "
                         f"{c.get('desired_linear_vel')} m/s, lookahead "
                         f"{c.get('lookahead_dist')} m")
    return lines


def main():
    out = Path(sys.argv[1])
    dest = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE
    rows = rows_for(out)
    with open(dest / "item8_results.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    head = "| " + " | ".join(COLUMNS[1:]) + " |"
    table = [head, "|" + "---|" * (len(COLUMNS) - 1)]
    table += ["| " + " | ".join("" if r[c] is None else str(r[c]) for c in COLUMNS[1:]) + " |"
              for r in rows]
    md = ["# item 8 다섯 컨트롤러 비교 결과", "", f"원본: `{out}`", "", "## 컨트롤러", "",
          *controller_lines(), "", "## 결과", "", *table, ""]
    (dest / "item8_results.md").write_text("\n".join(md), encoding="utf-8")
    print("\n".join(table))


if __name__ == "__main__":
    main()
