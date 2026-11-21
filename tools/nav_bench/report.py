import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as mt  # noqa: E402
import movers as mv  # noqa: E402

COLUMNS = ("time_s", "path_m", "lidar_min_m", "body_min_m", "doridori", "cpu_pct", "cmd_dt_p95_ms")
COUNTS = ("collisions", "blocked")
DIGITS = {"doridori": 0, "cpu_pct": 0, "time_s": 1, "cmd_dt_p95_ms": 1}
SPLIT = ("wait_s", "turn_s", "stop_s", "drive_s", "drive_vx")
LABELS = {"FollowPathGoal": "RPP", "FollowPathMPPI": "MPPI", "FollowPathRL": "RL",
          "FollowPathRLShim": "RL+shim"}
DIFFS = (("time_s", "시간 s"), ("path_m", "달린 길 m"), ("body_min_m", "몸 최소 거리 m"),
         ("doridori_per_min", "도리도리 /분"), ("drive_vx", "주행 vx m/s"))


def label(ctl):
    return LABELS.get(ctl, ctl)


def with_controllers(cfg, rows):
    names = list(cfg["controllers"])
    names += [c for c in dict.fromkeys(r["controller"] for r in rows) if c not in names]
    return {**cfg, "controllers": names}


# ---- 표 ----
def fmt(mean, sd, digits=2):
    if mean is None:
        return "-"
    return f"{mean:.{digits}f} ± {sd:.{digits}f}"


def band(row):
    if row.get("plan_in_band") is None:
        return "?"
    return "통로" if row["in_corridor"] else "밖"


def rates(legs, total):
    minutes = sum(r["time_s"] for r in legs) / 60.0
    metres = sum(r["path_m"] for r in legs)
    per_min = f"{total / minutes:.1f}/분" if minutes > 0 else "-"
    per_m = f"{total / metres:.2f}/m" if metres > 0 else "-"
    return f"({per_min}, {per_m})"


def groups(rows, cfg):
    for scenario in cfg["scenarios"]:
        for ctl in cfg["controllers"]:
            legs = [r for r in rows if r["scenario"] == scenario and r["controller"] == ctl]
            for b in ("통로", "밖", "?"):
                part = [r for r in legs if band(r) == b]
                if part:
                    yield scenario, ctl, b, part


def table(rows, cfg):
    lines = ["| 시나리오 | 컨트롤러 | 길 | 성공 | 시간 s | 달린 길 m"
             " | 라이다 최소 m | 몸 최소 거리 m"
             " | 도리도리 (판당, /분, /m) | 충돌 (합, /분, /m) | BLOCKED (합, /분, /m) | CPU %"
             " | cmd 간격 p95 ms |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for scenario, ctl, b, legs in groups(rows, cfg):
        ok = [r for r in legs if r["ok"]]
        cells = [scenario, ctl, b, f"{len(ok)}/{len(legs)}"]
        for col in COLUMNS:
            src = ok if col == "time_s" else legs
            cell = fmt(*mt.summary([r[col] for r in src]), DIGITS.get(col, 3))
            if col == "doridori":
                cell += " " + rates(legs, sum(r[col] for r in legs))
            cells.append(cell)
        counts = [f"{sum(r[c] for r in legs)} {rates(legs, sum(r[c] for r in legs))}"
                  for c in COUNTS]
        cells[9:9] = counts
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def split_table(rows, cfg):
    lines = ["| 시나리오 | 컨트롤러 | 길 | 첫 명령까지 s | 제자리 회전 s | 멈춤 s | 주행 s"
             " | 주행 중 평균 vx m/s |",
             "|---|---|---|---|---|---|---|---|"]
    for scenario, ctl, b, legs in groups(rows, cfg):
        ok = [r for r in legs if r["ok"] and r.get("drive_s") is not None]
        if ok:
            cells = [fmt(*mt.summary([r[c] for r in ok]), 3 if c == "drive_vx" else 1)
                     for c in SPLIT]
            lines.append("| " + " | ".join([scenario, ctl, b, *cells]) + " |")
    return "\n".join(lines)


def fairness_table(rows, cfg):
    lines = ["| 시나리오 | 컨트롤러 | 통로 판 | 계획 경로 띠 안 | 실제 궤적 띠 안"
             " | 출발 위치 오차 m (평균 ± sd, 최대) | 출발 방향 오차 ° (최대)"
             " | 옮긴 판 | 옮겨도 벗어난 판 | 다시 보낸 판 |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for scenario in cfg["scenarios"]:
        for ctl in cfg["controllers"]:
            legs = [r for r in rows if r["scenario"] == scenario and r["controller"] == ctl]
            if not legs:
                continue
            n = len(legs)
            errs = [r["start_err_m"] for r in legs if r.get("start_err_m") is not None]
            degs = [r["start_err_deg"] for r in legs if r.get("start_err_deg") is not None]
            pos = f"{fmt(*mt.summary(errs), 3)}, {max(errs):.3f}" if errs else "-"
            cells = [scenario, ctl, f"{sum(band(r) == '통로' for r in legs)}/{n}",
                     f"{sum(r.get('plan_in_band') is True for r in legs)}/{n}",
                     f"{sum(r.get('truth_in_band') is True for r in legs)}/{n}", pos,
                     f"{max(degs):.1f}" if degs else "-",
                     str(sum(bool(r.get("replaced")) for r in legs)),
                     str(sum(r.get("start_ok") is False for r in legs)),
                     str(sum(bool(r.get("retried")) for r in legs))]
            lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def diff_value(row, col):
    if col == "doridori_per_min":
        return row["doridori"] / (row["time_s"] / 60.0) if row["time_s"] > 0 else None
    if col == "time_s" and not row["ok"]:
        return None
    return row.get(col)


def diff_table(rows, cfg):
    lines = ["| 시나리오 | 차이 | 지표 | 평균 차이 ± 표준오차 | Welch t | 판 수 |",
             "|---|---|---|---|---|---|"]
    names = [c for c in cfg["controllers"] if any(r["controller"] == c for r in rows)]
    pairs = [(a, b) for i, a in enumerate(names) for b in names[:i]]
    pairs.sort(key=lambda p: (-names.index(p[0]), names.index(p[1])))
    for scenario in cfg["scenarios"]:
        legs = [r for r in rows if r["scenario"] == scenario and band(r) != "밖"]
        for a, b in pairs:
            for col, name in DIFFS:
                va = [diff_value(r, col) for r in legs if r["controller"] == a]
                vb = [diff_value(r, col) for r in legs if r["controller"] == b]
                diff, se, t = mt.welch(va, vb)
                if diff is None:
                    continue
                n = f"{sum(v is not None for v in va)}/{sum(v is not None for v in vb)}"
                tt = f"t {t:.1f}" if t is not None else "-"
                lines.append(f"| {scenario} | {label(a)} - {label(b)} | {name}"
                             f" | {diff:.2f} ± {se:.2f} | {tt} | {n} |")
    return "\n".join(lines)


def figure(rows, cfg, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))
    names = list(cfg["scenarios"])
    panels = (("time_s", "time (s, succeeded)"), ("doridori", "wz sign changes"),
              ("body_min_m", "min body clearance (m)"))
    for k, (col, title) in enumerate(panels):
        ax = axes[k]
        for j, ctl in enumerate(cfg["controllers"]):
            means, sds = [], []
            for s in names:
                src = [r[col] for r in rows if r["scenario"] == s and r["controller"] == ctl
                       and (r["ok"] or col != "time_s")]
                m, sd = mt.summary(src)
                means.append(m or 0.0)
                sds.append(sd or 0.0)
            w = 0.8 / len(cfg["controllers"])
            ax.bar([i + w * j for i in range(len(names))], means, w * 0.95, yerr=sds,
                   label=label(ctl), capsize=3)
        ax.set_xticks([i + 0.4 - 0.4 / len(cfg["controllers"]) for i in range(len(names))], names,
                      fontsize=8)
        ax.set_title(title, fontsize=9)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


def tracks(raw, cfg, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = list(cfg["scenarios"])
    fig, axes = plt.subplots(len(names), 1, figsize=(8, 3.6 * len(names)))
    for ax, s in zip(axes, names, strict=False):
        for r in mt.arena_rects():
            xs, ys = zip(*(r + r[:1]), strict=False)
            ax.fill(xs, ys, color="0.8")
        for rec in raw:
            if rec["scenario"] != s:
                continue
            xs = [p[1] for p in rec["truth"]]
            ys = [p[2] for p in rec["truth"]]
            ax.plot(xs, ys, lw=0.8, color=f"C{cfg['controllers'].index(rec['controller'])}")
        ax.set_xlim(0.0, 1.98)
        ax.set_ylim(0.0, 0.98)
        ax.set_aspect("equal")
        colours = ("blue", "orange", "green", "red", "purple", "brown", "pink", "gray", "olive",
                   "cyan")
        key = ", ".join(f"{colours[j % 10]} {label(c)}" for j, c in enumerate(cfg["controllers"]))
        ax.set_title(f"{s} ({key})", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


# ---- 실행 ----
def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--legs", required=True, nargs="+", help="folders with leg json files")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    cfg = mv.load(args.config)
    body, statics = mt.footprint(), mt.arena_rects()
    raw = [json.loads(p.read_text(encoding="utf-8"))
           for d in args.legs for p in sorted(Path(d).glob("*.json"))]
    rows = [mt.leg_metrics(rec, cfg, body, statics) for rec in raw]
    cfg = with_controllers(cfg, rows)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "legs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    text = "\n\n".join([table(rows, cfg), split_table(rows, cfg), fairness_table(rows, cfg),
                         diff_table(rows, cfg)])
    (out / "table.md").write_text(text + "\n", encoding="utf-8")
    figure(rows, cfg, out / "compare.png")
    tracks(raw, cfg, out / "tracks.png")
    print(text)


if __name__ == "__main__":
    main()
