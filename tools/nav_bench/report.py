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


# ---- 표 ----
def fmt(mean, sd, digits=2):
    if mean is None:
        return "-"
    return f"{mean:.{digits}f} ± {sd:.{digits}f}"


def table(rows, cfg):
    lines = ["| 시나리오 | 컨트롤러 | 성공 | 시간 s | 달린 길 m | 라이다 최소 m | 몸 최소 거리 m"
             " | 도리도리 | 충돌 | BLOCKED | CPU % | cmd 간격 p95 ms |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for scenario in cfg["scenarios"]:
        for ctl in cfg["controllers"]:
            legs = [r for r in rows if r["scenario"] == scenario and r["controller"] == ctl]
            if not legs:
                continue
            ok = [r for r in legs if r["ok"]]
            cells = [scenario, ctl, f"{len(ok)}/{len(legs)}"]
            for col in COLUMNS:
                src = ok if col == "time_s" else legs
                digits = DIGITS.get(col, 3)
                cells.append(fmt(*mt.summary([r[col] for r in src]), digits))
            cells[8:8] = [str(sum(r[c] for r in legs)) for c in COUNTS]
            lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def figure(rows, cfg, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))
    names = list(cfg["scenarios"])
    panels = (("time_s", "time (s, succeeded)"), ("doridori", "wz sign changes"),
              ("body_min_m", "min body clearance (m)"))
    for k, (col, label) in enumerate(panels):
        ax = axes[k]
        for j, ctl in enumerate(cfg["controllers"]):
            means, sds = [], []
            for s in names:
                src = [r[col] for r in rows if r["scenario"] == s and r["controller"] == ctl
                       and (r["ok"] or col != "time_s")]
                m, sd = mt.summary(src)
                means.append(m or 0.0)
                sds.append(sd or 0.0)
            ax.bar([i + 0.38 * j for i in range(len(names))], means, 0.36, yerr=sds,
                   label="RPP" if j == 0 else "MPPI", capsize=3)
        ax.set_xticks([i + 0.19 for i in range(len(names))], names, fontsize=8)
        ax.set_title(label, fontsize=9)
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
            rpp = rec["controller"] == cfg["controllers"][0]
            ax.plot(xs, ys, lw=0.8, color="C0" if rpp else "C1")
        ax.set_xlim(0.0, 1.98)
        ax.set_ylim(0.0, 0.98)
        ax.set_aspect("equal")
        ax.set_title(f"{s} (blue RPP, orange MPPI)", fontsize=9)
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
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "legs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    (out / "table.md").write_text(table(rows, cfg) + "\n", encoding="utf-8")
    figure(rows, cfg, out / "compare.png")
    tracks(raw, cfg, out / "tracks.png")
    print(table(rows, cfg))


if __name__ == "__main__":
    main()
