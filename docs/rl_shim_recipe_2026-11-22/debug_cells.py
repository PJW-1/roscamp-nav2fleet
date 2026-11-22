import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "nav_bench"))
import debug_extract as de  # noqa: E402
import metrics as mt  # noqa: E402
import movers as mv  # noqa: E402


def to_world(truth, odom, px, py):
    _, tx, ty, tyaw = truth
    _, ox, oy, oyaw = odom
    d = tyaw - oyaw
    dx, dy = px - ox, py - oy
    return (tx + math.cos(d) * dx - math.sin(d) * dy, ty + math.sin(d) * dx + math.cos(d) * dy)


def main():
    out = Path(sys.argv[1])
    bag = de.read_bag(out / "bag")
    body, statics = mt.footprint(), mt.arena_rects()
    report = json.loads((out / "debug_extract.json").read_text())
    for key, leg in report.items():
        rec = json.loads((out / key).read_text())
        t0 = rec["t_start"]
        for r in leg["rotations"]:
            if not r["early"]:
                continue
            end = t0 + r["t_end"]
            truth = de.nearest(rec["truth"], end)
            od = de.nearest(bag["odom"], end)
            w = math.copysign(abs(r["w"]) or 1.0, r["err"])
            hits, worst = [], None
            for cm in bag["costmap"]:
                if not end - 0.6 <= cm[0] <= end + 0.2:
                    continue
                for ww in (w, math.copysign(1.0, w)):
                    c = de.shim_check(cm, od, body, r["err"], ww)
                    if c["hit"]:
                        hx, hy = to_world(truth, od, c["hit"][1], c["hit"][2])
                        wall = min(mv.poly_dist([(hx, hy)], s) for s in statics)
                        hits.append((round(cm[0] - end, 2), ww, c["hit_s"], round(hx, 3),
                                     round(hy, 3), round(wall, 3)))
            shape = mv.to_world(body, truth[1], truth[2], truth[3])
            worst = min(mv.poly_dist(shape, s) for s in statics)
            print(f"{key} t {r['t_end']:.2f} pose {r['pose']} err {r['err']} w {r['w']} "
                  f"body-to-wall {worst:.3f} m; costmaps with a hit "
                  f"{len({h[0] for h in hits})}: {hits[:3]}")


if __name__ == "__main__":
    main()
