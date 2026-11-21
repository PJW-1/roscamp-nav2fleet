"""Sort the timed-out episodes of a play.py rollouts.npz by what was around our robot at the end.

For the last window_s of each timed-out episode of a tracked env:
  robot_still   a scripted robot came within near_m of our robot and moved less than still_m
                (parked at its last goal, pausing, or held at a node)
  robot_moving  a scripted robot came within near_m and kept moving (yield / back-off cycles,
                head-on in a corridor)
  mover         no robot near, but a person or cart within near_m
  alone         nothing moving near: our policy dithering or stuck at a box or a wall
Each row also says how our robot ended (the last tail_s): facing_wall (an obstacle within
wall_m ahead and no headway), stopped, dithering (turning in place or back and forth), moving
(still making way), and where: the nearest graph node within place_m, else "between nodes".
alone_summary is the standard alone-timeout number of every evaluation (play.py eval.json).
No Isaac:  python learning/store_arena/timeouts.py <rollouts.npz> [episode_s]
"""

import json
import sys

import numpy as np

KINDS = ("robot_still", "robot_moving", "mover", "alone")
MOTIONS = ("stopped", "dithering", "facing_wall", "moving")


def motion(us, front, cmd, wall_m=0.2, turn=0.3):
    """How our robot ended: us (T, 2) positions, front (T,) range ahead or None, cmd (T, 2)
    commanded (v, w) or None."""
    path = float(np.linalg.norm(np.diff(us, axis=0), axis=-1).sum())
    net = float(np.linalg.norm(us[-1] - us[0]))
    if front is not None and float(front.min()) < wall_m and net < 0.05:
        return "facing_wall"
    turning = cmd is not None and float(np.abs(cmd[:, 1]).mean()) > turn
    if path < 0.03:
        return "dithering" if turning else "stopped"
    return "dithering" if net < 0.05 or net < 0.5 * path else "moving"


def place(xy, node_xy, node_names, place_m=0.25):
    d = np.linalg.norm(node_xy - xy, axis=-1)
    return str(node_names[d.argmin()]) if d.min() < place_m else "between nodes"


def episodes(done: np.ndarray):
    """(start, end) step index pairs of the finished episodes of one env (end inclusive)."""
    ends = np.flatnonzero(done)
    starts = np.concatenate(([0], ends[:-1] + 1))
    return list(zip(starts, ends, strict=True))


def classify(z, episode_s=30.0, window_s=10.0, near_m=0.35, still_m=0.05, tail_s=3.0):
    z = {k: z[k] for k in z}  # an npz decompresses on every access: read it once
    dt = float(z["dt"])
    full = int(round(episode_s / dt))
    w = int(round(window_s / dt))
    tail = max(int(round(tail_s / dt)), 2)
    out = []
    for e in range(z["xy"].shape[1]):
        for s, t in episodes(z["done"][:, e]):
            if "outcome" in z:  # play.py records why each episode ended
                if z["outcome"][t, e] != 3:
                    continue
            elif t - s + 1 < full:  # older rollouts: by length (a last-step success counts too)
                continue
            # the done step t already shows the next episode's reset pose: leave it out
            a = max(s, t - w)
            us = z["xy"][a:t, e]
            kind = "alone"
            moved_us = float(np.linalg.norm(us[-1] - us[0]))
            if "robot_xy" in z:
                for r in range(z["robot_xy"].shape[2]):
                    if not z["robot_on"][s, e, r]:
                        continue
                    rx = z["robot_xy"][a:t, e, r]
                    if np.linalg.norm(rx - us, axis=-1).min() >= near_m:
                        continue
                    moved = np.linalg.norm(np.diff(rx, axis=0), axis=-1).sum()
                    k = "robot_still" if moved < still_m else "robot_moving"
                    kind = k if kind != "robot_still" else kind
            if kind == "alone" and "mover_xy" in z:
                for m in range(z["mover_xy"].shape[2]):
                    on = z["mover_on"][s, e, m]
                    mx = z["mover_xy"][a:t, e, m]
                    if on and np.linalg.norm(mx - us, axis=-1).min() < near_m:
                        kind = "mover"
            b = max(s, t - tail)
            row = {
                "env": e,
                "start": int(s),
                "kind": kind,
                "end_xy": [round(float(v), 3) for v in us[-1]],
                "moved_last_window_m": round(moved_us, 3),
                "motion": motion(
                    z["xy"][b:t, e],
                    z["front_m"][b:t, e] if "front_m" in z else None,
                    z["cmd"][b:t, e] if "cmd" in z else None,
                ),
            }
            if "node_xy" in z:
                row["place"] = place(us[-1], z["node_xy"], z["node_names"])
            out.append(row)
    return out


def alone_summary(z, rows, n_episodes=None):
    """The alone timeouts of classify rows: count, rate per 4096 episodes, where, how."""
    if n_episodes is None:
        n_episodes = sum(len(episodes(z["done"][:, e])) for e in range(z["xy"].shape[1]))
    alone = [r for r in rows if r["kind"] == "alone"]
    by_place = {}
    for r in alone:
        by_place[r.get("place", "?")] = by_place.get(r.get("place", "?"), 0) + 1
    return {
        "tracked_episodes": int(n_episodes),
        "timeouts": len(rows),
        "count": len(alone),
        "per_4096": round(4096 * len(alone) / max(n_episodes, 1), 1),
        "by_motion": {m: sum(r["motion"] == m for r in alone) for m in MOTIONS},
        "by_place": dict(sorted(by_place.items(), key=lambda kv: -kv[1])),
    }


if __name__ == "__main__":
    z = np.load(sys.argv[1])
    rows = classify(z, *(float(x) for x in sys.argv[2:3]))
    counts = {k: sum(r["kind"] == k for r in rows) for k in KINDS}
    n_ep = sum(len(episodes(z["done"][:, e])) for e in range(z["xy"].shape[1]))
    print(json.dumps({"tracked_episodes": n_ep, "timeouts": len(rows), "by_kind": counts}))
    print(json.dumps({"alone": alone_summary(z, rows, n_ep)}))
    for r in rows:
        print(json.dumps(r))
