"""Sort the timed-out episodes of a play.py rollouts.npz by what was around our robot at the end.

For the last window_s of each timed-out episode of a tracked env:
  robot_still   a scripted robot came within near_m of our robot and moved less than still_m
                (parked at its last goal, pausing, or held at a node)
  robot_moving  a scripted robot came within near_m and kept moving (yield / back-off cycles,
                head-on in a corridor)
  mover         no robot near, but a person or cart within near_m
  alone         nothing moving near: our policy dithering or stuck at a box or a wall
No Isaac:  python learning/store_arena/timeouts.py <rollouts.npz> [episode_s]
"""

import json
import sys

import numpy as np

KINDS = ("robot_still", "robot_moving", "mover", "alone")


def episodes(done: np.ndarray):
    """(start, end) step index pairs of the finished episodes of one env (end inclusive)."""
    ends = np.flatnonzero(done)
    starts = np.concatenate(([0], ends[:-1] + 1))
    return list(zip(starts, ends, strict=True))


def classify(z, episode_s=30.0, window_s=10.0, near_m=0.35, still_m=0.05):
    dt = float(z["dt"])
    full = int(round(episode_s / dt))
    w = int(round(window_s / dt))
    out = []
    for e in range(z["xy"].shape[1]):
        for s, t in episodes(z["done"][:, e]):
            if t - s + 1 < full:  # ended before the time limit: not a timeout
                continue
            # the done step t already shows the next episode's reset pose: leave it out
            a = max(s, t - w)
            us = z["xy"][a:t, e]
            kind = "alone"
            moved_us = float(np.linalg.norm(us[-1] - us[0]))
            if "robot_xy" in z.files:
                for r in range(z["robot_xy"].shape[2]):
                    if not z["robot_on"][s, e, r]:
                        continue
                    rx = z["robot_xy"][a:t, e, r]
                    if np.linalg.norm(rx - us, axis=-1).min() >= near_m:
                        continue
                    moved = np.linalg.norm(np.diff(rx, axis=0), axis=-1).sum()
                    k = "robot_still" if moved < still_m else "robot_moving"
                    kind = k if kind != "robot_still" else kind
            if kind == "alone" and "mover_xy" in z.files:
                for m in range(z["mover_xy"].shape[2]):
                    on = z["mover_on"][s, e, m]
                    mx = z["mover_xy"][a:t, e, m]
                    if on and np.linalg.norm(mx - us, axis=-1).min() < near_m:
                        kind = "mover"
            out.append(
                {
                    "env": e,
                    "start": int(s),
                    "kind": kind,
                    "end_xy": [round(float(v), 3) for v in us[-1]],
                    "moved_last_window_m": round(moved_us, 3),
                }
            )
    return out


if __name__ == "__main__":
    z = np.load(sys.argv[1])
    rows = classify(z, *(float(x) for x in sys.argv[2:3]))
    counts = {k: sum(r["kind"] == k for r in rows) for k in KINDS}
    n_ep = sum(len(episodes(z["done"][:, e])) for e in range(z["xy"].shape[1]))
    print(json.dumps({"tracked_episodes": n_ep, "timeouts": len(rows), "by_kind": counts}))
    for r in rows:
        print(json.dumps(r))
