"""Per-dimension mean and std of the actor's observation normalizer in an rsl_rl checkpoint.

A dimension whose std is near zero was constant in training; the exported policy then divides
real-robot variation of that input by almost nothing (the empty-floor policy's lidar sectors).
No Isaac needed:  python learning/store_arena/normalizer_std.py <model_*.pt> [out.csv]
"""

import csv
import dataclasses
import os
import sys

import torch

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "goal_reach"))
import task_spec as ts  # noqa: E402


def obs_names(dim: int) -> list[str]:
    """Names of a dim-long observation: the 41-dim layout plus whole intent robots (task_spec)."""
    extra = dim - ts.OBS_DIM
    for prio in (0, 1):  # the 2-robot layouts differ in size, so the first fit is the layout
        per = 1 + prio + 2 * ts.SPEC.intent_points
        if extra >= 0 and extra % per == 0:
            spec = dataclasses.replace(ts.SPEC, intent_robots=extra // per, intent_priority=prio)
            return ts.obs_names(spec)
    return [f"obs_{i}" for i in range(dim)]


def normalizer_stats(path: str) -> tuple[torch.Tensor, torch.Tensor]:
    ck = torch.load(path, map_location="cpu", weights_only=False)
    for key, sd in ck.items():
        if not isinstance(sd, dict) or "critic" in key:
            continue
        means = [k for k in sd if k.endswith("_mean") and "norm" in k]
        if means:
            m = sd[means[0]].flatten()
            var_key = means[0][: -len("_mean")] + "_var"
            std_key = means[0][: -len("_mean")] + "_std"
            std = sd[std_key] if std_key in sd else sd[var_key].sqrt()
            return m, std.flatten()
    raise KeyError(f"no normalizer in {path}: {list(ck)}")


if __name__ == "__main__":
    mean, std = normalizer_stats(sys.argv[1])
    names = obs_names(len(mean))
    rows = [
        (n, round(float(m), 5), round(float(s), 5))
        for n, m, s in zip(names, mean, std, strict=True)
    ]
    for r in sorted(rows, key=lambda r: r[2])[:6]:
        print("smallest std:", r)
    if len(sys.argv) > 2:
        with open(sys.argv[2], "w", newline="", encoding="utf-8") as fh:
            wr = csv.writer(fh)
            wr.writerow(("obs", "mean", "std"))
            wr.writerows(rows)
