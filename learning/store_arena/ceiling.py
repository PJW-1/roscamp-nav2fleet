"""Unavoidable failures: pair a policy run with a still-robot run of the same episodes.

Both runs are play.py with the same seed and set flags and --episodes 1 (every env counts its
first episode, which is drawn at the first reset before any policy acts; steps draw nothing
random, so the still run sees the same layout, movers and robots). For each set:
  still collision rate    episodes where our robot is hit even though it never moves
  share of failures       policy failures (collision, out of bounds, timeout) whose episode also
                          collides when standing still
  success ceiling         1 - still collision rate (a still-hit episode cannot be won by waiting;
                          the policy could still dodge it, so this is a ceiling for a waiting
                          strategy, not a hard bound)
No Isaac needed:  python learning/store_arena/ceiling.py <policy_run_dir> <still_run_dir>
"""

import json
import sys

import numpy as np


def load(run_dir: str):
    z = np.load(run_dir + "/episodes.npz")
    order = np.argsort(z["env"], kind="stable")
    return z["env"][order], z["code"][order], [str(k) for k in z["keys"]]


def ceiling(policy_env, policy_code, still_env, still_code, keys) -> dict:
    if not np.array_equal(policy_env, still_env):
        raise ValueError("the two runs do not count the same envs: use --episodes 1 for both")
    coll = keys.index("collision")
    success = keys.index("success")
    still_hit = still_code == coll
    fail = policy_code != success
    n = len(policy_code)
    return {
        "episodes": n,
        "policy_success": round(float((~fail).mean()), 4),
        "still_collision_rate": round(float(still_hit.mean()), 4),
        "policy_failures": int(fail.sum()),
        "failures_also_hit_still": int((fail & still_hit).sum()),
        "share_of_failures_hit_still": round(
            float((fail & still_hit).sum() / max(fail.sum(), 1)), 4
        ),
        "policy_success_on_still_hit": round(
            float((~fail & still_hit).sum() / max(still_hit.sum(), 1)), 4
        ),
        "success_ceiling": round(float(1.0 - still_hit.mean()), 4),
    }


if __name__ == "__main__":
    pe, pc, keys = load(sys.argv[1])
    se, sc, _ = load(sys.argv[2])
    print(json.dumps(ceiling(pe, pc, se, sc, keys), indent=1))
