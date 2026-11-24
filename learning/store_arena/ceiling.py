"""피할 수 없는 실패: 정책 실행과 같은 에피소드의 서 있는 로봇 실행을 짝짓는다.

두 실행 모두 같은 시드·설정의 play.py --episodes 1 (첫 에피소드는 정책이 움직이기 전에 정해짐).
설정마다:
  still collision rate    움직이지 않아도 부딪힌 에피소드 비율
  share of failures       정책 실패 가운데 서 있어도 부딪히는 에피소드 비율
  success ceiling         1 - still collision rate (기다리기 전략의 상한, 엄격한 한계는 아님)
Isaac 불필요:  python learning/store_arena/ceiling.py <policy_run_dir> <still_run_dir>
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
