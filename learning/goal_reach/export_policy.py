"""Package a trained goal-reach policy for the Nav2 controller plugin FollowPathRL.

Writes into --out:
  policy.onnx       the rsl_rl export (observation normalizer + MLP, no action clamp) as one file
                    (the export keeps its weights in policy.onnx.data next to it)
  policy_spec.json  every number the plugin needs, taken from task_spec: observation layout and
                    scales, lidar ray layout, action limits and rate limits, control period
and with --fixtures <dir> the parity fixtures read by pode_rl_controller's tests:
  obs_cases.json        scan -> 360 training rays -> observation (scan_to_rays + build_observation)
  action_cases.json     action sequences -> rate-limited (v, w) (action_to_target + rate_limit)
  inference_cases.json  observations -> actions from the checkpoint in PyTorch

The script also runs the exported ONNX in onnxruntime (python) on the inference cases and stops if
it differs from PyTorch by more than INFER_ATOL.

Needs torch, onnx and onnxruntime (Windows system python has all three; the Isaac venv has no
onnxruntime). Run from the repo root, e.g.
  python learning/goal_reach/export_policy.py \\
    --onnx D:/isaac/runs/goal_reach/base_4096/play_final/policy.onnx \\
    --checkpoint D:/isaac/runs/goal_reach/base_4096/model_299.pt \\
    --out pode/pode_rl_controller/policies/goal_reach_base_4096 \\
    --fixtures pode/pode_rl_controller/test/fixtures

JSON has no inf / nan: the fixtures write them as the YAML strings ".inf", "-.inf", ".nan", which
yaml-cpp (the plugin's reader; JSON is YAML) turns back into floats.
"""

import argparse
import json
import math
import os
import sys

import numpy as np
import onnx
import onnxruntime as ort
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import task_spec as ts  # noqa: E402

SPEC_FORMAT = 1
# float32 MLP 41-128-128-2: summation order differs between PyTorch and onnxruntime, so the two
# agree to float32 rounding (eps 1.2e-7) times the size of the sums, not exactly. Pass when
# |onnx - torch| <= INFER_ATOL + INFER_RTOL * |torch| (numpy.allclose). Inside the training range
# the actions are O(1) and the bound is about 2e-5, 3e-6 m/s of v after the action mapping. Off the
# training range the normalizer blows inputs up (empty-floor lidar sectors have std ~0) and actions
# reach ~100 before the clamp; the relative term (about 80 float32 steps) covers those.
INFER_ATOL = 1e-5
INFER_RTOL = 1e-5
NORMALIZER_EPS = 1e-2  # rsl_rl EmpiricalNormalization default: (x - mean) / (std + eps)


def obs_layout() -> list[str]:
    """Names of the observation entries in order, as task_spec.build_observation builds them."""
    return [f"lidar_sector_{i}" for i in range(ts.OBS_LIDAR)] + [
        "goal_dist",
        "goal_sin",
        "goal_cos",
        "v",
        "w",
    ]


def make_spec(source: dict) -> dict:
    s, lim = ts.SPEC, ts.ACTION_LIMITS
    return {
        "format": SPEC_FORMAT,
        "source": source,
        "control_dt": s.control_dt,
        "obs_dim": ts.OBS_DIM,
        "act_dim": ts.ACT_DIM,
        "obs_layout": obs_layout(),
        "lidar": {
            "rays": s.lidar_rays,
            "first_ray_deg": -180.0,
            "ray_step_deg": 360.0 / s.lidar_rays,
            "sectors": s.lidar_sectors,
            "min_range": s.lidar_min_range,
            "obs_max": s.lidar_obs_max,
        },
        "goal": {"obs_max": s.goal_obs_max, "success_radius": s.success_radius},
        "action": {
            "v_min": lim.v_min,
            "v_max": lim.v_max,
            "w_max": lim.w_max,
            "acc_v": lim.acc_v,
            "acc_w": lim.acc_w,
        },
    }


def scan_to_rays(ranges, angle_min, angle_inc, range_min, range_max, lidar_yaw, spec=ts.SPEC):
    """LaserScan -> the 360 training rays (ray k points at -180 + k deg in the base frame).

    The plugin's definition, written here as the reference for its C++ copy: ray k takes the scan
    beam nearest in angle (beam i points at lidar_yaw + angle_min + i * angle_inc in the base
    frame). A ray past the last beam wraps to beam 0 when within half a step of it, else it has no
    data. No data, non-finite, below range_min or above range_max all mean "no return" and become
    lidar_obs_max, as the Newton lidar's inf does in training. Ranges are kept as measured from
    the lidar centre, as in training (both lidars sit at the same place on the robot)."""
    n = len(ranges)
    out = np.empty(spec.lidar_rays)
    two_pi = 2.0 * math.pi
    for k in range(spec.lidar_rays):
        theta = math.radians(-180.0 + k * 360.0 / spec.lidar_rays)
        rel = (theta - lidar_yaw - angle_min) % two_pi
        i = math.floor(rel / angle_inc + 0.5)
        if i >= n:
            i = 0 if two_pi - rel <= 0.5 * angle_inc else -1
        r = ranges[i] if i >= 0 else math.inf
        if not math.isfinite(r) or r < range_min or r > range_max:
            r = spec.lidar_obs_max
        out[k] = min(r, spec.lidar_obs_max)
    return out


def enc(x: float):
    if math.isnan(x):
        return ".nan"
    if math.isinf(x):
        return ".inf" if x > 0 else "-.inf"
    return x


def obs_cases(rng) -> list[dict]:
    cases = []
    layouts = [
        # Gazebo pode_sim: 640 beams over [-pi, pi] (first and last beam coincide), rplidar_link
        # turned by pi
        (640, -math.pi, 2 * math.pi / 639, 0.05, 12.0, math.pi),
        # the training layout itself: the mapping must be the identity
        (360, -math.pi, 2 * math.pi / 360, 0.05, 12.0, 0.0),
        # a partial, offset scan: some rays have no beam
        (500, -2.5, 4.8 / 499, 0.1, 8.0, 0.3),
    ]
    for n, amin, ainc, rmin, rmax, yaw in layouts:
        r = rng.uniform(0.0, 3.0, n)
        special = rng.choice(n, size=n // 8, replace=False)
        fill = [math.inf, -math.inf, math.nan, 0.0, 0.03, 13.0]
        for j, idx in enumerate(special):
            r[idx] = fill[j % len(fill)]
        rays = scan_to_rays(r, amin, ainc, rmin, rmax, yaw)
        gd, ga = float(rng.uniform(0.0, 4.0)), float(rng.uniform(-math.pi, math.pi))
        v, w = float(rng.uniform(-0.05, 0.25)), float(rng.uniform(-1.0, 1.0))
        obs = ts.build_observation(
            torch.tensor(rays, dtype=torch.float32)[None],
            torch.tensor([gd]),
            torch.tensor([ga]),
            torch.tensor([[v, w]]),
        )[0]
        cases.append(
            {
                "angle_min": amin,
                "angle_increment": ainc,
                "range_min": rmin,
                "range_max": rmax,
                "lidar_yaw": yaw,
                "ranges": [enc(float(x)) for x in r],
                "goal_dist": gd,
                "goal_angle": ga,
                "v": v,
                "w": w,
                "rays": rays.tolist(),
                "obs": obs.tolist(),
            }
        )
    return cases


def action_cases(rng) -> list[dict]:
    cases = []
    for start in ((0.0, 0.0), (0.2, -0.8)):
        acts = rng.uniform(-1.5, 1.5, (40, 2)).astype(np.float32)
        acts[:5] = [[1.0, 1.0], [-1.0, -1.0], [3.0, -3.0], [0.0, 0.0], [0.5, -0.25]]
        cmd = torch.tensor([start], dtype=torch.float32)
        cmds = []
        for a in acts:
            cmd = ts.rate_limit(cmd, ts.action_to_target(torch.tensor(a)[None]), ts.SPEC.control_dt)
            cmds.append(cmd[0].tolist())
        cases.append({"start": list(start), "actions": acts.tolist(), "cmds": cmds})
    return cases


def torch_policy(checkpoint: str):
    sd = torch.load(checkpoint, map_location="cpu", weights_only=False)["actor_state_dict"]
    mean, std = sd["obs_normalizer._mean"], sd["obs_normalizer._std"]
    layers = [(sd[f"mlp.{i}.weight"], sd[f"mlp.{i}.bias"]) for i in (0, 2, 4)]

    def run(obs: torch.Tensor) -> torch.Tensor:
        x = (obs - mean) / (std + NORMALIZER_EPS)
        for j, (w, b) in enumerate(layers):
            x = x @ w.T + b
            if j < len(layers) - 1:
                x = torch.nn.functional.elu(x)
        return x

    return run


def inference_obs(rng, n: int) -> np.ndarray:
    """Observations in the ranges the policy sees: sectors in (0, 1], goal dist in [0, 1], a unit
    bearing vector, v in [-0.2, 1], w in [-1, 1]; the last few are out-of-range extremes."""
    obs = np.empty((n, ts.OBS_DIM), dtype=np.float32)
    obs[:, : ts.OBS_LIDAR] = rng.uniform(0.025, 1.0, (n, ts.OBS_LIDAR))
    obs[: n // 2, : ts.OBS_LIDAR] = 1.0  # empty floor: most sectors see nothing
    obs[:, ts.OBS_LIDAR] = rng.uniform(0.0, 1.0, n)
    ang = rng.uniform(-math.pi, math.pi, n)
    obs[:, ts.OBS_LIDAR + 1] = np.sin(ang)
    obs[:, ts.OBS_LIDAR + 2] = np.cos(ang)
    obs[:, ts.OBS_LIDAR + 3] = rng.uniform(-0.2, 1.0, n)
    obs[:, ts.OBS_LIDAR + 4] = rng.uniform(-1.0, 1.0, n)
    obs[-4:] = rng.uniform(-3.0, 3.0, (4, ts.OBS_DIM))
    return obs


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--onnx", required=True, help="policy.onnx written by play.py")
    p.add_argument("--checkpoint", required=True, help="the model_*.pt it was exported from")
    p.add_argument("--out", required=True)
    p.add_argument("--fixtures", default="")
    p.add_argument("--seed", type=int, default=7)
    args = p.parse_args()
    rng = np.random.default_rng(args.seed)
    os.makedirs(args.out, exist_ok=True)

    model = onnx.load(args.onnx)  # also reads policy.onnx.data
    onnx.checker.check_model(model)
    out_onnx = os.path.join(args.out, "policy.onnx")
    onnx.save(model, out_onnx, save_as_external_data=False)
    source = {
        "checkpoint": os.path.basename(os.path.dirname(args.checkpoint))
        + "/"
        + os.path.basename(args.checkpoint),
        "onnx_opset": [o.version for o in model.opset_import],
        "onnx_inputs": [i.name for i in model.graph.input],
        "onnx_outputs": [o.name for o in model.graph.output],
        "action_clamp_in_graph": False,
    }
    with open(os.path.join(args.out, "policy_spec.json"), "w", encoding="utf-8") as f:
        json.dump(make_spec(source), f, indent=1)
        f.write("\n")

    obs = inference_obs(rng, 32)
    ref = torch_policy(args.checkpoint)(torch.tensor(obs)).numpy()
    sess = ort.InferenceSession(out_onnx, providers=["CPUExecutionProvider"])
    ort_out = np.concatenate([sess.run(None, {"obs": o[None]})[0] for o in obs])
    diff = np.abs(ort_out - ref)
    excess = float((diff - INFER_ATOL - INFER_RTOL * np.abs(ref)).max())
    print(
        f"[EXPORT] onnxruntime {ort.__version__} vs torch {torch.__version__}: "
        f"max |diff| {diff.max():.3g}, max |action| {np.abs(ref).max():.3g}, "
        f"worst margin to the bound {-excess:.3g}"
    )
    if excess > 0:
        raise SystemExit(f"onnx and checkpoint differ beyond atol {INFER_ATOL}, rtol {INFER_RTOL}")

    if args.fixtures:
        os.makedirs(args.fixtures, exist_ok=True)
        files = {
            "obs_cases.json": obs_cases(rng),
            "action_cases.json": action_cases(rng),
            "inference_cases.json": {
                "atol": INFER_ATOL,
                "rtol": INFER_RTOL,
                "obs": obs.tolist(),
                "actions_torch": ref.tolist(),
                "actions_onnxruntime_python": ort_out.tolist(),
            },
        }
        for name, data in files.items():
            with open(os.path.join(args.fixtures, name), "w", encoding="utf-8") as f:
                json.dump(data, f)
                f.write("\n")
    print(f"[EXPORT] wrote {args.out}" + (f" and {args.fixtures}" if args.fixtures else ""))


if __name__ == "__main__":
    main()
