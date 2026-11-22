"""Package a trained goal-reach or store-arena policy for the Nav2 controller plugin FollowPathRL.

Writes into --out:
  policy.onnx       observation normalizer + MLP, no action clamp, as one file: the rsl_rl export
                    given with --onnx (it keeps its weights in policy.onnx.data next to it), or,
                    without --onnx, the same graph exported here from the checkpoint
  policy_spec.json  every number the plugin needs, taken from task_spec: observation layout and
                    scales, lidar ray layout, action limits and rate limits, control period; the
                    goal rule the policy was trained with (--goal-rule) and where it came from
and with --fixtures <dir> the parity fixtures read by pode_rl_controller's tests:
  obs_cases.json        scan -> 360 training rays -> observation (scan_to_rays + build_observation)
  action_cases.json     action sequences -> rate-limited (v, w) (action_to_target + rate_limit)
  inference_cases.json  observations -> actions from the checkpoint in PyTorch
  follow_cases.json     (goal rule lookahead) robot poses along paths -> progress index, lookahead
                        point, goal distance and bearing, goal and velocity observation entries,
                        from the training code: route_spec.follow (--route-spec) and task_spec

The script also runs the exported ONNX in onnxruntime (python) on the inference cases and stops if
it differs from PyTorch by more than INFER_ATOL.

Needs torch, onnx and onnxruntime (Windows system python has all three; the Isaac venv has no
onnxruntime). Run from the repo root, e.g.
  python learning/goal_reach/export_policy.py \\
    --onnx D:/isaac/runs/goal_reach/base_4096/play_final/policy.onnx \\
    --checkpoint D:/isaac/runs/goal_reach/base_4096/model_299.pt \\
    --out pode/pode_rl_controller/policies/goal_reach_base_4096 \\
    --fixtures pode/pode_rl_controller/test/fixtures/goal_reach_base_4096
The store-arena policy (route_spec.py taken from the commit it was trained at, e.g.
git show <commit>:learning/store_arena/route_spec.py > <tmp>/route_spec.py):
  python learning/goal_reach/export_policy.py \\
    --checkpoint D:/isaac/runs/store_arena/ppo_g/c1/model_499.pt \\
    --goal-rule lookahead --lookahead-m 0.40 --follow-window 20 --route-spec <tmp>/route_spec.py \\
    --source-branch feat/store-arena-2026-11-21 --source-commit <commit> \\
    --out pode/pode_rl_controller/policies/store_arena_4b \\
    --fixtures pode/pode_rl_controller/test/fixtures/store_arena_4b

JSON has no inf / nan: the fixtures write them as the YAML strings ".inf", "-.inf", ".nan", which
yaml-cpp (the plugin's reader; JSON is YAML) turns back into floats.
"""

import argparse
import importlib.util
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

SPEC_FORMAT = 2  # 2: goal rule (path_end | lookahead) and the policy's origin
# 3: format 2 plus "obs_blocks" (the optional blocks after the 41 base entries, task_spec
# obs_blocks) and one entry per block; written only for a policy that has such blocks (--set)
SPEC_FORMAT_BLOCKS = 3
# float32 MLP 41-128-128-2: summation order differs between PyTorch and onnxruntime, so the two
# agree to float32 rounding (eps 1.2e-7) times the size of the sums, not exactly. Pass when
# |onnx - torch| <= INFER_ATOL + INFER_RTOL * |torch| (numpy.allclose). Inside the training range
# the actions are O(1) and the bound is about 2e-5, 3e-6 m/s of v after the action mapping. Off the
# training range the normalizer blows inputs up (empty-floor lidar sectors have std ~0) and actions
# reach ~100 before the clamp; the relative term (about 80 float32 steps) covers those.
INFER_ATOL = 1e-5
INFER_RTOL = 1e-5
NORMALIZER_EPS = 1e-2  # rsl_rl EmpiricalNormalization default: (x - mean) / (std + eps)


def obs_layout(spec=ts.SPEC) -> list[str]:
    """Names of the observation entries in order, as task_spec.build_observation builds them."""
    return ts.obs_names(spec)


def block_entries(spec) -> dict:
    """policy_spec format 3 entries of the optional blocks present in spec."""
    lim = ts.ACTION_LIMITS
    out = {"obs_blocks": ts.obs_blocks(spec)}
    if "intent" in out["obs_blocks"]:
        raise ValueError("the intent block has no policy_spec entry yet")
    if spec.prev_action_obs:
        out["prev_action"] = {"v_scale": lim.v_max, "w_scale": lim.w_max}
    if spec.lidar_frames > 1:
        out["lidar_history"] = {
            "frames": spec.lidar_frames,
            "stride_steps": spec.lidar_stride_steps,
        }
    if spec.speed_cap_obs:
        lo, hi = spec.speed_cap_range
        out["speed_cap"] = {"min": lo, "max": hi, "obs_scale": lim.v_max}
    return out


def make_spec(source: dict, goal_rule: dict, spec=ts.SPEC) -> dict:
    s, lim = spec, ts.ACTION_LIMITS
    blocks = ts.obs_blocks(spec)
    if not spec.speed_cap_obs and tuple(spec.speed_cap_range) != (lim.v_max, lim.v_max):
        # the env maps v onto this cap, but the spec would tell the plugin v_max
        raise ValueError(f"speed_cap_range {spec.speed_cap_range} needs speed_cap_obs=1")
    return {
        "format": SPEC_FORMAT_BLOCKS if blocks else SPEC_FORMAT,
        "source": source,
        "control_dt": s.control_dt,
        "obs_dim": ts.obs_dim(spec),
        "act_dim": ts.ACT_DIM,
        "obs_layout": obs_layout(spec),
        **(block_entries(spec) if blocks else {}),
        "lidar": {
            "rays": s.lidar_rays,
            "first_ray_deg": -180.0,
            "ray_step_deg": 360.0 / s.lidar_rays,
            "sectors": s.lidar_sectors,
            "min_range": s.lidar_min_range,
            "obs_max": s.lidar_obs_max,
        },
        "goal": {"obs_max": s.goal_obs_max, "success_radius": s.success_radius, **goal_rule},
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


def obs_cases(rng, spec=ts.SPEC) -> list[dict]:
    """Scan -> rays -> observation; with optional blocks also their inputs (prev target in m/s
    and rad/s, earlier frames as scaled sectors newest first, speed cap in m/s)."""
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
        extra, blocks = {}, {}
        if spec.prev_action_obs:
            prev = [float(rng.uniform(-0.05, 0.16)), float(rng.uniform(-1.0, 1.0))]
            extra["prev_target"], blocks["prev_target"] = torch.tensor([prev]), prev
        if spec.lidar_frames > 1:
            h = rng.uniform(0.025, 1.0, (spec.lidar_frames - 1, spec.lidar_sectors))
            extra["lidar_history"] = torch.tensor(h[None], dtype=torch.float32)
            blocks["lidar_history"] = h.tolist()
        if spec.speed_cap_obs:
            cap = float(rng.uniform(*spec.speed_cap_range))
            extra["speed_cap"], blocks["speed_cap"] = torch.tensor([cap]), cap
        obs = ts.build_observation(
            torch.tensor(rays, dtype=torch.float32)[None],
            torch.tensor([gd]),
            torch.tensor([ga]),
            torch.tensor([[v, w]]),
            spec,
            **extra,
        )[0]
        cases.append(
            {
                **blocks,
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


def action_cases(rng, spec=ts.SPEC) -> list[dict]:
    """Action sequences -> rate-limited (v, w); with the speed cap block also at its two ends
    (the case's "speed_cap" is the v upper limit of the mapping)."""
    cases = []
    caps = [None] + (list(spec.speed_cap_range) if spec.speed_cap_obs else [])
    for cap in caps:
        for start in ((0.0, 0.0), (0.2, -0.8)):
            acts = rng.uniform(-1.5, 1.5, (40, 2)).astype(np.float32)
            acts[:5] = [[1.0, 1.0], [-1.0, -1.0], [3.0, -3.0], [0.0, 0.0], [0.5, -0.25]]
            cmd = torch.tensor([start], dtype=torch.float32)
            v_cap = None if cap is None else torch.tensor([cap])
            cmds = []
            for a in acts:
                target = ts.action_to_target(torch.tensor(a)[None], v_cap=v_cap)
                cmd = ts.rate_limit(cmd, target, ts.SPEC.control_dt)
                cmds.append(cmd[0].tolist())
            case = {"start": list(start), "actions": acts.tolist(), "cmds": cmds}
            if cap is not None:
                case["speed_cap"] = cap
            cases.append(case)
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


def load_route_spec(path: str):
    spec = importlib.util.spec_from_file_location("route_spec", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["route_spec"] = mod  # its dataclasses look their module up
    spec.loader.exec_module(mod)
    return mod


def follow_paths(rng) -> dict:
    """Paths in their own frame, before a random placement: points every `step` m with a little
    jitter (a planner's path is never evenly spaced, and exact spacing would put the lookahead
    target on a float tie)."""

    def line(n, step):
        return np.stack((np.arange(n) * step, np.zeros(n)), -1)

    turn = np.stack((np.full(24, 1.0), np.arange(1, 25) * 0.025), -1)
    paths = {
        "straight": line(81, 0.025),
        "shorter_than_lookahead": line(9, 0.025),
        "corner": np.concatenate((line(41, 0.025), turn)),
        "coarse_5cm": line(41, 0.05),
    }
    return {k: p + rng.uniform(-0.002, 0.002, p.shape) for k, p in paths.items()}


def follow_cases(rng, route_spec, lookahead: float, window: int) -> list[dict]:
    """Robot poses walked along each path (with sideways error, one jump past the follow window,
    then standing at the end), fed through route_spec.follow step by step as the env does."""
    cases = []
    for name, p0 in follow_paths(rng).items():
        th = float(rng.uniform(-math.pi, math.pi))
        rot = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        path = p0 @ rot.T + rng.uniform(-2.0, 2.0, 2)
        seg = np.diff(path, axis=0)
        cum = np.concatenate(([0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))))
        n = len(path)
        along = list(range(0, n, 3)) + [n - 1] * 3
        if n > window + 10:
            along.insert(4, along[3] + window + 6)  # the progress index moves at most window
        prog = torch.zeros(1, dtype=torch.long)
        steps = []
        for i in along:
            xy = path[min(i, n - 1)] + rng.uniform(-0.03, 0.03, 2)
            yaw = float(rng.uniform(-math.pi, math.pi))
            v, w = float(rng.uniform(-0.05, 0.25)), float(rng.uniform(-1.0, 1.0))
            xy_t = torch.tensor(xy[None], dtype=torch.float64)
            prog, _, la = route_spec.follow(
                torch.tensor(path[None]),
                torch.tensor(cum[None]),
                torch.tensor([n]),
                prog,
                xy_t,
                lookahead,
                window,
            )
            dist, ang = ts.goal_in_robot_frame(xy_t, torch.tensor([yaw], dtype=torch.float64), la)
            obs = ts.build_observation(
                torch.full((1, ts.SPEC.lidar_rays), ts.SPEC.lidar_obs_max, dtype=torch.float64),
                dist,
                ang,
                torch.tensor([[v, w]], dtype=torch.float64),
            )[0]
            steps.append(
                {
                    "xy": xy.tolist(),
                    "yaw": yaw,
                    "v": v,
                    "w": w,
                    "prog": int(prog[0]),
                    "lookahead_xy": la[0].tolist(),
                    "goal_dist": float(dist[0]),
                    "goal_angle": float(ang[0]),
                    "obs_goal_vel": obs[ts.OBS_LIDAR :].tolist(),
                }
            )
        cases.append({"name": name, "path": path.tolist(), "steps": steps})
    return cases


class CheckpointPolicy(torch.nn.Module):
    """The rsl_rl actor graph: (obs - mean) / (std + eps), then the ELU MLP, no action clamp."""

    def __init__(self, checkpoint: str):
        super().__init__()
        sd = torch.load(checkpoint, map_location="cpu", weights_only=False)["actor_state_dict"]
        self.register_buffer("mean", sd["obs_normalizer._mean"])
        self.register_buffer("std", sd["obs_normalizer._std"])
        layers = []
        for i in (0, 2, 4):
            out_dim, in_dim = sd[f"mlp.{i}.weight"].shape
            layers += [torch.nn.Linear(in_dim, out_dim), torch.nn.ELU()]
        self.mlp = torch.nn.Sequential(*layers[:-1])
        self.mlp.load_state_dict({k[4:]: v for k, v in sd.items() if k.startswith("mlp.")})

    def forward(self, obs):
        return self.mlp((obs - self.mean) / (self.std + NORMALIZER_EPS))


def export_from_checkpoint(checkpoint: str, path: str, spec=ts.SPEC) -> None:
    torch.onnx.export(
        CheckpointPolicy(checkpoint).eval(),
        torch.zeros(1, ts.obs_dim(spec)),
        path,
        input_names=["obs"],
        output_names=["actions"],
        opset_version=18,
        dynamo=False,
    )


def inference_obs(rng, n: int, spec=ts.SPEC) -> np.ndarray:
    """Observations in the ranges the policy sees: sectors in (0, 1], goal dist in [0, 1], a unit
    bearing vector, v in [-0.2, 1], w in [-1, 1], the optional blocks in theirs; the last few
    are out-of-range extremes."""
    dim = ts.obs_dim(spec)
    obs = np.empty((n, dim), dtype=np.float32)
    obs[:, : ts.OBS_LIDAR] = rng.uniform(0.025, 1.0, (n, ts.OBS_LIDAR))
    obs[: n // 2, : ts.OBS_LIDAR] = 1.0  # empty floor: most sectors see nothing
    obs[:, ts.OBS_LIDAR] = rng.uniform(0.0, 1.0, n)
    ang = rng.uniform(-math.pi, math.pi, n)
    obs[:, ts.OBS_LIDAR + 1] = np.sin(ang)
    obs[:, ts.OBS_LIDAR + 2] = np.cos(ang)
    obs[:, ts.OBS_LIDAR + 3] = rng.uniform(-0.2, 1.0, n)
    obs[:, ts.OBS_LIDAR + 4] = rng.uniform(-1.0, 1.0, n)
    i = ts.BASE_DIM
    if spec.prev_action_obs:
        obs[:, i] = rng.uniform(-0.4, 1.4, n)
        obs[:, i + 1] = rng.uniform(-1.0, 1.0, n)
        i += 2
    for _ in range(1, spec.lidar_frames):  # earlier frames near the current one
        jitter = rng.uniform(-0.05, 0.05, (n, ts.OBS_LIDAR))
        obs[:, i : i + ts.OBS_LIDAR] = np.clip(obs[:, : ts.OBS_LIDAR] + jitter, 0.025, 1.0)
        i += ts.OBS_LIDAR
    if spec.speed_cap_obs:
        lo, hi = (c / ts.ACTION_LIMITS.v_max for c in spec.speed_cap_range)
        obs[:, i] = rng.uniform(lo, hi, n)
        i += 1
    assert i == dim
    obs[-4:] = rng.uniform(-3.0, 3.0, (4, dim))
    return obs


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--onnx", default="", help="policy.onnx written by play.py, else made here")
    p.add_argument("--checkpoint", required=True, help="the model_*.pt it was exported from")
    p.add_argument("--out", required=True)
    p.add_argument("--fixtures", default="")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--goal-rule", choices=("path_end", "lookahead"), default="path_end")
    p.add_argument("--lookahead-m", type=float, default=0.0, help="ArenaSpec.lookahead_m")
    p.add_argument("--follow-window", type=int, default=0, help="ArenaSpec.follow_window")
    p.add_argument("--route-spec", default="", help="route_spec.py the policy was trained with")
    p.add_argument("--source-branch", default="")
    p.add_argument("--source-commit", default="")
    p.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="FIELD=VALUE",
        help="the TaskSpec observation fields the policy was trained with (train.py --set): "
        "prev_action_obs, lidar_frames, lidar_stride_steps, speed_cap_obs, speed_cap_range",
    )
    args = p.parse_args()
    spec = ts.make_spec(ts.parse_overrides(args.set))
    if args.goal_rule == "lookahead":
        if args.lookahead_m <= 0 or args.follow_window <= 0 or not args.route_spec:
            p.error("--goal-rule lookahead needs --lookahead-m, --follow-window and --route-spec")
        goal_rule = {
            "rule": "lookahead",
            "lookahead_m": args.lookahead_m,
            "follow_window": args.follow_window,
        }
    else:
        goal_rule = {"rule": "path_end"}
    rng = np.random.default_rng(args.seed)
    os.makedirs(args.out, exist_ok=True)

    out_onnx = os.path.join(args.out, "policy.onnx")
    if not args.onnx:
        export_from_checkpoint(args.checkpoint, out_onnx, spec)
    model = onnx.load(args.onnx or out_onnx)  # also reads policy.onnx.data
    onnx.checker.check_model(model)
    onnx.save(model, out_onnx, save_as_external_data=False)
    source = {
        "checkpoint": os.path.basename(os.path.dirname(args.checkpoint))
        + "/"
        + os.path.basename(args.checkpoint),
        "checkpoint_path": args.checkpoint.replace("\\", "/"),
        "onnx_from": "rsl_rl export" if args.onnx else "export_policy.py torch.onnx.export",
        "branch": args.source_branch,
        "commit": args.source_commit,
        "onnx_opset": [o.version for o in model.opset_import],
        "onnx_inputs": [i.name for i in model.graph.input],
        "onnx_outputs": [o.name for o in model.graph.output],
        "action_clamp_in_graph": False,
    }
    with open(os.path.join(args.out, "policy_spec.json"), "w", encoding="utf-8") as f:
        json.dump(make_spec(source, goal_rule, spec), f, indent=1)
        f.write("\n")

    width = torch.load(args.checkpoint, map_location="cpu", weights_only=False)[
        "actor_state_dict"
    ]["mlp.0.weight"].shape[1]
    if width != ts.obs_dim(spec):
        raise SystemExit(f"checkpoint takes {width} inputs, --set gives {ts.obs_dim(spec)}")
    obs = inference_obs(rng, 32, spec)
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
            "obs_cases.json": obs_cases(rng, spec),
            "action_cases.json": action_cases(rng, spec),
            "inference_cases.json": {
                "atol": INFER_ATOL,
                "rtol": INFER_RTOL,
                "obs": obs.tolist(),
                "actions_torch": ref.tolist(),
                "actions_onnxruntime_python": ort_out.tolist(),
            },
        }
        if args.goal_rule == "lookahead":
            files["follow_cases.json"] = {
                "lookahead_m": args.lookahead_m,
                "follow_window": args.follow_window,
                "cases": follow_cases(
                    rng, load_route_spec(args.route_spec), args.lookahead_m, args.follow_window
                ),
            }
        for name, data in files.items():
            with open(os.path.join(args.fixtures, name), "w", encoding="utf-8") as f:
                json.dump(data, f)
                f.write("\n")
    print(f"[EXPORT] wrote {args.out}" + (f" and {args.fixtures}" if args.fixtures else ""))


if __name__ == "__main__":
    main()
