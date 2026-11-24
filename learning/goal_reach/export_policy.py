"""학습한 정책을 Nav2 컨트롤러 플러그인 FollowPathRL 용으로 묶는다. --out 에 쓰는 것:
  policy.onnx       관측 정규화 + MLP (행동 클램프 없음). --onnx 로 받은 파일 또는 체크포인트에서 내보낸 것
  policy_spec.json  플러그인이 쓰는 값 전부 (관측 배치와 스케일, 라이다, 행동/변화율 한도, 목표 규칙)
--fixtures <dir> 이면 pode_rl_controller 테스트용 일치 검증 파일도 쓴다:
  obs_cases / action_cases / inference_cases / follow_cases .json

내보낸 ONNX 를 onnxruntime 으로 돌려 PyTorch 와 INFER_ATOL 이상 다르면 멈춘다.
torch, onnx, onnxruntime 이 필요하다. JSON 에 inf/nan 이 없어 픽스처는 YAML 문자열(".inf", ".nan")로 쓴다.
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

SPEC_FORMAT = 2  # 2: 목표 규칙(path_end | lookahead)과 정책 출처 포함
# float32 MLP 41-128-128-2: PyTorch 와 onnxruntime 은 합산 순서가 달라 float32 반올림 수준으로만 같다.
# |onnx - torch| <= INFER_ATOL + INFER_RTOL * |torch| 이면 통과 (학습 범위 밖에서는 행동이 커져 상대 오차 항이 필요).
INFER_ATOL = 1e-5
INFER_RTOL = 1e-5
NORMALIZER_EPS = 1e-2  # rsl_rl 기본: (x - mean) / (std + eps)


def obs_layout() -> list[str]:
    """task_spec.build_observation 순서의 관측 항목 이름."""
    return [f"lidar_sector_{i}" for i in range(ts.OBS_LIDAR)] + [
        "goal_dist",
        "goal_sin",
        "goal_cos",
        "v",
        "w",
    ]


def make_spec(source: dict, goal_rule: dict) -> dict:
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
    """LaserScan 을 학습 레이 360 개로 바꾼다 (레이 k 는 base 프레임 -180 + k deg).

    가장 가까운 각도의 빔을 쓰고, 마지막 빔을 넘으면 반 스텝 이내일 때 빔 0 으로 이어진다.
    데이터 없음/비유한/범위 밖은 모두 lidar_obs_max 로 둔다 (플러그인 C++ 복사본의 기준)."""
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
        # Gazebo pode_sim: 빔 640 개 [-pi, pi], rplidar_link 가 pi 돌아감
        (640, -math.pi, 2 * math.pi / 639, 0.05, 12.0, math.pi),
        # 학습 배치 그대로: 매핑이 항등이어야 함
        (360, -math.pi, 2 * math.pi / 360, 0.05, 12.0, 0.0),
        # 일부만 있고 어긋난 스캔: 빔이 없는 레이 존재
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


def load_route_spec(path: str):
    spec = importlib.util.spec_from_file_location("route_spec", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["route_spec"] = mod  # 모듈 안 dataclass 가 자기 모듈을 찾음
    spec.loader.exec_module(mod)
    return mod


def follow_paths(rng) -> dict:
    """각 경로를 자기 프레임에서, 간격 step m 에 약간의 흔들림을 줘서 만든다 (등간격이면 lookahead 가 동률이 됨)."""

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
    """경로를 따라 로봇 자세를 걸려(옆 오차, follow 창 밖 점프, 끝에서 정지) route_spec.follow 에 스텝별로 넣는다."""
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
            along.insert(4, along[3] + window + 6)  # 진행 인덱스는 window 이내로만 움직임
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
    """rsl_rl actor 그래프: (obs - mean) / (std + eps), ELU MLP, 행동 클램프 없음."""

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


def export_from_checkpoint(checkpoint: str, path: str) -> None:
    torch.onnx.export(
        CheckpointPolicy(checkpoint).eval(),
        torch.zeros(1, ts.OBS_DIM),
        path,
        input_names=["obs"],
        output_names=["actions"],
        opset_version=18,
        dynamo=False,
    )


def inference_obs(rng, n: int) -> np.ndarray:
    """정책이 보는 범위의 관측: 섹터 (0, 1], 목표 거리 [0, 1], 단위 방위, v [-0.2, 1], w [-1, 1]; 끝 몇 개는 범위 밖 극단값."""
    obs = np.empty((n, ts.OBS_DIM), dtype=np.float32)
    obs[:, : ts.OBS_LIDAR] = rng.uniform(0.025, 1.0, (n, ts.OBS_LIDAR))
    obs[: n // 2, : ts.OBS_LIDAR] = 1.0  # 빈 바닥: 대부분 섹터가 아무것도 못 봄
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
    args = p.parse_args()
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
        export_from_checkpoint(args.checkpoint, out_onnx)
    model = onnx.load(args.onnx or out_onnx)  # policy.onnx.data 도 읽음
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
        json.dump(make_spec(source, goal_rule), f, indent=1)
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
