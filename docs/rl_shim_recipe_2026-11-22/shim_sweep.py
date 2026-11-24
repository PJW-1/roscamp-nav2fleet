"""Kinematic 528-start sweep of store_arena_4b (ONNX) behind a RotationShim model (Nav2 Jazzy 1.3.13 logic).

modes:
  nominal  : engage when |err| > 0.785 at a new plan (every PLAN_DT), rotate at 1.0 rad/s, accel 2.0 rad/s^2,
             stop-decel sqrt(2 a |err|), hand over when |err| <= disengage (0.3925 default). Every new plan
             re-arms the check (rotate_to_heading_once false).
  abort_T  : the same rotation, but the shim hands the robot to the policy after T s of rotation whatever the
             error (what a rotation-collision refusal does), and does not engage again (the refusal repeats
             next to the wall).
err = bearing to the first route point >= 0.15 m from the robot, after the progress point (a new plan starts at
the robot). Clearance = URDF base_link box distance to the measured shelves and walls.
"""

import collections
import json
import math
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

REPO = Path("C:/Users/pjw/ros2camp/pd-plugin2")
for p in (REPO / "learning/store_arena", REPO / "learning/goal_reach", REPO / "pc/fleet_core",
          REPO / "sim/fleet_gz_sim"):
    sys.path.insert(0, str(p))
import arena_spec as arena  # noqa: E402
import route_spec as rs  # noqa: E402
import task_spec as ts  # noqa: E402

sys.path.insert(0, str(REPO / "learning/goal_reach"))
from export_policy import CheckpointPolicy  # noqa: E402

POLICY = REPO / "pode/pode_rl_controller/policies/store_arena_4b/policy.onnx"
CKPT = "D:/isaac/runs/store_arena/ppo_k/c2/model_998.pt"
sess = ort.InferenceSession(str(POLICY), providers=["CPUExecutionProvider"])
torch_pol = CheckpointPolicy(CKPT).eval()
torch.manual_seed(0)
_probe = torch.rand(64, 41)
_onnx = np.stack([sess.run(None, {"obs": _probe[i:i + 1].numpy()})[0][0] for i in range(64)])
with torch.no_grad():
    _gap = float(np.abs(torch_pol(_probe).numpy() - _onnx).max())
print(f"checkpoint vs store_arena_4b policy.onnx on 64 random obs: max |diff| {_gap:.2e}")
NOISE = 0.0
spec = arena.ARENA
site = arena.load_site(spec)
boxes = arena.arena_boxes(spec, site)
rects_np = np.array([(b.x0, b.y0, b.x1, b.y1) for b in boxes])
rects = arena.boxes_tensor(boxes).float()
BODY = np.array([(x, y) for x in np.linspace(-0.0745, 0.0387, 12) for y in (-0.047, 0.047)]
                + [(x, y) for x in (-0.0745, 0.0387) for y in np.linspace(-0.047, 0.047, 9)])
DT, ACC = 0.05, 2.0 * 0.05
ENGAGE, DISENGAGE, ROT_W, ROT_A, SAMPLE, PLAN_DT = 0.785, 0.785 / 2.0, 1.0, 2.0, 0.15, 1.0


def policy(obs):
    with torch.no_grad():
        return ts.action_to_target(torch_pol(obs.float()))


def clearance(x, y, yaw):
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    px = x[:, None] + c * BODY[:, 0] - s * BODY[:, 1]
    py = y[:, None] + s * BODY[:, 0] + c * BODY[:, 1]
    return rs.point_rect_distance(px, py, rects_np).min(axis=1)


def wrap(a):
    return torch.atan2(torch.sin(a), torch.cos(a))


def sample_err(path, npts, prog, x, y, yaw):
    n, lmax = path.shape[:2]
    j = torch.arange(lmax)[None, :]
    d = (path - torch.stack((x, y), 1)[:, None, :]).norm(dim=-1)
    ok = (j >= prog[:, None]) & (j < npts[:, None]) & (d >= SAMPLE)
    first = torch.where(ok, j, torch.full_like(j, lmax)).min(dim=1).values
    idx = torch.where(first < lmax, first, npts - 1)
    pt = path[torch.arange(n), idx]
    return wrap(torch.atan2(pt[:, 1] - y, pt[:, 0] - x) - yaw)


def run(path, cum, npts, start_xy, yaw0, mode, abort_t=None, w0=None, steps=1250, shim_on=True):
    n = path.shape[0]
    end = path[torch.arange(n), npts - 1]
    x, y, yaw = start_xy[:, 0].clone(), start_xy[:, 1].clone(), yaw0.clone()
    v = torch.zeros(n)
    w = torch.zeros(n) if w0 is None else w0.clone()
    prog = torch.zeros(n, dtype=torch.long)
    alive = torch.ones(n, dtype=torch.bool)
    armed = torch.full((n,), shim_on)
    rotating = torch.zeros(n, dtype=torch.bool)
    rot_time = torch.zeros(n)
    hand = [None] * n
    done_t = torch.full((n,), float("nan"))
    bmin = np.full(n, 9.0)
    t_bmin = np.zeros(n)
    p_bmin = np.zeros((n, 3))
    for step in range(steps):
        t = step * DT
        xy = torch.stack((x, y), 1)
        prog, _, la = rs.follow(path, cum, npts, prog, xy, spec.lookahead_m, spec.follow_window)
        if shim_on and mode == "nominal" and step > 0 and abs(t / PLAN_DT - round(t / PLAN_DT)) < 1e-6:
            armed |= alive
        err = sample_err(path, npts, prog, x, y, yaw)
        thresh = torch.where(rotating, torch.full_like(err, DISENGAGE), torch.full_like(err, ENGAGE))
        want = armed & (err.abs() > thresh)
        if mode == "abort":
            want &= rot_time < abort_t - 1e-9
        new_hand = (armed | rotating) & ~want & alive
        for i in torch.nonzero(new_hand & rotating).flatten().tolist():
            if hand[i] is None:
                hand[i] = (t, float(x[i]), float(y[i]), float(yaw[i]), float(v[i]), float(w[i]), float(err[i]))
        armed &= want
        if mode == "abort":
            armed &= rot_time < abort_t - 1e-9
        rotating = want
        o = torch.stack((x - 0.017 * torch.cos(yaw), y - 0.017 * torch.sin(yaw)), 1)
        ranges = arena.raycast_rects(o, yaw, rects)
        if NOISE > 0:
            ranges = torch.clamp(ranges + NOISE * torch.randn_like(ranges), min=0.05)
        d, a = ts.goal_in_robot_frame(xy, yaw, la)
        obs = ts.build_observation(ranges, d, a, torch.stack((v, w), 1))
        with torch.no_grad():
            tgt = policy(obs)
        tv, tw = tgt[:, 0].clone(), tgt[:, 1].clone()
        sign = torch.where(err > 0, 1.0, -1.0)
        rw = torch.clamp(sign * ROT_W, w - ROT_A * DT, w + ROT_A * DT)
        cap = torch.sqrt(2 * ROT_A * err.abs())
        rw = torch.where(rw.abs() > cap, sign * cap, rw)
        tv = torch.where(rotating, torch.zeros_like(tv), tv)
        tw = torch.where(rotating, rw, tw)
        rot_time = rot_time + rotating.float() * DT
        v_new = torch.where(rotating, torch.zeros_like(v), v + torch.clamp(tv - v, -ACC, ACC))
        w_new = torch.where(rotating, tw, w + torch.clamp(tw - w, -ACC, ACC))
        v = torch.where(alive, v_new, torch.zeros_like(v))
        w = torch.where(alive, w_new, torch.zeros_like(w))
        yaw = yaw + w * DT
        x = x + v * torch.cos(yaw) * DT
        y = y + v * torch.sin(yaw) * DT
        c = clearance(x.numpy(), y.numpy(), yaw.numpy())
        better = (c < bmin) & alive.numpy()
        bmin = np.where(better, c, bmin)
        t_bmin = np.where(better, (step + 1) * DT, t_bmin)
        p_bmin[better] = np.stack((x.numpy(), y.numpy(), yaw.numpy()), 1)[better]
        reached = alive & (torch.hypot(x - end[:, 0], y - end[:, 1]) < ts.SPEC.success_radius)
        done_t[reached] = (step + 1) * DT
        alive &= ~reached
        alive &= torch.tensor(c > 0.0)
    return bmin, t_bmin, done_t, p_bmin, hand


def nearest_box(x_, y_, h_):
    c, sn = math.cos(h_), math.sin(h_)
    px = x_ + c * BODY[:, 0] - sn * BODY[:, 1]
    py = y_ + sn * BODY[:, 0] + c * BODY[:, 1]
    dist = [rs.point_rect_distance(px, py, rects_np[j:j + 1]).min() for j in range(len(rects_np))]
    j = int(np.argmin(dist))
    k = int(np.argmin(rs.point_rect_distance(px, py, rects_np[j:j + 1])))
    return boxes[j].name, BODY[k]


def sweep():
    routes = rs.build_routes(spec, site, rects_np)
    sel = np.arange(0, routes.count, routes.layouts)
    H = 12
    yaws = np.linspace(-math.pi, math.pi, H, endpoint=False)
    R = len(sel)
    N = R * H
    ridx = np.repeat(sel, H)
    path = torch.tensor(routes.path[ridx], dtype=torch.float32)
    cum = torch.tensor(routes.cum[ridx], dtype=torch.float32)
    npts = torch.tensor(routes.npts[ridx])
    yaw0 = torch.tensor(np.tile(yaws, R), dtype=torch.float32)
    names = routes.node_names
    out = {}
    for label, mode, abort_t, shim_on in (("RL alone", "nominal", None, False),
                                          ("shim nominal", "nominal", None, True),
                                          ("shim abort 0.25 s", "abort", 0.25, True),
                                          ("shim abort 0.5 s", "abort", 0.5, True),
                                          ("shim abort 0.75 s", "abort", 0.75, True),
                                          ("shim abort 1.0 s", "abort", 1.0, True),
                                          ("shim abort 1.25 s", "abort", 1.25, True)):
        bmin, t_bmin, done_t, p_bmin, hand = run(path, cum, npts, path[:, 0], yaw0, mode, abort_t,
                                                 shim_on=shim_on)
        ok = ~np.isnan(done_t.numpy())
        graze = bmin <= 0.005
        nh = sum(h is not None for h in hand)
        print(f"== {label}: success {ok.sum()}/{N}, mean time {np.nanmean(done_t.numpy()):.2f} s, "
              f"grazes (body <= 0.005 m) {graze.sum()}, contact (<= 0) {(bmin <= 0).sum()}, "
              f"min body {bmin.min():.3f} m, handovers recorded {nh}")
        by = collections.Counter()
        rows = []
        for i in np.flatnonzero(graze):
            s_, g_ = routes.pairs[ridx[i] // routes.layouts]
            by[names[s_]] += 1
            x_, y_, h_ = p_bmin[i]
            box, bp = nearest_box(x_, y_, h_)
            hs = hand[i]
            htxt = ("no shim" if hs is None else
                    f"handover t {hs[0]:.2f} s yaw {math.degrees(hs[3]):+.0f} w {hs[5]:+.2f} err "
                    f"{math.degrees(hs[6]):+.0f}")
            rows.append(f"     {names[s_]}->{names[g_]} start {math.degrees(float(yaw0[i])):+.0f}: "
                        f"min {bmin[i]:.3f} at t {t_bmin[i]:.2f} s pose ({x_:.3f},{y_:.3f},"
                        f"{math.degrees(h_):+.0f}) to {box} body pt ({bp[0]:+.3f},{bp[1]:+.3f}); {htxt}")
        print("   grazes by start node:", dict(by))
        print("\n".join(rows))
        out[label] = {"success": int(ok.sum()), "n": N, "grazes": int(graze.sum()),
                      "contact": int((bmin <= 0).sum()), "min_body": float(bmin.min()),
                      "mean_time": float(np.nanmean(done_t.numpy())), "rows": rows,
                      "handovers": [h for h in hand if h is not None]}
    return out


def plan_at(rec, t_rel):
    t0 = rec["t_start"]
    plan_t, pts = max((pl for pl in rec["plans"] if pl[0] - t0 <= t_rel + 1e-6), key=lambda pl: pl[0],
                      default=rec["plans"][0])
    p = np.array(pts, dtype=np.float32)
    d = np.r_[0.0, np.cumsum(np.hypot(*np.diff(p, axis=0).T))].astype(np.float32)
    return plan_t - t0, p, d


def one(p, d, k, xy0, y0, w0, mode, abort_t, shim_on, noise):
    global NOISE
    NOISE = noise
    torch.manual_seed(1)
    path = torch.tensor(p)[None].repeat(k, 1, 1)
    cum = torch.tensor(d)[None].repeat(k, 1)
    npts = torch.tensor([len(p)] * k)
    res = run(path, cum, npts, torch.tensor([xy0] * k, dtype=torch.float32), torch.full((k,), y0), mode,
              abort_t, w0=torch.full((k,), w0), steps=600, shim_on=shim_on)
    NOISE = 0.0
    return res


def describe(label, res):
    bmin, t_bmin, done_t, p_bmin, hand = res
    i = int(np.argmin(bmin))
    x_, y_, h_ = p_bmin[i]
    box, bp = nearest_box(x_, y_, h_)
    hs = hand[i]
    htxt = "" if hs is None else (f"; shim handover t {hs[0]:.2f} s yaw {hs[3]:+.2f} rad w {hs[5]:+.2f} "
                                  f"err {hs[6]:+.2f} rad")
    k = len(bmin)
    many = f" [{k} seeds: grazes {(bmin <= 0.005).sum()}, min {bmin.min():.3f}, median {np.median(bmin):.3f}]"         if k > 1 else ""
    return (f"   {label}: min body {bmin[i]:.3f} m at t {t_bmin[i]:.2f} s pose ({x_:.3f},{y_:.3f},{h_:+.2f} rad)"
            f" to {box} body pt ({bp[0]:+.3f},{bp[1]:+.3f}){htxt}{many}")


GZ_HANDOVER = {0: (1.17, 0.67, 1.00), 1: (0.42, 0.11, 0.54), 2: (0.58, 0.17, 0.60)}


def repro(legs):
    lines = []
    for n, leg in enumerate(legs):
        rec = json.loads(Path(leg).read_text())
        th, yaw_h, w_h = GZ_HANDOVER[n]
        _, p0, d0 = plan_at(rec, 0.0)
        pth, ph, dh = plan_at(rec, th)
        lines.append(f"  Gazebo leg {n}: handover t {th:.2f} s at (1.660, 0.150, {yaw_h:.2f} rad), w {w_h:.2f}, "
                     f"plan of t {pth:.2f} s")
        lines.append(describe("shim nominal from Q2 yaw 0, noise 0",
                              one(p0, d0, 1, (1.66, 0.15), 0.0, 0.0, "nominal", None, True, 0.0)))
        lines.append(describe(f"shim abort at {th:.2f} s from Q2 yaw 0, noise 0",
                              one(p0, d0, 1, (1.66, 0.15), 0.0, 0.0, "abort", th, True, 0.0)))
        lines.append(describe("policy from the Gazebo handover state, noise 0",
                              one(ph, dh, 1, (1.66, 0.15), yaw_h, w_h, "nominal", None, False, 0.0)))
        lines.append(describe("policy from the Gazebo handover state, lidar noise 0.02 m",
                              one(ph, dh, 50, (1.66, 0.15), yaw_h, w_h, "nominal", None, False, 0.02)))
        lines.append(describe("policy from the Gazebo handover state with w 0, noise 0",
                              one(ph, dh, 1, (1.66, 0.15), yaw_h, 0.0, "nominal", None, False, 0.0)))
        lines.append(describe("RL alone from Q2 yaw 0, noise 0",
                              one(p0, d0, 1, (1.66, 0.15), 0.0, 0.0, "nominal", None, False, 0.0)))
    print("== Gazebo Q2->E-3 reproduction on the Gazebo plans")
    print(chr(10).join(lines))
    return lines


if __name__ == "__main__":
    lines = repro(sys.argv[1:4])
    if len(sys.argv) > 5 and sys.argv[5] == "repro-only":
        sys.exit(0)
    out = sweep()
    out["repro"] = lines
    Path(sys.argv[4]).write_text(json.dumps(out, indent=1), encoding="utf-8")
