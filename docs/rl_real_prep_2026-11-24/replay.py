"""실물 bag 스캔을 플러그인 관측(observation.cpp와 같은 규칙)으로 바꿔 정책에 넣는다.
왼쪽 뒤 고정 반사(빔 300..306)가 있을 때와 뺐을 때를 비교한다. 목표는 정면 0.4 m, v 0.1, w 0."""
import json, math, sys
import numpy as np
import onnxruntime as ort

scans = np.load(sys.argv[1])
policy_dir = sys.argv[2]
spec = json.load(open(policy_dir + "/policy_spec.json"))
R, meta = scans["ranges"].astype(np.float64), scans["meta"]
amin, inc, rmin, rmax = meta
lid = spec["lidar"]
yaw = math.pi
n = R.shape[1]
k = np.arange(lid["rays"])
theta = np.radians(lid["first_ray_deg"] + k * lid["ray_step_deg"])
rel = np.mod(theta - yaw - amin, 2 * math.pi)
idx = np.floor(rel / inc + 0.5).astype(int)
idx = np.where(idx >= n, np.where(2 * math.pi - rel <= 0.5 * inc, 0, -1), idx)
assert (idx >= 0).all()
sess = ort.InferenceSession(policy_dir + "/policy.onnx", providers=["CPUExecutionProvider"])
a = spec["action"]

def obs_of(ranges):
    r = ranges[:, idx]
    bad = ~np.isfinite(r) | (r < rmin) | (r > rmax)
    r = np.where(bad, lid["obs_max"], np.minimum(r, lid["obs_max"]))
    sec = r.reshape(len(r), lid["sectors"], -1).min(axis=2) / lid["obs_max"]
    tail = np.tile([0.4 / spec["goal"]["obs_max"], 0.0, 1.0, 0.1 / a["v_max"], 0.0], (len(r), 1))
    return np.hstack([sec, tail]).astype(np.float32), sec

o_raw, s_raw = obs_of(R)
M = R.copy(); M[:, 300:307] = np.inf
o_mask, s_mask = obs_of(M)
used = sorted(set(int(i) for i in idx if 296 <= i <= 310))
print("training rays hitting beams 296..310:", used, "-> rays", [int(j) for j in k[(idx >= 300) & (idx <= 306)]])
print("sector 33 raw: mean", s_raw[:, 33].mean() * 2, "m; masked mean", s_mask[:, 33].mean() * 2, "m")
def act(o):
    out = np.vstack([sess.run(None, {"obs": o[i:i + 1]})[0] for i in range(len(o))])
    av = np.clip(out[:, 0], -1, 1); aw = np.clip(out[:, 1], -1, 1)
    return a["v_min"] + (av + 1) * 0.5 * (a["v_max"] - a["v_min"]), aw * a["w_max"]
v0, w0 = act(o_raw); v1, w1 = act(o_mask)
dv, dw = np.abs(v0 - v1), np.abs(w0 - w1)
print("target v raw mean %.4f masked %.4f; |dv| median %.4f p95 %.4f max %.4f m/s" % (v0.mean(), v1.mean(), np.median(dv), np.percentile(dv, 95), dv.max()))
print("target w raw mean %.3f masked %.3f; |dw| median %.3f p95 %.3f max %.3f rad/s" % (w0.mean(), w1.mean(), np.median(dw), np.percentile(dw, 95), dw.max()))
print("reverse commands (v<0): raw %d masked %d of %d" % ((v0 < 0).sum(), (v1 < 0).sum(), len(v0)))
