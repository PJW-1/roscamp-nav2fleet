"""급행 지나기마다 실제 위치(sim/truth)로 진열대 A와 북쪽 벽까지 남은 거리를 잰다.

사용: python3 express_metrics.py TRUTH.csv BATCH.log [--passes]  (한 파일에 여러 판, 로그로 나눔)
     python3 express_metrics.py NAME=TRUTH.csv ... [--passes]      (판마다 파일 하나)
TRUTH.csv: wall,sim_t,robot,x,y,yaw (truth_logger.py).
발자국은 pode_nav_config nav2_params.yaml:286 다각형이다(Gazebo 충돌 모양과 같다고 가정).
"""
import csv
import math
import os
import re
import sys
from collections import defaultdict
from datetime import datetime
from zoneinfo import ZoneInfo

KST = ZoneInfo("Asia/Seoul")
SHELF_A = (0.63, 0.77, 1.56, 0.803)
WALL_N = 0.98
FOOT = [(0.040, 0.021), (0.036, 0.031), (0.029, 0.056), (-0.029, 0.056), (-0.049, 0.049),
        (-0.071, 0.042), (-0.076, 0.037), (-0.076, -0.037), (-0.071, -0.042), (-0.049, -0.049),
        (-0.029, -0.056), (0.029, -0.056), (0.036, -0.031), (0.040, -0.021)]


def foot_points(x, y, yaw, step=0.004):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for (ax, ay), (bx, by) in zip(FOOT, FOOT[1:] + FOOT[:1]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        for i in range(n):
            px, py = ax + (bx - ax) * i / n, ay + (by - ay) * i / n
            pts.append((x + c * px - s * py, y + s * px + c * py))
    return pts


def rect_dist(px, py, r):
    x0, y0, x1, y1 = r
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    if dx == 0.0 and dy == 0.0:
        return -min(px - x0, x1 - px, py - y0, y1 - py)
    return math.hypot(dx, dy)


def runs_from_log(path):
    """판 이름, 시작, 끝(epoch). 로그 시각은 KST로 읽는다(실행하는 PC의 시간대와 무관).

    날짜는 --date=YYYY-MM-DD로 주고, 없으면 로그 파일이 마지막으로 바뀐 날을 쓴다(자정을 넘긴 묶음은 안 된다)."""
    opt = [a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--date=")]
    day = opt[0] if opt else datetime.fromtimestamp(os.path.getmtime(path), KST).strftime("%Y-%m-%d")
    out, cur = [], None
    for line in open(path):
        m = re.match(r"== (\S+) (\S+) start (\d\d:\d\d:\d\d)", line)
        if m:
            cur = [m.group(1), datetime.fromisoformat(f"{day} {m.group(3)}").replace(tzinfo=KST).timestamp(), None]
            out.append(cur)
        m = re.match(r"== (\S+) exit \S+ end (\d\d:\d\d:\d\d)", line)
        if m and cur and cur[0] == m.group(1):
            cur[2] = datetime.fromisoformat(f"{day} {m.group(2)}").replace(tzinfo=KST).timestamp()
    return out


def passes(rows):
    """급행 띠(진열대 A 북쪽, x 0.55~1.75) 안의 연속 구간."""
    out, cur = [], []
    for r in rows:
        inside = 0.55 <= r[1] <= 1.75 and r[2] > 0.805
        if inside and (not cur or r[0] - cur[-1][0] < 2.0):
            cur.append(r)
        elif inside:
            out.append(cur)
            cur = [r]
        elif cur and r[0] - cur[-1][0] >= 2.0:
            out.append(cur)
            cur = []
    if cur:
        out.append(cur)
    return [p for p in out if len(p) >= 5]


def stuck_at_entry(p, window=3.0, move=0.02):
    """들머리(x 1.40~1.66)에서 3 s 동안 2 cm 안에 머문 적이 있나."""
    ent = [r for r in p if 1.40 <= r[1] <= 1.66]
    for i, a in enumerate(ent):
        for b in ent[i + 1:]:
            if b[0] - a[0] >= window:
                if math.hypot(b[1] - a[1], b[2] - a[2]) < move:
                    return True
                break
    return False


def measure(p):
    shelf = min(min(rect_dist(px, py, SHELF_A) for px, py in foot_points(x, y, yaw))
                for _, x, y, yaw in p)
    wall = min(WALL_N - max(py for _, py in foot_points(x, y, yaw)) for _, x, y, yaw in p)
    arc = [r[2] for r in p if 1.56 <= r[1] <= 1.75]
    lane = [r[2] for r in p if 0.63 <= r[1] <= 1.56]
    corner = min((math.hypot(r[1] - 1.56, r[2] - 0.803) for r in p), default=None)
    return {"t0": round(p[0][0], 1), "t1": round(p[-1][0], 1), "shelf_raw_m": shelf,
            "shelf_clear_cm": round(shelf * 100, 1), "wall_clear_cm": round(wall * 100, 1),
            "arc_max_y": round(max(arc), 3) if arc else None,
            "lane_min_y": round(min(lane), 3) if lane else None,
            "lane_max_y": round(max(lane), 3) if lane else None,
            "corner_centre_m": round(corner, 3) if corner is not None else None,
            "stuck_entry": stuck_at_entry(p)}


def read_truth(path):
    return [(float(r["wall"]), float(r["sim_t"]), r["robot"], float(r["x"]), float(r["y"]),
             float(r["yaw"])) for r in csv.DictReader(open(path))]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    show = "--passes" in sys.argv
    if len(args) == 2 and args[1].endswith(".log"):
        rows = read_truth(args[0])
        runs = [(name, w0, w1, rows) for name, w0, w1 in runs_from_log(args[1])]
    else:
        runs = []
        for a in args:
            name, path = a.split("=", 1)
            runs.append((name, -math.inf, math.inf, read_truth(path)))
    print("| run | robot passes | min shelf clear cm | passes < 0.5 cm | min wall clear cm "
          "| arc max y | lane y range | entry stuck |")
    print("|---|---|---|---|---|---|---|---|")
    for name, w0, w1, rows in runs:
        w1 = w1 or math.inf
        by = defaultdict(list)
        for wall, t, robot, x, y, yaw in rows:
            if w0 <= wall <= w1:
                by[robot].append((t, x, y, yaw))
        ms = []
        for robot, rr in sorted(by.items()):
            rr.sort()
            for p in passes(rr):
                m = measure(p)
                m["robot"] = robot
                ms.append(m)
        if not ms:
            print(f"| {name} | 0 | - | - | - | - | - | - |")
            continue
        # 0.5 cm 비교는 반올림 전 값으로 한다
        print(f"| {name} | {len(ms)} | {round(min(m['shelf_raw_m'] for m in ms) * 100, 1)} "
              f"| {sum(m['shelf_raw_m'] < 0.005 for m in ms)} | {min(m['wall_clear_cm'] for m in ms)} "
              f"| {max(m['arc_max_y'] or 0 for m in ms)} "
              f"| {min(m['lane_min_y'] or 9 for m in ms)}~{max(m['lane_max_y'] or 0 for m in ms)} "
              f"| {sum(m['stuck_entry'] for m in ms)} |")
        if show:
            for m in ms:
                print("   ", m)


if __name__ == "__main__":
    main()
