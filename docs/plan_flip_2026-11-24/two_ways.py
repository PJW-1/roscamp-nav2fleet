"""남쪽 진열대 줄을 넘는 세 길(틈 S, 동쪽 E, 서쪽 W)의 NavFn 비용과 길이를 v7 costmap 지도로 잰다.

실행(저장소 뿌리에서): python3 docs/plan_flip_2026-11-24/two_ways.py [a|a1]
- 정적 지도 sim/fleet_gz_sim/maps/arena_v7_<갈래>.pgm, keepout 마스크 arena_v7_keepout.pgm(1 cm).
- 부풀리기는 nav2_params global_costmap 값(발자국 반폭 0.06 + 덧댐 0.01 = 내접 0.07, 반경 0.15, 배율 15).
- NavFn 칸 값 50 + 0.8 * cost(253 이상은 못 지남). 8방향 다익스트라로 근사한다(NavFn 파면과 같지 않다).
- 길 셋은 진열대 줄(y 0.365~0.405)의 두 칸을 막아 가른다.
"""
from __future__ import annotations

import heapq
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parents[2]
MAPS = ROOT / "sim/fleet_gz_sim/maps"
ORIGIN = (-0.15, -0.15)
INSCRIBED, RADIUS, SCALE = 0.07, 0.15, 15.0
BAND = (0.365, 0.405)
GAP = (0.99, 1.25)
EAST_END, WEST_END = 1.56, 0.70


def load(variant: str):
    res = {"a": 0.025, "a1": 0.01}[variant]
    occ = np.array(Image.open(MAPS / f"arena_v7_{variant}.pgm"))[::-1] < 128  # 행 0 = y 아래
    mask = np.array(Image.open(MAPS / "arena_v7_keepout.pgm"))[::-1] < 128
    h, w = occ.shape
    ys = ORIGIN[1] + (np.arange(h) + 0.5) * res
    xs = ORIGIN[0] + (np.arange(w) + 0.5) * res
    dist = distance_transform_edt(~occ) * res
    cost = np.where(dist <= INSCRIBED, 253.0,
                    np.where(dist <= RADIUS + 1e-9,
                             np.floor(252 * np.exp(-SCALE * (dist - INSCRIBED))), 0.0))
    cost[occ] = 254
    # keepout은 칸 가운데에서 마스크를 읽고 부풀리지 않는다
    mi = np.clip(((xs - ORIGIN[0]) / 0.01).astype(int), 0, mask.shape[1] - 1)
    mj = np.clip(((ys - ORIGIN[1]) / 0.01).astype(int), 0, mask.shape[0] - 1)
    cost[mask[np.ix_(mj, mi)]] = 254
    return res, xs, ys, cost


def blocked(xs, ys, wall, ways: str):
    band = (ys >= BAND[0]) & (ys <= BAND[1])
    cut = np.zeros_like(wall, dtype=bool)
    if "S" not in ways:
        cut |= np.outer(band, (xs > GAP[0]) & (xs < GAP[1]))
    if "E" not in ways:
        cut |= np.outer(band, xs > EAST_END)
    if "W" not in ways:
        cut |= np.outer(band, xs < WEST_END)
    return wall | cut


def dijkstra(cell, wall, goal):
    h, w = cell.shape
    best = np.full((h, w), math.inf)
    step = np.full((h, w), -1, dtype=np.int64)
    gj, gi = goal
    best[gj, gi] = 0.0
    todo = [(0.0, gj, gi)]
    moves = [(dj, di, math.hypot(dj, di)) for dj in (-1, 0, 1) for di in (-1, 0, 1) if dj or di]
    while todo:
        spent, j, i = heapq.heappop(todo)
        if spent > best[j, i]:
            continue
        for dj, di, d in moves:
            nj, ni = j + dj, i + di
            if 0 <= nj < h and 0 <= ni < w and not wall[nj, ni]:
                total = spent + d * (cell[j, i] + cell[nj, ni]) / 2
                if total < best[nj, ni]:
                    best[nj, ni] = total
                    step[nj, ni] = j * w + i
                    heapq.heappush(todo, (total, nj, ni))
    return best, step


def navfn(cell, wall, goal):
    """NavFn updateCell의 2차 근사 갱신(4방향 파면)을 힙 순서로 돈다. 값은 칸 비용 x 칸 수."""
    h, w = cell.shape
    pot = np.full((h, w), math.inf)
    gj, gi = goal
    pot[gj, gi] = 0.0
    todo = [(0.0, gj, gi)]
    done = np.zeros((h, w), dtype=bool)
    while todo:
        spent, j, i = heapq.heappop(todo)
        if done[j, i] or spent > pot[j, i]:
            continue
        done[j, i] = True
        for nj, ni in ((j - 1, i), (j + 1, i), (j, i - 1), (j, i + 1)):
            if not (0 <= nj < h and 0 <= ni < w) or wall[nj, ni] or done[nj, ni]:
                continue
            lr = min(pot[nj, ni - 1] if ni > 0 else math.inf,
                     pot[nj, ni + 1] if ni < w - 1 else math.inf)
            ud = min(pot[nj - 1, ni] if nj > 0 else math.inf,
                     pot[nj + 1, ni] if nj < h - 1 else math.inf)
            ta, tc = min(lr, ud), max(lr, ud)
            hf = cell[nj, ni]
            dc = tc - ta
            if dc >= hf:
                value = ta + hf
            else:
                d = dc / hf
                value = ta + hf * (-0.2301 * d * d + 0.5307 * d + 0.7040)
            if value < pot[nj, ni]:
                pot[nj, ni] = value
                heapq.heappush(todo, (value, nj, ni))
    return pot


def walk(step, start, res):
    w = step.shape[1]
    j, i = start
    total = 0.0
    while step[j, i] >= 0:
        nj, ni = divmod(int(step[j, i]), w)
        total += math.hypot(nj - j, ni - i) * res
        j, i = nj, ni
    return total


class Ways:
    def __init__(self, variant: str):
        self.res, self.xs, self.ys, cost = load(variant)
        self.wall = cost >= 253
        self.cell = 50 + 0.8 * cost
        self._memo = {}

    def index(self, x, y):
        return (int(math.floor((y - ORIGIN[1]) / self.res)),
                int(math.floor((x - ORIGIN[0]) / self.res)))

    def field(self, goal, way):
        key = (goal, way)
        if key not in self._memo:
            wall = blocked(self.xs, self.ys, self.wall, way)
            best, step = dijkstra(self.cell, wall, self.index(*goal))
            self._memo[key] = (best, step, navfn(self.cell, wall, self.index(*goal)))
        return self._memo[key]

    def measure(self, start, goal):
        """{길: (NavFn 비용 m, 8방향 최소 비용 길의 길이 m)}.
        비용 m = 비용 / 50 x 칸 크기(빈 칸만 지나면 길이와 비슷하다)."""
        out = {}
        for way in "SEW":
            best, step, pot = self.field(goal, way)
            j, i = self.index(*start)
            if math.isfinite(best[j, i]) and math.isfinite(pot[j, i]):
                out[way] = (pot[j, i] / 50 * self.res, walk(step, (j, i), self.res))
        return out


def gap(m) -> tuple[str, str, float]:
    """가장 싼 길, 다음 길, 비용 차이 비율(다음 / 가장 싼 - 1).
    길 셋의 비용이 같으면 진열대 줄을 넘지 않는 쌍(같은 쪽)이라 갈림이 없다(비율 inf)."""
    order = sorted(m, key=lambda k: m[k][0])
    if len(order) < 2:
        return (order[0] if order else ""), "", math.inf
    if m[order[-1]][0] - m[order[0]][0] < 1e-9:
        return order[0], "", math.inf
    return order[0], order[1], m[order[1]][0] / m[order[0]][0] - 1


THRESHOLD = 0.15  # 두 길 비용 차이가 이 비율 안이면 "비슷하다"(기록된 자리 (1.05, 0.55)는 a 10.2 %, a1 13.8 %)
SPEED, TURN = 0.12, 1.0  # 시간 어림: nav_speed_mps, nav_turn_rps(Q2에 닿은 방향과 Q2 방향의 차이만큼 돈다)


def grid(site, step=0.025):
    for row in range(int(round(0.98 / step)) + 1):
        for k in range(int(round(1.98 / step)) + 1):
            x, y = round(k * step, 4), round(row * step, 4)
            if site.zone_at(x, y) == "HALL":
                yield x, y


def via_cost(ways, start, via, goal):
    """출발 -> via -> goal 두 조각. (비용 m, 길이 m, 첫 조각 비율, 둘째 조각 비율)."""
    first, second = ways.measure(start, via), ways.measure(via, goal)
    fa, _, fr = gap(first)
    sa, _, sr = gap(second)
    return first[fa][0] + second[sa][0], first[fa][1] + second[sa][1], fr, sr


def seconds(length, via, goal, goal_yaw):
    arrive = math.atan2(goal[1] - via[1], goal[0] - via[0])
    return length / SPEED + abs(math.remainder(arrive - goal_yaw, math.tau)) / TURN


def main():
    sys.path.insert(0, str(ROOT / "pc/fleet_core"))
    from fleet_core.fleet_map import load_fleet_map
    site = load_fleet_map("v7")
    variant = sys.argv[1] if len(sys.argv) > 1 else "a"
    ways = Ways(variant)
    q2 = site.node_pose("Q2")
    print(f"variant {variant}, res {ways.res}, threshold {THRESHOLD:.0%}")
    print("== 출발 -> Q2: 길별 (NavFn 비용 m, 길이 m), 가장 싼 길, 다음 길, 차이 비율")
    for start in [(1.05, 0.55), (0.9, 0.55), (0.8, 0.6), (0.68, 0.6), (1.2, 0.55), (1.3, 0.55),
                  (1.4, 0.55), (1.05, 0.62), (1.12, 0.48), (0.34, 0.68), (0.45, 0.45)]:
        m = ways.measure(start, q2)
        a, b, r = gap(m)
        print(" ", start, {k: (round(c, 3), round(n, 3)) for k, (c, n) in m.items()},
              a, b, f"{r:.3f}")
    print("== 지도 splits(Q2) 사각형마다: 2.5 cm 격자 HALL 점")
    for split in site.splits:
        via = site.node_pose(split.via)
        rows = []
        for x, y in grid(site):
            if not any(r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in split.rects):
                continue
            direct = ways.measure((x, y), site.node_pose(split.to))
            if len(direct) < 2:
                continue
            a, b, r = gap(direct)
            cost, length, fr, sr = via_cost(ways, (x, y), via, site.node_pose(split.to))
            rows.append(((x, y), r, fr, sr, cost / direct[a][0] - 1, length - direct[a][1]))
        ties = [row for row in rows if row[1] < THRESHOLD]
        print(f"  {split.to} via {split.via} {split.rects}: 점 {len(rows)}, 그 가운데 비슷함 {len(ties)}")
        print(f"    첫 조각 차이 비율 최소 {min(r[2] for r in rows):.3f} at "
              f"{min(rows, key=lambda r: r[2])[0]}, 둘째 조각 {min(r[3] for r in rows):.3f}")
        print(f"    NavFn 비용 늘어남(나눔/곧장 가장 싼 길 - 1) 최대 {max(r[4] for r in rows):.3f}, "
              f"평균 {sum(r[4] for r in rows) / len(rows):.3f}")
        print(f"    길이 차(나눔 - 곧장 가장 싼 길) {min(r[5] for r in rows):+.3f} ~ "
              f"{max(r[5] for r in rows):+.3f} m")
    print("== 기록된 자리 (1.05, 0.55): 어림 시간(길이 / 0.12 + Q2에서 돌기)")
    start = (1.05, 0.55)
    for name in ("GAP_S", "M_E"):
        via = site.node_pose(name)
        cost, length, fr, sr = via_cost(ways, start, via, q2)
        print(f"  via {name}: 길이 {length:.3f} m, 비용 {cost:.3f}, 첫 조각 비율 {fr:.3f}, "
              f"둘째 {sr:.3f}, 어림 {seconds(length, via, q2, 0.0):.1f} s")
    print("== 노드 출발(through 사슬로 Q2 goal 하나가 되는 자리): 곧장 차이 비율, via별 비용 늘어남과 첫 조각 비율")
    for name in ("O_OUT", "O_IN", "E-1", "E-2", "E-3"):
        start = site.node_pose(name)
        direct = ways.measure(start, q2)
        a, b, r = gap(direct)
        cells = []
        for via_name in ("GAP_S", "M_E"):
            cost, length, fr, sr = via_cost(ways, start, site.node_pose(via_name), q2)
            cells.append(f"via {via_name} {cost / direct[a][0] - 1:+.3f} 첫 {fr:.3f}")
        print(f"  {name} {start}: {a} {b} {r:.3f} | " + " | ".join(cells))


if __name__ == "__main__":
    main()
