"""배치 v7의 Gazebo 경기장과 시나리오: 월드 치수, 지도, keepout, gz_v7(_random)."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytest.importorskip("fleet_core")
pytest.importorskip("fake_robots")
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_gz_sim import arena_maps, fleet_setup  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
BOXES = [(0.68, 0.77, 1.56, 0.81), (0.68, 0.365, 0.99, 0.405), (1.25, 0.365, 1.56, 0.405),
         (0.24, 0.0, 0.26, 0.10), (0.50, 0.0, 0.52, 0.10), (0.76, 0.0, 0.78, 0.10)]


@pytest.fixture(scope="module")
def site():
    return load_fleet_map("v7")


def _boxes(path):
    root = ET.fromstring(path.read_text(encoding="utf-8"))
    out = []
    for part in root.iter("collision"):
        if not part.get("name").startswith("shelf_"):
            continue
        pose = [float(v) for v in part.find("pose").text.split()]
        size = [float(v) for v in part.find("geometry/box/size").text.split()]
        out.append((round(pose[0] - size[0] / 2, 4), round(pose[1] - size[1] / 2, 4),
                    round(pose[0] + size[0] / 2, 4), round(pose[1] + size[1] / 2, 4)))
    return out


def test_v7_world_has_the_shelves_and_dividers(site):
    found = fleet_setup.arena_for("v7", PKG)
    assert sorted(_boxes(Path(found.world))) == sorted(BOXES)


def test_v7_charging_stalls_and_express_are_open_in_the_fine_map(site):
    grid = arena_maps.occupancy(arena_maps.FINE_M, site)

    def free(x, y):
        i, j = arena_maps.cell_of(x, y, arena_maps.FINE_M, site)
        return not grid[j][i]
    for x in (0.12, 0.38, 0.64):         # 칸 한가운데
        assert free(x, 0.05)
    assert not free(0.25, 0.05) and not free(0.51, 0.05) and not free(0.77, 0.05)
    assert free(1.0, 0.85) and free(1.0, 0.95)   # 급행 0.18 안


def test_v7_standing_nodes_are_outside_the_keepout(site):
    grid = arena_maps.keepout(arena_maps.FINE_M, site)
    for name in ("N_O", "N_E", "G", "Q1", "Q2", "Q3", "O_Q", "E-1", "E-2", "E-3", "P-1", "P-2", "P-3"):
        x, y = site.node_pose(name)
        i, j = arena_maps.cell_of(x, y, arena_maps.FINE_M, site)
        assert not grid[j][i], name


@pytest.mark.parametrize("name", ["gz_v7", "gz_v7_random"])
def test_v7_scenarios_fit_the_layout(site, name):
    data = fleet_setup.read_scenario(PKG / "scenarios" / f"{name}.yaml")
    assert data["map"] == "v7"
    assert len(data["orders"]) == 8 and len(data["robots"]) == 3
    for seed in (None, 4):
        fleet_setup.check_fits(data, name, "v7", seed)
    starts = fleet_setup.start_nodes(site, data)
    assert list(starts.values()) == ["P-3", "P-2", "P-1"]


def test_v7_random_scenario_has_the_random_block():
    data = fleet_setup.read_scenario(PKG / "scenarios" / "gz_v7_random.yaml")
    assert data["random"] == {"order_gap_s": [5, 40],
                              "low_battery": {"from_s": 20, "to_s": 150, "charge_s": 30}}
    assert fleet_setup.has_low_battery(data, 4)


@pytest.mark.parametrize("seed", [None, 4])
def test_v7_random_scenario_expands_to_one_low_battery_per_robot_and_spread_orders(seed):
    from fake_robots.scenario import expand_random
    data = fleet_setup.read_scenario(PKG / "scenarios" / "gz_v7_random.yaml")
    out = expand_random(data, seed)
    low = [f for f in out["faults"] if f["type"] == "low_battery"]
    assert sorted(f["robot"] for f in low) == ["pinky1", "pinky2", "pinky3"]
    assert all(20 <= f["at_s"] <= 150 and f["duration_s"] == 30 for f in low)
    times = [0.0] + [order["at_s"] for order in out["orders"]]
    gaps = [b - a for a, b in zip(times, times[1:])]
    # at_s는 쌓인 시각을 0.1 s로 반올림한 값이라 두 at_s의 차는 참 간격과 0.1 s까지 다르다
    assert len(gaps) == 8 and all(5 - 0.1 - 1e-9 <= gap <= 40 + 0.1 + 1e-9 for gap in gaps), gaps


def test_v7_committed_maps_are_the_generator_output(tmp_path, site):
    from fleet_gz_sim import arena
    written = arena.generate(tmp_path, site)
    expected = {"worlds/arena_v7.sdf"} | {f"maps/arena_v7{suffix}.{ext}"
                                          for suffix in ("", "_fine", "_keepout")
                                          for ext in ("yaml", "pgm")}
    assert set(written) == expected
    for name in written:
        assert (tmp_path / name).read_bytes() == (PKG / name).read_bytes(), name
    for suffix, resolution in (("", arena_maps.COARSE_M), ("_fine", arena_maps.FINE_M),
                               ("_keepout", arena_maps.FINE_M)):
        text = (PKG / "maps" / f"arena_v7{suffix}.yaml").read_text(encoding="utf-8")
        assert f"image: arena_v7{suffix}.pgm\n" in text
        assert f"resolution: {resolution}\n" in text
        assert "origin: [-0.15, -0.15, 0.0]\n" in text


# ---- 5 cm 전역 costmap 흉내: 커밋된 지도(5 cm)와 keepout 마스크(1 cm)를 읽는다 ----
COARSE, ORIGIN, INSCRIBED = 0.05, (-0.15, -0.15), 0.09


def _read_pgm(path):
    """P5 그림을 줄 아래부터의 막힘 격자로(0이 막힘)."""
    data = path.read_bytes()
    words, at = [], 0
    while len(words) < 4:
        end = data.index(b"\n", at)
        words += data[at:end].split(b"#")[0].split()
        at = end + 1
    w, h = int(words[1]), int(words[2])
    rows = [data[at + r * w: at + (r + 1) * w] for r in range(h)]
    return [[v == 0 for v in row] for row in reversed(rows)]


def _costmap():
    """칸마다 (막힘, 이유). 진열대·벽 칸 중심에서 내접 0.09 안이거나, 칸 중심의 1 cm 마스크가
    keepout이면 막힘(Nav2 KeepoutFilter는 칸 중심에서 마스크를 읽고 부풀리지 않는다)."""
    occ = _read_pgm(PKG / "maps" / "arena_v7.pgm")
    keep = _read_pgm(PKG / "maps" / "arena_v7_keepout.pgm")
    h, w = len(occ), len(occ[0])
    reach = int(INSCRIBED / COARSE) + 1
    blocked = [[False] * w for _ in range(h)]
    kept = [[False] * w for _ in range(h)]
    for j in range(h):
        for i in range(w):
            if occ[j][i]:
                for dj in range(-reach, reach + 1):
                    for di in range(-reach, reach + 1):
                        if (0 <= j + dj < h and 0 <= i + di < w
                                and COARSE * (di * di + dj * dj) ** 0.5 <= INSCRIBED + 1e-9):
                            blocked[j + dj][i + di] = True
            cx, cy = ORIGIN[0] + (i + 0.5) * COARSE, ORIGIN[1] + (j + 0.5) * COARSE
            fi, fj = int((cx - ORIGIN[0]) / 0.01), int((cy - ORIGIN[1]) / 0.01)
            kept[j][i] = keep[fj][fi]
    return [[blocked[j][i] or kept[j][i] for i in range(w)] for j in range(h)], occ, kept


def _cell(x, y):
    return int((x - ORIGIN[0]) / COARSE), int((y - ORIGIN[1]) / COARSE)


def _joined(grid, a, b, shut=()):
    """4 이웃 BFS. shut: 일부러 막아 보는 사각형들(그 길만 남겼을 때 이어지는지 보려고)."""
    from collections import deque

    def closed(i, j):
        cx, cy = ORIGIN[0] + (i + 0.5) * COARSE, ORIGIN[1] + (j + 0.5) * COARSE
        return grid[j][i] or any(x0 <= cx <= x1 and y0 <= cy <= y1 for x0, y0, x1, y1 in shut)
    start, goal = _cell(*a), _cell(*b)
    assert not closed(*start) and not closed(*goal), (a, b)
    seen, todo = {start}, deque([start])
    while todo:
        i, j = todo.popleft()
        if (i, j) == goal:
            return True
        for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if (0 <= nj < len(grid) and 0 <= ni < len(grid[0]) and (ni, nj) not in seen
                    and not closed(ni, nj)):
                seen.add((ni, nj))
                todo.append((ni, nj))
    return False


def test_v7_the_crossing_gap_is_shut_for_nav2_on_the_5cm_costmap():
    """건넘 틈(x 0.99~1.25)을 막는 keepout이 두 진열대 끝과 구멍 없이 이어진다: 남쪽 진열대 줄의
    어느 열도 위아래로 지나갈 수 없다(막힘 칸 또는 keepout 칸이 그 열의 진열대 높이에 있다)."""
    grid, occ, kept = _costmap()
    j0, j1 = _cell(0.0, 0.365)[1], _cell(0.0, 0.405 - 1e-6)[1]
    for x in [0.70 + 0.05 * k for k in range(18)]:  # 0.70 ~ 1.55
        i = _cell(x, 0.0)[0]
        assert any(occ[j][i] or kept[j][i] for j in range(j0, j1 + 1)), x
    assert _joined(grid, (1.12, 0.55), (1.12, 0.25)) is True  # 둘레로는 이어진다
    assert not _joined(grid, (1.12, 0.55), (1.12, 0.25),
                       shut=[(0.0, 0.30, 0.68, 0.45), (1.56, 0.30, 1.98, 0.45)])


@pytest.mark.parametrize("shut, pairs", [
    # 서쪽 끝(x < 0.68)만 남김: 동쪽 통로를 막아 본다
    ([(1.56, 0.30, 1.98, 0.45)], [("N_O", "Q1"), ("N_O", "Q3"), ("N_E", "Q1")]),
    # 동쪽 통로(1.65~1.72)만 남김: 서쪽 끝을 막아 본다. G는 그 통로 바로 위라 가장 빠듯하다
    ([(0.0, 0.30, 0.68, 0.45)], [("N_E", "G"), ("G", "Q3"), ("N_E", "Q1"), ("N_O", "Q1"),
                                 ("N_E", "Q3")]),
])
def test_v7_north_and_south_stay_joined_by_each_way_alone(site, shut, pairs):
    grid, _, _ = _costmap()
    for a, b in pairs:
        assert _joined(grid, site.node_pose(a), site.node_pose(b), shut), (a, b, shut)


# ---- 칸막이 3 위 끝: Nav2 goal 경로가 모서리를 긁지 않게 ----
DIVIDER_3_KEEP = (0.71, 0.0, 0.86, 0.20)  # 서쪽 끝 0.71: P-3에 선 로봇 발자국(0.64 + 0.06) 밖


def test_v7_the_top_of_divider_3_is_kept_out_for_nav2(site):
    """Gazebo 1~3판: 남쪽 마당을 서쪽으로 가던 로봇이 y ~0.16으로 와서 칸막이 3 위 끝(0.76~0.78,
    0.10)을 긁었다(3판 pinky2: RPP 충돌 경고 약 400회, NAV_ABORTED 3회 뒤 ISOLATED). 그 둘레를
    y 0.20까지 막아 Nav2 goal 경로의 중심이 그 구간에서 y 0.20 위로 다니게 한다. 칸 고정 경로
    (P-3 <-> E-3, FollowPath)는 local costmap만 보므로 걸리지 않는다."""
    assert DIVIDER_3_KEEP in site.keepout_extra
    grid = arena_maps.keepout(arena_maps.FINE_M, site)

    def kept(x, y):
        i, j = arena_maps.cell_of(x, y, arena_maps.FINE_M, site)
        return grid[j][i]
    for x in (0.72, 0.77, 0.85):
        assert kept(x, 0.12) and kept(x, 0.195), x
    assert not kept(0.77, 0.205) and not kept(0.64, 0.11), "above it and the P-3 bay stay open"


@pytest.mark.parametrize("a, b", [("E-3", "Q1"), ("E-3", "G"), ("E-3", "Q3"), ("E-3", "O_Q"),
                                  ("E-3", "N_O"), ("E-1", "E-3")])
def test_v7_the_south_yard_stays_open_above_the_divider_keepout(site, a, b):
    """칸막이 keepout(y < 0.20)과 남쪽 진열대 inflation 사이로(5 cm 격자에서 y 0.225, 0.275 줄)
    남쪽 마당이 이어진다."""
    grid, _, _ = _costmap()
    assert _joined(grid, site.node_pose(a), site.node_pose(b)), (a, b)
    # 동쪽으로 가는 길은 칸막이 3 위를 지나야 한다: 서쪽 끝을 막아도 이어진다
    if b in ("Q1", "G", "Q3"):
        assert _joined(grid, site.node_pose(a), site.node_pose(b),
                       shut=[(0.0, 0.40, 0.68, 0.99)]), (a, b)
