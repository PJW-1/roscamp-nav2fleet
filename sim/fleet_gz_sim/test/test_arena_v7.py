"""배치 v7의 Gazebo 경기장과 시나리오: 월드 치수, 지도, keepout, gz_v7(_random)."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

pytest.importorskip("fleet_core")
pytest.importorskip("fake_robots")
from fleet_core.fleet_map import load_fleet_map  # noqa: E402
from fleet_gz_sim import arena_maps, fleet_setup  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
# 칸막이 3(0.76~0.78)은 2026-11-17에 뺐다
# 북쪽 진열대 A는 2026-11-17에 서쪽으로 0.05 늘렸다(0.68 -> 0.63),
# 2026-11-18에 북쪽 면을 실측 0.803으로, 2026-11-20에 시뮬만 1 cm 물려 0.793으로, 2026-11-21에 바퀴 여유로 0.783으로(sim_shelves)
# 2026-11-23 줄자 실측: 진열대 A 서쪽 끝 0.70, 남쪽 서쪽 진열대 서쪽 끝 0.65
BOXES = [(0.70, 0.77, 1.56, 0.783), (0.65, 0.365, 0.99, 0.405), (1.25, 0.365, 1.56, 0.405),
         (0.24, 0.0, 0.26, 0.10), (0.50, 0.0, 0.52, 0.10)]


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
    assert not free(0.25, 0.05) and not free(0.51, 0.05)
    assert free(0.77, 0.05)  # 칸막이 3이 있던 자리
    assert free(1.0, 0.85) and free(1.0, 0.95)   # 급행 0.17(실측) 안
    # 시뮬 진열대 A 북쪽 면 0.783: 1 cm 지도에서 0.78~0.79 칸까지 진열대, 0.79~0.80 칸은 빈칸
    assert not free(1.0, 0.785) and free(1.0, 0.795)
    # 북쪽 벽 안쪽 면은 SLAM 실측 0.98(2026-11-17): 1 cm 지도에서 0.98~0.99 칸은 벽이다
    assert free(1.0, 0.975) and not free(1.0, 0.985)


def test_v7_real_kit_maps_are_drawn_with_the_measured_shelf(tmp_path):
    # 실물 키트 지도는 --real: 시뮬 전용 진열대 대신 실측 진열대(북쪽 면 0.803)로 그린다
    from fleet_gz_sim import arena
    arena.main(["--layout", "v7", "--real", "--out", str(tmp_path)])
    site = load_fleet_map("v7")
    assert arena.real_site(site).sim_shelves == site.shelves
    assert site.sim_shelves != site.shelves  # 원본은 그대로
    lines = (tmp_path / "maps" / "arena_v7_fine.yaml").read_text(encoding="utf-8").splitlines()
    assert lines[1] == "# 실물 키트: /tmp/real_v7/maps/의 지도를 tools/real_pode/maps로 복사한다"
    # 머리 줄 명령은 셸에 그대로 쳐도 돈다(--out만 시험 폴더로 바꿔 같은 파일이 나오는지 본다)
    import shlex
    words = shlex.split(lines[0].removeprefix("# 생성 파일: "))
    assert words[:3] == ["python3", "-m", "fleet_gz_sim.arena"]
    args = words[3:]
    assert args[args.index("--out") + 1] == "/tmp/real_v7"
    args[args.index("--out") + 1] = str(tmp_path / "again")
    arena.main(args)
    for path in sorted((tmp_path / "maps").iterdir()):
        assert (tmp_path / "again" / "maps" / path.name).read_bytes() == path.read_bytes()
    real = arena_maps.pgm(arena_maps.FINE_M, arena.real_site(site))
    assert (tmp_path / "maps" / "arena_v7_fine.pgm").read_bytes() == real
    assert real != arena_maps.pgm(arena_maps.FINE_M, site)
    assert (tmp_path / "maps" / "arena_v7_a.yaml").exists()


def test_v7_real_maps_are_never_written_over_the_sim_files(capsys):
    from fleet_gz_sim import arena
    with pytest.raises(SystemExit):
        arena.main(["--layout", "v7", "--real"])
    assert "실물 지도는 --out" in capsys.readouterr().err


def test_v7_standing_nodes_are_outside_the_keepout(site):
    grid = arena_maps.keepout(arena_maps.FINE_M, site)
    for name in ("O_IN", "O_OUT", "M_E", "Q1", "Q2", "E-1", "E-2", "E-3", "P-1", "P-2", "P-3"):
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


def test_v7_varied_scenario_mixes_order_bursts_with_staggered_low_batteries(site):
    """더 다양한 상황. 주문이 몰렸다 쉬고,
    로봇마다 다른 때에 배터리가 떨어진다."""
    from fake_robots.scenario import load_scenario
    path = PKG / "scenarios" / "gz_v7_varied.yaml"
    data = fleet_setup.read_scenario(path)
    fleet_setup.check_fits(data, "gz_v7_varied", "v7")
    assert list(fleet_setup.start_nodes(site, data).values()) == ["P-3", "P-2", "P-1"]
    assert len(data["orders"]) == 12 and fleet_setup.has_low_battery(data)
    times = [order["at_s"] for order in data["orders"]]
    gaps = [b - a for a, b in zip(times, times[1:], strict=False)]
    assert min(gaps) == 0 and max(gaps) >= 90, gaps
    sizes = {sum(order["items"].values()) for order in data["orders"]}
    assert sizes >= {1, 2, 3, 4}
    low = [f for f in data["faults"] if f["type"] == "low_battery"]
    assert sorted(f["robot"] for f in low) == ["pode1", "pode2", "pode3"]
    assert len({f["at_s"] for f in low}) == 3 and len({f["duration_s"] for f in low}) == 3
    assert {f["type"] for f in data["faults"]} == {"low_battery"}
    scenario = load_scenario(path, site)
    assert scenario.expect.orders_shipped == 12 and scenario.expect.parked_while_backlog is None


def test_v7_rush_scenario_sends_twelve_orders_at_once_without_faults(site):
    """주행 성능을 재는 판(2026-11-14): 주문 12건이 처음 12초 안에 다 들어오고 고장이 없다.
    걸린 시간(duration_s)이 곧 처리 성능이다."""
    from fake_robots.scenario import load_scenario
    path = PKG / "scenarios" / "gz_v7_rush.yaml"
    data = fleet_setup.read_scenario(path)
    fleet_setup.check_fits(data, "gz_v7_rush", "v7")
    assert list(fleet_setup.start_nodes(site, data).values()) == ["P-3", "P-2", "P-1"]
    assert len(data["orders"]) == 12
    assert max(order["at_s"] for order in data["orders"]) <= 12
    assert not data.get("faults") and "random" not in data
    scenario = load_scenario(path, site)
    assert scenario.expect.orders_shipped == 12


def test_v7_obstacle_schedule_puts_small_boxes_on_nav_goal_edges_only(site):
    """고정 경로는 Gazebo에서 충돌 검사를 하지 않는다. 상자는 Nav2 goal 이음 위에만 놓는다.
    상자가 작아 로봇이 스스로 돌아갈 틈이 남는다
    (장애물이 생기면 회피가 보여야 한다)."""
    from fleet_core.fleet_map.model import NAV_GOAL
    from fleet_gz_sim.obstacles import load_schedule
    schedule = load_schedule(PKG / "obstacles" / "v7_yard_mix.yaml", site)
    assert [one.edge for one in schedule.obstacles] == [("E-3", "Q2"), ("O_OUT", "E-3"),
                                                         ("O_IN", "E-1"), ("O_IN", "M_E")]
    for one in schedule.obstacles:
        assert site.edge(*one.edge).mode == NAV_GOAL
        assert one.at_s >= 15, "sim_obstacles가 뜬 뒤"
    assert max(schedule.size_m[:2]) <= 0.08
    # 남쪽 통로의 상자 남쪽에 로봇 폭(0.112) + 양쪽 0.03의 틈이 남는다
    south = schedule.obstacles[0]
    assert south.y - schedule.size_m[1] / 2 >= 0.112 + 0.06, south


@pytest.mark.parametrize("seed", [None, 4])
def test_v7_random_scenario_expands_to_one_low_battery_per_robot_and_spread_orders(seed):
    from fake_robots.scenario import expand_random
    data = fleet_setup.read_scenario(PKG / "scenarios" / "gz_v7_random.yaml")
    out = expand_random(data, seed)
    low = [f for f in out["faults"] if f["type"] == "low_battery"]
    assert sorted(f["robot"] for f in low) == ["pode1", "pode2", "pode3"]
    assert all(20 <= f["at_s"] <= 150 and f["duration_s"] == 30 for f in low)
    times = [0.0] + [order["at_s"] for order in out["orders"]]
    gaps = [b - a for a, b in zip(times, times[1:], strict=False)]
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


V7_GAP = (0.99, 0.365, 1.25, 0.405)  # 남쪽 진열대 사이 건넘 틈
BOTH_ENDS = [(0.0, 0.30, 0.65, 0.45), (1.56, 0.30, 1.98, 0.45)]  # 서쪽 끝과 동쪽 통로


def test_v7_the_crossing_gap_is_open_for_nav2_on_the_5cm_costmap(site):
    """팀 결정 2026-11-17: 건넘 틈(x 0.99~1.25)은 지름길이라 막지 않는다. 커밋된 마스크에 틈 안
    keepout 칸이 없고, 서쪽 끝과 동쪽 통로를 다 막아도 5 cm costmap에서 틈으로 남북이 이어진다.
    내접 0.09로 막고 남는 열은 틈 가운데 셋(칸 중심 x 1.075, 1.125, 1.175)이다."""
    assert V7_GAP not in site.keepout_extra
    grid, occ, kept = _costmap()
    j0, j1 = _cell(0.0, 0.365)[1], _cell(0.0, 0.405 - 1e-6)[1]
    for x in (1.075, 1.125, 1.175):
        i = _cell(x, 0.0)[0]
        assert not any(kept[j][i] or grid[j][i] for j in range(j0, j1 + 1)), x
    for x in (1.025, 1.225):  # 진열대 끝 칸 옆 열은 내접 반지름 안이라 닫힌다
        i = _cell(x, 0.0)[0]
        assert any(grid[j][i] for j in range(j0, j1 + 1)), x
    assert _joined(grid, (1.125, 0.55), (1.125, 0.25), shut=BOTH_ENDS)


def test_v7_the_crossing_gap_has_no_keepout_cell_in_the_fine_mask(site):
    """1 cm 마스크(Nav2 KeepoutFilter가 읽는 것)의 틈 안 칸은 하나도 닫히지 않는다. 칸막이 둘레의
    keepout_extra는 그대로 닫혀 있다(아래 칸막이 시험)."""
    keep = _read_pgm(PKG / "maps" / "arena_v7_keepout.pgm")
    x0, y0, x1, y1 = V7_GAP
    shut = []
    for k in range(int(round((x1 - x0) / 0.01))):
        for m in range(int(round((y1 - y0) / 0.01))):
            x, y = x0 + 0.005 + 0.01 * k, y0 + 0.005 + 0.01 * m
            fi, fj = int((x - ORIGIN[0]) / 0.01), int((y - ORIGIN[1]) / 0.01)
            if keep[fj][fi]:
                shut.append((round(x, 3), round(y, 3)))
    assert shut == []


@pytest.mark.parametrize("shut, pairs", [
    # 서쪽 끝(x < 0.65)만 남김: 동쪽 통로와 건넘 틈을 막아 본다
    ([(1.56, 0.30, 1.98, 0.45), V7_GAP], [("O_IN", "Q1"), ("O_IN", "Q2"), ("O_OUT", "Q2")]),
    # 동쪽 통로(1.65~1.72)만 남김: 서쪽 끝과 건넘 틈을 막아 본다
    ([(0.0, 0.30, 0.65, 0.45), V7_GAP],
     [("O_IN", "Q1"), ("O_IN", "Q2"), ("O_OUT", "Q2"), ("M_E", "Q2")]),
])
def test_v7_north_and_south_stay_joined_by_each_way_alone(site, shut, pairs):
    """틈이 열려 있어도(2026-11-17) 끝 길 하나만으로 남북이 이어지는지 보려고 틈도 막는다."""
    grid, _, _ = _costmap()
    # 막은 틈이 정말 틈 칸을 닫는지: 틈만 지나는 길이 끊긴다
    assert not _joined(grid, (1.125, 0.55), (1.125, 0.25), shut=[*BOTH_ENDS, V7_GAP])
    for a, b in pairs:
        assert _joined(grid, site.node_pose(a), site.node_pose(b), shut), (a, b, shut)


def test_v7_work_is_joined_to_the_hall_for_its_nav_goal_way_out(site):
    """W -> M_E는 Nav2 goal로 나간다: W는 WORK_DOCK keepout
    안이지만, 커밋된 마스크가 그 도크를 나가는 곧은 선 둘레를 열어 두어 5 cm costmap에서 W 칸부터
    M_E까지 이어진다. 연 주머니는 막다른 곳이라 그 둘레의 도크는 그대로 막혀 있다."""
    grid, _, kept = _costmap()
    assert _joined(grid, site.node_pose("W"), site.node_pose("M_E"))
    for x, y in ((1.875, 0.625), (1.875, 0.375), (1.775, 0.675)):
        i, j = _cell(x, y)
        assert kept[j][i], (x, y)


# ---- 칸막이 3은 없다(2026-11-17): 칸막이도 그 둘레 keepout도 뺐다 --------
DIVIDER_3_KEEP = (0.71, 0.0, 0.86, 0.20)  # 칸막이 3이 있을 때의 둘레 차단


def test_v7_divider_3_and_its_keepout_are_gone(site):
    """충전 자리는 그대로 두고 3번 칸막이만 없앴다. 그 둘레를 막던 keepout도 뺀다: 막을 모서리가
    없다. 1 cm 지도와 마스크 모두 그 자리가 열려 있고, P-3에서 동쪽으로 y 0.20 아래로 이어진다."""
    assert DIVIDER_3_KEEP not in site.keepout_extra
    grid = arena_maps.keepout(arena_maps.FINE_M, site)
    occ = arena_maps.occupancy(arena_maps.FINE_M, site)

    def at(g, x, y):
        i, j = arena_maps.cell_of(x, y, arena_maps.FINE_M, site)
        return g[j][i]
    for x in (0.72, 0.77, 0.85):
        for y in (0.05, 0.12, 0.195):
            assert not at(grid, x, y) and not at(occ, x, y), (x, y)


def test_v7_p3_joins_the_south_yard_to_the_east_below_y_020(site):
    """5 cm costmap(내접 inflation + keepout)에서 P-3 앞(y 0.165)과 동쪽 (0.85, 0.15)이 y 0.20
    아래로만 이어진다. 칸막이 3이 있을 때는 그 둘레 keepout 때문에 끊겼다."""
    grid, _, _ = _costmap()
    assert _joined(grid, (0.64, 0.165), (0.85, 0.15), shut=[(0.0, 0.20, 1.98, 0.98)])


# ---- 칸막이 1, 2 위 끝: 칸을 나와 동쪽으로 가는 Nav2 goal 경로 ----
DIVIDER_1_KEEP = (0.19, 0.0, 0.31, 0.20)  # 칸막이 가운데에서 0.06: 5 cm 칸 둘(0.20~0.30)이 닫힌다
DIVIDER_2_KEEP = (0.45, 0.0, 0.57, 0.20)


@pytest.mark.parametrize("keep, divider_x, bays_x", [(DIVIDER_1_KEEP, 0.25, (0.12, 0.38)),
                                                     (DIVIDER_2_KEEP, 0.51, (0.38, 0.64))])
def test_v7_the_tops_of_dividers_1_and_2_are_kept_out_for_nav2(site, keep, divider_x, bays_x):
    """견고함 판 R3, R4(2026-11-15): P-1을 나온 pode3가 y 0.165에서 Q2로 가는 Nav2 goal을 받아
    5 cm 격자의 y 0.15~0.20 줄을 따라 동쪽으로 갔고, 칸막이 1 위 끝(0.24~0.26, 0.10)을 긁어 실제
    방향이 추정과 47~70도 어긋났다(주문 12건 가운데 7건에서 멈춤). 그때 칸막이 3에 있던 것과 같이
    그 둘레를 y 0.20까지 막는다(칸막이 3과 그 둘레는 2026-11-17에 뺐다). 양쪽 칸은 열려 있다."""
    assert keep in site.keepout_extra
    grid = arena_maps.keepout(arena_maps.FINE_M, site)

    def kept(x, y):
        i, j = arena_maps.cell_of(x, y, arena_maps.FINE_M, site)
        return grid[j][i]
    for x in (keep[0] + 0.01, divider_x, keep[2] - 0.01):
        assert kept(x, 0.12) and kept(x, 0.195), x
    assert not kept(divider_x, 0.205), "above it stays open"
    for x in bays_x:
        for dx in (-0.06, 0.0, 0.06):
            assert not kept(x + dx, 0.11) and not kept(x + dx, 0.19), (x, dx)


@pytest.mark.parametrize("x, divider_x, next_bay_x", [
    (0.119, 0.25, 0.38), (0.134, 0.25, 0.38), (0.366, 0.51, 0.64), (0.382, 0.51, 0.64)])
@pytest.mark.parametrize("y", [0.165, 0.195])
def test_v7_a_robot_that_left_its_bay_passes_the_next_divider_only_above_it(
        x, divider_x, next_bay_x, y):
    """칸을 나온 로봇(Gazebo 판들에서 본 자리: x는 칸 가운데에서 2 cm 안, y 0.165~0.20)의 칸은
    열려 있고 옆 칸 앞과 이어지지만, 그 사이 칸막이 위쪽(y 0.20부터)을 막으면 끊긴다: 5 cm
    costmap에서 칸막이 위 끝을 지나는 y 0.15~0.20 줄이 닫혀 있다."""
    grid, _, _ = _costmap()
    east = (next_bay_x, 0.175)
    assert _joined(grid, (x, y), east), (x, y)
    above = (divider_x - 0.10, 0.20, divider_x + 0.10, 0.98)
    assert not _joined(grid, (x, y), east, shut=[above]), (x, y)


@pytest.mark.parametrize("bay", ["P-1", "P-2", "P-3"])
@pytest.mark.parametrize("dx", [-0.05, -0.02, 0.0, 0.02, 0.05])
@pytest.mark.parametrize("y", [0.12, 0.15, 0.165, 0.195])
def test_v7_the_cell_a_robot_leaves_its_bay_from_is_not_kept_out(site, bay, dx, y):
    """칸막이 차단이 칸 안이나 칸을 나온 로봇의 출발 칸을 닫으면 Nav2
    계획이 실패한다. 칸 가운데에서 0.05 m까지 치우친 자리, y 0.12~0.20의 5 cm 칸은 keepout이
    아니다(낮은 y에서 칸막이 inflation에 걸리는 칸은 이 차단과 상관없이 전부터 있었다)."""
    _, _, kept = _costmap()
    i, j = _cell(site.node_pose(bay)[0] + dx, y)
    assert not kept[j][i], (bay, dx, y)


@pytest.mark.parametrize("a, b", [("E-3", "Q1"), ("E-3", "Q2"), ("E-1", "Q2"), ("E-3", "O_OUT"),
                                  ("E-3", "O_IN"), ("E-1", "E-3")])
def test_v7_the_south_yard_stays_open_above_the_divider_keepout(site, a, b):
    """칸막이 keepout(y < 0.20)과 남쪽 진열대 inflation 사이로(5 cm 격자에서 y 0.225, 0.275 줄)
    남쪽 마당이 이어진다."""
    grid, _, _ = _costmap()
    assert _joined(grid, site.node_pose(a), site.node_pose(b)), (a, b)
    # 동쪽으로 가는 길은 남쪽 마당으로 이어진다: 서쪽 끝을 막아도 이어진다
    if b in ("Q1", "Q2"):
        assert _joined(grid, site.node_pose(a), site.node_pose(b),
                       shut=[(0.0, 0.40, 0.65, 0.98)]), (a, b)


# ---- 가운데 통로 갈래 지도(A: 2.5 cm, B: 칸 중심 규칙). 기본 지도는 위 시험이 그대로 본다 ----
def _free_rows(grid, x, y0, y1, resolution, site):
    """x 열의 y0~y1에서 빈칸 수와 빈칸 덩어리의 (아래 경계, 위 경계)."""
    i, j0 = arena_maps.cell_of(x, y0, resolution, site)
    _, j1 = arena_maps.cell_of(x, y1, resolution, site)
    oy = arena_maps.map_origin(site)[1]
    free = [j for j in range(j0, j1 + 1) if not grid[j][i]]
    assert free == list(range(free[0], free[-1] + 1)), "빈칸은 한 덩어리"
    return (len(free), round(oy + free[0] * resolution, 4),
            round(oy + (free[-1] + 1) * resolution, 4))


@pytest.mark.parametrize("resolution, paint, count, low, high", [
    (0.05, "touch", 6, 0.45, 0.75),  # 기본: 상자가 걸친 칸은 막힘
    (0.05, "centre", 7, 0.40, 0.75),  # B
    (0.025, "touch", 13, 0.425, 0.75),  # A
    (0.025, "centre", 15, 0.40, 0.775),  # A+B
])
def test_v7_middle_corridor_free_cells_at_x_095(site, resolution, paint, count, low, high):
    """가운데 통로(진열대 사이 y 0.405~0.77)를 x 0.95에서 가로지르는 빈칸."""
    grid = arena_maps.occupancy(resolution, site, paint)
    assert _free_rows(grid, 0.95, 0.385, 0.79, resolution, site) == (count, low, high)


def test_the_default_paint_is_the_touch_rule(site):
    assert arena_maps.occupancy(0.05, site) == arena_maps.occupancy(0.05, site, "touch")
    assert arena_maps.pgm(0.05, site) == (PKG / "maps" / "arena_v7.pgm").read_bytes()
    with pytest.raises(ValueError, match="paint"):
        arena_maps.occupancy(0.05, site, "middle")


@pytest.mark.parametrize("resolution", [0.05, 0.025])
def test_the_centre_rule_paints_a_subset_of_the_touch_rule(site, resolution):
    touch = arena_maps.occupancy(resolution, site, "touch")
    centre = arena_maps.occupancy(resolution, site, "centre")
    assert all(t or not c for tr, cr in zip(touch, centre, strict=True)
               for t, c in zip(tr, cr, strict=True))
    assert centre != touch


@pytest.mark.parametrize("resolution", [0.05, 0.025])
def test_the_centre_rule_keeps_every_thin_divider(site, resolution):
    """2 cm 칸막이는 칸 중심을 하나도 덮지 않는다. 그래도 사라지지 않고 한 칸 폭 줄로 남는다."""
    from fleet_gz_sim import arena
    grid = arena_maps.occupancy(resolution, site, "centre")
    for box in arena.obstacles(site):
        i, j = arena_maps.cell_of((box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2, resolution, site)
        assert grid[j][i], box.name
    # 칸막이 1(x 0.24~0.26, y 0~0.10)은 x로 한 칸
    j = arena_maps.cell_of(0.25, 0.05, resolution, site)[1]
    ox = arena_maps.map_origin(site)[0]
    row = [i for i in range(len(grid[0]))
           if grid[j][i] and 0.15 < ox + (i + 0.5) * resolution < 0.35]
    assert len(row) == 1


@pytest.mark.parametrize("resolution, paint", [(0.025, "touch"), (0.05, "centre"),
                                               (0.025, "centre")])
def test_v7_variant_walls_leave_no_diagonal_leak_and_dividers_stay_joined(site, resolution,
                                                                           paint):
    """칸 중심 규칙은 상자마다 따로 칠해 맞닿은 상자 사이에 대각 틈을 낼 수 있다.
    v7에서는 벽 밖 빈칸이 8 이웃으로도 안쪽에 닿지 않고, 칸막이는 남쪽 벽과 이어진다."""
    from collections import deque

    from fleet_gz_sim import arena
    grid = arena_maps.occupancy(resolution, site, paint)
    h, w = len(grid), len(grid[0])
    ox, oy = arena_maps.map_origin(site)
    x0, y0, x1, y1 = site.bounds()
    seen, todo = {(0, 0)}, deque([(0, 0)])
    while todo:
        i, j = todo.popleft()
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                a, b = i + di, j + dj
                if 0 <= a < w and 0 <= b < h and not grid[b][a] and (a, b) not in seen:
                    seen.add((a, b))
                    todo.append((a, b))
    assert not [(a, b) for a, b in seen if x0 < ox + (a + 0.5) * resolution < x1
                and y0 < oy + (b + 0.5) * resolution < y1]
    wall_j = arena_maps.cell_of(0.5, -0.001, resolution, site)[1]
    for box in arena.shelves(site):
        if box.y0 > 1e-9:
            continue  # 남쪽 벽에 닿는 칸막이만
        i, j = arena_maps.cell_of((box.x0 + box.x1) / 2, box.y1 - 0.001, resolution, site)
        assert all(grid[k][i] for k in range(wall_j, j + 1)), box.name


def test_a_cell_centre_on_a_box_edge_counts_as_inside():
    # 원점 0, 0.05 칸: 칸 0 중심 0.025, 칸 1 중심 0.075
    grid = arena_maps.centre_filled([(0.025, 0.0, 0.075, 0.05)], 4, 2, 0.05, (0.0, 0.0))
    assert grid == [[True, True, False, False], [False] * 4]
    # 중심을 하나도 덮지 않는 상자: 가운데(0.0875)가 든 칸 하나
    grid = arena_maps.centre_filled([(0.08, 0.0, 0.095, 0.05)], 4, 1, 0.05, (0.0, 0.0))
    assert grid == [[False, True, False, False]]
    # 가운데가 칸 경계(0.05)에 놓여도 한 칸만
    grid = arena_maps.centre_filled([(0.026, 0.0, 0.074, 0.05)], 4, 1, 0.05, (0.0, 0.0))
    assert sum(grid[0]) == 1 and not grid[0][2] and not grid[0][3]
    # 격자 밖 상자는 칸을 칠하지 않는다
    assert arena_maps.centre_filled([(0.3, 0.0, 0.4, 0.05)], 4, 1, 0.05, (0.0, 0.0)) == [
        [False] * 4]


def test_v7_committed_nav_variant_maps_are_the_generator_output(tmp_path, site):
    from fleet_gz_sim import arena
    written = arena.generate_nav_variants(tmp_path, site)
    assert set(written) == {f"maps/arena_v7_{v}.{ext}" for v in ("a", "b", "ab")
                            for ext in ("yaml", "pgm")}
    for name in written:
        assert (tmp_path / name).read_bytes() == (PKG / name).read_bytes(), name
    for variant, resolution in (("a", 0.025), ("b", 0.05), ("ab", 0.025)):
        text = (PKG / "maps" / f"arena_v7_{variant}.yaml").read_text(encoding="utf-8")
        assert f"image: arena_v7_{variant}.pgm\n" in text
        assert f"resolution: {resolution}\n" in text
        assert "origin: [-0.15, -0.15, 0.0]\n" in text
    assert arena_maps.NAV_VARIANTS == {"a": (0.025, "touch"), "b": (0.05, "centre"),
                                       "ab": (0.025, "centre")}


def test_only_v7_has_nav_variant_maps():
    from fleet_gz_sim import arena
    assert arena.NAV_VARIANT_LAYOUTS == ("v7",)
    for layout in ("d2", "d", "c", "d2_shelf"):
        for variant in arena_maps.NAV_VARIANTS:
            assert not (PKG / "maps" / f"arena_{layout}_{variant}.yaml").exists()


def test_v7_corridor_box_keeps_one_box_in_the_middle_corridor_for_the_whole_run(site):
    """가운데 통로 재현(2026-11-17): v7_yard_mix의 상자 4와 같은 자리·크기, 한 번 놓고 판 내내 둔다.
    O_IN에 선 로봇이 놓기를 막지 않는다(sim_obstacles clearance_m 기본 0.35)."""
    from fleet_core.fleet_map.model import NAV_GOAL
    from fleet_gz_sim.obstacles import Planner, load_schedule
    mix = load_schedule(PKG / "obstacles" / "v7_yard_mix.yaml", site)
    schedule = load_schedule(PKG / "obstacles" / "v7_corridor_box.yaml", site)
    [box] = schedule.obstacles
    assert box.edge == ("O_IN", "M_E") and box.fraction == 0.4
    assert site.edge(*box.edge).mode == NAV_GOAL
    assert (box.x, box.y) == pytest.approx((0.952, 0.595))
    assert (box.x, box.y) == pytest.approx((mix.obstacles[3].x, mix.obstacles[3].y))
    assert schedule.size_m == mix.size_m
    # at_s는 시뮬 절대 시각이고 노드가 뜨기 전 항목은 건너뛴다: 노드가 늦게 떠도 되게 30 s
    assert box.at_s == 30, "sim_obstacles가 뜬 뒤"
    assert box.duration_s >= 1800
    assert schedule.repeat_every_s is None
    o_in = site.node_pose("O_IN")
    assert math.dist(o_in, (box.x, box.y)) > 0.35 + 0.1
    for start_s in (1.0, 25.0):  # 노드의 첫 tick이 늦어도(시뮬 25 s) 놓는다
        planner = Planner(schedule, start_s=start_s, max_delay_s=20.0)
        step = planner.step(box.at_s, [o_in], 0.35)
        assert [occ.obstacle for occ in step.spawn] == [box]


def test_v7_world_north_wall_inner_face_is_the_slam_measurement():
    """2026-11-17 SLAM: 북쪽 벽 안쪽 면 y 0.98. 커밋된 월드의 wall_north도 그 자리다."""
    from fleet_gz_sim import arena
    root = ET.fromstring((PKG / "worlds" / "arena_v7.sdf").read_text(encoding="utf-8"))
    boxes = {c.get("name"): c for c in root.iter("collision")}
    y = float(boxes["wall_north"].find("pose").text.split()[1])
    assert y - arena.WALL_M / 2 == pytest.approx(0.98)
