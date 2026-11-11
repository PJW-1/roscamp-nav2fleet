# 어디서 복사했나

이 레포의 첫 커밋은 `PJW-1/roscamp-freefloor`의 `main` 커밋 `9375a80`에서 아래 경로를 **그대로 복사**한 것이다(fork가 아니다). 그 뒤의 커밋에서 자유바닥·레인 전용 코드를 덜어 내고 새 교통 층을 만든다.

| 이 레포의 경로 | 원래 경로 (`roscamp-freefloor` 9375a80) | 비고 |
|---|---|---|
| `interfaces/fleet_interfaces` | 같음 | |
| `pc/fleet_core`, `pc/fleet_manager`, `pc/central` | 같음 | 첫 커밋에는 `traffic/floor/*`와 레인 전용 파일도 들어 있고, 다음 커밋에서 지운다 |
| `pinky/pinky_fsm`, `pinky/pinky_nav_config`, `pinky/pinky_dock` | 같음 | |
| `sim/fake_robots`, `sim/fleet_gz_sim` | 같음 | 옛 지도·site·시나리오는 다음 커밋에서 지운다 |
| `tools/` | 같음 | `replay.py`와 시험 틀이 쓰는 것만 남긴다 |
| `arm/`, `bringup/` | 같음 | |
| `docs/핑키_계약.md`, `docs/실측_2026-11-10.md`, `docs/GG_공통설계.md` | 같음 | |
| `.gitattributes`, `.gitignore`, `ruff.toml`, `iponic.repos`, `config/`, `scripts/` | 같음 | 레포 설정, 벤더 의존(pinky_pro), 로봇 목록, 인터페이스 점검 |

가져오지 않은 것: `.github/`(CI), `README.md`(새로 씀), `docs/`의 작업 기록들과 `docs/freefloor/`, `docs/관제_동작.md`, `docs/개선점_2026-11-06.md`.

옛 작업 기록은 원래 레포에 있다: <https://github.com/PJW-1/roscamp-freefloor/tree/9375a80/docs>

확인하는 법: 첫 커밋의 디렉터리 tree 해시가 원래 커밋과 같다.

```
git rev-parse <첫 커밋>:pc            # 이 레포
git rev-parse 9375a80:pc              # roscamp-freefloor
```

## 지운 것

1단계 뼈대에서 `skeleton` 브랜치의 두 커밋으로 지웠다. 지운 파일은 첫 커밋(`1da1e61`)에 그대로 있다(`git show 1da1e61:<경로>`).

| 커밋 | 지운 것 | 이유 |
|---|---|---|
| `ebef4bf` 자유바닥 | `pc/fleet_core/fleet_core/traffic/floor/`(칸 잠금, 칸 길 찾기, 지름길, 교차 상자, 통로 비우기), `data/floor_map*.yaml`, `data/feature_floor_map.yaml`, `data/*.pgm`, `data/fleet_params_real_arena.yaml`, 자유바닥 전용 파라미터, 심판과 안전 측정의 자유바닥 갈래, `tools/floor_view.py`, `tools/compare_floor.py`, `tools/scenarios/`, 자유바닥 시험 | Nav2가 길을 짜므로 필요 없다 |
| `9c90d6d` 레인 전용 | `traffic/controller.py`(레인 제어기), `lane_map.py`, `lane_path.py`, `blocking.py`, `detour.py`, `off_lane.py`, `release_floor.py`, `data/lane_map.yaml`, `sim/fake_robots/maps/`(옛 지도 전부), 옛 지도에 묶인 시나리오(`feature_*`, `warehouse_rush`, `off_lane`), `tools/model_check.py`, `tools/pressure_sweep.py`, `tools/convert_grid_map.py`, `tools/baseline.py`, 시험 지도(`test/grid_map.py`, `test/data/grid_cells.yaml`), 레인 전용 파라미터, 그 코드의 시험 | 새 그래프·구역 층이 대신한다 |

남긴 시나리오 8개(`basic_orders` 등)는 고장 종류와 형식의 본보기다. 지운 레인 지도의 지점 이름(`P-6` 등)을 쓰므로 새 지도에 맞춰 고쳐야 돈다. `sim/fleet_gz_sim`은 다른 트랙이 따로 정리한다.
