# v7 진열대 끝 줄자 실측 반영 (2026-11-23)

브랜치 `fix/v7-tape-2026-11-23` (origin/main 9cbd413에서). 관제(`fleet_map_v7.yaml`)와 시뮬(월드, 시뮬 지도)만 바꿨다.
실물 키트 지도(`tools/real_pode/maps/`: AMCL `arena_real_slam`, costmap `arena_v7_a`, keepout, fine)는 건드리지 않았다.
사용자가 11-24 스캔을 본 뒤 정한다.

## 무엇을, 왜

사용자가 2026-11-23 16:57에 줄자로 잰 값이다(x는 서쪽 벽 안쪽 면에서). 그림과 2 cm 넘게 다른 것만 고쳤다.

| 무엇 | 그림(전) | 실측(후) |
|---|---|---|
| 북쪽 긴 진열대 A 서쪽 끝 | x 0.63 | **x 0.70** (길이 0.93 -> 0.86) |
| 남쪽 서쪽 진열대 서쪽 끝 | x 0.68 | **x 0.65** (길이 0.31 -> 0.34) |

나머지는 2 cm 안이라 그대로 뒀다: 가운데 틈 0.26(실측 0.25), 통로 0.365(0.36), 급행 0.197(0.198), 동쪽 끝 1.56.
`shelves`와 `sim_shelves` 둘 다 바꿨고 y 값은 그대로다(A: 실측 0.77~0.803, 시뮬 0.77~0.783).

### HALL 띠: 0.63 -> 0.645 (사용자 확인 필요)

진열대 A 서쪽의 HALL 띠 `[0.50, 0.77, 0.63, 0.803]`은 진열대 끝에 붙어 있었다.

- 0.70까지 늘리면 띠 모서리 (0.70, 0.803)이 O_X (0.71, 0.868)에서 0.066이다. O_X에 선 로봇 몸(0.083)이 HALL에 든다.
  `test_v7_outbound_waits_at_the_west_end_of_the_express_too`가 실패한다. 이 시험은 O_X에서 HALL까지 0.09 넘게 떨어져 있어야 한다고 본다.
- 0.63에 두면 x 0.63~0.70 바닥이 어느 구역도 아니다. 그 안에서 노드 밖으로 보고된 로봇은 배치(`placement.py:24`)와
  새 길 찾기(`goals.py:64`)에서 거절된다(Codex가 짚었고 코드에서 확인했다).
- 고른 값은 **0.645**다(진열대 끝 0.70 - 몸 반폭 0.056). 로봇 중심이 진열대에 닿지 않고 설 수 있는 바닥은 HALL이다.
  O_X에서 HALL까지는 0.098 -> 0.092가 됐다. 그래도 몸 밖이고(> 0.085), 놓기 거리 0.135 안이다. 그래서 O_X의 동작은
  전과 같다: 오기 전에 쥐었던 HALL은 기다리는 동안 그대로 쥔다. 0.645~0.70 띠는 진열대 자리처럼 어느 구역도 아니다.
- 노드와 길은 옮기지 않았다. 지나갈 수 있던 틈도 닫지 않았다.

## 여유가 바뀐 곳 (관제 노드와 길에서 진열대 셋까지, 전 -> 후, m)

`fleet_core` 지도에서 노드 점과 곧은 선(고정 경로는 점)을 쟀다. 5 mm 넘게 줄어든 것만 적는다.

| 노드나 길 | 전 | 후 | 메모 |
|---|---|---|---|
| E-3 <-> O_OUT (Nav2 goal) | 0.122 | 0.103 | 새 남쪽 진열대 서쪽 아래 모서리 (0.65, 0.365) |
| E-1/E-2/E-3 사이, P-3 <-> E-3, E-3 <-> Q2 | 0.128~0.131 | 0.125 | E-3 (0.64, 0.24)가 이제 진열대 서쪽 끝(0.65)에서 1 cm 서쪽 아래다 |
| E-2 <-> O_OUT, E-2 <-> E-1 | 0.325 | 0.298 | |
| O_IN -> E-3 곧은 선(이음 없음) | 0.082 | 0.055 | 원래 잇지 않는 쌍이다. 잇지 않는 까닭이 더 강해졌다 |

늘어난 곳: O_IN 0.202 -> 0.255, out_in(O_IN -> O) 0.182 -> 0.247, O 0.351 -> 0.420, O_OUT 0.474 -> 0.517.
O_X는 진열대 A 위 면에서 0.065 그대로다(서쪽 끝에서 1 cm 동쪽이라 아직 위 면 바로 북쪽이다).
서쪽 기둥(서쪽 벽과 남쪽 서쪽 진열대 사이)은 0.68 -> 0.65로 3 cm 좁아졌지만 여전히 0.65 m 폭이다.

## 바꾼 파일

- `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml`: `shelves`, `sim_shelves`, HALL 띠, 머리 주석과 그림, O_IN과 O_X와 M_E 주석.
- `sim/fleet_gz_sim/worlds/arena_v7.sdf`, `sim/fleet_gz_sim/maps/arena_v7{,_a,_b,_ab,_fine}.pgm`: 생성기로 다시 만들었다.
  `cd sim/fleet_gz_sim && PYTHONPATH=../../pc/fleet_core:. python3 -m fleet_gz_sim.arena --layout v7 --out .`
  (`--real`은 돌리지 않았다). keepout과 yaml들은 바이트까지 같다(HALL은 shared라 keepout에 들지 않는다).
- 시험:
  - `pc/fleet_core/test/test_layout_v7.py`: `SHELVES`, `SIM_SHELF_A`, 진열대 A 길이 0.86, HALL 띠 0.645(진열대 끝에서 몸 반폭 안),
    진열대 자리 확인 점 (0.64, 0.79) -> (0.71, 0.79)(0.64는 이제 바닥), 주석 수.
  - `sim/fleet_gz_sim/test/test_arena_v7.py`: `BOXES`, 서쪽 끝을 막는 사각형 0.68 -> 0.65.
  - `sim/fleet_gz_sim/test/test_fleet_setup.py`: 진열대 A 서쪽 끝면 상자 `0.6,0.74,0.66,0.813` -> `0.67,0.74,0.73,0.813`.
  - `sim/fake_robots/test/test_world_local_yield.py`: hall_obstacle v7 씨드 6 -> 11. 새 배치에서 씨드 6은 두 대가 마주치지 않는다.
    씨드 1~10에는 이 판이 없고 11, 12, 13에서 나온다. 단언은 그대로다.
  - `tools/test/test_real_pode.py`: 키트 지도 생성 비교. 키트 파일 가운데 `arena_v7.pgm`, `arena_v7_a.pgm`, `arena_v7_fine.pgm`
    (`KIT_PENDING_TAPE_2026_11_23`)은 옛 진열대 끝(A 0.63, 남쪽 서쪽 0.68)으로 다시 그린 것과 바이트까지 같아야 하고,
    지금 진열대로 그린 것과는 달라야 한다. 나머지 키트 파일은 전처럼 지금 생성 결과와 같아야 한다.
    키트 지도를 다시 만들면 목록을 비운다(그러지 않으면 시험이 실패한다).
  - `learning/store_arena/test_arena_spec.py`: yaml 글자 바꾸기 시험의 찾는 글과 기대값. 이 PC에는 torch가 없어 **돌리지 못했다**.

## 시험 결과 (이 PC, Ubuntu + Jazzy, 파일마다 pytest, 최종 f6d4e34 기준 + 키트 시험 고침)

| 묶음 | 이 브랜치 | main 9cbd413 |
|---|---|---|
| `pc/fleet_core` | 1230 passed, 7 xfailed | 같은 수(시험 수정 전후 개수 같음) |
| `sim/fleet_gz_sim` | 489 passed, 24 failed, 93 skipped | 실패 24는 main에서도 같다: `test_dock_stub` 10, `test_launch_wiring` 1, `test_sim_nav_params` 11, `test_stand_in_battery` 2(이 PC 환경) |
| `sim/fake_robots` | 993 passed, 11 skipped, 30 xfailed | 씨드를 바꾸기 전 `test_world_local_yield` 1 실패, main은 12 passed |
| `tools/test/test_real_pode.py` + `test_nav_bench.py` | 149 passed, 2 skipped(키트 시험 고친 뒤 `-k kit` 4 passed) | |
| `learning/store_arena` | 돌리지 못함(torch 없음) | |

## Gazebo 전후 (gz_v7 한 판씩, 도메인 61, GUI)

명령(이 PC):

```bash
S=<scratchpad>; W=<worktree>
# 관제와 시뮬 두 패키지만 오버레이로(전: origin/main을 git archive로 푼 것, 후: 이 브랜치 916e6ea)
colcon build --base-paths $SRC/pc/fleet_core $SRC/sim/fleet_gz_sim --packages-select fleet_core fleet_gz_sim \
  --build-base $S/ovl_X/build --install-base $S/ovl_X/install
export SIM_BATCH_INSTALL=$HOME/ros2camp/podedrive_ws/install/setup.bash SIM_BATCH_ARGS="rviz:=none gui:=true nav_max_speed:=0.12"
bash $W/docs/baseline_sim_2026-11-19/sim_batch.sh $S/gz_runs/base $S/ovl_base/install/setup.bash v7:gz_v7
bash $W/docs/baseline_sim_2026-11-19/sim_batch.sh $S/gz_runs/new2 $S/ovl_new2/install/setup.bash v7:gz_v7
```

나머지 패키지(fake_robots, fleet_manager, Nav2 설정 등)는 두 판 모두 같은 `~/ros2camp/podedrive_ws/install`이다.
여유는 `truth.csv`(Gazebo 실제 위치)의 로봇 중심에서 그 판의 시뮬 진열대 상자까지다.

| 지표 | 전(main) | 후(이 브랜치) |
|---|---|---|
| 통과, 출고 | True, 8/8 | True, 8/8 |
| 경기 시간 | 221.0 s | 217.9 s |
| ROBOT_ISOLATED / BLOCKED 사건 | 0 / 0 | 0 / 0 |
| 놓기 early / near, 최소 놓기 거리 | 0 / 0, 0.110 | 0 / 0, 0.114 |
| 로봇 사이 최소 | 0.206 | 0.197 |
| LOCAL_YIELD | 2 (pode2, 1 s) | 2 |
| RPP "collision ahead" 경고 줄 | 19 | 18 |
| 진열대 A 최소 중심 거리(서쪽 끝 x < 0.85 쪽) | 0.076 (pode1, (0.691, 0.859), 급행 줄) | 0.075 (pode3, (0.759, 0.858), 급행 줄) |
| 남쪽 서쪽 진열대 최소(서쪽 끝 x < 0.80 쪽) | 0.137 (pode2, (0.607, 0.249)) | 0.126 (pode2, (0.605, 0.247)) |
| 남쪽 동쪽 진열대 최소 | 0.157 | 0.163 |
| 벽 최소 | 0.100 (출발 자리) | 0.099 (급행 들머리) |

- 진열대 A는 두 판 모두 급행 줄 위에서 가장 가깝다(면에서 0.075~0.076, 바퀴 바깥 끝 0.0766과 거의 같다: 11-21부터 알려진 값).
  새 서쪽 끝 모서리 둘레에서 더 가까워진 곳은 없다.
- 남쪽 서쪽 진열대는 E-3 둘레에서 1.1 cm 가까워졌다(진열대가 3 cm 서쪽으로 늘어난 만큼). 몸 반경 0.083보다는 4 cm 넘게 멀다.
- HALL을 0.63에 둔 중간 판(94825ea)도 한 판 돌렸다: True 8/8, 217.6 s, 격리 0, 진열대 A 0.077, 남쪽 서쪽 0.127.
- 한 판씩이라 시간 차이(3 s)는 판 사이 흔들림 안이다.

## 리뷰

Codex(gpt-6.1-terra) 세 번. 1차: HALL 띠를 0.63에 두면 바닥이 구역 밖(위 HALL 0.645로 고침), 시험 주석 0.082(고침).
2차: xfail이 생성 오류도 덮음(대기 목록 비교로 바꿈), O_X 주석 수(고침).
3차: 대기 그림 셋은 다르기만 하면 통과함(옛 진열대 끝으로 다시 그린 것과 바이트 비교로 고침). 이 고침의 짧은 리뷰는 밀고 난 뒤 돈다.

## 하지 않은 것

- 실물 키트 지도(`tools/real_pode/maps/`): 사용자가 11-24 스캔을 본 뒤 정한다. 그때 `--real`로 다시 만들고
  `KIT_MAP_SHA256`과 `KIT_PENDING_TAPE_2026_11_23`을 고친다.
- `learning/store_arena` 시험: torch가 없어 이 PC에서 돌리지 못했다.
- `docs/baseline_sim_2026-11-19/express_metrics.py`의 `SHELF_A`(0.63): 11-19 기록을 다시 세는 스크립트라 그대로 뒀다.
