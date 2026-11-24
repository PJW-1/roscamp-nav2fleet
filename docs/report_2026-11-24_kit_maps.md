# 실물 키트 지도를 줄자 반영 도면으로 다시 만들어 비교 (2026-11-24 밤 작업)

브랜치 `feat/kit-maps-tape-2026-11-24` (origin/main b2aea4d에서). 계획: Drive "AMCL · costmap 지도를 실제 치수로 맞추기" 2판의 CLI 지시문.
실물 적용은 하지 않았다(deploy.sh, 로봇 접속 없음). launch 기본값도 그대로다(로봇 AMCL `arena_real_slam`, costmap `arena_v7_a`; Gazebo v7 기본 갈래 a).

## 결론 세 줄

1. **새 AMCL 후보 `arena_v7_fine`(1 cm 도면)은 지금 SLAM 지도보다 실제에 가깝지만, 급행이 아직 2~3 cm 좁다.** 급행 폭: 줄자 0.198, 11-23 scan 0.213(lidar 거리 치우침 약 +1.1 cm/쪽을 빼면 약 0.19), 새 지도 0.170, SLAM 0.140. 모자란 몫은 진열대 A 북쪽 면이다: 관제 `shelves`의 0.803이 lidar 높이에서는 약 0.78~0.79다(벽 0.98은 맞다). 같은 scan을 지도마다 다시 맞추면 AMCL이 옆 위치를 가르지 못하는 폭이 SLAM 6.5 cm → 새 지도 4.5 cm로 준다(면을 0.783으로 그린 시뮬 지도면 2.75 cm).
2. **costmap 1 cm(`arena_v7_a1`)는 급행 빈칸을 0.150 → 0.170으로 넓힌다(계획이 기대한 약 0.19는 진열대 면이 0.803인 한 나오지 않는다).** Gazebo gz_v7 한 판씩: 두 판 모두 8/8 출고, 격리 0, 급행 북쪽 벽 최소 여유 4.47 cm 대 4.50 cm, `collision ahead` 줄 25 대 16(거의 모두 pode3, 급행 밖)으로, 이 두 판에서는 급행의 최소 벽 여유가 비슷했다. 판이 하나씩이라 반복 주행의 차이나 충돌 가능성까지는 말하지 못한다. 1 cm 판은 `path_not_connected` 1건(Q2→W 갱신, 재시도로 풀림)으로 판정 False였다. 판 하나라 원인이 costmap인지는 가르지 못했다(아래).
3. **11-24 실물에서 확인할 것:** lidar 높이(0.125 m)에서 북쪽 벽 ~ 진열대 A 북쪽 면 거리(줄자), lidar 거리 치우침(벽에서 아는 거리에 세워 scan 한 번), 새 지도로 AMCL을 띄워 O_X 등 node에서 `amcl_pose`와 줄자 위치의 차이(체크리스트 3단계 표).

## 한눈에

| 무엇 | 결과 |
|---|---|
| 키트 지도 다시 그림 | `arena_v7`, `arena_v7_a`, `arena_v7_fine`을 `--real`로 다시 만들었다(진열대 A 서쪽 끝 0.70, 남쪽 서쪽 0.65). keepout은 바이트까지 같다. `KIT_PENDING_TAPE_2026_11_23`은 비웠다. |
| 1 cm costmap 지도 | 생성기에 갈래 `a1`(1 cm, 걸친 칸 칠하기 = `arena_v7_a`와 같은 규칙)을 더했다. 키트 `tools/real_pode/maps/arena_v7_a1`, 시뮬 `sim/fleet_gz_sim/maps/arena_v7_a1`. 같은 규칙, 같은 칸이라 그림은 `arena_v7_fine`과 바이트까지 같다. Gazebo에서는 `nav_map_variant:=a1`. |
| AMCL 후보 | 새 `arena_v7_fine`. `arena_real_slam`은 그대로 두었다. |
| 급행 폭(지도) | SLAM 0.140, 옛 costmap 0.150, 새 1 cm 0.170, 줄자 0.198 |
| 급행 폭(11-23 scan) | 0.213 (p10 0.207, p90 0.218, scan 223개). 서쪽 HALL에서 잰 안쪽 세로가 1.002(줄자 0.98)라 lidar가 쪽마다 약 1.1 cm 길게 잰다고 보면 약 0.19 |
| 지도마다 위치 다시 맞추기 | 했다(AMCL likelihood_field 점수, scan 하나씩). 아래 표 |
| Gazebo 1 cm 대 2.5 cm | 1 cm(a1): 판정 False(`path_not_connected` 1, 재시도로 풀림), 8/8, 218.1 s, 격리 0, BLOCKED_WAIT 1. 2.5 cm(a): True, 8/8, 218.9 s, 격리 0, BLOCKED_WAIT 0. 급행 벽 최소 4.47 / 4.50 cm |
| 시험 | `tools/test/test_real_pode.py` 124 passed, `sim/fleet_gz_sim` 파일마다 모두 통과(실패 0), `pc/fleet_core/test/test_layout_v7.py` 25 passed |

## 사용자가 정할 것

1. **진열대 A 북쪽 면을 0.803으로 둘지.** 줄자(급행 0.198)와 11-23 scan(약 0.19)은 lidar 높이의 면이 약 0.78~0.79라고 말한다. 0.803은 11-18 실측이다. 키트 지도는 관제 `shelves`를 그대로 그리므로, 면을 고치려면 `fleet_map_v7.yaml`의 `shelves` A를 고치거나(관제 여유 검사도 바뀐다) 키트 지도에만 다른 값을 쓰는 길 중에서 정해야 한다. 이번에는 배치(관제 지도)를 바꾸지 않았다. 시뮬은 이미 0.783(`sim_shelves`)이다.
2. **AMCL 지도: A(새 `arena_v7_fine`) / B(다시 SLAM).** scan마다 AMCL 점수식으로 맞춘 이 비교로는 A가 SLAM보다 낫다(옆 위치를 못 가르는 폭 6.5 → 4.5 cm. SLAM은 진열대 A가 띠 둘로 그려져 옆으로 미끄러진다). 면을 0.783으로 그린 시뮬 지도에서는 2.75 cm였다. AMCL 입자 필터를 돌린 결과가 아니므로 실제 위치 추정 성능은 11-24 실물 확인(결론 3)으로 본다.
3. **costmap: 2.5 cm(`arena_v7_a`) 유지 / 1 cm(`arena_v7_a1`).** Gazebo 한 판씩으로는 급행에서 차이가 없고, 1 cm 판에 `path_not_connected` 1건과 마당 BLOCKED_WAIT 1건이 있었다(원인 미확정). 바꾸려면 몇 판 더 비교한다. 실물 launch 기본은 바꾸지 않았다.
4. 갈래 이름 `a1`: `c`는 배치 이름(`arena_c`)과 겹치고 시험이 없는 갈래의 예로 쓰고 있어 피했다. 다른 이름을 원하면 바꾼다.

## 알아야 할 포인트

- 계획 표의 "1 cm면 약 0.19"는 진열대 면 0.783(시뮬 값)일 때 나온다. 키트(`--real`)는 0.803을 그리므로 1 cm 지도의 급행은 0.810~0.980 = 0.170이다.
- 키트 1 cm 지도(`arena_v7_fine`)와 costmap 1 cm 지도(`arena_v7_a1`)는 같은 그림이다. 이름을 나눈 것은 launch 인자가 각자 가리키게 하려고다.
- global costmap은 정적 층이 지도 해상도를 따르므로 1 cm 지도를 주면 global costmap도 1 cm 칸이 된다(약 228 x 128칸). 이것을 쓰는 것은 마당 Nav2 goal(NavFn)과 관제의 경로 검사다. **RPP 충돌 검사(`collision ahead`)는 local costmap(lidar만, 2.5 cm)을 보므로 지도 칸과 상관없다.** 고정 경로인 급행 주행도 global costmap을 쓰지 않는다. 그래서 Gazebo에서 급행 여유나 경고 수가 1 cm로 달라질 까닭은 작다.
- inflation(global 15.0/0.15)은 m 단위라 칸이 바뀌어도 반경은 같다.
- 11-23 기록의 SLAM 급행 폭 0.135는 같은 그림을 0.5 cm 어긋나게 읽은 값이다. 이번 `map_widths.py`는 map_server 규칙(그림 맨 아래 줄 = origin)으로 읽어 0.140(진열대 점유 0.800~0.825, 벽 0.965부터)이다.

## 지도 폭 (m, `docs/kit_maps_2026-11-24/map_widths.py`, 출력 `map_widths.txt`)

기준점이 든 빈칸 구간을 칸 경계로 읽었다. 괄호는 구간 끝. 급행 세 기둥(x 0.8, 1.1, 1.4)과 통로 두 기둥(x 0.8, 1.4)은 지도마다 같은 값이라 한 줄로 적었다.

| 어디 | 줄자 | `arena_real_slam` (AMCL 지금) | 옛 `arena_v7_a` (main b2aea4d, 로봇에 배포된 costmap, 2.5 cm) | 새 `arena_v7_fine` (AMCL 후보, 1 cm) | 새 `arena_v7_a1` (costmap 후보, 1 cm) | 새 `arena_v7_a` (2.5 cm, 참고) |
|---|---|---|---|---|---|---|
| 급행 x=0.8, 1.1, 1.4 | 0.198 | 0.140 (0.825~0.965) | 0.150 (0.825~0.975) | 0.170 (0.810~0.980) | 0.170 (0.810~0.980) | 0.150 (0.825~0.975) |
| 가운데 통로 x=0.8, 1.4 | 0.360 | 0.325 (0.425~0.750) | 0.325 (0.425~0.750) | 0.360 (0.410~0.770) | 0.360 | 0.325 |
| 남쪽 통로 x=0.8, 1.4 | 0.360 | 0.340 (0.010~0.350) | 0.350 | 0.360 | 0.360 | 0.350 |
| 가운데 칸막이 틈 y=0.385 | 0.250 | 0.225 (1.025~1.250) | 0.250 (1.000~1.250) | 0.260 (0.990~1.250) | 0.260 | 0.250 |
| 진열대 A 서쪽 끝~서쪽 벽 | 0.700 | 0.615 | 0.625 | 0.700 | 0.700 | 0.700 |
| 칸막이 서쪽 끝~서쪽 벽 | 0.650 | 0.665 | 0.675 | 0.650 | 0.650 | 0.650 |
| 진열대 A 동쪽 끝~동쪽 벽 | 0.420 | 0.365 | 0.400 | 0.420 | 0.420 | 0.400 |
| 칸막이 동쪽 끝~동쪽 벽 | 0.420 | 0.365 | 0.400 | 0.420 | 0.420 | 0.400 |
| 안쪽 세로 x=0.3 | 0.980 | 0.955 | 0.975 | 0.980 | 0.980 | 0.975 |
| 안쪽 가로 y=0.6 | 1.980 | 1.955 | 1.975 | 1.980 | 1.980 | 1.975 |

새 1 cm 지도는 급행 하나만 빼고 줄자와 1 cm 안이다(틈 0.26은 관제 도면 값 그대로, 줄자 0.25).

## 11-23 rosbag scan 겹치기 (지도마다 위치를 따로 다시 맞춤)

자료: `results/2026-11-23/robot/1123_ds_off2/pode1/bag/`(data/real-2026-11-23 브랜치, 끔 2 판 pode1, 387.6 s). 스크립트 `docs/kit_maps_2026-11-24/scan_overlay.py`, 출력 `scan_overlay.json`, 그림 `docs/kit_maps_2026-11-24/scan_overlay.png`(위: 맞춘 scan 점(빨강)과 지도, 아래: 점마다 그려진 면까지 부호 있는 거리).

- 고른 scan: 기록 위치(tf map->odom, odom->base_footprint)가 급행 곧은 구간(x 0.80~1.45, y 0.83~0.91, 동서에서 10도 안, 회전 0.15 rad/s 아래)인 scan 223개(급행 지나기 다섯 번). 로봇에서 0.10 m 안의 점(자기 몸)은 뺐다.
- 위치 다시 맞추기: AMCL을 다시 돌리지 않고, scan 하나씩 로봇 설정의 likelihood_field 점수(sigma_hit 0.05, z_hit 0.5, z_rand 0.5)가 가장 큰 자리를 기록 위치 둘레 ±10 cm, ±6도에서 찾았다. 모든 빔을 썼다(AMCL은 60개).
- "옆 위치를 못 가르는 폭": 맞춘 자리에서 y로만 옮기며 점수가 최고의 98 % 이상인 폭. 지도의 통로가 실제보다 좁으면 벽 점과 진열대 점이 둘 다 점유 띠 안에 드는 자리가 여럿이라 이 폭이 커진다.

| 지도 | 그려진 급행(진열대 면 ~ 벽 면) | 맞춘 자리 - 기록 자리, 옆(y) 가운데 (p10~p90) | 옆 위치를 못 가르는 폭(가운데) | 벽 점 - 그려진 벽 면 | 진열대 점 - 그려진 진열대 면 |
|---|---|---|---|---|---|
| `arena_real_slam` | 0.140 (0.825~0.965) | -0.6 cm (-2.5 ~ 0.0) | 6.5 cm | +1.1 cm | -6.2 cm (아래 띠에 맞음) |
| 옛 `arena_v7_a` | 0.150 (0.825~0.975) | +1.1 cm (-0.9 ~ +2.4) | 7.0 cm | +1.7 cm | -4.5 cm |
| 새 `arena_v7_fine` = `arena_v7_a1` | 0.170 (0.810~0.980) | +1.0 cm (-0.7 ~ +2.0) | 4.5 cm | +1.3 cm | -3.0 cm |
| 참고: 시뮬 `arena_v7_fine`(진열대 A 0.783) | 0.190 (0.790~0.980) | +0.9 cm (-1.0 ~ +1.5) | 2.75 cm | +1.1 cm | -1.1 cm |

지도와 상관없는 값:

| | 값 | 줄자 |
|---|---|---|
| scan 하나에서 잰 급행 폭(벽 점 y 가운데값 - 진열대 점 y 가운데값) | 0.213 (p10 0.207, p90 0.218, n 223) | 0.198 |
| 서쪽 HALL(x 0.15~0.45)에서 로봇 바로 남북 가장 가까운 점 사이 | 1.002 (가운데값, n 128. p10 0.876은 다른 물체에 걸린 scan) | 0.980 |

읽는 법:

- 벽 점은 어느 지도에서나 그려진 벽 면보다 1.1~1.7 cm 바깥에 있다. 안쪽 세로를 2.2 cm 길게 잰 것과 맞다: lidar가 짧은 거리를 쪽마다 약 1 cm 길게 잰다(추측: 한 판, 한 로봇).
- 그 치우침을 빼면 lidar 높이의 급행은 약 0.19 m다. 진열대 점은 시뮬 지도(면 0.79로 그림)에서 -1.1 cm, 즉 치우침만큼만 어긋나고, 새 키트 지도(0.81)에서는 -3.0 cm다. **진열대 A 북쪽 면은 lidar 높이에서 약 0.78~0.79**다.
- SLAM 지도는 진열대 A를 두 점유 띠(0.75~0.775, 0.800~0.825)로 그렸다. 다시 맞추면 scan의 진열대 점은 위 띠가 아니라 아래 띠(약 0.763)에 들고 벽 점은 벽 띠 안(약 0.976)에 든다. 위 띠는 lidar가 보는 면보다 약 4 cm 앞에 그려진 셈이다. 띠가 둘이라 옆으로 6.5 cm를 옮겨도 점수가 거의 같다.
- "맞춘 자리 - 기록 자리"는 로봇이 SLAM 지도로 AMCL을 돌린 기록과 비교한 것이다. 새 지도로 바꾸면 로봇이 믿는 자리가 가운데값으로 약 1 cm 북쪽으로 옮긴다(추측: 실제 AMCL은 입자 평균이고 scan 사이를 이어 간다).
- 한계: 판 하나, 로봇 하나(pode1), 급행 곧은 구간만. scan 한 바퀴(0.27 s) 동안의 움직임은 고치지 않았다(동서로 갈 때 벽과 나란한 쪽으로만 밀려 옆 거리에는 거의 들지 않는다).

## Gazebo gz_v7 (1 cm costmap 대 2.5 cm, 한 판씩)

같은 빌드(`~/gg_build/pd-kitmaps`, 이 브랜치)로 갈래만 바꿔 한 판씩 돌렸다. 도메인 61, `rviz:=none gui:=true nav_max_speed:=0.12`, `docs/baseline_sim_2026-11-19/sim_batch.sh`. 잠금 `.heavy_run.lock`을 쥐고 00:27~00:35에 돌렸다.

```bash
export SIM_BATCH_INSTALL=$HOME/gg_build/pd-kitmaps/install/setup.bash
export SIM_BATCH_ENV="WAYLAND_DISPLAY=wayland-0 GALLIUM_DRIVER=d3d12"
SIM_BATCH_ARGS="rviz:=none gui:=true nav_max_speed:=0.12 nav_map_variant:=a1" bash sim_batch.sh ~/gz_runs/kitmaps_a1 - a1:gz_v7
SIM_BATCH_ARGS="rviz:=none gui:=true nav_max_speed:=0.12 nav_map_variant:=a"  bash sim_batch.sh ~/gz_runs/kitmaps_a  - a25:gz_v7
python3 docs/kit_maps_2026-11-24/gz_metrics.py a1=~/gz_runs/kitmaps_a1/a1 a=~/gz_runs/kitmaps_a/a25
```

| 지표 | 1 cm (`a1`) | 2.5 cm (`a`, 지금 기본) |
|---|---|---|
| 판정(summary pass) | **False** (실패 검사: `path_not_connected` 1) | True |
| 출고, 경기 시간 | 8/8, 218.1 s | 8/8, 218.9 s |
| ROBOT_ISOLATED | 0 | 0 |
| BLOCKED_WAIT (관제 사건) | 1 (pode3, +178.7 s, P-1로 가던 중 LOCAL_YIELD 2 s 뒤) | 0 |
| LOCAL_YIELD 사건 | 2 | 4 |
| GOAL_RETRY | 1 (pode1 `PATH_NOT_CONNECTED`, +77.6 s) | 0 |
| RPP "collision ahead" 줄 | 25 (pode3 22, pode2 3) | 16 (pode3 16) |
| 급행 북쪽 벽 최소(발자국, Gazebo 실제 위치) | 0.0447 (pode1, x 1.553, y 0.878) | 0.0450 (pode1, x 1.224, y 0.878) |
| 진열대 A 위 면 최소(로봇 중심, 시뮬 면 0.783) | 0.0765 (pode1) | 0.0789 (pode2) |
| 최소 놓기 거리 | 0.106 | 0.101 |

- 이 두 판에서는 급행(고정 경로)의 최소 벽 여유가 비슷했다(판 하나씩이라 반복 차이는 모른다). 두 판 모두 급행 줄 y 0.878 부근에서 벽 4.5 cm, 11-23 판(벽 0.099는 출발 자리)과 같은 수준이다. 예상대로다: 급행 주행과 RPP 충돌 검사는 global costmap을 쓰지 않는다.
- `path_not_connected`: pode1이 Nav2 goal로 Q2에 닿은 순간(+77.5 s) 관제가 order를 갱신해 Q1에서 잇는 W 경로(시작 (1.849, 0.26))를 보냈고, Q2 (1.66, 0.15)에 선 로봇은 시작점에서 0.15 m보다 멀어 거절했다. 관제가 2 s 뒤 다시 보내 풀렸다. 갱신 시점의 경합으로 보이고 global costmap과 직접 이어지는 고리는 찾지 못했다(추측, 판 하나).
- BLOCKED_WAIT 1과 `collision ahead` 줄 차이는 pode3이 마지막에 P-1로 가는 마당 Nav2 goal(이것은 global costmap을 쓴다)에서 났다. 판 하나라 판 사이 흔들림과 가르지 못했다. 1 cm를 기본으로 하려면 같은 비교를 몇 판 더 돌려야 한다.

## 바꾼 파일

- `sim/fleet_gz_sim/fleet_gz_sim/arena_maps.py`: `NAV_VARIANTS`에 `"a1": (FINE_M, TOUCH)`.
- `sim/fleet_gz_sim/maps/arena_v7_a1.{yaml,pgm}`: 생성기 출력(`python3 -m fleet_gz_sim.arena --layout v7 --out .`). 다른 시뮬 파일은 바이트까지 같다.
- `sim/fleet_gz_sim/launch/gz.launch.py`, `one_pode.launch.py`: `nav_map_variant` 설명 글에 a1. 기본값은 그대로(빈 값 = v7은 a).
- `tools/real_pode/maps/arena_v7{,_a,_fine}.pgm` 다시 그림, `arena_v7_a1.{yaml,pgm}` 새로. `python3 -m fleet_gz_sim.arena --layout v7 --real --out /tmp/real_v7_kit`의 출력을 복사했다.
- 시험: `tools/test/test_real_pode.py`(`KIT_MAP_SHA256` 새 값과 `arena_v7_a1`, `KIT_PENDING_TAPE_2026_11_23` 비움, 생성 비교에 `arena_v7_a1`), `sim/fleet_gz_sim/test/test_arena_v7.py`, `test_launch_wiring.py`, `test_one_pode_wiring.py`, `test_fleet_setup.py`(갈래 목록에 a1). 느슨하게 한 단언은 없다.
- `tools/real_pode/README.md` 지도 줄.
- `docs/kit_maps_2026-11-24/`: `map_widths.py`, `map_widths.txt`, `scan_overlay.py`, `scan_overlay.json`, `scan_overlay.png`, `gz_metrics.py`.

## 시험 결과 (이 PC WSL Ubuntu + Jazzy, ROS_DOMAIN_ID 47, 빌드 `~/gg_build/pd-kitmaps`)

```
tools/test/test_real_pode.py: 124 passed in 122.60s
sim/fleet_gz_sim (파일마다): test_arena 73, test_arena_v7 114, test_dock_stand_in 2, test_dock_stub 21,
  test_drive_check 5, test_fleet_check 26, test_fleet_setup 32, test_fleet_view 41, test_isolation 31,
  test_launch_wiring 73, test_obstacles 73, test_one_pode_wiring 24, test_quaternion 6, test_ready 3,
  test_recovery_probe 15, test_sim_nav_params 44, test_stage2_plan 23, test_stand_in_battery 2,
  test_timing_agreement 5 passed (실패 0)
pc/fleet_core/test/test_layout_v7.py: 25 passed
```

갈래 이름을 처음에 `c`로 했을 때 `test_fleet_setup.py::test_an_unknown_nav_map_variant_or_a_layout_without_it_is_refused`가 실패했다(이 시험은 `c`를 없는 갈래의 예로 쓴다). 그래서 시험은 두고 이름을 `a1`로 바꿨다.

## 리뷰

Codex(gpt-6.1-terra) 한 번(b2aea4d..6750f07). 확정 셋을 고쳤다(ee5aa36, 66a3102): 폭 표의 옛 costmap 열 이름(로봇에 배포된 main b2aea4d 지도임을 밝힘), `gz_metrics.py`가 `BLOCKED` 부분 문자열을 세던 것(관제 사건 code마다 셈), `scan_overlay.py`가 지도 밖 점을 가장자리 칸으로 읽던 것(가장 먼 거리로 봄). 추측 하나(Powell 결과가 격자 후보보다 나쁠 수 있음)도 더 나은 쪽을 쓰게 고쳤다. 고친 뒤 위 표의 겹치기 값은 다시 낸 값이다(SLAM 행이 바뀌었다: 진열대 점이 늘 아래 띠에 맞는다). 고친 것의 짧은 재리뷰(6750f07..66a3102): 결함 없음.

## 하지 않은 것

- 실물 적용(배포, launch 기본값 변경).
- 진열대 A 북쪽 면 값 고치기(사용자가 정할 것 1).
- AMCL 노드를 bag으로 다시 돌리기: scan마다 같은 점수식으로 맞추는 쪽으로 대신했다(입자 필터의 시간 이음은 없다).
