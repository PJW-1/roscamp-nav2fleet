# v7 빈 바닥 쓰기 시험 결과와 결정 (2026-11-17)

사람과 나중의 AI가 읽는 기록이다. 표시: [확정]은 데이터나 코드에서 본 것, [추측]은 해석이다.
방법과 경로 계획만 잰 결과는 `worklog_corridor_costmap_ab_2026-11-17.md`에 있다.
이 문서는 Gazebo 판과 한 대 재현 판의 결과, 오늘의 결정, 남은 일을 적는다.

## 1. 요청과 결정

사용자 요청(11-17, 원문):

- "1번같은경우는 A와 B로 테스트를해봐야되겠는데 지금 실제론 공간이 남는데 너무 못쓰는느낌이야"
- "저틈을 막으면안되지 지름길인데 안되면 고쳐야되는거고"
- inflation 시험: "1번 2번 같이 진행해서테스트해"

오늘 사용자가 정한 것:

| 결정 | 내용 |
|---|---|
| 기본값 | A+inflation(aI)을 시뮬과 실제 로봇 모두의 기본값으로 쓴다. global costmap 0.025 m 걸친 칸 지도, `footprint_padding` 0.01, inflation 15.0/0.15. 오늘 main에 병합한다. main 병합 커밋은 병합 기록 참조 |
| 남쪽 진열대 틈 | 열어 둔다. 막아서 실패를 피하지 않는다 |
| 모서리 고침 | (가)만 한다: ClearEntireCostmap BT 노드 둘에 노드별 `server_timeout`(브랜치 `fix/bt-clear-timeout`, 1000 ms). (나) FollowPathGoal lookahead 단축과 (다) 후진 복구는 하지 않았다. 결정이 필요하다 |
| 버린 것 | B와 baseI. 둘 다 shelf_3 모서리에서 로봇이 멈춘 채 남았다 |

변형 정의:

| 이름 | global costmap |
|---|---|
| base | 0.05 m 지도(걸친 칸), padding 0.03, inflation 6.0/0.25 |
| A | 0.025 m 걸친 칸 지도 + 전역 `footprint_padding` 0.01 |
| B | 0.05 m 지도, 칸 중심 규칙 |
| baseI | base + inflation 15.0/0.15 |
| aI | A + inflation 15.0/0.15 |

## 2. 판 조건

- 세 대 판: 시나리오 `gz_v7_varied`, 주문 12건, 도메인 61, 한 번에 한 판. 변형마다 한 번만 돌렸다. 반복해서 잰 값이 아니다.
- 공통 인자: `layout:=v7 scenario:=gz_v7_varied obstacles:=v7_yard_mix battery_low_pct:=20 use_order_model:=true sim_dock_stub:=true rviz:=fleet gui:=true fleet_record:=true`.
- 실행 스크립트: `run_corridor.sh`(A, base, B), `run_inflation.sh`(baseI, aI). 둘 다 scratchpad `gz/`에 있다.
- A, base, B는 브랜치 `exp/corridor-costmap-ab` 680fda8의 빌드다. baseI, aI는 같은 빌드에 inflation 덮어쓰기 파일(`global_inflation_tight.yaml`, `global_padding_0p01_inflation_tight.yaml`)을 더했다.
- 한 대 판: `one_pode.launch.py` + `drive_check`, 브랜치 `exp/corridor-repro`(상자 6번 이동)와 `exp/corridor-repro-gap`(틈 12번 이동). 도메인 61, 한 번에 하나.
- 세 대 판의 위치는 로봇이 보고한 값(AMCL)이다. 실제(Gazebo) 위치가 아니다. 끝난 판은 `robots.csv`, 끝나지 않은 B와 baseI는 `fleet_record.jsonl`과 `recovery_probe.csv`에서 읽었다. 단 `safety.json`의 로봇 사이 거리는 Gazebo truth로 잰다(`scenario_monitor_node`).
- 한 대 판의 `track`, `final`, 몸-모서리 거리는 Gazebo truth(`/<robot>/sim/truth`)에서 왔다. AMCL은 `amcl_max_error_m`(오차) 계산에만 쓴다.
- 옛 판은 기본값이 바뀌기 전에 돌렸다. 그대로 재현하려면 여기 적은 커밋에서 돌린다. 변형 명령만 쓰면 지금은 새 기본값(padding 0.01, inflation 15.0/0.15) 위에서 돈다.
- B와 baseI는 판이 끝나지 않아 `summary.json`과 `safety.json`이 없다(아래 표).

## 3. 결과 표

### 3-1. 세 대 판

| 판 | 결과 | 시간 | BLOCKED 표본 | ZONE_MARKED_BLOCKED | NO_START | RPP collision ahead 줄 | 가장 가까운 두 로봇 |
|---|---|---|---|---|---|---|---|
| base | PASS 12/12 | 658.0 s | 102 | 5 | 4 | 896 | 0.187 m |
| A | PASS 12/12 | 573.7 s | 3 | 0 | 0 | 226 | 0.142 m |
| aI | PASS 12/12 | 572.7 s | 6 | 0 | 0 | 138 | 0.201 m |
| baseI | 출고 12건(565.7 s), 요약 없음. pode1이 모서리에 멈춰 자동 중지 | - | (미집계) | (미집계) | (미집계) | 308 | (safety.json 없음) |
| B | 출고 12건, 요약 없음. pode3이 모서리에 멈춘 채 약 936 s까지 두었다가 중지 | - | (미집계) | (미집계) | (미집계) | 1128 | (safety.json 없음) |

읽는 법:

- [확정] BLOCKED는 `robots.csv`의 상태 표본 수다. 막힘 사건 수가 아니다. `BLOCKED_WAIT` 이벤트는 base 6, A 1, aI 1이다.
- [확정] 가장 가까운 두 로봇은 `safety.json`의 `min_gap_m`이다. 몸 사이 틈이 아니라 로봇 중심 사이 거리다. A는 near 2건(0.159 m, 0.158 m)이 있었고 base와 aI는 0건이다.
- [확정] 표의 PASS 세 판은 위반 0이다. 판이 끝난 뒤 Nav2 컨테이너가 `Magick: abort due to signal 11`로 죽었다. 순서는 scenario PASS 뒤 launch 종료 중이다. 결과와 무관하다고는 확인하지 못했다.
- [확정] B는 12건 출고 기록이 있지만 `summary.json`이 없어 PASS가 아니다. baseI도 같다.
- [추측] 시간과 BLOCKED 수 차이를 지도 설정 탓으로 읽는 것은 한 판씩이라 근거가 약하다. 시나리오 입력과 출고 순서는 같았지만 진행 시각은 판마다 달랐다(V-010 출고: A 464.7 s, base 563.5 s).

shelf_3 동쪽 모서리를 남쪽으로 돌 때 로봇 중심과 모서리 (1.56, 0.405)의 최소 거리(`robots.csv` 표본, 몇 초 간격):

| 판 | 돈 횟수 | 중심-모서리 최소 거리 |
|---|---|---|
| base | 4 | 0.079 ~ 0.095 m |
| A | 2 | 0.087 ~ 0.108 m |
| aI | 4 | 0.115 ~ 0.150 m |

[확정] 표본 간격이 있어 실제 최소는 더 작을 수 있다. 추천 초안의 base 범위는 0.079~0.086이었는데 데이터는 0.095까지다(pode1, 584~606 s, 표본 35개). 데이터 값을 쓴다.

### 3-2. 한 대, 가운데 통로 상자(6번 이동)

상자는 통로 안 (0.952, 0.595)에 놓인 작은 장애물이다. 이동은 O_IN>M_E와 M_E>O_IN을 3번씩 번갈아 한다.

| 변형 | 결과 | 어느 길 | 이동 시간(sim s) |
|---|---|---|---|
| base | 6/6 | 6번 모두 남쪽 마당(y 최소 0.15~0.17) | 14.8 ~ 18.6 |
| A | 1번 성공, 2번 NAV_ABORTED(2번에서 중단) | 1번 남쪽 마당. 2번은 통로에 들어가 x 1.155까지 갔다가 되돌아나와 모서리 (1.613, 0.387)에서 멈춤 | 1번 49.4, 2번 34.8(중단) |
| baseI | 6/6 | 6번 모두 남쪽 마당 | 13.5 ~ 28.2 |
| aI(2차) | 6/6 | 2번째와 4번째(M_E>O_IN)만 통로에 들어갔다가 남쪽 마당으로 나옴. 나머지 4번은 처음부터 남쪽 마당 | 16.1 ~ 116.2. 2번째 98.0, 4번째 116.2, 6번째 37.4, 나머지 16~24 |

- [확정] 4변형 모두 통로를 지나 상자 옆을 통과한 이동은 0번이다(`through_corridor`가 전부 false).
- [확정] aI 2번째와 4번째 이동은 통로 안(x 1.10~1.58, y 0.54~0.58)에서 왔다 갔다 하다 끝내 동쪽으로 나와 남쪽 마당으로 돌았다. 오래 걸린 값은 이 두 번이다. 추천 초안의 "x 1.14~1.57, 60~80 s"는 이동 전체 시간과 좌표만으로는 구간을 가를 수 없어 이동 시간만 적는다. 초안의 A "x 1.22에서 되돌아감"은 데이터 최소 x가 1.155라서 고쳤다.
- [확정] aI 첫 시도(`corr1_aI`)는 Nav2 시작 실패로 유효한 이동 결과가 없다. 위 표는 2차(`corr1_aI_r2`)다.
- [확정] aI 2번째와 4번째 이동은 모서리를 x 약 1.70(중심-모서리 약 0.14 m)으로 돌았다.

### 3-3. 한 대, 남쪽 진열대 틈(12번 이동)

틈을 연 지도에서 M_E>GAP_N, GAP_N>GAP_S, GAP_S>GAP_N, GAP_N>M_E를 3번 돈다.

| 변형 | 결과 | 틈 통과 | 몸-모서리 최소(기록된 계산값) |
|---|---|---|---|
| base | 12/12 | 6번 | 0.046 m |
| aI(2차) | 12/12 | 6번 | 0.056 m |

- [확정] 틈 통과 6번에서 몸-모서리는 base 0.046~0.061, aI 0.056~0.063이다. `gap_corner_body_min_m`은 footprint와 위치 표본으로 계산한 값이라 연속 시간의 실제 최소를 보장하지 않는다.
- [확정] aI 첫 시도(`gap_open_aI`)는 Nav2가 뜨기 전에 목표를 보내 벽시계 0.11 s(sim 0.07 s) 만에 NAV_ABORTED가 났고 유효한 이동 결과가 없다. 2차를 쓴다.

### 3-4. 갈린 장면(남쪽 통로 서쪽에 선 pode3 옆 지나가기)

- [확정] A: pode2가 pode3 옆(남쪽 통로 서쪽, 약 (0.64, 0.12))을 423.9~428.2 s에 지났다. RECOVERING 표본이 하나 있었다.
- [확정] base: pode2가 같은 장면에서 425.8~515.1 s 사이에 BLOCKED 표본 66개를 남겼다(x 0.36~0.54, y 0.19~0.30). 이어서 pode1도 543.9~572.4 s에 (0.48~0.52, 0.28~0.30)에서 BLOCKED 36개였다. 둘 다 막힘 표시 뒤 가운데 통로로 돌아갔다.
- [추측] base의 0.05 칸과 padding 0.03에서는 서 있는 로봇 옆이 막히고 A에서는 열린다. 경로 계획만 잰 결과와 같은 방향이다. 그러나 판마다 한 번이고 `/plan` 기록이 없다. 지도 해상도와 padding이 같이 바뀌어 효과를 가를 수 없다. 원인은 확정하지 못했다.

## 4. shelf_3 동쪽 모서리 정지

멈춘 경우는 셋이다.

| 경우 | 변형 | 멈춘 곳(출처) | 비고 |
|---|---|---|---|
| 세 대 판 B | B | pode3, 약 (1.59, 0.37), 보고(AMCL) 위치. 111~127 s에 도착 | 끝(약 936 s)까지 그 자리. 출고 12건 |
| 세 대 판 baseI | baseI | pode1, (1.638, 0.402) 부근, 보고(AMCL) 위치. 126.7 s 이후 | 542 s NAV_ABORTED, ROBOT_ISOLATED |
| 한 대 A | A | (1.613, 0.387), Gazebo truth `final` | 2번째 이동 NAV_ABORTED |

[확정]

- 통로 폭: 진열대 끝 x 1.56과 WORK_DOCK keepout 경계 x 1.72 사이 0.16 m다. 로봇 반폭은 0.056 m다. keepout은 물리 벽이 아니다.
- 한 대 A 로그(`corr1_a.launch.log` 294~308줄): collision ahead, NavFn이 (1.61, 0.36)에서 경로 없음, controller patience, Path is empty, Goal failed 순서다.
- B: 나중의 truth는 (1.602, 0.394)에 서쪽을 향하고, 보고 yaw는 약 84도 틀렸다. baseI: 복구 시작 때(127.8 s) truth와 AMCL yaw 차이 43도였다. 이 truth-보고 값은 각각 한 표본이다.
- ClearEntireCostmap(global) 시간 초과 경고: B 102번, baseI 100번, base 15번, aI 2번, A 0번, 한 대 판 6개 launch 로그 모두 0번. 기본 `server_timeout`은 20 ms다(`nav2_params.yaml:76`).
- 일반 `nav_goal` BT에는 Spin과 BackUp이 없다. FSM의 BackUp은 앞의 로봇에 양보할 때만 조건부로 쓴다. 정적 선반에 눌린 로봇은 후진으로 빠지지 않는다.
- FollowPathGoal은 RPP다: lookahead_time 0.8, 속도 비례 0.10~0.25 m, rotate_to_heading 0.25 rad (`avoidance.yaml:26-56`).

[추측]

- 원인을 가르지 못한다. 계획(NavFn이 모서리에 붙은 길을 그림)과 제어(RPP가 모서리를 안쪽으로 잘라 접촉)를 구별하려면 그 순간의 계획 좌표와 시각이 붙은 자세가 필요한데 남기지 않았다.
- 모서리를 L/2(약 0.08 m)만큼 잘라 닿았다는 설명은 근거가 없다. 입증하지 못했다.
- clear 시간 초과가 정지의 원인이라는 증거도 없다. clear가 성공해도 정적 선반은 지워지지 않고 로봇도 움직이지 않는다. 시간 초과를 늘리는 것은 모서리 고침이 아니다.
- 멈춘 두 판에서 clear 시간 초과가 약 100번이었던 것은 로봇이 멈춰 있는 동안 복구 루프가 계속 돈 결과일 수 있다. 원인이라고 읽지 않는다.
- "B와 baseI가 모서리 비용을 낮췄다"는 설명(B는 끝 칸이 빠짐, baseI는 inflation이 좁음)도 가설이다. 한 번씩이라 증거가 약하다.
- 한 번에 한 판이므로 aI가 모서리에 안전하다고 결론내지 않는다.

## 5. 상자 옆 통로

- [확정] 어느 변형도 상자 옆을 지나가지 못했다(3-2).
- [확정] base와 baseI는 통로에 들어가지 않고 남쪽 마당으로만 갔다. A와 aI는 통로에 들어갔다가 상자 앞 약 0.15~0.2 m(x 1.10~1.16)에서 되돌아왔다.
- [추측] 원인은 모른다. 계획이 상자 쪽 길을 막혔다고 본 것인지, controller가 되돌렸는지, 둘을 가를 기록이 없다. 상자 양쪽의 남는 폭(약 0.15와 0.135 m)은 계산값이고 이 시험에서 독립 확인하지 못했다.
- [추측] 지도 설정만으로는 로봇이 그 공간을 쓰게 되지 않는다. 한 판씩이므로 설정 탓이 아니라고도 단정하지 않는다.
- 세 대 판의 상자 시간대(371.7~432.8 s 등)는 분석 스크립트가 로그 시각에서 60 s를 빼서 만든 값이라 `robots.csv` 시간축과 같다고 확인되지 않았다. 이 문서에는 쓰지 않는다. 통로 안 표본 그룹 수는 A 2, base 4, aI 4다.

## 6. 병합본 확인 판(main에 올리기 전)

- 브랜치 `merge/drive-2026-11-17`: aI 기본값(c518d36), 틈 열기, BLOCKED 히스테리시스(fleet, pode), Clear 대기 한도 1000 ms를 합친 빌드다. 이 넷을 한 판에 같이 돌린 것은 처음이다. 빌드는 12:51(d7a3cc5까지). 뒤의 커밋은 문서와 시험만 바꿨다.
- 인자: 2절의 공통 인자 그대로. `nav_map_variant`와 `nav_overrides`는 주지 않았다(기본값 = aI). 도메인 61, 한 번.
- 결과 [확정, summary.json, safety.json, robots.csv, events.csv, 로그]:

| 항목 | 병합본 | 참고: aI 판 |
|---|---|---|
| 결과 | PASS 12/12, 위반 0 | PASS 12/12 |
| 걸린 시간 | 572.5 s | 572.7 s |
| BLOCKED 표본 | 38(모두 pode2) | 6 |
| BLOCKED_WAIT / ZONE_MARKED_BLOCKED / NO_START | 2 / 2 / 2 | 1 / 0 / 0 |
| RPP "collision ahead" | 551줄 | 138줄 |
| ClearEntireCostmap 시간 초과 경고 | 0 | 2 |
| safety.json near / min_gap_m(truth 중심 거리) | 0 / 0.166 m | 0 / 0.201 m |

- BLOCKED 38표본은 한 장면이다. pode2가 428.99~458.70 s에 (0.53~0.63, 0.28~0.31)에서 Q2로 가다 막혔다. 그때 pode3은 (0.619, 0.129)에 IDLE로 서 있었다. base 판에서 막혔던 장면과 같은 자리다. 막힘 표시(E-2->E-3, here->Q2) 뒤 다른 길로 가서 판은 PASS했다.
- 판이 PASS로 끝난 뒤 launch를 내릴 때 Nav2 컨테이너가 SIGINT에 5 s 안에 안 꺼져 SIGTERM, SIGKILL로 내려갔다. 그래서 실행 스크립트의 rc가 1이다.
- [추측] aI 판과 다른 숫자(BLOCKED, collision 줄)가 합친 변경 때문인지 판마다 다른 것인지는 한 판으로 가를 수 없다. aI 판에서는 이 장면에서 pode2가 pode3 옆을 지나갔다.

## 7. 남은 일

| 일 | 상태 |
|---|---|
| 모서리 (나) lookahead 단축, (다) 후진 복구 | 미결정. (다)는 사용자가 뺀 BackUp을 다시 쓰는 일이라 결정이 필요하다 |
| 모서리 진단 | 계획 좌표(`/plan`)와 모서리를 도는 시각의 자세를 기록해 계획과 제어를 가른다 |
| drive_check | `exp/corridor-repro`의 `drive_check.py:280-283`은 `move_to` 서버와 IDLE만 기다린다. Nav2 lifecycle이 active가 되는 것은 기다리지 않는다. scratchpad 실행 스크립트(`run_repro_rerun2.sh`)가 따로 기다려 돌아갔다. 코드 고침은 하지 않았다 |
| 반복 | 세 대 판과 한 대 판은 한 번씩이다. 반복해서 확인한다. 병합본 판의 남쪽 통로 서쪽 막힘(6절)도 함께 본다 |
| 실제 아레나 | padding 0.01은 실제 위치 오차를 모르는 채 정한 값이다. 실제 로봇 위치 오차를 잰 뒤 다시 본다 |
| 지그재그 | Smac 2D와 smoother 작업은 위 일을 끝낸 뒤에 한다 |

## 8. 참조 데이터(임시 폴더, 사라질 수 있다)

- 세 대 판: scratchpad `gz/corr_a`, `corr_base`, `corr_b`, `corr_baseI`, `corr_aI`(robots.csv, events.csv, safety.json, summary.json, fleet_record.jsonl)와 `gz/corr_*.log`. 분석 `gz/analyze_game.py`.
- 한 대 판: `gz/repro/`의 `corr1_base.json`, `corr1_a.json`, `corr1_baseI.json`, `corr1_aI_r2.json`, `gap_open_base.json`, `gap_open_aI_r2.json`.
- 경로 계획만 잰 결과(headless): `corridor_ab/oout_planner.log`. 상자 없이 O_IN->M_E 길이 base 1.149, baseI 1.205, A 1.165, aI 1.189 m(직선 1.157)다.
- 병합본 판: `gz/final_main/`와 `gz/final_main.log`, 시험 로그 `gz/final/`.
- 진단: `b_stuck_diag2.md`, `corner_stuck_pattern.md`, `corner_analysis/`.
