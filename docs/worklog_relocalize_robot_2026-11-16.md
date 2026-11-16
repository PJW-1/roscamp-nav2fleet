# 작업 기록: 위치 다시 찾기(Relocalize) 로봇 쪽 (2026-11-16)

브랜치 `feat/relocalize-robot`(기반 `feat/relocalize` 09a28d5). 관제 쪽은 다른 에이전트가 맡는다.

## 왜

Gazebo에서 pode1 몸체를 0.5 m 순간이동시키자 AMCL이 0.65 m 어긋난 채 남았다. 로봇은 도크 W 66 cm 앞에서 "도착"했고 도킹이 세 번 실패해 관제가 격리했다. 지금은 사람이 다시 넣어야 한다. 관제가 격리하기 전에 `/{pode}/relocalize`를 부르면 로봇이 선 채로 위치를 다시 찾는다.

## 계약

`interfaces/fleet_interfaces/action/Relocalize.action`(d2e0906 → d8d3123 → 9a746b5 → 09a28d5). 작업 중 세 번 바뀌어 세 번 rebase했다. 마지막 판에 맞췄다. `.action`과 `contract.yaml`은 고치지 않았다.

## 바꾼 것

| 파일 | 내용 |
|---|---|
| `pode/pode_fsm/pode_fsm/relocalize.py` (새 파일) | ROS 없는 순수 로직. `distance_field`(상한까지 정확한 유클리드 거리 변환, numpy), `scan_points`, `scan_match`, `std_from_covariance`(계약대로 sqrt(max(var_x, var_y))), `is_good`, `failure_reason`, `is_busy`, 단계 기계 `Relocalizer` |
| `pode/pode_fsm/pode_fsm/relocalize_ros.py` (새 파일) | 액션 서버 `RelocalizeServer`, Nav2 Spin 실행기 `SpinRunner`(기존 `GoalRunner` 재사용) |
| `pode/pode_fsm/pode_fsm/node.py` | 서버 생성(이름은 contract 키 `relocalize`), 도는 동안 `LOCALIZING` 보고, 정지 때 회전 취소 |
| `pode/pode_fsm/pode_fsm/server_ros.py` | 도는 동안 move/dock/retreat goal 거절(`refuse`). 수락은 노드 락 안에서 예약(`pending`)하고, relocalize의 바쁨 검사가 그 예약을 본다. 그래도 틈에 들어온 goal은 `NAV_ABORTED`로 끝낸다 |
| `pode/pode_fsm/pode_fsm/params.py`, `params_ros.py` | `RelocalizeParams`(줄마다 한국어 주석), ROS 파라미터 `relocalize_*` |
| `pode/pode_fsm/package.xml` | `std_srvs`, `python3-numpy` |
| `pode/pode_fsm/test/test_relocalize.py` (새 파일) | 순수 로직 56개 |
| `pode/pode_fsm/test/test_ros_relocalize.py` (새 파일) | FakeAmcl(지도, 스캔, amcl_pose, 두 서비스, spin)로 실제 노드를 시험 14개 |
| `pode/pode_nav_config/params/nav2_params.yaml` | behavior_server에 `nav2_behaviors::Spin`(`spin`) 추가, `max_rotational_vel` 1.0 → 0.5 → 0.6(리뷰 반영), AMCL `recovery_alpha_slow` 0.001, `recovery_alpha_fast` 0.1 |
| `pode/pode_nav_config/test/test_nav_config.py` | spin 플러그인을 기대하게 바꿈, recovery alpha 시험 추가. **맡은 파일 목록 밖**이지만 yaml을 바꾸면 깨지는 시험이라 함께 고쳤다 |

`sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py`는 고치지 않았다. 시뮬은 실물 yaml에 `SIM_AMCL`만 덮으므로 spin과 recovery alpha가 그대로 들어간다.

## 동작

1. **바쁨**: `MOVING`, `DOCKING`, `RECOVERING`, `BLOCKED`이거나, 수락만 되고 아직 FSM에 안 넘어간 이동 goal이 있거나(10 s 뒤 잊음), 다른 relocalize가 돌거나 그 회전이 아직 멈추지 않았으면 goal을 받고 바로 `success=false, reason=BUSY, stage=""`로 끝낸다. 상태나 지금 일은 바꾸지 않는다(LOCALIZING도 내지 않는다).
2. **시작**: `request_nomotion_update`를 `service_wait_s`(1 s) 기다린다. `allow_global`이면 `reinitialize_global_localization`도 기다린다. 지도(`map`)가 아직 없어도 `NO_LOCALIZER`, 단계는 `""`.
3. **scan_match**: 점을 AMCL 자세와 레이저 오프셋으로 지도에 옮기고, 그 점에서 벽 칸(값 ≥ 65)의 네모까지 거리가 `match_tol_m`(0.06) 안인 비율. 거리 변환(칸 중심 거리, `match_tol_m + 2·해상도`까지)으로 먼저 거르고(칸 중심 거리 > tol + 대각선이면 맞지 않음), 남은 점은 주변 벽 칸 네모까지 거리를 모두 잰다. 지도 밖 점은 맞지 않음, 유효한 점이 0개면 0.
4. **NOMOTION**: 새 스캔마다(최소 `nomotion_period_s` 0.2 s 간격) 무이동 갱신을 부른다. 새 amcl_pose(이번 실행 시작 뒤이고 마지막으로 쓴 것보다 새 시각)마다 std_xy, std_yaw와 scan_match를 잰다. scan_match는 보관한 스캔(최근 20개) 중 시각이 가장 가까운 것으로 재고, 차이가 0.15 s를 넘으면 0이다. 그 시각의 스캔이 아직 안 왔으면 줄에 두고 그 시각 이후의 스캔이 올 때 차례로 잰다. 전역 재초기화 전의 것은 버리고, 회전 중에 나온 것(회전 결과 때까지 받은 마지막 스캔 시각 이하)은 회전 뒤 판정에 넣지 않는다. std_xy < 0.10, std_yaw < 0.15, scan_match ≥ 0.75가 5번 이어지면 성공. 공분산만 작아서는 성공하지 않는다.
5. **SPIN**(`allow_spin`이고 NOMOTION이 몫 안에 못 끝냈을 때): `spin` 액션에 2π. 도는 동안에도 무이동 갱신은 부르지만 판정은 회전이 끝난 뒤부터 한다. 거절이나 시간 초과(701)가 아닌 중단(충돌 703 등)이면 `SPIN_BLOCKED`, 단 `allow_global`이면 GLOBAL로 간다(그때는 다시 돌지 않는다).
6. **GLOBAL**(`allow_global`): 재초기화 서비스를 부르고, 답이 오기 전의 추정은 버린다. 돌 수 있으면 다시 한 바퀴, 그다음 판정.
7. **시간 몫**: 전체 T(goal의 `timeout_s`, 0 이하면 30 s). 뒤 단계가 있으면 NOMOTION은 0.25 T, GLOBAL이 뒤에 있으면 SPIN은 0.5 T까지. 뒤 단계가 없으면 그 단계가 T 끝까지 쓴다. 회전의 `time_allowance`는 단계 끝 − 지금 − `settle_s`(3 s), 최소 `min_spin_allowance_s`(1 s). Jazzy Spin은 0을 무제한으로 본다. 시간이 지난 뒤 온 추정으로는 성공하지 않는다.
8. **실패 사유**: 추정이 한 번도 없거나 좋았지만 K번 못 이었으면 `TIMEOUT`, std가 크면 `NOT_CONVERGED`, std는 작은데 스캔이 안 맞으면 `SCAN_MISMATCH`.
9. **취소·시간 초과**: 회전을 취소하고 회전 결과(멈춤 확인)를 `spin_stop_wait_s`(2 s)까지 기다린 뒤 결과를 낸다. 그동안은 계속 LOCALIZING이고 이동 goal을 받지 않는다. 2 s 안에 확인이 없으면 결과는 내되 멈춤을 확인할 때까지(상한: 회전을 보낸 시각 + time_allowance + 2 s, pode_fsm의 단조 시계로 잰다. Nav2는 ROS 시계로 재므로 시뮬 시간이 느리면 이 상한이 Nav2의 멈춤보다 먼저 올 수 있다) LOCALIZING을 유지하고 이동 goal을 받지 않는다. 회전 결과를 잃은 것은 멈춤이 아니다. 취소 응답이 '이미 끝남'일 때만 멈춤으로 본다.
10. **결과**: `stage`, `pose_valid`(시작 때 받은 amcl_pose가 있고 판정에 쓴 추정이 있을 때만 참), `stamp`(마지막으로 받은 새 amcl_pose의 시각. 회전 중이나 재초기화 답을 기다리던 중의 것일 수도 있다. 끝난 뒤 온 것은 넣지 않는다. `pose`, 품질 값, 실패 사유도 이 추정에서 온다), `start_pose`(시작 때 마지막 amcl_pose), `pose`, `std_xy_m`, `std_yaw_rad`, `scan_match`, `fresh_updates`(판정에 쓴 새 추정 수. 회전 중의 것은 세지 않는다).
11. amcl_pose는 transient_local로 구독한다(Jazzy AMCL과 같은 QoS). 그래서 시작 전에 받아 둔 마지막 추정을 `start_pose`로 쓴다.

## 시험

명령(내 폴더에만 빌드):

```
bash <scratchpad>/build.sh fleet_interfaces      # colcon --build-base/--install-base를 scratchpad의 reloc_ws로
bash <scratchpad>/build.sh fake_robots
bash <scratchpad>/build.sh fleet_core            # 실제로는 pode/sim 패키지들이 함께 빌드됐다(아래 열린 점)
# test.sh: ROS_DOMAIN_ID=72, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, 내 install 소싱
bash <scratchpad>/test.sh -q test/test_relocalize.py
bash <scratchpad>/test.sh -q -rs test/
bash <scratchpad>/test_nav.sh -q test/
```

시험 먼저: 순수 시험을 쓰고 돌려 실패를 확인한 뒤 구현했다.

```
E   ImportError: cannot import name 'RelocalizeParams' from 'pode_fsm.params' (...)
1 error in 0.23s
```

구현 뒤:

```
49 passed in 0.23s
```

계약 d8d3123(std_xy 정의, stage, fresh_updates)에 맞춘 시험을 먼저 넣고 실패를 확인:

```
FAILED test/test_relocalize.py::test_std_xy_is_the_contract_definition_not_the_eigenvalue
FAILED test/test_relocalize.py::test_missing_localizer_ends_at_once - Asserti...
FAILED test/test_relocalize.py::test_fresh_updates_counts_only_judged_estimates
3 failed, 48 passed in 0.36s
```

고친 뒤 `51 passed in 0.16s`.

ROS 시험 첫 실행에서 한 개가 틀렸다. 시험이 틀렸다(시간이 다 됐을 때 사유는 마지막 추정으로 정하는 설계다. std 0.5 m라 `NOT_CONVERGED`가 맞다). 시험을 고쳤다.

```
FAILED test/test_ros_relocalize.py::test_timeout_stops_the_spin_before_the_result
1 failed, 12 passed in 30.48s
```

첫 커밋(938df90, ec472a5) 때:

```
$ test.sh -q test/test_relocalize.py test/test_ros_relocalize.py
64 passed in 32.03s

$ test.sh -q -rs test/            # pode_fsm 전체
SKIPPED [6] test/test_ros_matches_fake_pode.py: written for the lane map; not yet rewritten for the zone map
504 passed, 6 skipped in 101.10s (0:01:41)

$ test_nav.sh -q test/            # pode_nav_config 전체
103 passed in 8.71s
```

리뷰 반영(fc31a26, afeefb9). 시험을 먼저 넣고 실패를 확인했다:

```
$ test.sh -q test/test_relocalize.py          # nearest_field가 아직 없다
ERROR test/test_relocalize.py
1 error in 0.19s
(구현 뒤, 내 시험 실수 하나: 30 - 7.5 - 20 = 2.5)
FAILED test/test_relocalize.py::test_spin_allowance_is_never_zero - assert 2....
1 failed, 54 passed in 0.17s
(시험을 settle_s=25로 고친 뒤)
55 passed in 0.22s

$ test.sh -q test/test_ros_relocalize.py -k slow_spin   # 늦게 멈추는 회전, 고치기 전
E       assert True is False      (이동 goal이 수락됐다)
1 failed, 13 deselected in 7.63s
(고친 뒤. 가짜 spin이 time_allowance를 무시해 시간 초과 판은 실제와 달라 취소 판으로 바꿨다)
1 passed, 13 deselected in 8.52s

$ test_nav.sh -q test/test_nav_config.py      # 0.5 rad/s일 때
E       assert (((2 * 3.141592653589793) / 0.5) + 0.15625) < 12.0
1 failed, 56 passed in 0.18s
```

fc31a26 뒤 전체: `509 passed, 6 skipped, 1 warning in 113.18s (0:01:53)`, pode_nav_config `104 passed in 8.74s`.

2차 리뷰 반영(fe810ce, d1e0972). 모서리 칸 시험은 import 실패로 먼저 빨갛게 됐고, fc31a26 구현으로 같은 두 점을 재 보니 `(0.052, 0.098) 0.0`, `(0.098, 0.052) 1.0`이었다(결함 확인).

fe810ce 뒤 전체 `510 passed, 6 skipped in 111.11s (0:01:51)`.

마지막(044021e 뒤):

```
$ test.sh -q test/test_relocalize.py test/test_ros_relocalize.py   # fe810ce 때
70 passed in 39.88s

$ test.sh -q -rs test/            # pode_fsm 전체
SKIPPED [6] test/test_ros_matches_fake_pode.py: written for the lane map; not yet rewritten for the zone map
510 passed, 6 skipped in 106.36s (0:01:46)      # 044021e 뒤

$ test_nav.sh -q test/            # pode_nav_config 전체
104 passed in 8.55s

$ (pc/fleet_core) pytest test/test_contract.py test/test_contract_check.py   # 읽기만, 고치지 않음
18 passed in 0.07s
```

## Gazebo에서 볼 것

로봇마다 namespace 안에서(`/<ns>/...`):

- 서버: 액션 `/<ns>/relocalize`(fleet_interfaces/action/Relocalize)
- 구독: `/<ns>/scan`(LaserScan, sensor QoS), `/<ns>/amcl_pose`(transient_local), `/<ns>/map`(5 cm, transient_local), TF `base_footprint` → 스캔 frame(정적, 한 번 읽어 둔다)
- 부르는 것: 서비스 `/<ns>/request_nomotion_update`, `/<ns>/reinitialize_global_localization`(std_srvs/Empty), 액션 `/<ns>/spin`(nav2_msgs/action/Spin)
- 알림: `/<ns>/state`가 도는 동안 `LOCALIZING`(자세 칸은 비움)

손으로 부르기:

```
ros2 action send_goal /pode1/relocalize fleet_interfaces/action/Relocalize \
  "{allow_spin: true, allow_global: false, timeout_s: 30.0}" --feedback
```

코디네이터가 볼 것:

- pode_fsm 로그 `relocalize started ...`, `relocalize ended: OK|<사유> std_xy ... scan_match ...`, 경고 `spin server is not available`, `spin did not confirm the stop in time`, `no base_footprint->...`
- behavior_server가 `spin` 플러그인을 올리는지(새 yaml이 들어간 Nav2로 띄워야 한다)
- 순간이동 시험: 0.5 m 옮긴 뒤 무이동 단계만으로는 대개 못 모을 것으로 본다(입자가 0.5 m 밖에 없으면 무이동 갱신은 옮겨 주지 못한다). recovery alpha가 무작위 입자를 넣고 회전이 스캔을 모아야 한다. `allow_global`이 필요할 수 있다. 결과의 `stage`, `fresh_updates`, `scan_match`를 적어 달라
- 회전 중 다른 로봇이 가까우면 Spin이 충돌로 멈출 수 있다. 그러면 `allow_global`이 없을 때 `SPIN_BLOCKED`, 있으면 GLOBAL로 간다(GLOBAL에서 다시 막히면 판정을 이어 간다)
- Jazzy Spin의 충돌 검사는 회전 시작 직후 앞을 다 보지 못한다(Codex, 상류 구현). 좁은 곳에서는 지켜볼 것

## Codex

| 커밋 | 상태 |
|---|---|
| 938df90 feat(pode_fsm) | 도착해서 읽음. [확정] 8개 중 7개를 직접 확인하고 fc31a26에서 고쳤다(회전 멈춤 확인, 이동 goal 수락 경쟁, 결과 stamp, 늦은 전역 응답, 시간 뒤 성공, allowance 0, 칸 중심 거리). [추측] 1개(스캔보다 먼저 온 amcl_pose)도 고쳤다. 시간 초과 사유를 품질로 정하는 점은 설계대로 두고 열린 점에 적었다. 서비스 대기 중 취소가 최대 1~2 s 늦는 점도 열린 점 |
| ec472a5 feat(pode_nav_config) | 도착해서 읽음. 한 바퀴가 12 s 몫에 안 들어간다는 지적을 계산으로 확인하고 afeefb9에서 0.6 rad/s로 고쳤다. recovery alpha가 정상 주행에도 입자를 넣을 수 있다는 [추측]은 열린 점 |
| fc31a26 fix(pode_fsm) | 도착해서 읽음. [확정] 7개를 직접 확인했다. 1(앞 회전이 도는 중 새 relocalize), 4(pending 누수), 5(모서리 칸: fc31a26 코드로 재어 한쪽 점이 0.0으로 나옴), 6(이른 스캔이 기다림을 풂), 7(재초기화 전 추정)을 fe810ce에서 고쳤다. 2(멈춤 확인 없이 기한만 지나면 허용), 3(결과 잃고 취소 중 응답이면 다시 확인 못 함)은 Nav2가 time_allowance를 어길 때만 생기므로 열린 점으로 두었다 |
| afeefb9 fix(pode_nav_config) | 도착해서 읽음. 시험의 가속 모델이 Jazzy와 다르다, 12 s를 옮겨 적었다(둘 다 [확정]). d1e0972에서 Jazzy 속도 규칙으로 적분하게 바꾸고 출처를 적었다 |
| fe810ce fix(pode_fsm) | 도착해서 읽음. [확정] 3개를 확인했다. 3(회전 중 추정이 회전 뒤 판정에 들어감)은 67fce03에서 고쳤다. 1, 2(넘김 콜백이 예약 만료보다 늦을 때)는 만료를 10 s로 늘리고 한계로 적었다. scan_match 이웃 검사는 오류 없음 |
| d1e0972 test(pode_nav_config) | 도착해서 읽음. 결함 없음(속도식이 Jazzy와 같고 0.5에서 실패함을 확인) |
| 67fce03 fix(pode_fsm) | 도착해서 읽음. [확정] 3개(노드 시계 경계가 늦은 콜백 때 좋은 추정을 버림, 같은 시각은 통과, 앞 goal의 늦은 결과가 경계를 옮김)를 확인하고 044021e에서 고쳤다. 시험을 먼저 쓰지 못했다(실행기 타이밍 경쟁) |
| 044021e fix(pode_fsm) | 도는 중(보고에 적는다). 시험을 먼저 쓰지 못했다(같은 이유) |
| 이 기록(a 커밋, 아래 b 커밋) | 첫 판 리뷰 도착해서 읽음: 같은 시각 스캔이라는 설명, stamp의 뜻, 멈춤 상한의 시계, SPIN_BLOCKED 단정, 7.5 s의 조건을 고쳤다. 고친 판 리뷰는 보고에 적는다 |
| 이 기록 | 커밋 뒤 돌린다 |

## 열린 점

- **0.5 m 순간이동을 실제로 되돌리는지는 Gazebo에서 아직 재지 않았다.** 시뮬은 코디네이터 몫이다. 판정 문턱(0.10 m, 0.15 rad, 0.75, K 5)과 시간 몫은 첫 값이다.
- scan_match는 5 cm 지도(`map`)로 잰다. AMCL은 1 cm 지도(`map_fine`)를 읽는다. 지도에 없는 물체(다른 로봇, 사람)가 많으면 맞음 비율이 떨어져 `SCAN_MISMATCH`가 날 수 있다.
- 시간이 다 됐을 때 사유는 마지막 추정으로 정한다. 그래서 회전 중 시간이 끝나도 `TIMEOUT`이 아니라 `NOT_CONVERGED`일 수 있다. `TIMEOUT`은 추정이 없었거나 좋았지만 K번 못 이었을 때만이다. 관제가 다르게 읽는다면 맞춰야 한다.
- 회전 결과를 잃거나 Spin 서버가 없으면 `SPIN_BLOCKED`로 본다(돌 수 없었다는 뜻으로).
- 기본 30 s에 `allow_global`이고 SPIN이 몫 끝까지 가면 GLOBAL 단계가 7.5 s뿐이라(SPIN이 일찍 막히면 더 남는다) 전역 뒤 한 바퀴(약 11 s)를 다 돌 수 없다. 전역을 쓸 때는 관제가 `timeout_s`를 늘려야 한다(예: 60 s).
- 시작 때 서비스를 기다리는 동안(최대 1 s, 전역이면 2 s) 취소와 시간이 바로 듣지 않는다.
- recovery alpha는 정상 주행 중에도 스캔 맞음이 떨어지면 입자를 넣는다(Codex [추측]). 실물·시뮬 주행에서 위치가 흔들리는지 볼 것.
- 이동 goal 예약은 넘김 콜백이 끝내 오지 않을 때를 대비해 10 s 뒤 잊는다. 콜백이 그보다 늦으면 다른 goal의 예약을 지울 수 있다(Codex fe810ce 1, 2. 실제로는 ms 단위라 두었다).
- 회전 멈춤을 끝내 확인하지 못하면(Nav2가 time_allowance를 넘겨 돌거나, 결과를 잃고 취소 응답이 '취소 중'이면) 보낸 시각 + time_allowance + 2 s에 이동 거절을 푼다. 영원히 묶이지 않게 하려는 선택이다(Codex fc31a26 2, 3).
- 정지 확인을 기다리는 2 s 안에 회전 결과가 오지 않으면 결과는 낸다(계약의 '회전을 멈춘 뒤 결과'를 이 고장 때만 어긴다). 대신 멈춤 확인까지 LOCALIZING과 이동 거절을 유지한다.
- `max_rotational_vel`은 Spin만 쓴다(Codex 확인. BackUp, Wait는 쓰지 않는다). `simulate_ahead_time`은 BackUp과 함께 쓰므로 바꾸지 않았다.
- 빌드 스크립트의 `--packages-select fleet_core`가 fleet_core는 빌드하지 않고 pode/sim 패키지 9개를 빌드했다. 시험은 `pc/fleet_core`를 PYTHONPATH에 넣어 돌렸다. 원인은 보지 않았다.
- 작업 중 내 scratchpad의 `build.sh`를 다른 세션이 덮어썼다(다른 install 폴더 `/tmp/claude-1000/fakeabort_ws`를 쓰는 내용). 되돌리지 않았고, 그 뒤로는 빌드할 일이 없었다.
- 계약에서 모자란 것: 없다. 이동 goal을 거절하면 MoveTo에는 사유가 없어 관제는 이유를 모른다(계약이 거절을 정했으므로 그대로 따랐다).
