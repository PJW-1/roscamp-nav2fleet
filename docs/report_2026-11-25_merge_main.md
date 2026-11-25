# 보고서: main 병합과 병합 뒤 훑기 (2026-11-25 새벽)

11-24 밤 사용자 승인으로 세 갈래를 main에 합쳤다: `feat/interface-align-2026-11-24`(계약 0.7, 팔 이름, no-motion),
`docs/rl-real-prep-2026-11-24`(RL 실물 준비 점검), RL 갈래(`feat/rl-shim-recipe-2026-11-22`, `feat/rl-acd-train-2026-11-24`
등, 주석 정리 포함). 병합은 작업 브랜치 `merge/main-2026-11-25`에서 하고 main은 빨리감기로만 옮겼다(force push 없음).
그 뒤 병합 결과를 한 번 훑어(어긋남 18건) 고칠 것은 고쳤다. 로봇 접속, deploy, 실물 명령은 하지 않았다.

## 1. 커밋

| 커밋 | 내용 |
|---|---|
| bbf7228 | Merge feat/interface-align-2026-11-24 |
| 65b90c7 | Merge docs/rl-real-prep-2026-11-24 |
| d4aac2b | 병합으로 들어온 영어 주석을 한국어 한 줄 요약으로(주석 규칙) |
| 2502483 | no-motion 대역이 준비를 보고하지 않아 로봇이 LOCALIZING에 머물던 것 |
| f16adf1 | Merge RL 갈래(레시피, acd2 학습, 형식 3 shim) |
| f7c53fb | plan_split 예시 params를 기본 파일에서 다시 만듦, 학습 arena가 GAP_S 경유 노드를 건너뜀, 수레 시험 기준(아래 4절) |
| 724410d | 첫 tick 전에 온 no-motion goal이 NAV_UNAVAILABLE로 끝나던 것, `nav_rl`이 지운 shim 파일을 가리키던 것 |
| d3b9b3a, eef3fbd, 0d43c7c | parity 시험: acd2를 더하고, 저장소에 onnx가 없는 후보는 추론 비교만 건너뜀. 4b는 onnx가 꼭 있어야 함 |
| ff7a78a | Merge fix/sweep-docs-2026-11-25: 훑기의 문서·주석 어긋남 정리 |

## 2. 병합으로 생긴 의미 충돌 (고침)

- **no-motion goal이 NAV_UNAVAILABLE로 끝남.** interface-align의 no-motion 대역과 main의 "Nav2 서버가 보이기 전에는
  LOCALIZING, 못 보낸 goal은 NAV_UNAVAILABLE"(6133ba9)이 만나, 대역을 쓰는 로봇이 첫 tick 전에 받은 goal을 거절했다.
  대역은 처음부터 준비된 것이므로 `_start_no_motion`에서 준비를 알린다(724410d, 시험 22개 통과).
- **`nav_rl:=rl_shim`이 지운 파일을 가리킴.** RL 갈래가 RL 설정을 `follow_path_rl.yaml` 하나(shim으로 감싼 FollowPathRL과
  단독 FollowPathRLAlone)로 합쳤는데 launch는 옛 shim 파일 이름을 썼다. 두 값 모두 그 파일을 쓰게 하고, 사용자가 준
  controller 목록에는 RL controller 둘을 한 번씩만 더한다(724410d, `test_launch_wiring.py` 통과).
- **parity 시험.** acd2 fixture가 시험 목록에 없었고, 저장소에 onnx가 없는 c·acd 시험은 새로 받은 저장소에서 실패한다.
  세 번 고쳐(Codex 지적 두 번, 세 번째 검수에서 결함 없음) 지금은 모든 후보의 시험을 만들되 onnx가 없으면 추론 비교만
  건너뛰고, 저장소에 있어야 하는 4b는 없으면 실패한다. 6/6 통과, 4b onnx를 치우면 실패하는 것도 확인했다.

## 3. 훑기 18건과 처리

| 번호 | 무엇 | 처리 |
|---|---|---|
| 1 | Gazebo `nav_rl:=rl_shim`이 지운 파일을 가리킴 | 고침(724410d) |
| 2 | 로봇 상태 발행 5 Hz(pode_fsm, IR-34 의도 값)와 문서·주석의 10 Hz | 문서·주석을 5 Hz로(ff7a78a). FakePode 기본 10 Hz는 사용자 결정(6절) |
| 3 | `pc.launch.py` 사용 예가 한 줄로 뭉개짐 | 고침(ff7a78a) |
| 4 | `tools/real_pode/README.md`가 스크립트와 다름 | 고침(ff7a78a) |
| 5 | Retreat가 NAV_UNAVAILABLE로 끝날 수 있는데 Retreat.action과 계약 문서에 없음 | 사용자 결정(6절) |
| 6 | `/fleet/force_ship` 거절 코드가 계약에 없음 | 사용자 결정(6절) |
| 7 | 계약 문서의 넘김 거리 0.15 m(코드는 0.12 m) | 고침(ff7a78a) |
| 8 | plan_split 사각형 근거 스크립트가 옛 진열대 끝 0.65를 씀 | 0.70으로(ff7a78a). 다시 돌려 15 % 동점 점 245개가 모두 지도의 splits 사각형 안에 있음을 확인 |
| 9 | parity 시험 목록과 onnx | 고침(2절) |
| 10 | Gazebo 시험·launch의 옛 RL 설명 | 고침(724410d, ff7a78a) |
| 11 | Gazebo README의 layout 기본값 | 고침(ff7a78a) |
| 12 | `docs/IFP_공통설계.md`의 옛 값(SubmitOrder, vision 토픽, blocked_mark_s 등) | 고침(ff7a78a). 없어진 파라미터 10개는 "없어짐"으로 표시 |
| 13 | 계약 버전 번호 두 체계(0.11~0.13-draft와 0.7) | 번호 이력을 문서에 적음(ff7a78a) |
| 14 | 맨 위 README "지금 상태" | 고침(ff7a78a) |
| 15 | 주행 계획서의 sim_shelves 설명 | 고침(ff7a78a) |
| 16 | 병합 잔여(중복 문장, 줄 번호 인용, 환경 변수 설명) | 고침(ff7a78a) |
| 17 | 수레 시험 기준을 낮춤 | 4절 |
| 18 | `fleet_manager_node`의 `_dispatch`가 ReportTripCmd를 조용히 버림 | 이미 알려진 W-21. 사용자 결정(6절) |

## 4. 느슨하게 한 기준

- `learning/store_arena/test_mover_spec.py`: 수레가 경로를 가로지르는 비율의 하한을 0.2에서 0.12로 낮췄다(사람은 0.2 유지).
  고친 v7 진열대에서 수레는 반지름이 커서 시드 8개의 값이 0.15~0.21이었다(f7c53fb).
- `pode_rl_controller` parity: 저장소에 onnx가 없는 후보(c, acd, acd2)는 fixture 검사만 하고 추론 비교는 건너뛴다.
  저장소는 4b의 onnx만 커밋하므로, 나머지는 학습 PC에서 onnx를 넣어야 비교한다. 4b는 없으면 실패한다.

## 5. 시험

WSL(Jazzy, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 도메인 44~47)에서 병합 판 전체를 한 번 돌렸다:
fleet_core 1416 passed(7 xfailed), fake_robots 1168 passed(1 skipped, 30 xfailed), fleet_manager 125 passed(42 skipped),
pode_nav_config 131, pode_avoidance 26, pode_peer_obstacles 19, pode_plan_keep 50, pode_scan_filter 23 passed.
이때 실패한 것은 셋이었고 모두 설명된다.

- pode_fsm `test_ros_no_motion.py` 2건, fleet_gz_sim `test_launch_wiring.py` 3건: 2절의 두 의미 충돌. 고친 뒤 통과.
- tools `test_deploy_copies_every_repo_dependency`: 시험 사본에 `.git`이 없어서 난 환경 문제다. 같은 계산을 저장소에서
  다시 해 빠진 의존성이 없음을 확인했다.

ff7a78a(문서 정리 병합) 뒤에는 바뀐 곳만 다시 돌렸다(아래 표는 실제 출력).

| 시험 | 결과 |
|---|---|
| pode_fsm `test_ros_no_motion.py`, `test_no_motion.py` | 22 passed |
| fleet_gz_sim `test_launch_wiring.py`, `test_sim_nav_params.py` | 130 passed |
| fleet_core `test_layout_v7_split.py` | 6 passed |
| tools `test_real_pode.py` | 156 passed, 1 failed(`test_deploy_copies_every_repo_dependency`: 시험 사본에 `.git`이 없어 `git ls-files`가 실패, 위와 같은 환경 문제) |
| 바뀐 launch 파일 셋 `ast.parse`, `ros2 launch iponic_bringup pc.launch.py --show-args` | 통과 |

## 6. 사용자가 정할 것

2026-11-25 사용자 결정: 1~3은 권고대로 고치고, 4는 해결로 둔다. 고친 내용은 `docs/worklog_merge_report_leftovers_2026-11-25.md`.
5는 같은 날 사용자 지시로 동기화했다(nav2fleet 818e60d = pode-drive 4572a23).

1. **Retreat의 NAV_UNAVAILABLE.** `retreat.py`는 Nav2 서버가 없을 때 이 코드로 끝내지만 `Retreat.action`에는 없다.
   권고: `Retreat.action`에 `REASON_NAV_UNAVAILABLE`을 더하고 계약 문서를 고친다. interfaces 변경이라 팀 interfaces 담당의 확인이 필요하다.
2. **force_ship 거절 코드.** 관제는 `NOT_HELD`, `ROBOT_FAULT`, `UNKNOWN_ORDER`를 그대로 돌려주는데 `contract.yaml`의
   `request_rejected`에는 없다. 권고: 계약 코드로 바꾼다(UNKNOWN_ORDER → NOT_FOUND, NOT_HELD → ORDER_STATE). ROBOT_FAULT는 대응 코드가 없어 계약에 더할지 정해야 한다.
3. **FakePode 상태 주기.** pode_fsm은 5 Hz(IR-34)인데 가짜 로봇 기본은 10 Hz다. 시뮬을 실물에 맞추려면 5 Hz로 바꾼다. 바꾸면 fake_robots 시험의 시간 여유를 다시 봐야 한다.
4. **ReportTripCmd(W-21).** 아직 보내는 곳이 없어 지금 동작에는 영향이 없다. 적어도 `case _` 경고를 둘지 정한다.
5. **nav2fleet 동기화.** nav2fleet main은 f653332(pode-drive 8d106e5 사본)이고 이번 병합은 맞추지 않았다. 동기화는 사용자가 말할 때만 한다.
