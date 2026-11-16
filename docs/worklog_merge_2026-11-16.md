# 구조 정리 브랜치 main 병합 기록 (2026-11-16)

사용자 요청: "수정사항들 메인에 병합해서 최신화해놔 그리고 이상없는지 테스트도 돌리고", 이어서 "시뮬로 테스트도 한번".

## 1. 병합한 것

origin/main 7c7d04b에서 갈라진 브랜치 여섯 개를 통합 브랜치에 `--no-ff`로 합친 뒤 main에 fast-forward로 올렸다(7c7d04b..2a06d72). 충돌은 없었다.

| 브랜치 | 내용 | 작업 기록 |
|---|---|---|
| fix/package-deps | package.xml 의존 선언(pode 4개, fleet_core rclpy, fleet_manager test_depend) | docs/worklog_package_deps_2026-11-16.md |
| docs/zone-traffic-map | ZoneTraffic 구조 문서 | docs/ZoneTraffic_구조.md |
| fix/action-cancel | pode_fsm 액션 래퍼 3개를 action_client.py GoalRunner로 공통화, BackUp 결함 3개 수정 | docs/worklog_action_cancel_2026-11-16.md |
| feat/zone-observe | 읽기 전용 관측 API(Views.live_block), sim의 zone 내부 import 제거, 경계 시험 | docs/worklog_zone_observe_2026-11-16.md |
| test/tick-order | 시험 하네스 기본 tick 순서를 FleetManager.tick으로, 순서 비교 시험 | docs/worklog_tick_order_2026-11-16.md |
| feat/params-validate | FleetParams 검증, stall_s가 로봇 blocked_after_s 10 s + 2 s보다 짧으면 경고(오류 아님) | docs/worklog_params_validate_2026-11-16.md |

통합 브랜치에서 더한 커밋은 하나다: 90b8425(구조 문서의 하네스 설명을 병합 뒤 기본 tick 순서에 맞춤. 통합 범위 Codex 리뷰 지적).

## 2. 시험

- 환경: ROS_DOMAIN_ID 68·69, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, 빌드는 scratch 폴더(공용 install은 당시 다른 세션의 Gazebo가 쓰고 있었다).
- 경고 커밋 전 트리(0b1f54f)에서 전체를 돌리고, 경고 커밋을 합친 뒤(2a06d72) 바뀐 부분을 다시 돌렸다.

| 시험 | 결과 | 트리 |
|---|---|---|
| colcon build 15개 패키지 | 성공 | 0b1f54f, 2a06d72 |
| fleet_core (ZONE_TICK_ORDER manager / zone) | 각 1048 passed, 1 skipped | 2a06d72 |
| pode_fsm | 440 passed, 6 skipped | 0b1f54f |
| pode_avoidance / peer_obstacles / scan_filter / nav_config | 26 / 18 / 23 / 102 passed | 0b1f54f |
| fleet_manager (아래 두 파일 제외) | 112 passed | 0b1f54f |
| fleet_manager test_node_wiring.py | 24 passed | 2a06d72 |
| fleet_gz_sim | 527 passed | 0b1f54f |
| fleet_gz_sim test_timing_agreement.py | 4 passed | 2a06d72 |
| tools | 96 passed | 0b1f54f |
| fake_robots | 1000 passed (658 s) | 0b1f54f |

돌리지 않은 것: fleet_manager의 test_scenario_integration.py, test_fms_restart.py. 도메인 50~59를 스스로 잡아 사용자 도메인 57과 겹칠 수 있다.

## 3. Gazebo 한 판

- 명령: `ros2 launch fleet_gz_sim gz.launch.py layout:=v7 scenario:=gz_v7_rush use_order_model:=true sim_dock_stub:=true rviz:=fleet gui:=true`, 도메인 61, main 2a06d72를 scratch install로 빌드.
- 비교 대상: 같은 명령의 병합 전 판(gz_measure_1116b, 다른 세션, 공용 install 7c7d04b).

| 항목 | 병합 뒤(2a06d72) | 병합 전(7c7d04b) |
|---|---|---|
| 판정 | PASS | PASS |
| 주문 | 12/12 출하, partial 0, held 0 | 12/12 출하 |
| 걸린 시간 | 273.0 s | 271.2 s |
| 위반(violations) | 없음 | 없음 |
| 구역 해제 | 60회, early 0, near 0 | 60회, early 0, near 0 |
| 최소 로봇 간격 | 0.220 m | 0.224 m |
| 막힘·회복 이벤트(BLOCKED, STALL, BACKUP 등) | 0 | 0 |
| 시계 겹침 경고(backwards in time, TF_OLD_DATA) | 0 | - |

- 두 판 모두 한 번씩이라 시간 차이 1.8 s를 성능 차이로 볼 근거는 없다.
- 이 판에서는 후진(BackUp)과 회복이 한 번도 일어나지 않았다. 그래서 fix/action-cancel의 BackUp 취소·결과 처리 경로는 Gazebo에서 확인되지 않았다(단위 시험 test_action_runners.py로만 확인).
- 종료 때 nav2 컨테이너 3개가 "Magick: abort due to signal 11"로 죽고 launch가 exit 1을 낸다. 시나리오 모니터가 끝나 launch가 내리는 중에 생기며, 병합 전 판(gz_measure_1116b, gz_order_1116, gz_rename)의 launch.log에도 같은 줄이 있다.
- 결과 폴더: /tmp/claude-1000/-home-ane-ros2camp/255eff68-4c77-42b3-b9fc-325ae2037b61/scratchpad/gz_main_1116 (임시 폴더라 지워질 수 있다).

## 4. 남은 일

- Pode에 올리기: pode_fsm 코드와 package.xml 4개가 바뀌었다. `tools/real_pode/deploy.sh pinky@<주소>` 뒤 로봇에서 colcon build. 로봇과 주소는 사용자가 정한다.
- 공용 작업 공간 install(nav2fleet_ws/install)은 2a06d72로 다시 빌드했다(15개 패키지 성공).
- 회복 시간 초과(pode_fsm recovery.py:77)에서 CancelBackUp과 경로 재전송이 함께 나가는 문제는 고치지 않았다. 사용자가 2026-11-16에 "안전 우선이 아니라 최상의 퍼포먼스 우선"이라고 정해, 안전 쪽 인계 장치 제안은 거두고 다시 평가하기로 했다.
