# 실물 핑키 실행 묶음과 pinky_27a6 정리 (2026-11-16)

브랜치 `feat/real-pinky-kit`(`main` 48ad1d4에서 갈라짐). 커밋은 a03a61d, eb95c78, d7c272e, e1f58d3, 6a885fe, 3de4fc1, 890e96e입니다. 로봇에서 launch를 띄운 적은 없습니다. 이 문서의 결과는 모두 시험과 Gazebo 기준입니다.

## 1. 로봇 pinky_27a6에서 한 일

- 접속: 공유기 192.168.0.200, 사용자 `pinky`(키 인증). Raspberry Pi, ROS 2 Jazzy, 제조사 pinky_pro(rpi4 브랜치). 로봇 도메인 12.
- 정리: 예전 작업 폴더를 지우지 않고 `~/archive_2026-11-10/`로 옮겼습니다(`ff_real`, `ff_real_ws`, `ff_ws`, `maps_ff`, `pinky_real.launch.py`). 그대로 둔 것: `r2g`, `robot_*_gateway.py`, `motion_guard.py`, `maps/`, `work/`.
- 교체: `~/nav2fleet_ws/src`에 `fleet_interfaces`와 `pinky/` 아래 6개 패키지(`pinky_fsm`, `pinky_nav_config`, `pinky_avoidance`, `pinky_peer_obstacles`, `pinky_scan_filter`, `pinky_dock`)를 넣고 로봇에서 `colcon build`(7개 성공, 4분 23초). 빌드한 내용은 eb95c78 기준입니다. 그 뒤 커밋은 `tools/`와 시험만 바꿔서 다시 빌드하지 않았습니다.
- 실행 묶음: `~/nav2fleet_real/`(launch, 스크립트, v7 지도 사본). `~/nav2fleet_ws/src/SOURCE_COMMIT`에 마지막으로 배포한 커밋을 적습니다.
- 속도 0.12로 params 사본을 로봇에서 만들어 확인했습니다: FollowPath, FollowPathThrough, Nav2 goal 컨트롤러(`avoidance.yaml`의 follow_path_goal)가 0.12, 후진은 0.05 그대로입니다.

## 2. 실행 묶음 `tools/real_pinky/`

| 파일 | 하는 일 |
|---|---|
| `real_pinky.launch.py` | 제조사 bringup(namespace를 그룹에 적용), `cmd_watchdog`, `nav2.launch.py`, `pinky_fsm`. 프로세스 하나가 끝나면 모두 내립니다 |
| `make_params.py --speed S --out DIR` | Nav2 params와 회피 params의 속도 줄만 바꾼 사본을 만듭니다. 쓰기 전에 바꾼 글을 YAML로 읽어 그 세 값 말고 바뀐 것이 없는지 봅니다. 0 < S ≤ 0.25를 반올림 전에 보고, 소수 넷째 자리까지 반올림해 0이 되면 거부합니다 |
| `start.sh NS X Y YAW [SPEED=0.12]` | params 사본 → env → `setsid`로 launch. `logs/start.lock`으로 두 번 시작을 막고, 5 s 안에 pid 파일이 없으면 TERM 후 약 2 s 기다려 사라졌는지 알립니다(KILL은 안 씀) |
| `stop.sh`, `env.sh` | 내리기, 환경(도메인 12, SUBNET) |
| `deploy.sh HOST` | `git archive HEAD`를 패키지 폴더마다 `rsync --delete`로 보내고, 묶음은 `~/nav2fleet_real`로 보냅니다(이쪽은 `--delete` 없음) |
| `cmd_watchdog.py` | 마지막 움직이는 명령 뒤 약 0.40~0.45 s에 속도 0을 보냅니다 |

시험: `python3 -m pytest -q tools/test` → 96 passed(890e96e). `pinky_nav_config` 시험은 패키지를 source하고 도메인 73에서 102 passed(eb95c78).

## 3. 이전에 짚은 1~3번 항목의 처리

- 속도: 실물 기본값은 상한이 없어 세 컨트롤러의 목표 속도(`desired_linear_vel`)가 0.25입니다. 실제로 내는 속도가 늘 그 값이라는 뜻은 아닙니다. 실행 묶음은 Nav2 goal 컨트롤러까지 세 군데를 `start.sh`의 속도(기본 0.12, 바꿀 수 있음)로 맞춘 사본을 씁니다. `pinky_fsm`의 `horizon_cap_mps`(기본 0.25)도 사본의 속도에 맞춥니다.
- keepout: 실물 `nav2.launch.py`는 `keepout_mask`가 비어 있어 꺼져 있습니다. 실행 묶음은 `maps/arena_v7_keepout.yaml`을 기본으로 넘깁니다.
- 시작 자세: `start.sh`는 X, Y, YAW를 꼭 받아 AMCL 초기 자세로 넘깁니다. launch만 직접 띄우면 기본값 P-1(0.12, 0.10, 90°)을 씁니다.

## 4. 실물과 Gazebo 기본값이 다른 것(실행 묶음에서도 그대로)

- 도킹: `pinky_dock`이 비어 있어 FSM의 dock()이 실패합니다. 관제는 DOCK_FAILED로 받고 2 s 뒤 다시 시도합니다. 물러났다 다시 들어가는 길은 최대 10 s 동안 잡아 보고, 길이 없거나 막혔거나 시간이 지나면 제자리에서 다시 시도합니다. 처음을 포함해 3회 실패하면 ROBOT_ISOLATED로 세웁니다. 그래서 실물에서는 주문 하나를 끝까지 돌릴 수 없고, 도크 앞에서 멈추는 것이 정상입니다.
- 스캔 필터: `narrow_strips`가 비어 있어 꺼져 있습니다.
- peer 장애물 층: 한 대만 달리므로 끕니다.
- 고정 선 컨트롤러의 충돌 검사: 켜져 있습니다(Gazebo는 꺼짐).

## 5. 첫 실물 시험 전에 확인할 것

- 지도: v7 지도(1.98 x 0.99 m)를 AMCL에 씁니다. 11-10 실측은 2.005 x 1.015 m라 줄자로 다시 재야 합니다.
- 배터리: 제조사 bringup의 배터리 토픽 이름을 확인하지 못해 `battery_topic`을 비워 두었습니다. 배터리는 -1(모름)로 알립니다.
- launch 안 컴포넌트: 종료 규칙은 프로세스만 봅니다. 달리기 전에 기록에서 "Failed to load"나 active가 되지 못한 lifecycle 노드를 확인합니다.
- 이 PC에는 `pinky_fsm`이 없어 launch의 FSM 단계는 import와 horizon cap 계산까지만 확인했습니다.
- 알려진 한계(README): `stop.sh`는 그룹 이름을 확인하지 않습니다. `rsync --delete`는 심볼릭 링크를 따라갈 수 있습니다. 시간 초과 때 pid 번호가 다른 프로세스에 다시 쓰일 수 있습니다. `nav2.launch.py`는 /tmp에 임시 yaml을 남깁니다.

## 6. Gazebo 확인 판(같은 날, 화면 확인용)

`main` 48ad1d4, v7, `gz_v7_rush`, 3대, 주문 모델 켬, 도킹은 시뮬레이션 대역, 도메인 61.

- 주문 12건 모두 출고, 275.7 s.
- 로봇 중심 간 최소 거리 0.236 m(fleet_check, Gazebo truth 위치).
- 기록된 표본에서 이른 해제, 구역 밖 점유, keepout 침범 0. 로그의 "backwards in time" 0건.

이 한 판은 화면 확인용이며 안전성을 증명하지 않습니다.

## 7. 리뷰

Codex 리뷰를 커밋마다 돌렸습니다. 실제 결함은 둘이었습니다: 0.25001이 반올림으로 통과하던 것(d7c272e), 시간 초과 때 내렸는지 확인하지 않던 것(6a885fe, 3de4fc1). 나머지 지적은 README의 알려진 한계로 적거나 시험을 보강했습니다(890e96e).
