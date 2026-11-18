# main 병합: 두 대 실물 시험 전 (2026-11-18)

사용자 요청(11-18): "fix/followups 브랜치랑 리얼 아레나 슬램 맵 브랜치 메인에 병합해, 테스트 한 번 돌리고, 핑키 쪽 코드 갱신할 거 준비하고. 주행 테스트 두 대로 할 거니까." 청소로봇 용도 한 대는 "일단 기다려, 두 대 실물 시험 먼저"라고 해서 이번 범위에서 뺐다.

병합 전 main은 ad2cd83(다른 세션의 refactor/yield-owner 병합)이었다. 브랜치 `merge/real-test-2026-11-18`에서 만들었다.

## 1. 들어간 것

| 브랜치 | 넣은 방법 | 내용 | 기록 |
|---|---|---|---|
| `fix/followups-2026-11-18` (40354a4) | 빨리 감기(main ad2cd83 위에 있었다) | 매장 상황을 v7에서 서게 고침, 쓰지 않는 station_left와 역 시간 추정 파라미터 제거, replay가 없는 파라미터를 건너뛰고 빠진 필수 파라미터는 거부, fleet_manager 시험이 실패한 판 폴더를 남김 | `docs/worklog_overnight_2026-11-18.md`, `docs/worklog_v7_store_2026-11-18.md` |
| `feat/real-arena-slam-map` (01b689b) | 병합 커밋 689e298(acfa45b에서 갈라졌다) | 실물 AMCL이 SLAM 지도 `arena_real_slam`을 읽음(전역 costmap은 도면 지도 그대로), v7 북쪽 벽 0.99 -> 0.98, 급행 중심선 0.90 -> 0.895, 시뮬과 실물 지도 다시 만듦 | `docs/worklog_real_slam_map_2026-11-17.md` |

| `feat/start-peer-robots` (e3c6ac1, cfa0de3) | 빨리 감기(689e298 위에서 만들었다) | `start.sh`가 `PEER_ROBOTS`로 peer 층을 켠다(5절). 두 대 실물 시험 준비로 이 세션에서 만들었다 | 이 기록 5절 |

- 병합은 충돌 없이 됐다.

## 2. 시험

병합 커밋 689e298에서 돌렸다.

- `colcon build`(arm_control, arm_vision 제외): 13개 패키지 통과.
- `colcon test`(ROS_DOMAIN_ID 66): fleet_interfaces, fleet_core, fake_robots, fleet_gz_sim, iponic_bringup, central, pode_* 여섯. 3400개 중 실패 0, 오류 0, 건너뜀 213.
- fleet_manager 시험은 돌리지 않았다(도메인 50~59를 스스로 잡는데, 그 대역은 사용자가 쓴다).
- `tools/test/test_real_pode.py`(colcon 밖): start.sh 수정을 넣은 cfa0de3에서 58 passed.
- 매장 상황 다섯을 v7 씨드 1로 병합 전(40354a4)과 병합 뒤(689e298)에 직접 돌려 `failures`를 비교했다. 스크립트는 세션 임시 폴더에 있다(저장소 밖).
  - 넷(person_in_express, person_at_dock, corridor_box, box_stays)은 양쪽 다 0이다.
  - box_removed는 양쪽 다 `I5_STANDING` 2개다(pode3이 J_E에 섬). 병합 전 55.6/60.7 s와 132.2/137.3 s, 병합 뒤 55.2/60.3 s와 132.0/137.1 s.
  - 병합 전 값은 `worklog_v7_store_2026-11-18.md`에 적힌 값과 같다. 병합으로 새 실패는 생기지 않았다. 시각이 0.2~0.4 s 당겨진 것은 급행 중심선 이동 때문으로 본다 [추측].
- Gazebo 판은 돌리지 않았다.

## 3. 병합 리뷰가 짚은 것 (Codex, 689e298)

- 두 브랜치 사이에 좌표나 구역 이름이 직접 부딪치는 곳은 찾지 못했다. 매장 상황의 급행 장애물은 구역 경계에서 가운데를 계산하므로 새 0.895를 따른다.
- 병합 전부터 있던 제약:
  - `start.sh`는 peer 층을 켤 수 없다(5절).
  - 실물에는 도킹이 없다(`pode_fsm` dock()이 DOCK_UNAVAILABLE). WORK/OUTBOUND 주문은 도크에서 멈춘다.
- 급행 중심선 0.895는 도면 기준으로 북쪽 벽까지 몸 원 여유가 약 2 mm다. 기록된 실측 벽값으로는 음수다. 벽 측정에 ±1~2 cm 불확실성이 있어 단정하지 못한다. 두 대가 출고를 바꿔 가며 `O_X`에서 기다리는 장면을 실물에서 눈으로 본다.
- `test_store_situations`의 v7 끝까지 돌기 시험은 행 길이와 배치·씨드만 보고 `failures`는 보지 않는다. 그래서 병합 전후로 직접 돌려 비교했다(2절).
- `test_zone_release`의 O_OUT 앞 정지 시험은 로봇을 O_OUT 0.04 앞에 직접 둔다. 실제 멈춘 자리는 더 검사하지 않는다(SLAM 기록에 적힌 절충).

## 4. 두 대 실물 시험 순서

모두 파일에서 읽은 것이다. 실물에서 돌려 본 것은 아니다.

- **같은 커밋:** 로봇 두 대와 관제 PC에 같은 main 커밋을 쓴다.
- **도메인:** `tools/real_pode/env.sh`는 `ROS_DOMAIN_ID=12`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`이다. PC에서도 같은 값을 export한다.
- **관제 PC를 먼저 띄운다.** 로봇은 관제 heartbeat가 1.5 s 끊기면 선다(`real_pode.launch.py`의 `heartbeat_timeout_s`).
  - `ros2 launch iponic_bringup pc.launch.py use_fake:=false map_file:=v7 robots:="pode1 pode2"`
  - `robots` 기본값은 `"pode1 pode2 pode3"`다. 두 대면 꼭 준다. 이름은 로봇 namespace와 글자까지 같아야 한다.
  - `map_file`을 안 주면 d2 배치를 읽는다.
  - `use_fake:=false`에서는 `fleet_manager`만 뜬다. 가짜 central이 없으니 주문은 바깥(OrderBoard 토픽)에서 와야 한다.
- **로봇마다 배포:**
  - 옛 이름(pinky)으로 배포된 로봇이면 배포 전에 옛 launch를 옛 `stop.sh`로 내리고 `~/nav2fleet_ws/src/pinky/`, `build/`, `install/`을 치운다.
  - PC에서 `tools/real_pode/deploy.sh pinky@<주소>`(커밋된 것만 간다). 로봇에서 `cd ~/nav2fleet_ws && colcon build`.
- **로봇마다 시작:** v7 주차 자리는 P-1 (0.12, 0.10), P-2 (0.38, 0.10), P-3 (0.64, 0.10), 모두 yaw 90도다.
  - 예: `PEER_ROBOTS=pode1,pode2 ~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708`, `PEER_ROBOTS=pode1,pode2 ~/nav2fleet_real/start.sh pode2 0.38 0.10 1.5708`.
  - 지도는 인자 없이 기본값이 쓰인다. AMCL은 `arena_real_slam`, 전역은 `arena_v7_a`, keepout은 `arena_v7_keepout`.
  - 끌 때는 로봇마다 `~/nav2fleet_real/stop.sh`.
- **peer 층:** 5절.

## 5. peer 층과 start.sh

- `real_pode.launch.py`의 `peer_obstacles` 기본값은 false다. README는 두 대 이상이면 `peer_obstacles:=true peer_robots:=pode1,pode2`를 쓰라고 한다.
- 병합 전 `start.sh`는 namespace와 초기 자세만 launch에 넘겨서 이 둘을 줄 수 없었다. launch를 직접 부르면 `start.sh`가 하는 속도 사본, `env.sh`, pid 기록(`stop.sh`가 쓴다)이 빠진다.
- 그래서 `feat/start-peer-robots`(e3c6ac1)를 같이 넣었다.
  - 환경 변수 `PEER_ROBOTS`(예: `pode1,pode2`)가 있으면 `start.sh`가 launch에 `peer_obstacles:=true peer_robots:=<목록>`을 붙인다. 없거나 비면 launch 명령은 예전과 같다. 인자 4/5개 검사도 그대로다.
  - 이름은 `^[a-z_][a-z0-9_]*(,[a-z_][a-z0-9_]*)*$`만 받는다(ROS 이름은 숫자로 시작할 수 없다, cfa0de3). 틀리면 아무것도 띄우기 전에 exit 2로 멈춘다.
  - 두 로봇에 같은 목록(두 이름 다)을 줘도 된다. peer 노드가 자기 이름을 뺀다(`pode/pode_peer_obstacles/pode_peer_obstacles/body.py`의 `others()`).
  - 시험(`tools/test/test_real_pode.py`, 대역 `ros2`로 launch 인자만 확인): 새 시험 3개를 먼저 넣었을 때 7 failed 47 passed, 바꾼 뒤 54 passed.
  - 리뷰 반영(cfa0de3): 시험이 ros2 인자 전체를 예전 인자와 비교하고 여러 목록을 쓴다. 숫자로 시작하는 이름 두 경우를 먼저 넣어 2 failed를 본 뒤 정규식을 고쳐 58 passed.
  - 실제 launch와 로봇에서는 아직 돌려 보지 않았다.

## 6. 로봇에 올린 것 (11-18, 사용자 지시 "27a6 핑키 ssh로 접속해서 최신화", "하나는 07d7")

| 로봇 | 주소 | 전 상태 | 한 일 |
|---|---|---|---|
| pinky_27a6 | 192.168.0.200 (MAC e4:5f:01:b9:27:a7) | 03a100d가 `pode_*` 이름으로 배포돼 있었다. 옛 `pinky_*` 없음. launch 안 돎 | cfa0de3 배포, `colcon build` 7개 패키지 통과(50.1 s) |
| pinky_07d7 | 192.168.0.201 (MAC d8:3a:dd:26:07:d8) | 이 저장소 코드 없음(어제 SLAM은 제조사 bringup만 썼다). launch 안 돎 | cfa0de3 첫 배포, `colcon build` |

- 배포는 `feat/start-peer-robots` worktree의 HEAD(cfa0de3)에서 했다. 이 기록 커밋만 빼면 main과 같은 코드다. 로봇의 `~/nav2fleet_ws/src/SOURCE_COMMIT`에 cfa0de3을 적었다.
- 빌드 기록: 로봇의 `~/nav2fleet_ws/build_cfa0de3.log`.
- 07d7의 `.bashrc`는 `ROS_DOMAIN_ID=4`(BLE 설정)다. `start.sh`는 `env.sh`로 12를 쓰지만, 로봇에서 손으로 `ros2` 명령을 칠 때는 먼저 `source ~/nav2fleet_real/env.sh`를 한다. 27a6의 `.bashrc`는 12다.
- peer 원 반지름은 0.075다(11-17 결정). 상대 위치는 각 로봇의 AMCL `/<robot>/state`에서 오므로 AMCL이 틀리면 원도 그만큼 밀린다.
