# 실물 시험을 디스커버리 서버로: net.sh, pc_env.sh, 서버 스크립트 (2026-11-19)

Drive 문서 `nav2fleet_다음_작업_순서_2026-11-19`의 0번(디스커버리 서버). 사용자 지시(11-19): "그럼 진행해"(바로 앞에서 정한 것: env.sh와 PC 쪽을 코드로 굳히고, pc_env.sh가 셋업까지 읽고, 서버 실행 스크립트를 둔다).

## 결론

- 실물 시험의 로봇(`start.sh` → `env.sh`)과 PC 터미널(`pc_env.sh`, `rviz_pc.sh`)이 기본으로 관제 PC의 Fast DDS 디스커버리 서버 `192.168.0.252:11811`에 붙는다. `DISCOVERY_SERVER=off`를 붙이면 예전처럼 멀티캐스트로 찾는다.
- 로봇 명령(`start.sh pode1 …`)과 RViz 명령(`rviz_pc.sh pode1 pode2`)은 그대로다. 새로 생긴 것은 서버 터미널 하나(`discovery_server.sh`)와 PC 관제 터미널의 `source pc_env.sh` 한 줄이다.
- PC에서 돌린 시험은 모두 통과했다(127 통과, 2 건너뜀). **실물 로봇과 Wi-Fi에서는 아직 돌리지 않았다.** 로봇에 배포하지도 않았다.
- 브랜치 `feat/discovery-server`: 212d1f7(코드와 시험), 9b97339(README), cf8c5a1(리뷰 반영: `pc_env.sh` 실패 전달, 시험 보강), b2b70df(리뷰 반영: README), 그리고 이 기록. main에는 합치지 않았다.

## 1. 바꾼 것 (`tools/real_pode/`)

| 파일 | 내용 |
|---|---|
| `net.sh` (새 파일) | 통신 설정을 한곳에 모은다. 로봇(`env.sh`)과 PC(`pc_env.sh`)가 같이 읽는다. 먼저 `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `ROS_LOCALHOST_ONLY`, `ROS_STATIC_PEERS`를 지우고, `ROS_DOMAIN_ID=12`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`을 둔다. `DISCOVERY_SERVER`(기본 `192.168.0.252:11811`, `off`면 서버 없음)로 `ROS_DISCOVERY_SERVER`를 정하고, `SUPER=1`이면 `ROS_SUPER_CLIENT=TRUE`(off일 때는 무시) |
| `env.sh` (로봇) | ROS, `~/pinky_pro`, `~/nav2fleet_ws`를 읽는 것은 같다. 도메인과 SUBNET 줄을 빼고 옆의 `net.sh`를 읽는다. ROS setup 경로는 `ROS_SETUP`으로 바꿀 수 있다(시험용) |
| `pc_env.sh` (새 파일) | PC 터미널에서 source한다. `/opt/ros/jazzy/setup.bash`와 `~/ros2camp/nav2fleet_ws/install/setup.bash`를 읽고(`ROS_SETUP`, `NAV2FLEET_WS_SETUP`로 바꿀 수 있다) `net.sh`를 읽는다. setup을 읽는 동안 nounset을 끄고, 부른 셸에서 켜져 있었을 때만 다시 켠다. ROS setup이 없거나 실패, 작업공간 setup이 있는데 실패, `net.sh`를 못 읽으면 셸을 닫지 않고 `return 1`(cf8c5a1). 작업공간 파일이 아예 없으면 알리고 ROS만 읽는다. 보조 변수를 남기지 않는다 |
| `discovery_server.sh` (새 파일) | ROS를 읽고 서버 변수를 지운 뒤 `exec fastdds discovery -i 0 -p 11811`(앞에서 돈다). 주소를 주지 않아 유선과 Wi-Fi 모두에서 듣는다. 끌 때는 Ctrl+C |
| `rviz_pc.sh` | 직접 두던 setup, 도메인, SUBNET 줄을 `SUPER=1 source pc_env.sh`로 바꿨다. `DISCOVERY_SERVER=off rviz_pc.sh …`도 된다 |
| `README.md` | 파일 표, "디스커버리 서버 (기본)" 절(시험 날 순서, 서버 없이 돌리기, 알아 둘 것), "첫 실행 순서" 6번 (서브에이전트가 쓰고 내가 코드와 대조했다) |

도메인 번호는 팀이 아직 정하지 않았다(11-19 확인). 정해지면 `net.sh`의 한 줄만 바꾸면 로봇, PC 관제, RViz가 같이 바뀐다.

## 2. 시험 날 명령 (9줄)

| 터미널 | 줄 |
|---|---|
| PC 서버 | `cd ~/ros2camp/nav2fleet_ws/src/roscamp-nav2fleet && tools/real_pode/discovery_server.sh` (켜 둔다) |
| PC 관제 | `source ~/ros2camp/nav2fleet_ws/src/roscamp-nav2fleet/tools/real_pode/pc_env.sh` 뒤 평소의 `ros2 launch iponic_bringup pc.launch.py …` |
| 로봇 pode1 | `ssh pinky@192.168.0.200` 뒤 `PEER_ROBOTS=pode1,pode2 DOCK_STUB=1 ~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708` |
| 로봇 pode2 | `ssh pinky@192.168.0.201` 뒤 `PEER_ROBOTS=pode1,pode2 DOCK_STUB=1 ~/nav2fleet_real/start.sh pode2 0.38 0.10 1.5708` |
| PC RViz | `tools/real_pode/rviz_pc.sh pode1 pode2` |

확인용 터미널(선택): `SUPER=1 source …/pc_env.sh`, `ros2 daemon stop`, `ros2 daemon start`, 몇 초 뒤 `ros2 node list`.

## 3. 확인한 것

- **시험을 먼저 썼다.** 코드 전에 돌려 `test_real_pode.py`의 새 시험 25개, `test_real_rviz.py`의 3개가 실패하는 것을 봤다.
- **시험 결과:** `python3 -m pytest -q tools/test/test_real_pode.py tools/test/test_real_rviz.py` → 212d1f7에서 120 통과, cf8c5a1에서 127 통과, 둘 다 2 건너뜀(각 1분 53초). cf8c5a1의 새 실패 경로 시험 5개는 고치기 전 코드에서 실패하는 것을 먼저 봤다. 건너뛴 2개는 이번에 건드리지 않은 시험(`test_real_pode.py:321`)으로, 작업공간을 읽지 않은 셸이라 `pode_fsm not installed`로 건너뛴다.
- 새 시험이 보는 것:
  - 다른 값(도메인 6, Cyclone, LOCALHOST, 옛 서버 주소, 슈퍼 클라이언트, LOCALHOST_ONLY, STATIC_PEERS)이 남은 셸에서 시작해도 결과가 기본값, 다른 주소, off, SUPER=1이 된다.
  - 한 셸에서 super → 보통 → off로 다시 source해도 옛 값이 남지 않고, 앞에 붙인 `SUPER`, `DISCOVERY_SERVER`가 셸에 남지 않는다.
  - 부른 셸이 `set -u`이든 아니든 `net.sh`가 동작한다. `pc_env.sh`는 정의되지 않은 변수를 읽는 setup 대역을 읽고도 nounset 상태를 원래대로 둔다.
  - `env.sh`는 다른 폴더에서 source해도 옆의 `net.sh`를 읽는다.
  - 진짜 `start.sh`+`env.sh`+`net.sh`를 돌려, 대역 `ros2`(launch 자리)가 받은 환경이 서버 모드이고 off일 때 멀티캐스트 모드인지 본다.
  - 대역 `rviz2` 세 개가 모두 도메인 12, fastrtps, 서버 주소, 슈퍼 클라이언트로 뜨는지, off일 때 서버 변수가 비는지 본다.
  - `discovery_server.sh`가 `exec`로 `fastdds discovery -i 0 -p 11811`이 되고(대역의 PID가 스크립트 PID와 같다, 포트는 `net.sh` 기본 주소와 같다), 남아 있거나 setup이 export한 서버 변수가 서버에 가지 않고, ROS setup이 없으면 1로 끝나는지 본다.
  - `pc_env.sh`가 ROS setup 실패, 작업공간 setup 실패, `net.sh` 없음에서 1을 돌려주고, 실패 경로에서도 부른 셸의 nounset을 원래대로 두는지 본다.
- **PC의 진짜 setup으로**(cf8c5a1에서 다시 확인): 진짜 ROS setup과 작업공간 setup은 둘 다 0을 돌려준다. 빈 환경(`env -i`), `set -u`, `SUPER=1 source pc_env.sh` → rc 0, nounset 그대로 켜짐, `SUPER`와 보조 변수는 남지 않음, 도메인 12 / fastrtps / SUBNET / `192.168.0.252:11811` / `ROS_SUPER_CLIENT=TRUE`, `ros2 pkg prefix fleet_manager`, `pode_fsm`이 PC 작업공간을 가리킴.
- **진짜 fastdds로 `discovery_server.sh`:** 4초 띄워 `0.0.0.0:11811` 수신, "Server is running"을 봤다. 프로세스 그룹에 SIGINT(Ctrl+C와 같다)를 보내 "Server shut down", 남은 프로세스 0, 포트 비었음을 확인했다. `exec` 뒤에는 스크립트 프로세스가 `fastdds`(sh) 자리가 되고 python과 `fast-discovery-server`가 같은 그룹에 있어, 터미널의 Ctrl+C가 셋 모두에 간다.
  - 실수 하나: 첫 SIGINT는 `setsid`가 새로 갈라져 나온 탓에 다른 그룹에 갔다. 서버가 남은 것을 보고 정확한 그룹(sh, python, 서버 3개)에 다시 보내 껐다.

## 4. 돌리지 않은 것 (사용자 몫)

- 로봇 배포와 빌드: `tools/real_pode/deploy.sh pinky@192.168.0.200`, `.201`. 그 뒤 로봇마다 `cd ~/nav2fleet_ws && colcon build`(로봇의 마지막 배포는 cfa0de3이고, 그 뒤 interfaces와 pode 패키지가 바뀌었다).
- 배포 전 로봇 확인: `rmw_fastrtps_cpp`가 있는지, `.bashrc`의 `RMW_IMPLEMENTATION`, Fast DDS/Cyclone XML 경로 변수, Fast DDS 버전.
- 실물 확인: 로봇 두 대 사이 데이터(서로의 토픽 한 건), Wi-Fi에서 서버를 껐다 켰을 때(이미 찾은 연결은 유지, 새 노드는 못 찾음, 다시 켜면 회복), off 모드, 관제와 짧게 같이 돌리기, 멀티캐스트 캡처(PC에서 로봇 IP로 걸러 보는 것은 약한 대용).
- 팔(JetCobot)은 이번에 넣지 않았다.

## 5. 알아 둘 것

- **배포 뒤에는 서버가 기본이다.** 서버를 켜지 않고 `start.sh`만 돌리면 로봇과 PC가 서로 찾지 못한다. 서버를 먼저 켜거나, 예전처럼 하려면 양쪽 모두 `DISCOVERY_SERVER=off`.
- 서버 아래에서는 도메인 번호로 갈라지지 않는다(11-19 PC 시험). 같은 서버에 붙으면 다른 도메인끼리도 보인다.
- `ROS_DISCOVERY_SERVER`를 PC `~/.bashrc`에 넣지 않는다. Gazebo 시뮬은 이 변수가 있으면 띄우지 않는다(`sim/fleet_gz_sim/fleet_gz_sim/isolation.py:18`, `:60`).
- 로봇 `.bashrc`의 도메인은 서로 다르다(27a6 = 12, 07d7 = 4). `start.sh`는 `net.sh`로 12를 덮는다. 로봇에서 ros2 명령을 손으로 칠 때는 먼저 `source ~/nav2fleet_real/env.sh`.
- 그대로 둔 것(사용자 결정 대기): `isolation.py:3`의 "기본 도메인 6" 주석, Drive 문서의 "팀 규칙은 6" 문장.

## 6. 리뷰 (Codex, 커밋마다)

- **212d1f7 (코드):** 기본 동작과 다시 source하는 경로는 맞다고 봤다. 받아들인 지적:
  - `pc_env.sh`가 setup이나 `net.sh` 실패에도 0을 돌려줘, `rviz_pc.sh`가 옛 설정으로 뜰 수 있었다 → cf8c5a1에서 고쳤다(시험 먼저).
  - 서버 시험이 `exec`와 서버 변수 지우기를 보지 않았다. RViz 시험이 창 셋을 다 세지 않고 빈 값과 없는 값을 가르지 않았다. ROS가 없는 실패 경로 시험이 nounset 복원을 보지 않았다 → cf8c5a1에서 보강했다. 이 넷은 코드는 이미 맞았고 시험만 약했다.
  - "서버 없이 예전 명령 그대로 쓰던 흐름이 바뀐다": 의도한 변화다. README와 이 기록 5절에 적었다.
- **9b97339 (README):** 받아들인 지적은 b2b70df에서 고쳤다(서브에이전트가 쓰고 내가 코드와 대조했다).
  - 첫 실행 순서에서 서버를 로봇보다 먼저 켠다.
  - PC 명령은 저장소 루트에서 친다. `rviz_pc.sh`에 경로를 붙인다.
  - 작업공간 경고가 보이면 PC 작업공간을 먼저 빌드한다.
  - `SUPER`, `DISCOVERY_SERVER`는 앞에 붙여 쓰고 export하지 않는다.
  - 다른 포트를 쓰면 서버를 직접 그 포트로 띄운다.
  - 시험 결과마다 "PC 한 대, Wi-Fi·로봇 미확인" 범위를 붙인다.
- **받아들이지 않은 지적 하나:** "확인 절에서 새 설정을 source한 뒤 daemon을 멈추면 옛 daemon을 못 멈출 수 있다." ros2 daemon은 도메인마다 따로다(포트 11511 + `ROS_DOMAIN_ID`, `/opt/ros/jazzy/lib/python3.12/site-packages/ros2cli/daemon/__init__.py:34-37`). 새 셸에서 멈추고 다시 띄우는 것은 그 셸이 쓸 도메인 12 daemon이므로 순서가 맞다.
- **cf8c5a1, b2b70df, 이 기록:** 리뷰를 커밋 직후 시작했다. 결과는 다음 커밋에 덧붙인다.
