# 이름 변경: pinky → Pode, GG → IFP (2026-11-16)

브랜치 `feat/rename-pode`. 바탕은 `feat/real-pinky-kit`(192c221)이고, 그 위에 커밋 4167641, bf02395, f42a626을 쌓았습니다.

## 1. 왜

팀 발표 자료(IFP 주간 진행 보고, "Architecture 변경" 쪽, v2 11/13)에서 이름을 정했습니다.

| 구분 | 전 | 후 |
|---|---|---|
| 프로젝트 | GG (Give Gift) | IFP (Intelligent Flying Pod) |
| 주행 로봇 ×3 | PINKY-PRO | Pode |
| 작업 공간 양팔 | Jetcobot ×2 | Phody |
| 출고 구역 팔 | Jetcobot ×1 | Phony |

이번에는 주행 로봇(Pode)과 프로젝트 이름만 바꿨습니다. 팔(`arm1~3`)은 바꾸지 않았습니다.

## 2. 바꾼 것 (4167641, 파일 305개)

- 로봇 이름: `pinky1~4` → `pode1~4`. 토픽, 액션, 시나리오, 관제 기본값, 시험, golden 기록이 모두 바뀌었습니다.
- 폴더와 패키지:
  - `pinky/` → `pode/`
  - `pinky_fsm`, `pinky_nav_config`, `pinky_avoidance`, `pinky_peer_obstacles`, `pinky_scan_filter`, `pinky_dock` → `pode_*`
  - 같이 바뀐 것: package.xml, setup.py, setup.cfg, CMakeLists, resource 표시 파일, 실행 파일 이름
- 계약: `contract.yaml`의 자리표 `{pinky}` → `{pode}`. 코드의 `name(..., pinky=robot)` → `pode=robot`. `config/robots.yaml`의 `type: pinky` → `type: pode`.
- 실물 묶음: `tools/real_pinky/` → `tools/real_pode/`, `real_pinky.launch.py` → `real_pode.launch.py`, `test_real_pinky.py` → `test_real_pode.py`.
- 그 밖의 파일과 클래스:
  - 파일: `fake_pinky_node` → `fake_pode_node`, `one_pinky.launch.py` → `one_pode.launch.py`, `pinky_sim.urdf.xacro` → `pode_sim.urdf.xacro`, `pinky_nav.rviz` → `pode_nav.rviz`
  - 클래스: `PinkyFsm` → `PodeFsm`, `FakePinky` → `FakePode`
  - 코드 주석의 "핑키"도 "Pode"로 바꿨습니다.
- 방법: 스크립트로 바꿨습니다. 그대로 둘 이름을 먼저 표시해 두고, `pinky` / `Pinky` / `PINKY` / `핑키`를 바꾼 뒤 표시를 되돌렸습니다. 경로는 `git mv`로 옮겼습니다. `docs/`와 `*/results/`(실행 기록)는 건드리지 않았습니다.

## 3. 그대로 둔 것

- 제조사 이름: `pinky_pro`, `pinky_description`, `pinky_bringup`, `pinky_navigation`, `pinky_gz`(`pinky_gz.urdf.xacro`, `pinky_gz_sim`), `urdf/pinky.urdf.xacro`
- 외부 이름: 팀 저장소의 `pinky_multi_sim`, `~/pinky_deps_ws`, `~/pinky_pro`
- 실제 장비 이름: 로봇 호스트 `pinky_27a6`, `pinky_06c5`, ssh 사용자 `pinky@`, `/home/pinky`
- git 브랜치 이름: `feat/pinky-drive`, `feat/real-pinky-kit`
- 저장소 이름 `roscamp-nav2fleet`, 작업 공간 `nav2fleet_ws`, 팀 이름 `iponic`(`iponic_bringup`)
- 날짜가 붙은 기록 문서(`worklog_*_2026-*`, `실측_2026-11-10.md`)와 `결과_*`, `results/` 폴더는 그때 이름 그대로 둡니다. 다만 `worklog_nav2base_2026-11-12.md`의 계약 문서 링크 하나는 대상만 새 파일 이름으로 고쳤습니다.
- 팀 아키텍처 그림의 FMS 모듈 이름 "pinky"(core, pinky, arm)는 그림에 적힌 이름이라 `IFP_공통설계.md`에서 그대로 둡니다.
- fleet 패키지(`fleet_core`, `fleet_manager`, `fleet_interfaces`, `fleet_gz_sim`)에 `ifp_` 접두어는 붙이지 않았습니다.

## 4. 문서 (bf02395)

- `docs/GG_공통설계.md` → `docs/IFP_공통설계.md`. 제목을 "IFP (Intelligent Flying Pod)"로 바꾸고, 제목 아래에 이름 변경 안내를 넣었습니다.
- `docs/핑키_계약.md` → `docs/Pode_계약.md`
- 날짜 없는 문서 7개도 같은 규칙으로 바꿨습니다: `COPIED_FROM.md`, `order_모델.md`, `팀레포_옮기기.md`, `막힘_처리_결과.md`, `관제_구역교통.md`, `주행_계획서.md`, `회피_약속.md`.
- 외부 문서의 실제 제목은 그대로 둡니다: Confluence "주제1 : GG", Drive "02_프로젝트_설계_초안_GG", "Pinky Pro 3대 시뮬레이션 관제 정리".

## 5. 리뷰에서 나온 것 (f42a626)

- golden 기록의 칸 맞춤(실제 결함):
  - `zone_harness.py:132`는 로봇 이름을 7칸에 맞춥니다.
  - `pinky1`(6자)에서 `pode1`(5자)로 줄어 공백이 한 칸 더 필요했습니다.
  - `test_zone_order_off.py`가 실패했고, golden 기록(`zone_off_golden.txt`) 132줄의 공백을 고친 뒤 통과했습니다(`zone_harness.py`는 그대로).
- `IFP_공통설계.md`에서 그림의 모듈 이름 하나가 `pode`로 잘못 바뀐 것을 `pinky`로 되돌렸습니다.
- `팀레포_옮기기.md`의 `order_fsm.py` 경로에 모듈 폴더가 빠져 있었습니다. 이름 변경 전부터 있던 오류입니다.
- `tools/real_pode/README.md`에 옛 이름으로 배포된 로봇에 다시 배포하는 순서를 적었습니다.
  - 새 `stop.sh`는 옛 `real_pinky.launch.py`를 내리지 못합니다.
  - `deploy.sh`는 옛 `pinky_*` 소스와 빌드를 지우지 않습니다.

## 6. 확인

- 빌드: 15개 패키지 모두 성공했습니다. 별도 작업 공간에서 `/opt/ros/jazzy`, `~/pinky_pro`, `~/pinky_deps_ws`를 source한 상태입니다.
- 시험 (패키지마다 `python3 -m pytest -q test`, 도메인 71~79, LOCALHOST, Fast DDS):

| 패키지 | 결과 |
|---|---|
| pc/fleet_core | 처음 1 failed, 744 passed: `test_zone_order_off` golden 칸 맞춤(5절). 고친 뒤 745 passed |
| pode/pode_fsm | 389 passed, 6 skipped |
| pode/pode_nav_config | 102 passed |
| pode/pode_avoidance | 26 passed |
| pode/pode_peer_obstacles | 18 passed |
| pode/pode_scan_filter | 23 passed |
| sim/fake_robots | 1000 passed |
| sim/fleet_gz_sim | 527 passed |
| tools/test | 96 passed |
| pc/fleet_manager | 124 passed, 1 warning |

- fleet_core의 고친 뒤 결과(745)는 로컬 `nav2fleet_ws` 새 빌드로 다시 돌린 것입니다. 나머지는 첫 실행 결과입니다. 고친 golden은 fleet_core 시험만 읽습니다.
- 로컬 작업 공간: 로컬 `main`을 f42a626으로 앞으로 옮겼습니다. 옛 `build/`, `install/`, `log/`는 `~/ros2camp/archive_2026-11-16_pinky_build/`로 옮겼고, 다시 빌드해 15개 모두 성공했습니다. 설치된 것에 `pinky_*`는 없습니다.

### Gazebo 확인 판 (화면 확인용, 한 판)

- 설정:
  - 로컬 `main` f42a626, 로컬 `nav2fleet_ws` 새 빌드
  - v7, `gz_v7_rush`, 3대(`pode1~3`), 주문 모델 켬, 도킹은 시뮬레이션 대역
  - 도메인 61, LOCALHOST, Gazebo 화면과 RViz
- 결과:
  - 세 대 모두 IDLE이 된 뒤 관제가 시작했습니다.
  - 주문 12건 모두 출고, 275.8 s, scenario PASS(위반 0). 같은 날 이름 변경 전 확인 판은 275.7 s였습니다.
- fleet_check(Gazebo truth 위치, 기록된 표본 기준):
  - 로봇 중심 간 최소 거리 0.237 m(전 판 0.236 m)
  - 쥔 구역 밖 표본 0, keepout 침범 0, 실시간 배율 0.999
- 로그:
  - "backwards in time" 0건.
  - 주행 중 ERROR는 1줄입니다: `pode1` bt_navigator "Failed to get result for follow_path in node halt!"(Nav2가 하던 목표를 멈출 때 내는 줄).
  - 시나리오가 끝난 뒤 내리는 단계에서 다음이 나왔습니다. 결과 파일은 모두 쓰인 뒤입니다.
    - Nav2 컨테이너 3개가 exit -6으로 끝났습니다.
    - fleet_check가 rclpy 종료 예외로 exit 1로 끝났습니다.
    - "cannot use Destroyable" 줄들이 나왔습니다.
  - 이름 변경 전 판의 종료 로그는 남아 있지 않아 비교하지 못했습니다.
- 이 한 판은 이름이 끝까지 이어지는지 보는 확인이며, 안전성을 증명하지 않습니다.

## 7. 남은 것

- 로봇 pinky_27a6에는 아직 옛 이름(`pinky_*`, 커밋 3de4fc1)이 빌드되어 있습니다. 다시 배포하려면 README의 순서를 따릅니다.
- 팔 이름(`arm1~3` → Phody, Phony)은 바꿀지와 이름 짓는 방식을 팀과 정해야 합니다.
- 팀 아키텍처 그림의 FMS 모듈 이름 "pinky"는 그림을 그린 팀원이 정할 일입니다.
