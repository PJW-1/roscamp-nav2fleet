# main 병합: 실물 시험 전 최신화 (2026-11-17)

사용자 요청(11-17): "전체적으로 메인에 병합해서 최신화해. 주행 쪽이든 관제 쪽이든 이제 실물 시험 할 거니까, 맵만 읽으면."
병합 전 main은 9261ad5였다. 브랜치 `merge/real-test-2026-11-17`에서 만들어 main으로 빨리 감기했다.

## 1. 들어간 것

| 브랜치 | 내용 | 기록 |
|---|---|---|
| `feat/v7-no-divider3` (74220fb..1cc4cdb) | 충전 칸막이 3과 그 keepout을 뺐다. 진열대 A를 서쪽으로 0.63까지 늘렸다(0.68 -> 0.63). world, 시뮬 지도, `tools/real_pode/maps`를 다시 만들었다 | `docs/worklog_v7_divider3_shelfA_2026-11-17.md` |
| `fix/local-padding-zero` (bc5adb8..2e70916) | local_costmap `footprint_padding` 0(전에는 안 적어서 기본 0.01). peer 층 원 반지름 기본값 0.075(전 0.08) | `docs/worklog_p3_gap_fix_2026-11-17.md` |
| `docs/oout-wobble-measure` | 문서와 분석 스크립트만 | `docs/worklog_oout_wobble_2026-11-16.md` |
| `exp/wedge-clear-timeout-off`의 기록 파일 하나 | 문서만. 그 브랜치의 코드(BT clear 대기 끔)는 실험이라 넣지 않았다 | `docs/worklog_wedge_p3_diagnosis_2026-11-17.md` |

- 기록 문서는 쓴 날의 상태를 적은 것이다. 예를 들어 O_OUT 흔들림 기록(11-16)의 "local padding 기본 0.01"과 O_IN 좌표는 그 뒤에 바뀌었다. 지금 값은 설정 파일과 이 기록을 본다.
- `docs/oout_wobble_2026-11-16/`의 분석 스크립트는 import하면 바로 실행된다. 직접 실행하는 용도이고, 어느 패키지도 이를 부르지 않는다.

## 2. 넣지 않은 것

- `feat/nav-goal-smooth`: goal 컨트롤러가 굽은 길마다 서지 않게 하는 avoidance.yaml 변경이다. 커밋에 "Not merged into main; the user decides"라고 적혀 있다. 비용으로 주차 들어갈 때 한 판에 2~4 s 멈춤이 있고, 장애물 판에서는 이득이 없었다.
- `feat/nav-goal-bt-backup`: 보류 중이다. 끼임을 줄인다는 근거가 없고, 계약서도 BackUp을 쓰지 않는다고 적는다.
- `exp/p3-scene-test`: P-3 장면 시험 도구다. 실험용이라 넣지 않았다.
- `feat/relocalize-*`: B안이고 병합하지 않는다.
- `feat/rename-pode`: 다른 세션의 문서다.

## 3. 시험

- `colcon build`(arm_control, arm_vision 제외): 13개 패키지가 통과했다.
- `colcon test`(ROS_DOMAIN_ID 66): fleet_interfaces, fleet_core, fake_robots, fleet_gz_sim, iponic_bringup, central, pode_* 여섯 패키지를 돌렸다. 3407개 시험 가운데 실패 0, 오류 0, 건너뜀 212다.
- fleet_manager 시험은 돌리지 않았다. 이 시험은 도메인 50~59를 스스로 잡는데, 그 대역은 사용자가 쓴다.
- Gazebo 세 대 판은 이 병합으로 돌리지 않았다.
  - 배치 변경은 브랜치에서 두 판 PASS 12/12였다(`worklog_v7_divider3_shelfA_2026-11-17.md`).
  - padding 0과 원 0.075는 P-3 장면 반복 시험만 했다. 세 대 판은 없다.

## 4. 실물 시험 전에 알 것

모두 파일에서 읽은 것이다. 실물에서 돌려 본 것은 아니다.

- **로봇과 PC에 같은 커밋을 올린다.** 관제는 `fleet_map_v7.yaml`을, 로봇은 `tools/real_pode/maps`를 쓴다. 이번 병합이 둘 다 바꿨다.
- **로봇:**
  - 옛 이름(pinky)으로 배포된 로봇이면 배포 전에 옛 launch를 옛 `stop.sh`로 내린다. `deploy.sh`가 `~/nav2fleet_real/`의 `stop.sh`를 새것으로 덮어쓰고, 새 `stop.sh`는 옛 launch를 찾지 못한다. `~/nav2fleet_ws/src/pinky/`와 `build/`, `install/`도 배포 전에 치운다. `deploy.sh`는 옛 것을 지우지 않는다(README "옛 이름으로 배포된 로봇에 다시 배포할 때").
  - PC에서 `tools/real_pode/deploy.sh pinky@<주소>`를 실행한다(커밋된 것만 간다). 그다음 로봇에서 `cd ~/nav2fleet_ws && colcon build`를 한다(`tools/real_pode/README.md` 첫 실행 순서).
  - `start.sh`는 실행할 때마다 `make_params.py`로 저장소의 nav2_params.yaml 사본을 만든다. 사본에서는 속도 줄만 바뀌므로 padding 0이 실물에도 들어간다.
  - 실물에서는 고정 경로 컨트롤러(FollowPath, FollowPathThrough, FollowPathReverse)와 FollowPathGoal의 충돌 검사가 켜져 있다. 이 검사도 padding 0 발자국을 쓴다. 화면 받침 충돌 메시는 발자국 앞끝보다 2.1 mm 나온다.
- **peer 층:**
  - 실물 기본값은 꺼짐이다(`real_pode.launch.py`의 `peer_obstacles` false).
  - `start.sh`는 namespace와 초기 자세만 넘긴다. 두 대 이상에서 peer 층을 켜려면 launch를 직접 부르거나 start.sh를 고쳐야 한다. 원 0.075는 peer 층을 켰을 때만 효과가 있다.
- **지도:**
  - 실물 기본값은 `arena_v7_a`(2.5 cm), `arena_v7_fine`(1 cm), `arena_v7_keepout`이다. 셋 다 칸막이 3이 없고 진열대 A가 0.63부터인 배치로 다시 만들어졌다.
  - 실제 경기장이 이 배치와 다르면 SLAM 지도를 기준으로 관제 지도를 맞춘다(`docs/주행_계획서.md` 9절).
  - SLAM 지도를 쓸 때는 `real_pode.launch.py`에 `map:=`, `map_fine:=`, `keepout_mask:=`로 넘긴다. `start.sh`는 인자를 4개(namespace, x, y, yaw)나 5개(+속도)만 받는다. 지도 인자를 붙여 6개 이상이 되면 인자 개수 검사에서 멈추고, 속도 없이 하나만 붙여 5개가 되면 그 값이 속도 자리로 들어가 `make_params.py`에서 실패한다. 그래서 launch를 직접 부르거나 `start.sh`를 고쳐야 한다.
  - keepout은 `python3 tools/make_keepout.py --layout <관제 지도.yaml> --map <지도.yaml>`로 만든다. 이 도구는 지도 그림에서 크기와 원점만 읽고, 막는 자리는 `--layout`의 관제 지도 구역에서 그린다. 관제 지도를 고쳤다면 고친 yaml 경로를 `--layout`에 준다.
- **관제 PC:**
  - `pc.launch.py use_fake:=false`에서 `map_file`을 안 주면 d2 배치를 읽는다(`pc/fleet_core/fleet_core/fleet_map/fleet_map.py` 31줄 `DEFAULT_LAYOUT = "d2"`). 2026-11-19부터는 기본이 `v7`이라 안 주면 v7을 읽는다.
  - v7 경기장이면 `ros2 launch iponic_bringup pc.launch.py use_fake:=false map_file:=v7`로 띄운다.
- **도메인:** `tools/real_pode/env.sh`는 12다. 관제도 같은 도메인으로 띄운다.
- 도킹은 실물에 없다(DOCK_UNAVAILABLE). 주문 하나를 끝까지 돌릴 수 없다.
