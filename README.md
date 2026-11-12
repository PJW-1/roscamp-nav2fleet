# roscamp-nav2fleet

핑키 3대가 1.98 × 0.99 m 경기장에서 바구니를 나르는 입출고 관제다. 로봇은 넓은 곳을 Nav2 goal(NavigateToPose)로 스스로 길을 짜서 다니고, 도크 진입과 긴 좁은 길(급행) 한 곳만 관제가 준 선을 FollowPath로 따라간다. 관제는 "정확히 어디로 달리나"가 아니라 "어느 구역에 들어가도 되나"를 정한다.

## 구조

```mermaid
flowchart LR
  central[중앙 서버] -->|주문| orders
  subgraph pc[관제 PC]
    orders[주문] -->|로봇 부르기| traffic[교통: 구역 허락]
    orders -->|채우기, 교환| armjobs[팔 작업]
  end
  armjobs -->|작업 지시| arms[팔 3대]
  traffic -->|가도 되는 곳까지 목표| fsm
  subgraph robot[핑키 3대]
    fsm[핑키 상태 기계] -->|넓은 곳| nav[Nav2 길 찾기]
    fsm -->|도크, 급행| follow[선 따라가기]
    avoid[회피: 라이다, 카메라] --> nav
  end
  fsm -->|위치, 상태| traffic
```

| 층 | 하는 일 |
|---|---|
| 관제: 주문, 팔 | 그대로 |
| 관제: 교통 (새로) | 길 그래프(지점, 구역), 구역 허락, 목표 쪼개기, 급행 방향, 도크 줄, 막힘 처리 |
| 로봇: pinky_fsm | `mode`에 따라 Nav2 goal 또는 FollowPath. Retreat. 상태 보고 |
| 로봇: Nav2 | AMCL, bt_navigator(NavigateToPose), planner_server, controller_server(goal용 컨트롤러 하나 + FollowPath 셋) |
| 로봇: 회피 (팀원) | 라이다 + 비전 → costmap, goal용 컨트롤러 조정 |

판단(`pc/fleet_core`, `pinky/pinky_fsm/pinky_fsm/core.py`)은 ROS를 모르고, ROS 노드는 메시지만 옮긴다.

## 배치

경기장은 1.98 × 0.99 m이고 세 가지 배치를 지도로 넣었다(`pc/fleet_core/fleet_core/data/fleet_map_<배치>.yaml`). 어느 것을 쓸지는 아직 정하지 않았다. 비교표는 [작업 기록](docs/worklog_nav2base_2026-11-12.md)에 있다.

| 배치 | 모양 | 넓은 구역 | 급행(폭 0.20 m) |
|---|---|---|---|
| `d2`(기본) | 북쪽 벽을 따라 급행, 그 아래는 통째로 마당 | `HALL` 하나(3대) | 진열대 뒤, y 0.79~0.99 |
| `d` | `d2`에 가운데 진열대를 더해 마당을 서쪽, 동쪽과 그 사이 통로 둘로 나눔 | `HALL_W`, `HALL_E`(3대씩), 통로 `CORR_M`, `CORR_S`(1대씩) | `d2`와 같음 |
| `c` | 가운데에 급행, 위아래로 통로 | 네 모퉁이 마당(2대씩), 통로 `CORR_N`, `CORR_S`(1대씩) | 가운데, y 0.395~0.595 |

`d2_shelf`는 실제 후보가 아니라 아래 절차를 시험하려고 `d2`에 짧은 진열대(마당 한가운데, 0.30 m) 하나만 더한 시험용 배치다. 기본 목록(`d2 d c`)에는 없고 이름을 주면 돈다.

## 배치 바꾸기

배치 이름 목록은 `pc/fleet_core/fleet_core/fleet_map/fleet_map.py`의 `LAYOUTS`(실제 후보)와 `TEST_LAYOUTS`(시험용) 한 곳이다. 아래에서 `<이름>`은 새 배치 이름이고, 명령은 모두 저장소 루트에서 시작한다. 손으로 고치는 파일은 지도 yaml과 이 목록 둘이고, Gazebo 경기까지 돌리려면 시나리오 파일 하나가 더 든다.

1. 지도 파일: `fleet_map_d2.yaml`을 `pc/fleet_core/fleet_core/data/fleet_map_<이름>.yaml`로 복사해 `name: arena_<이름>`, `shelves`, `zones`, `nodes`, `paths`, `nav`를 고친다. 진열대는 두께 0.015 m로 적는다.
2. 이름 넣기: `fleet_map.py`의 `LAYOUTS` 또는 `TEST_LAYOUTS`에 `<이름>`을 더한다. 시험은 이 목록을 읽어 새 배치를 함께 돈다.
3. 지도 검사: 읽을 때 몸 여유와 길 이음을 보고, 진열대 끝 여유(0.15 m)를 어기면 경고를 낸다. 넓은 구역 정원의 합은 로봇 수(3) 이상이어야 한다(`test_zone_capacity.py`).

   ```bash
   (cd pc/fleet_core && python3 -c "import logging; logging.basicConfig(); from fleet_core.fleet_map import load_fleet_map as L; print(L('<이름>').name)")
   (cd pc/fleet_core && python3 -m pytest -q test/test_fleet_map.py test/test_zone_capacity.py)
   ```
4. 월드와 지도 만들기(Gazebo 경기장, keepout 마스크 포함): 만든 `worlds/`, `maps/` 파일은 함께 커밋한다. 시험이 커밋한 파일과 만드는 코드가 같은지 본다.

   ```bash
   (cd sim/fleet_gz_sim && PYTHONPATH=../../pc/fleet_core:. python3 -m fleet_gz_sim.arena --layout <이름>)
   ```
5. keepout 다시 만들기(실제 경기장 SLAM 지도용): 진열대는 마스크에 들어가지 않는다. 급행, 도크, 갈림목 구역을 바꿨을 때만 달라진다.

   ```bash
   python3 tools/make_keepout.py --layout <이름> --map <지도.yaml>
   ```
6. Gazebo 시나리오(Gazebo 경기까지 돌릴 때만): `sim/fleet_gz_sim/scenarios/gz_d2.yaml`을 `gz_<이름>.yaml`로 복사해 `map: <이름>`만 고친다. `layout:=<이름>`으로 띄우면 시나리오를 생략했을 때 이 파일을 찾는다. 한 대만 도는 `one_pinky`는 `gz_<이름>_one.yaml`이 따로 든다.
7. 상황 시험(머리 없는 세계): 흉내 모드 셋 x 씨드 셋, 몇 분 걸린다. 표 아래에 못 지킨 기대가 나온다.

   ```bash
   python3 tools/situations.py -l <이름> --avoid all --seeds 1 2 3
   ```

`d2_shelf`로 이 절차를 한 번 돌렸다(걸린 시간과 결과는 작업 기록).

## 빌드와 시험 (WSL, ROS 2 Jazzy)

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=45 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # 6은 쓰지 않는다
colcon build
source install/setup.bash
cd pc/fleet_core && python3 -m pytest -q test      # 패키지 폴더마다 같은 식
```

- 핑키 패키지는 벤더 의존(`iponic.repos`의 `pinky_pro`)이 함께 빌드되어 있어야 한다.
- `fleet_manager` 시험은 ROS 도메인 50~59를 코드에서 쓴다. 한 기계에서 한 번에 하나만 돌린다.

## 돌려 보기

```bash
# 머리 없는 세계(ROS 없음): 상황 열 가지 x 배치 x 씨드의 결과 표
python3 tools/situations.py -l d2 d c --seeds 1 2 3   # 시험용 배치는 -l d2_shelf

# Gazebo: 핑키 3대와 관제, 배치 d2, 주문 8개
ros2 launch fleet_gz_sim gz.launch.py layout:=d2

# Gazebo: 핑키 한 대만(관제 없음), 다른 터미널에서 정해진 구간을 달리게 한다
ros2 launch fleet_gz_sim one_pinky.launch.py gui:=false rviz:=false
ros2 run fleet_gz_sim drive_check --layout d2 --plan stage2

# 실제 지도에 맞춘 keepout 마스크 만들기
python3 tools/make_keepout.py --layout d2 --map <지도.yaml>
```

Gazebo 쪽 인자와 장애물 일정은 [sim/fleet_gz_sim/README.md](sim/fleet_gz_sim/README.md)에 있다.

## order 모델 (제안, 기본 꺼짐)

이동 하나를 조각 목록으로 한 번에 보내는 방식이다(VDA 5050의 order). 관제가 구역을 잡아 준 조각이 base이고, 아직 잡지 않은 조각이 horizon이다. 로봇은 base 안에서는 서지 않고 다음 조각으로 넘어간다. 관제는 로봇이 base 끝에 닿기 2초 전에 다음 구역을 잡아 update로 base를 늘리고, 못 잡으면 로봇이 base 끝에서 기다린다. 통신이 끊겨도 로봇은 base 끝까지만 간다.

```bash
ros2 launch fleet_gz_sim gz.launch.py layout:=d2 use_order_model:=true   # Gazebo
python3 tools/order_situations.py --seeds 1 2 3                          # 머리 없는 세계에서 끄고 켠 표
```

`use_order_model`을 끄면(기본) 관제가 내는 명령과 사건은 전과 같다. 규칙과 정한 값은 [docs/order_모델.md](docs/order_모델.md)에, 측정은 [작업 기록](docs/worklog_nav2base_2026-11-12.md) 8절에 있다.

![RViz의 관제 화면: 굵은 실선이 base](docs/img/order_base_horizon.png)

관제 화면(`rviz:=fleet`, 기본)은 로봇마다 base를 굵은 실선으로, horizon을 반투명 점선으로 그린다. 위 그림은 Gazebo 세 대 경기(D2, order 모델 켬)를 시작하고 110초 뒤의 화면이다. pinky1(파랑)의 base가 N_E에서 X_E로, pinky2(초록)의 base가 P-3에서 W_Q 쪽으로 보인다. 이 순간에는 horizon이 짧아 잘 보이지 않고, 처음 카메라 자리 때문에 경기장 서쪽 끝이 잘렸다.

## 지금 상태

| 있는 것 | 아직 없는 것 |
|---|---|
| 구역 교통 층: 구역 허락, 급행 방향, 도크 줄, 막힘 처리(기다림, 표시, 돌아가기, Retreat), 교착 감시 | 실물에서의 확인: 아무것도 실물에서 재지 않았다 |
| 핑키: MoveTo `mode`(`follow_path`, `nav_goal`), Retreat, keepout 필터, 비전 장애물 자리(기본 꺼짐) | 충돌 검사를 켠 채 고정 경로 주행: Gazebo에서 통과하는 설정을 찾지 못했다 |
| 시험 환경: 머리 없는 세계(상황 열 가지), Gazebo 세 배치 | 회피(라이다와 비전으로 피하기)는 팀원 몫이고 자리만 있다 |
| 측정: 머리 없는 세계 90번, Gazebo 세 대 경기 11판(작업 기록 4절) | Gazebo 세 대 경기의 안정성: 관제를 고친 뒤 장애물 없는 네 판 가운데 두 판만 주문 8건을 끝냈다. 마당에서 로봇의 몸이 닿고, 위치 추정이 흐른 뒤 격리되는 판이 있다 |

## 문서

| 문서 | 내용 |
|---|---|
| [docs/관제_구역교통.md](docs/관제_구역교통.md) | 교통 규칙, 사건 코드, 값 |
| [docs/회피_약속.md](docs/회피_약속.md) | 관제와 회피가 나눠 맡는 선(제안) |
| [docs/핑키_계약.md](docs/핑키_계약.md) | 핑키와 관제 사이의 계약(0.11-draft는 제안) |
| [docs/order_모델.md](docs/order_모델.md) | order 모델의 규칙, 켜는 법, 정한 값(제안) |
| [docs/worklog_nav2base_2026-11-12.md](docs/worklog_nav2base_2026-11-12.md) | 만든 것, 측정, 가정, 남은 일 |
| [docs/COPIED_FROM.md](docs/COPIED_FROM.md) | 복사한 곳과 지운 것 |
