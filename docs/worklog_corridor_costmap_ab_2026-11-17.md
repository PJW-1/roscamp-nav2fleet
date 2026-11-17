# 가운데 통로 costmap 갈래 A, B, A+B (2026-11-17)

브랜치 `exp/corridor-costmap-ab`(origin/main 956140d에서). Gazebo에서만 고를 수 있게 했고 기본 동작은 바꾸지 않았다.
Gazebo 판은 아직 돌리지 않았다(이 문서의 명령으로 돌린다).

## 1. 문제

- v7 가운데 통로(진열대 사이 y 0.405~0.77, x 약 0.5~1.65)는 폭 0.365 m, 로봇은 폭 0.112 m다.
- 5 cm Nav2 지도(`arena_maps.occupancy`)는 상자가 걸친 칸을 모두 막는다. x 0.95에서 통로의 빈칸은 6칸(y 0.45~0.75)이다(시험으로 확인).
- global costmap(발자국 0.12 사각형, `footprint_padding` 0.03, 내접 0.09)에서 로봇 중심이 설 수 있는 칸은 4칸(y 0.525~0.675 중심)이다
  (4절의 한 열 모델 계산. Gazebo costmap에서 잰 값이 아니다).
- 그래서 상자 하나(`v7_yard_mix`의 상자 4, (0.952, 0.595), 0.08 m)가 통로를 계획상 막는다고 본다(4절 모델). 로봇 한 대(peer 층의
  반지름 0.08 m 원)가 막는다는 것은 부모 세션의 판단이고 여기서 계산하지 않았다. 상자 옆 틈 0.150 m, 0.135 m는 형상에서 계산한 값이다.

## 2. 갈래

| 갈래 | 5 cm 지도 대신 | 해상도 | 칠하기 | 덮어쓰기 |
|---|---|---|---|---|
| base | `arena_v7.yaml`(그대로) | 0.05 | 걸친 칸(touch) | 없음 |
| A | `arena_v7_a.yaml` | 0.025 | 걸친 칸 | `nav_overrides/global_padding_0p01.yaml` 또는 `global_padding_0.yaml` |
| B | `arena_v7_b.yaml` | 0.05 | 칸 중심(centre) | 없음 |
| A+B | `arena_v7_ab.yaml` | 0.025 | 칸 중심 | A와 같은 덮어쓰기 |

- 칸 중심 규칙(`arena_maps.centre_filled`): 칸 중심이 벽·진열대 상자 안(경계 포함)일 때만 막는다.
  한 축에서 칸 중심을 하나도 덮지 않는 얇은 상자(2 cm 칸막이)는 그 축에서 상자 가운데가 든 칸 하나를 막는다.
  이 예외가 없으면 칸막이 셋이 0.05, 0.025 지도 모두에서 사라진다(칸 중심이 0.225/0.275, 0.2375/0.2625라 0.24~0.26을 비켜 간다).
- 1 cm 지도(AMCL)와 keepout 마스크(1 cm)는 모든 갈래에서 그대로다. 월드도 그대로다.
- 덮어쓰기 파일은 global costmap의 `footprint_padding`만 바꾼다(0.03 -> 0.01, 0.0). local costmap, controller, 충돌 검사는 그대로다.
  `resolution`은 덮지 않는다(3절).
- 지도 다시 만들기: `python3 -m fleet_gz_sim.arena --layout v7`이 기본 7개 파일과 갈래 6개 파일을 쓴다. 다른 배치는 갈래 지도를 만들지 않는다.

## 3. StaticLayer 해상도 확인

- 부모 세션의 짐작("non-rolling global costmap은 StaticLayer가 지도 해상도로 맞춘다")은 맞다.
- 설치본: `ros-jazzy-nav2-costmap-2d 1.3.13-1noble.20260921.024946`. 소스는 설치되어 있지 않아, 같은 태그
  `navigation2/1.3.13`의 `nav2_costmap_2d/plugins/static_layer.cpp`를 받아 읽었다. 설치된 `/opt/ros/jazzy/lib/liblayers.so`에
  같은 로그 문구 `StaticLayer: Resizing costmap to %d X %d at %f m/pix`가 들어 있다.
- `StaticLayer::processMap`(static_layer.cpp 199~215행):

```cpp
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
    master->getSizeInCellsY() != size_y ||
    !isEqual(master->getResolution(), new_map.info.resolution, EPSILON) ||
    ...
    !layered_costmap_->isSizeLocked()))
  {
    ...
    layered_costmap_->resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x,
      new_map.info.origin.position.y,
      true);
```

- 우리 global costmap에는 `rolling_window`가 없다(기본 false, `nav2_params.yaml`에서 `rolling_window: true`는 local costmap 278행뿐).
  `LayeredCostmap::size_locked_`는 false로 시작한다(layered_costmap.cpp 71행). 그래서 지도를 받으면 master costmap 전체가
  지도의 해상도·크기·원점으로 바뀐다. `resolution: 0.05` 파라미터는 처음 크기에만 쓰이고 덮인다.
- 결론: `resolution: 0.025`만 주면 5 cm 지도에서는 아무것도 바뀌지 않는다. A에는 2.5 cm 지도가 필요하다(`arena_v7_a`, `arena_v7_ab`).
  반대로 B 지도(5 cm)에 `resolution: 0.025`를 덮어도 5 cm로 돈다. obstacle layer와 inflation도 master 해상도를 따른다.

## 4. 계획상 통로 폭 (모델 계산, Gazebo 측정 아님)

로봇 중심 칸 = 칸 중심에서 내접 반지름(0.06 + padding) 안에 막힌 칸 중심이 없는 칸. x 0.952 한 열만 본다.
상자 4는 네 면을 모두 막힌 칸으로 흉내 냈다. 실제 obstacle layer는 관측 위치, 칸 양자화, 시간에 따른 표시·지우기에 따라 다르므로
이 모델이 실제보다 더 막는지 덜 막는지는 말할 수 없다.
스크립트: 부모 세션 스크래치패드의 `corridor_cells.py`(커밋하지 않음).

```
base res 0.05  pad 0.03 inscribed 0.09: no box  4 [0.525]..[0.675] | box4  0 []
base res 0.05  pad 0.01 inscribed 0.07: no box  4 [0.525]..[0.675] | box4  0 []
base res 0.05  pad 0.0  inscribed 0.06: no box  4 [0.525]..[0.675] | box4  0 []
a    res 0.025 pad 0.03 inscribed 0.09: no box  7 [0.5125]..[0.6625] | box4  0 []
a    res 0.025 pad 0.01 inscribed 0.07: no box  9 [0.4875]..[0.6875] | box4  1 [0.4875]
a    res 0.025 pad 0.0  inscribed 0.06: no box  9 [0.4875]..[0.6875] | box4  1 [0.4875]
b    res 0.05  pad 0.03 inscribed 0.09: no box  5 [0.475]..[0.675] | box4  1 [0.475]
b    res 0.05  pad 0.01 inscribed 0.07: no box  5 [0.475]..[0.675] | box4  1 [0.475]
b    res 0.05  pad 0.0  inscribed 0.06: no box  5 [0.475]..[0.675] | box4  1 [0.475]
ab   res 0.025 pad 0.03 inscribed 0.09: no box  9 [0.4875]..[0.6875] | box4  0 []
ab   res 0.025 pad 0.01 inscribed 0.07: no box 11 [0.4625]..[0.7125] | box4  3 [0.4625, 0.4875, 0.7125]
ab   res 0.025 pad 0.0  inscribed 0.06: no box 11 [0.4625]..[0.7125] | box4  3 [0.4625, 0.4875, 0.7125]
```

- 5 cm에서는 padding을 0.03에서 0까지 줄여도 이 열은 그대로다(세로 거리 0.05는 늘 안, 0.10은 늘 밖).
- 2.5 cm(A)는 padding을 함께 줄여야 상자 옆이 열린다. padding 0.01과 0.0은 이 열에서 같다.
- 한 열만 본 것이라 상자 길이(x 0.912~0.992) 전체를 지나는 길이 있는지는 Gazebo에서 봐야 한다.

## 5. Gazebo 명령 (v7 장애물 판: `gz_v7_varied` + `v7_yard_mix`)

판의 설정은 기존 견고함 판(`docs/주행_계획서.md` 183행, `docs/worklog_layout_v7_2026-11-13.md` 157행)과 같다.
설치는 이 브랜치를 빌드한 scratch install을 쓴다(이 작업 트리에서는 `install_x`). 도메인은 43~101 가운데 사람이 쓰지 않는 것
(사람은 57을 쓴다), 판마다 다른 `result_dir`.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ane/pinky_deps_ws/install/setup.bash
source <이 브랜치의 install>/setup.bash
export ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
GAME="layout:=v7 scenario:=gz_v7_varied obstacles:=v7_yard_mix battery_low_pct:=20 use_order_model:=true sim_dock_stub:=true rviz:=fleet gui:=true"
OV=$(ros2 pkg prefix fleet_gz_sim)/share/fleet_gz_sim/nav_overrides

# base
ros2 launch fleet_gz_sim gz.launch.py $GAME result_dir:=gz_result/corr_base
# A (2.5 cm, padding 0.01). padding 0은 global_padding_0.yaml
ros2 launch fleet_gz_sim gz.launch.py $GAME nav_map_variant:=a nav_overrides:=$OV/global_padding_0p01.yaml result_dir:=gz_result/corr_a
# B (칸 중심, 5 cm, padding 그대로)
ros2 launch fleet_gz_sim gz.launch.py $GAME nav_map_variant:=b result_dir:=gz_result/corr_b
# A+B (2.5 cm 칸 중심, padding 0.01)
ros2 launch fleet_gz_sim gz.launch.py $GAME nav_map_variant:=ab nav_overrides:=$OV/global_padding_0p01.yaml result_dir:=gz_result/corr_ab
```

- `nav_map_variant`는 v7에만 있다. 다른 배치에 주면 지도 파일이 없다며 띄우기 전에 멈춘다. 모르는 이름도 멈춘다.
- `nav_overrides`는 파일 하나만 받는다. 다른 덮어쓰기(예: `feat/nav-goal-smooth`의 값)와 함께 쓰려면 두 내용을 한 파일에 합친다.
- 확인할 것: global costmap 해상도(A, A+B는 0.025). `StaticLayer: Resizing costmap to ... m/pix`는 INFO 로그라 기본
  `nav_log_level:=warn`에서는 안 보인다. 보려면 `nav_log_level:=info`를 더하거나(로그가 많아진다), 판 도중에
  `ros2 topic echo --once --field info.resolution /pode1/global_costmap/costmap`로 본다. 상자 4가 놓인 동안 가운데 통로로 계획이 나는지,
  경기 시간, `collision ahead`, 자가 복구, 최소 거리.

## 6. 시험

- 먼저 쓴 시험(실패 확인 뒤 구현): 칠하기 규칙(x 0.95 통로 빈칸 수: touch 0.05 6칸 y 0.45~0.75, centre 0.05 7칸 y 0.40~0.75,
  touch 0.025 13칸 y 0.425~0.75, centre 0.025 15칸 y 0.40~0.775), 경계 포함, 얇은 상자 유지, centre ⊂ touch,
  갈래 지도가 생성기 출력과 같음, 기본 칠하기와 기본 지도가 그대로, v7에만 갈래 지도, `arena_for`의 갈래 선택,
  launch 배선(`map`만 바뀌고 `map_fine`, `keepout_mask`, 월드는 그대로; 기본값 빈칸; 오류), 덮어쓰기 파일이 global padding만 바꿈,
  setup.py가 `nav_overrides`를 설치함.
- 기존 생성 파일(월드, 지도, keepout, 1 cm 지도)은 생성기를 다시 돌린 뒤에도 바이트가 같다(`git status`에 바뀐 파일 없음, 기존 시험도 통과).
  `tools/real_pode/`, `pode/pode_nav_config`, `pc/`는 건드리지 않았다.
- fleet_gz_sim 전체(이 작업 트리의 `install_x`, `ROS_DOMAIN_ID=67`), Codex 반영 뒤, 그대로:

```
........................................................................ [ 12%]
........................................................................ [ 25%]
........................................................................ [ 38%]
........................................................................ [ 51%]
........................................................................ [ 64%]
........................................................................ [ 77%]
........................................................................ [ 90%]
........................................................                 [100%]
560 passed in 20.51s
```

  (앞에 d2 경로 경고 `fleet map arena_d2: the corner of path ... passes 0.115 m ... wants 0.15`가 여러 줄 찍힌다. 기존 출력이다.)
- fleet_core는 바꾸지 않아 돌리지 않았다. ruff는 이 컴퓨터에 없어 flake8로 새 줄만 봤다(새 경고 없음).

## 7. Codex 검수 반영

- 0e7b0cd 검수: (1) 칸 중심 규칙은 상자마다 따로 칠해 맞닿은 상자 사이에 대각 틈을 낼 수 있다 -> 사실(합성 예로 확인).
  v7 갈래 지도 셋에서 벽 밖 빈칸이 8 이웃으로 안쪽에 닿지 않고 칸막이가 남쪽 벽과 이어짐을 시험으로 넣었다(통과, 코드 변경 없음).
  (2) 경계 포함에 1e-6 칸(0.05 m에서 50 nm) 허용이 있다 -> 사실, 소수 오차용이라 두고 docstring에 적었다.
  (3) d2에 갈래를 주면 고칠 수 없는 생성 명령을 권했다 -> 사실, 시험 먼저 고쳐 "only for layout v7"로 거부한다.
- ce30172 검수: StaticLayer 로그가 INFO라 기본 `nav_log_level:=warn`에서 안 보인다 -> 사실, 5절에 확인법을 고쳤다.
  1절의 모델 값을 측정처럼 적었다, 상자 모델이 실제보다 덜 막는다는 단정 -> 받아들여 고쳤다.
  StaticLayer 행 번호가 틀렸다(187행부터라고) -> 받은 1.3.13 원본에서 `if (!layered_costmap_->isRolling() ...`는 199행,
  `true);`는 215행이라 고치지 않았다.
