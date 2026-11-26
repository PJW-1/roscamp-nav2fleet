# 11-26 실물 두 대 판: keeper 뒤집힘과 LOCAL_YIELD→BLOCKED 원인

자료: `/home/ane/ros2camp/real_data/2026-11-26/` (bag `1126_real` = pode2, `1126_real_pode1` = pode1, 관제 기록 `fleet_*_launch.log`/`fleet_*.jsonl`, 로봇 launch 기록 `bags/pode{1,2}_logs/`). 같은 bag은 이 브랜치 `results/2026-11-26/robot/`에 zstd로 있다. 시각은 KST. ROS 그래프 없이 bag만 읽었다.

## 결론

- **(a) keeper: 표본이 너무 작아 판단할 수 없다.** nav_goal 조각은 로봇마다 판마다 3개(모두 12개)다. planner `/plan` 뒤집힘은 첫째 판(keeper 꺼짐) pode1 O_OUT→Q2에서 2건(1 s 사이에 북쪽 통로로 갔다 돌아옴), 둘째 판(keeper 켬) 0건이다. `/plan`은 planner 출력이라 keeper가 바꾸지 못한다. 둘째 판은 planner가 뒤집지 않아 keeper가 할 일이 없었으므로 2 → 0은 keeper 효과가 아니다. keeper 규칙을 첫째 판 `/plan`에 다시 돌리면 두 건 모두 막는다(새 길 1.80 m가 남은 길 1.56 m보다 충분히 짧지 않음, 막힘 검사는 열림으로 가정).
- **(b) 세 사건 모두 nav_goal 조각이고 앞 물체 때문이 아니다.** pode_fsm nav_goal 정체 감시(1 s 동안 5 cm 못 감)가 발동했고, 사건 1·2는 대기 2 s 동안에도 5 cm를 못 가 BLOCKED가 됐다. 이 감시는 제자리 회전을 진행으로 거의 치지 않는다.
  - 사건 1 (급행 판, pode2, E-1): 목표 0.049 m 안에서 목표 방향 90°로 156° 제자리 회전 중이었다. 원인은 pode_fsm 정체 규칙이다.
  - 사건 2 (첫째 판, pode1, E-2): 목표에서 4.3~6.1 cm(허용 0.05 m 경계)를 오가며 RPP가 목표 방향 회전과 경로 방향 회전을 번갈아 했다(4.7 s). 그 사이 RPP collision ahead로 세 번 멈췄다(pode1 트레이 기둥 추정). pode2(중심 거리 0.27~0.31 m), 벽·진열대(발자국에서 0.13 m 넘게)는 원인이 아니다.
  - 사건 3 (첫째 판, pode1, O_OUT→Q2, 1 s 뒤 MOVING): planner 경로 뒤집힘(20:09:03.86 남쪽 → 북쪽, 04.89 되돌아옴) 동안 1.3 s 멈췄다. keeper가 막으려는 바로 그 경우다.

## (a) keeper

`PLAN_KEEP=1` → `plan_keep:=true` → `pode_plan_keep` 노드 + BT `..._goal_keep.xml`(ComputePathToPose가 `compute_path_keep`를 거침). 새 경로가 같은 goal의 다른 길(first_look OTHER, 남은 길에서 0.20 m 넘게 벗어남)이면, 남은 길이 전역 costmap에서 열려 있고 새 길이 max(0.30 m, 20 %) 넘게 짧지 않은 한 지킨 길을 돌려주고 `plan_kept`로 낸다. bag에는 `plan_kept`가 없다.

방법: 11-24 `count_flips` 정의 그대로. nav_goal 조각 = 관제 `SEGMENT_DONE (nav_goal)`(시작 = 앞 조각 끝)이고, 목표 0.06 m 안에서 끝나는 `/plan`만 센다. 스크립트 `keeper.py`, 결과 `keeper_legs.json`.

| 판 | keeper | 로봇 | nav_goal 조각 | /plan 수 | planner 뒤집힘 | keeper 재생 뒤 |
|---|---|---|---|---|---|---|
| 첫째 | 꺼짐 | pode1 | 3 (E-1→Q2, O_OUT→Q2, O_OUT→E-2) | 15+17+14 | 2 (O_OUT→Q2) | 0 |
| 첫째 | 꺼짐 | pode2 | 3 (E-2→Q2, O_OUT→Q2, O_OUT→E-1) | 13+14+9 | 0 | 0 |
| 둘째 | 켬 | pode1 | 3 (E-2→Q2, O_OUT→Q2, O_OUT→E-1) | 12+13+9 | 0 | 0 |
| 둘째 | 켬 | pode2 | 3 (E-1→Q2, O_OUT→Q2, O_OUT→E-2) | 14+14+8 | 0 | 0 |
| 급행(참고) | 꺼짐 | pode2 | 2 | 15+9 | 0 | 0 |

뒤집힘 두 건(`flips_run1_pode1.png`): 20:09:03.86에 남은 1.56 m(남쪽 통로, 경로 가운데 y 약 0.16)가 1.80 m(북쪽, y 약 0.57)로 바뀌었고, 로봇은 따르지 않고 멈췄다. 20:09:04.89에 1.54 m 남쪽 길로 돌아왔다. (확인: bag `/pode1/plan`에서 x 0.7~1.4 구간 y 평균이 02.83 0.16 → 03.86 0.57 → 04.89 0.16이다.)

판단: 둘째 판에서 keeper가 길을 붙잡은 일이 없다. 12개 조각 중 1개에서 한 순간 2건으로, 11-24 Gazebo 장애물 판(RPP 기준 판당 7~17건, `docs/report_2026-11-24_plan_flip.md`)보다 훨씬 드물다. keeper 효과를 보려면 같은 판을 훨씬 많이 돌려야 한다. keeper가 켜진 것은 launch 기록 `plan_keep` 프로세스 시작 줄로만 확인되고, keeper 로그 줄이 기록에 없다. 다음 판에는 `/pode*/plan_kept`도 bag에 넣는다.

## (b) 판정 코드

`pode_fsm`의 `yield_nav.py`, `nav_goal.py`, `params.py`: 기준점에서 0.05 m(`nav_progress_m`) 움직이면 기준점을 갱신한다. 제자리 회전 0.2 rad마다 `turned_at`을 갱신하되, 마지막 5 cm 이동 뒤 4 s(`nav_turn_window_s`) 안에서만이다. `now - max(anchor_at, turned_at) ≥ 1 s`(`block_report_s`)이면 RECOVERING/wait다. 대기 2 s(`local_wait_s`) 안에 5 cm 움직이면 MOVING(회전은 치지 않음), 아니면 앞 0.5 m 직선 0.2 m 띠 안 상대 로봇 + 양보 순서일 때 후진, 아니면 BLOCKED다. 보고되는 `obstacle_x/y`는 local costmap 앞 ±0.12 m 띠에서 가장 가까운 치명 칸일 뿐 판정 입력이 아니다.

RPP: collision detection 0.6 s, local costmap 2.5 cm, `obstacle_min_range` 0.0, 발자국 뒤 끝 −0.076. 충돌이면 경고 + 그 주기 명령 0. `regulated_linear_scaling_min_speed` 0.05 m/s가 정체 기준(1 s에 5 cm)과 같다. RPP collision ahead 경고 수: pode1 첫째 판 21, 둘째 판 6 / pode2 모든 판 0.

### 사건 1 (급행 판, pode2 혼자, O_OUT→E-1 (0.12, 0.24, 90°))

19:16:16.4~18.0 cmd v 0.050~0.063으로 접근 → 18.0 목표 0.049 m → 회전 시작(−114° → 90°, 시계 방향 156°) → 18.16 RECOVERING/wait → 18.4~21.2 cmd ω −1.0, 위치 (0.138, 0.279) 그대로 → 20.16 BLOCKED → 21.25 SUCCEEDED, 후진으로 P-1. 회전 전 앞 가장 가까운 점 0.26 m, 발자국에서 가장 가까운 점 0.10~0.13 m(서쪽 벽), RPP 경고 0건. 보고 장애물 (−0.01, 0.27)은 서쪽 벽 scan 점이다.

### 사건 2 (첫째 판, pode1, O_OUT→E-2 (0.38, 0.24, 90°))

20:10:10.0 목표 0.061 m → 10.5 0.045 m, 목표 방향(+) 회전 → 10.93~11.08 충돌 경고 4번(cmd ω 0) → 11.5~13.0 0.053~0.061 m, yaw +47°까지 → 13.5~16.0 경로 방향(−60°)으로 되돌림(경고 13.79 / 13.84 / 14.34) → 14.74 RECOVERING/wait(마지막 5 cm 이동 약 10.0 뒤 회전만 4.7 s) → 15.19~15.28 경고 3번 → 16.0~16.5 0.046~0.050 m, 다시 + 회전 → 16.77 BLOCKED(yaw −34°, pode2는 서남서 0.28 m라 앞 직선 띠 밖) → 16.84~19.28 경고 5번, yaw 86° → 19.80 SUCCEEDED(0.036 m), 23.2 RETREAT_DONE P-2.

|v| ≤ 0.001 내내. 거리 흔들림은 제자리 회전 중 map 자세 흔들림(1~2 cm)이다. pode2는 P-1 (0.125, 0.123), 중심 거리 0.27~0.31 m. 기둥이 아닌 가장 가까운 점은 발자국에서 0.13~0.17 m이고, RPP 검사 범위(회전 반지름 0.0845 m, 앞 0.6 s × 최대 0.12 m/s) 안에 기둥이 아닌 점은 없다. scan마다 기둥 점 8~19개가 회전 반지름 + 반 칸 안에 있다. 보고 장애물 (0.49, 0.10)은 0.22 m 앞 실제 scan 점이다. 원인: (1) 허용 0.05 m 경계에서 RPP 회전 목표가 바뀜(도리도리), (2) pode_fsm 정체 규칙, (3) 기둥 칸으로 인한 RPP 짧은 정지(추정).

### 사건 3 (첫째 판, pode1, O_OUT→Q2, 1 s 뒤 MOVING)

20:09:00.95 시작 → 02.40 경고 1번 → 03.86 `/plan` 남쪽(1.56 m) → 북쪽(1.80 m) → 04.0 cmd v 0.12 → 0, (0.318, 0.494)에서 멈춤 → 04.35~04.51 경고 4번 → 04.77 RECOVERING/wait → 04.89 `/plan` 남쪽 복귀(1.54 m) → 05.98 5 cm 이동, MOVING. pode2는 W 도킹(1.55 m 떨어짐). 기둥이 아닌 가장 가까운 점 0.26 m, 앞 띠 안 점 없음.

### 제안 (이번에 고치지 않음)

- nav_goal 정체 규칙이 목표 앞 제자리 회전을 정체로 센다. 대기 중 회전을 진행으로 치거나 목표 허용 안에서는 세지 않으면 사건 1·2의 BLOCKED 보고가 없었을 것이다.
- pode1 트레이 기둥이 local costmap에 찍힌다(`obstacle_min_range` 0.0). 둘째 판에도 경고 6번(20:13:59.5, 20:16:10.3~11.2)이 났지만 LOCAL_YIELD로 이어지지는 않았다.

## 한계

- bag에 `plan_kept`와 keeper 로그가 없다. 둘째 판에서 keeper가 아무것도 안 했다는 판단은 planner 뒤집힘 0건에만 기댄다.
- costmap이 bag에 없어 RPP 경고를 기둥 탓으로 보는 것은 추정이다.
- keeper 재생은 막힘 검사가 늘 열림이라고 가정하고, 두 경로의 끝점을 목표로 쓴다(이 조각들에서는 요청 목표와 같은 Q2·E 점).
- `keeper.py`는 조각 경계 앞뒤로 0.5 s씩 넓혀 `/plan`을 모은다. 표의 `/plan` 수는 1~2개 많을 수 있다. 뒤집힘 두 건은 조각 한가운데(시작 뒤 3~4 s)라 영향이 없다.
- `events.py`의 그림은 사건 시각의 자세에 그 직전 scan을 겹친다(scan 주기 0.1 s). 1 rad/s로 도는 동안에는 점이 지도에서 최대 약 6°까지 어긋나 보인다. 표의 `cmd_vel`은 0.3 s 넘게 메시지가 없으면 0으로 적는다.
- 위 셋은 Codex 검토(`~/codex-review.md`, 11-26 20:35)에서 나온 지적이다. 결론의 숫자(뒤집힘 2건, 경고 수, E-1 앞 회전)는 bag과 launch 기록으로 따로 확인했다.

## 다시 돌리기

```bash
WS=/home/ane/ros2camp/podedrive_ws
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$WS/install/fleet_interfaces/lib/python3.12/site-packages
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$WS/install/fleet_interfaces/lib
python3 extract.py /tmp/bags.pkl; python3 keeper.py /tmp/bags.pkl .; python3 events.py /tmp/bags.pkl .
```
