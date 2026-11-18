# 8차 실물 시험 준비: WORK 입구 원호, RViz, 세 대 시나리오 (2026-11-18 밤)

브랜치 `fix/w-entry-arc-2026-11-18`(01b2fe4에서 갈랐다). 시험 브랜치 `test/real-run8-2026-11-18`은 run7(ce116ea)에 아래 세 브랜치를 합친 것이다.

| 브랜치 | 내용 |
|---|---|
| `fix/w-entry-arc-2026-11-18` | WORK 입구 원호 R 0.17, Q2 y 0.15, 시험 보강 |
| `feat/real-rviz-tests-2026-11-18` | PC RViz 실행 스크립트(전체 창 + 로봇마다 한 창)와 시험 |
| `test/real-three-robots-2026-11-18` | 세 대 실물 시나리오 `real_three_orders` |

## 1. 왜 했나

- run7_1(11-18 밤, pode2 한 대, 1 cm AMCL 지도)은 끝까지 돌았다. 사용자: "잘되네".
- 남은 점(사용자): WORK 스테이션에 들어갈 때(아웃바운드 말고) 너무 가깝게 돈다. 조금 더 멀리서 호를 그리면 좋겠다.
- 진단: Q1에서 W로 드는 r 0.10 모서리(pode_fsm이 꺾인 점을 r 0.10으로 둥글린다)가 로봇이 실제로 따라가는 반지름(약 0.11~0.13)보다 작다. 그래서 동쪽 벽 쪽으로 부풀었다. Codex 판정: 그럴듯하지만 이것만으로 증명되지는 않는다.

## 2. 바꾼 것 (지도 `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml`)

- `q_step`/`work_in`: Q2에서 W까지 반지름 0.17 원호 하나. 중심 (1.69, 0.32). 점은 약 10도 간격이라 pode_fsm의 r 0.10 둥글리기가 거의 손대지 않는다.
- Q1 (1.8449, 0.26) -> (1.8491, 0.26), yaw = 원호 접선 69.3도. 명령의 마지막 yaw가 목적지 노드의 yaw라서 접선과 맞춰야 한다.
- Q1 y 상한 0.26: Q1에 선 로봇이 WORK_DOCK(y 0.395)을 놓으려면 0.135(놓기 거리) 떨어져야 한다.
- Q2 y 0.16 -> 0.15: 실물은 길 끝에서 1~2 cm 못 미쳐 선다. 원호 위에서 Q1보다 2 cm 앞에 서도 Q2까지 0.203으로 노드 놓기 거리 0.20을 넘는다(옛 Q2면 0.194로 Q2를 쥔 채 남는다). 여유는 약 3 mm로 작다.
- 반지름 0.17은 위 두 조건(Q1 y <= 0.26, Q1-Q2 놓기 여유)이 허락하는 거의 최대다.
- Gazebo 월드 `sim/fleet_gz_sim/worlds/arena_v7.sdf`는 생성기로 다시 만들었다. AMCL 지도와 keepout은 그대로다.

## 3. 시험

- 새 시험 `test_v7_w_entry_is_one_wide_arc_through_q1`: 원호 점이 모두 R 0.17 위에 있음, 꺾임 10.5도 이하, Q1이 원호 위이고 yaw가 접선, Q1-WORK_DOCK >= 0.135, 동쪽 벽 몸통 여유 >= 0.045, Q1보다 2 cm 앞 점에서 Q2까지 >= node_clear.
- Q2 좌표를 박아 둔 시험 두 개는 지도에서 읽게 바꿨다. E-3->Q2 간격 고정값 0.14 -> 0.139(바닥 0.126은 그대로).
- 함께 고친 옛 결함: `sim/fake_robots/test/test_ros_dock_wait.py`가 W를 (1.88, 0.495)로 박아 두었다. 77725b3에서 W가 x 1.86으로 옮겨진 뒤 실패하고 있었다. 지도에서 읽게 고쳤다(302a95e, 13 passed).
- 결과(run8 머지 기준): fleet_core 1106 passed 7 xfailed, fleet_gz_sim 587 passed, tools 156 passed.
- fake_robots는 느리다. 파일별로 150초 제한을 두고 돌리니 앞의 16개 파일은 통과했고, `test_ros_pode.py`는 제한에 잘려 1 failed 19 errors가 났다.
  같은 파일을 제한 없이 run7과 run8에서 나란히 돌리니 둘 다 58 passed(각 3분 45초)였다. 잘린 것이 원인이고 원호 탓이 아니다.
  나머지 파일은 제한을 늘려 다시 돌리는 중이다(결과는 다음 기록에).

## 4. RViz (`tools/real_pode/rviz_pc.sh pode1 pode2 [pode3]`)

- 전체 창 하나(첫 로봇의 TF를 읽는다)와 로봇마다 한 창. 로봇 TF가 `/<ns>/tf`에 있어서 창 하나가 TF 트리 하나만 본다.
- PC 오버레이(`NAV2FLEET_WS_SETUP`, 없으면 건너뜀)와 `ROS_SETUP`을 덮어쓸 수 있다.
- 시험은 가짜 rviz2로 실제 호출 인자를 한 개씩 확인한다(f815012, 10 passed; run8에는 아직 안 합쳤다).

## 5. 세 대 시나리오

- `real_three_orders` = `real_two_orders` + pode3 (P-3). robots 말고 나머지는 같다는 것을 시험이 YAML 전체로 비교한다.
- 관제의 로봇 목록은 launch 인자 `robots`에서 온다. 시나리오의 robots는 monitor(채점)만 읽는다.

## 6. 배포와 다음 시험

- run8의 로봇 쪽 패키지는 run7과 같다. 두 대(pode1 = pinky_27a6 .200, pode2 = pinky_07d7 .201)에 11bc026을 배포하고 로봇에서 빌드했다(7 packages, rc 0). 지도 파일 md5가 run8과 같다.
- PC 오버레이: fleet_core, fleet_manager, fake_robots를 run8에서 빌드.
- 다음: 두 대 시험(order 켬, peer 켬, 고정 경로 충돌 검사 끔) + RViz. 잴 것: W 입구 원호의 실제 부풂(1~2 cm인지), Q1에 선 로봇이 Q2를 놓는 시점.
- 그다음 세 대. 06c5(.202)는 팀원이 쓰던 로봇이라 사용자 허락이 먼저다.
