# 3대 경기 결과(Gazebo, G2와 G3)

충돌 검사: 마당(FollowPathGoal) 켬, 고정 경로 끔

- 트리: wt-nf-gztest(브랜치 sites-world-test). 부모 커밋은 sites aadeac3 + world 74e8264다.
- 공통: `game.sh <이름> layout:=<배치> fixed_line_collision_check:=false`, 시뮬 도킹 대역(sim_dock_stub) 켬, 화면 없음, 속도 0.12 m/s, ROS 도메인 60.
- 시나리오: gz_<배치>(주문 8건, 3대). 시나리오 한도 1800 s.
- 열의 뜻:
  - 끝 시각: 마지막 관제 이벤트의 시뮬 시각(s).
  - 이동 한 건: 도크나 주차 자리를 떠나 다음 도크나 주차 자리에 닿기까지의 평균(s). 실제 위치로 잰다(rec_truth.py, g2c부터).
  - 정지: fleet_check의 정지 횟수(3대 합).
  - 대기: 구역을 기다린 횟수(ZONE_WAIT)와 fleet_robots.csv의 wait_zone이 찬 시간의 합(s).
  - 최소 거리: 로봇 중심 사이 최소 거리(m, 실제 위치).
  - Retreat: RETREAT_START 횟수. 급행: EXPRESS를 받은 횟수. 방향: LANE_DIRECTION_SET/HANDOVER.

## G2: D2

| 경기 | 판정 | 주문 | 끝 시각 | 이동 한 건 | 정지 | 대기(횟수/합 s) | 최소 거리 | DEADLOCK_SEEN | OUTSIDE_ZONE | Retreat | 급행 | 방향 SET/HANDOVER | 멈춘 것 | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g2b | FAIL(시간 초과) | 1/8 | 157.8 | 기록 없음 | 8 | 2 / 65.0 | 0.204(pinky1-pinky2) | 0 | 0 | 6 | 1 | 1/0 | pinky2 격리(80 s), 그 뒤 진행 없음 | 0.985 |
| g2c | 멈춤(감시견이 끔) | 2/8 | 742.7 | 42.3(10건) | 53 | 2 / - | **0.09**(pinky2-pinky3, 633 s) | 0 | 0 | 7 | 1 | 1/0 | pinky2가 N_E 바로 아래 마당에 섰고, pinky3이 N_E로 못 가 742 s에 격리 | 0.903 |

| g2d | FAIL(parked_while_backlog 3) | **8/8** | 481.1(시나리오 505 s) | 36.5(25건) | 67 | 3 / 177.0 | **0.099**(pinky2-pinky3) | 0 | 0 | 16 | 1 | 1/0 | 없음. GOAL_STALLED 2(후진 1은 GOAL_ENDED_AT_TARGET 1, 이동 1은 재시도로 이어짐) | 0.965 |
| g2e | 멈춤(감시견) | 3/8 | 275.3 | 37.1(12건) | 21 | 2 / - | 0.129(pinky2-pinky3), 몸은 닿지 않음(틈 0.016 m) | 0 | 1 | 9 | 1 | 1/0 | pinky2가 N_E > X_E 길에서 진열대 동쪽 끝 옆에 끼어 섰고, 보고 위치는 벽 너머로 흘러 출고 도킹 3번 실패, 219 s 격리(B-G4) | 0.973 |

- 트리: g2e는 sites 65901fc(+ nav2-base 02563a8). g2b, g2c는 wt-nf-gztest(sites aadeac3 + world 74e8264, 69e3fea 없음). g2d부터는 wt-nf-sites의 sites f11df6b(aadeac3 + nav2-base 69e3fea, world 포함)다.
- g2d: 주문 8건을 모두 냈다. FAIL은 시나리오 판정 `parked_while_backlog`(묶이지 않은 주문이 있는데 주차 자리에 들어감) 3번 때문이다. 85.0 s pinky2, 158.6 s pinky1, 240.6 s pinky2다. 관제가 도크가 찼을 때 주차 자리(P-2)를 대기 자리로 쓰는 것(`DOCK_QUEUED ... waits at P-2`)과 판정이 맞지 않는다.
- g2d: GOAL_ENDED_AT_TARGET 1번(9.3 s 서 있음). 후진 W > X_W가 X_W 0.006 m에서 yaw 오차 0.156 rad로 끝나지 않았다(gz_bugs.md B-G1, 원인 확정).
- g2d: 마당 keepout 침범(Nav2 goal 중, 실제 위치, 발자국 0.12 m 사각형): 15번, 가장 깊게 0.05 m, 가장 길게 2.38 s, 합 13.0 s. 장애물 없는 경기의 기준값이다.
- g2e(65901fc): `HALL_YIELD ... out of the way` 0번, ZONE_MARKED_BLOCKED 0번, ROBOT_IDLE 0번, 격리 1번(pinky2, DOCK_FAILED), GOAL_ENDED_AT_TARGET 1번(pinky3 P-2). 감시견이 시뮬 681 s에 끝냈다. gz_bugs.md B-G4.
- 69e3fea(목표 끝에 닿은 실패를 도착으로 봄)는 g2b, g2c 모두 들어 있지 않다. GOAL_ENDED_AT_TARGET 열은 69e3fea가 든 경기(g2d부터)에만 뜻이 있다.
- g2b: pinky2가 출고 도크에서 X_E로 물러난 뒤 끝에 닿고도 후진 goal이 끝나지 않았다. 재시도 둘이 바로 FAILED로 끝나 80 s에 격리됐다. 격리된 로봇이 J_E와 HALL을 쥐어 pinky1이 끝까지 기다렸다. gz_bugs.md B-G1.
- g2c: gz_bugs.md B-G2. 마당에서 로봇이 서로 길을 막았고, 관제는 교착으로 보지 않았다(DEADLOCK_SEEN 0). ZONE_MARKED_BLOCKED 16번, BLOCKED_WAIT 57번, HALL_YIELD 12번. 감시견(관제 로그가 420 s 조용함)이 시뮬 1136 s에 경기를 끝내서 summary.json과 events.csv가 없다. 숫자는 rec.csv(실제 위치, 관제 이벤트)와 fleet_check.json에서 냈다.
- g2c의 후진 7번은 모두 RETREAT_DONE으로 끝났다. 끝의 yaw 오차는 최대 0.055 rad였다(O > X_E 두 번은 -0.032, 0.055).
- 최소 거리 0.09 m는 발자국(0.12 x 0.12 m) 두 개가 겹치는 거리다. 시뮬 안에서 두 로봇이 닿았다.

## G3: C와 D(배치 고르기용, 같은 열)

트리: g3c1, g3d1은 wt-nf-sites의 sites f11df6b(aadeac3 + nav2-base 69e3fea, world 포함). g3c2부터는 sites 65901fc(+ nav2-base 02563a8: B-G2 비켜 주기, B-G3 통로 나가기). GOAL_ENDED_AT_TARGET 열이 뜻이 있다.

| 경기 | 배치 | 판정 | 주문 | 끝 시각 | 이동 한 건 | 정지 | 대기(횟수/합 s) | 최소 거리 | DEADLOCK_SEEN | OUTSIDE_ZONE | Retreat | 급행(방향 SET/HANDOVER) | GOAL_ENDED_AT_TARGET | 멈춘 것 | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g3c1 | C | 멈춤(감시견) | 3/8 | 214.0 | 60.8(2건) | 12 | 0 / - | 0.214(pinky1-pinky3) | 0 | 0 | 6 | 3/0 | 0 | pinky1이 북쪽 통로 (1.39, 0.73)에서 막힌 뒤 관제가 다시 보내지 않음(B-G3) | 0.969 |
| g3d1 | D | 멈춤(감시견) | 5/8 | 430.6 | 34.2(23건) | 37 | 7 / - | 0.099(pinky2-pinky3) | 0 | 0 | 13 | 2/0 | 0 | pinky3이 CORR_M에서 막힌 뒤(396.2 s, "tried again in 60 s") 다시 보내지 않음. B-G3과 같은 모양 | 0.975 |
| g3c2 | C | FAIL(parked_while_backlog 5) | **8/8** | 679.2(시나리오 698 s) | 175.6(6건, 대기 포함) | 34 | 8 / 699.0 | 0.194(pinky1-pinky2) | 0 | 0 | 16 | 6/0 | 1 | pinky3이 CORR_N (1.32, 0.72)에 240~480 s 갇힘(아래), 그 뒤 풀림 | 0.974 |
| g3d2 | D | 멈춤(감시견) | 7/8 | 392.6 | 41.4(19건) | 32 | 3 / - | 0.12(pinky2-pinky3), 몸은 닿지 않음(틈 0.010 m) | 0 | 0 | 14 | 1/0 | 0 | pinky1이 작업 도킹 3번 실패로 353.7 s 격리(WORK_DOCK을 쥔 채). 그 직전 314~323 s 실제 위치는 (0.26, 0.54) 근처에서 제자리로 돌았고 보고 위치는 0.40 m 흘렀다(B-G5) | 0.953 |

- g3c1: 감시견이 시뮬 618 s에 끝내 summary.json이 없다. 숫자는 rec.csv와 fleet_check.json에서 냈다. 이동 한 건은 끝난 이동이 2건뿐이라 대표값이 아니다.
- g3d1: 트리 f11df6b(02563a8 없음). 감시견이 시뮬 837 s에 끝냈다. GOAL_STALLED 1번(이동), 후진 13번은 모두 끝났다(yaw 오차 0.12 이하).

- g3c2(65901fc): 주문 8건을 모두 냈다. FAIL은 parked_while_backlog 5번뿐이다. `HALL_YIELD ... out of the way` 0번, ZONE_MARKED_BLOCKED 10번, ROBOT_IDLE 0번, 격리 0번, GOAL_ENDED_AT_TARGET 1번. 몸은 닿지 않았다(최소 틈 0.072 m).
- g3c2: pinky3이 CORR_N의 (1.32, 0.72)에서 240 s부터 480 s까지 움직이지 못했다. 근처에 다른 로봇은 없었다. 관제는 02563a8대로 N_W, N_E, P-3, P-2를 번갈아 보냈고(ZONE_MARKED_BLOCKED 10번, NAV_ABORTED 재시도), 480 s쯤 빠져나왔다. g3c1에서 pinky1이 선 자리(1.39, 0.73)와 같은 곳이다. 그 자리는 진열대 북쪽 끝(y 0.61)에서 로봇 중심까지 0.11 m, 몸 가장자리까지 약 0.05 m다. 발자국은 keepout 안에 있지 않았다(깊이 0). Nav2가 왜 그 자리에서 길을 못 내는지는 확인하지 못했다(pinky_fsm 로그 `navigate_to_pose aborted: 104`).
- g3d2(65901fc, g3c2·g2e와 같은 트리와 설정): `HALL_YIELD ... out of the way` 0번, ZONE_MARKED_BLOCKED 0번, ROBOT_IDLE 0번, 격리 1번, GOAL_ENDED_AT_TARGET 0번, B-G4식 멈춤(실제 고정, 보고 0.2 m 이상) 1번. 후진 14번 모두 끝남. keepout 침범 13번(최대 0.06 m, 합 23.2 s). 감시견이 시뮬 808 s에 끝냈다.
- 같은 트리(65901fc)에서 배치 비교: C g3c2 8/8(698 s), D g3d2 7/8(격리로 멈춤), D2 g2e 3/8(B-G4), D2 g3exp 8/8(481 s, 상자가 놓이지 않은 장애물 없는 경기).

## 로봇 몸이 닿았나(리드 질문 c)

몸은 pinky_description의 base_link 충돌 메시 경계(x -0.075~0.039 m, y -0.041~0.047 m)로 보고, 실제 위치에서 두 몸 사이 틈을 쟀다(touch.py).

| 경기 | 최소 중심 거리 | 몸 사이 최소 틈 | 닿았나 |
|---|---|---|---|
| g2c | 0.09 m | 0(149.1 s pinky1-pinky3, 중심 0.098 m) | 닿았다 |
| g2d | 0.099 m | 0(173.6 s pinky1-pinky2, 중심 0.113 m) | 닿았다 |
| g3c1 | 0.214 m | 0.124 m | 닿지 않았다 |
| g3d1 | 0.099 m | 0(292.4 s pinky2-pinky3, 중심 0.111 m) | 닿았다 |
| g3c2 | 0.194 m | 0.072 m | 닿지 않았다 |
| g3d2 | 0.12 m | 0.010 m | 닿지 않았다 |
| g2e | 0.129 m | 0.016 m | 닿지 않았다 |
| g3yard | 0.099 m | 0(208.4 s pinky1-pinky2) | 닿았다 |
| g3exp | 0.09 m | 0(276.4 s pinky2-pinky3) | 닿았다 |
| g3exp2 | 급행 가운데 (0.99, 0.89), 0.08 m 상자, 시뮬 15 s부터(sites 5a18d1f 일정) | 0/8 | 150.0 | 10 | 0.209, 몸은 닿지 않음 | 0 | 0 | 1 | 1 | pinky2가 급행에서 상자에 부딪쳐 섰는데(실제 x 0.90) 보고 위치는 출고 도크까지 갔고, 도킹 3번 실패로 75.8 s 격리. 그 뒤 진행 없음 | 0.998 |

- g2d의 최소 거리 0.099 m에서 몸은 닿았다. 메시 경계 상자 기준이라 실제 메시보다 조금 크게 본다. 그래도 경계 상자가 겹쳤으므로 시뮬 안에서 부딪쳤다고 본다.

## G3: 장애물 경기(D2)

트리: sites 65901fc(nav2-base 02563a8 포함). 3대, 주문 8건. B-G4 열은 실제 위치가 0.05 m 안에 머무는 동안 보고 위치가 0.2 m 이상 움직인 경우의 수다(freeze.py, 처음 10 s 제외).

| 경기 | 장애물 | 주문 | 끝 시각 | 정지 | 최소 거리(몸 닿음) | ZONE_MARKED_BLOCKED | HALL_YIELD | 격리 | B-G4 | 멈춘 것 | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|
| g3yard | 마당 가운데 (0.99, 0.65), 0.12 m 상자, 시뮬 60~180 s | 3/8 | 270.9 | 26 | 0.099(닿음, 208.4 s pinky1-pinky2) | 2 | 1 | 2 | 1 | pinky1과 pinky2가 W_Q 근처 (0.26~0.36, 0.45~0.55)에서 엉켰고, 둘 다 도킹 3번 실패로 격리(256.4 s, 270.9 s) | 0.981 |
| g3exp | 급행 상자 예정이었으나 놓이지 않음(at_s 1이 노드보다 일러 건너뜀) | **8/8**(시나리오 481 s, FAIL은 parked_while_backlog 3) | 457.6 | 37 | 0.09(닿음, 276.4 s pinky2-pinky3) | 1 | 4 | 0 | 0 | 없음 | 0.942 |

### 마당 상자와 keepout 침범(실제 위치, Nav2 goal 중인 로봇, 발자국 0.12 m 사각형)

| 경기 | 상자 | 침범 횟수 | 가장 깊게 | 가장 길게 | 합 |
|---|---|---|---|---|---|
| g3yard | 있음(60~180 s) | 8번(그중 7번이 상자가 있던 동안) | 0.04 m | 2.69 s | 5.27 s |
| g2d(비교, 장애물 없음) | 없음 | 15번 | 0.05 m | 2.38 s | 13.0 s |
| g2e(비교, 장애물 없음) | 없음 | 8번 | 0.04 m | 2.98 s | 7.24 s |

- 상자가 마당에 있어도 keepout 침범은 장애물 없는 경기보다 늘지 않았다. 가장 깊은 것도 0.04 m다.
- g3yard는 상자 때문이 아니라 마당에서 두 로봇이 부딪쳐 멈췄다. 208.4 s에 pinky1과 pinky2의 몸이 닿았고(touch.py), 203.1~209.6 s에 pinky2의 실제 위치는 (0.363, 0.558) 근처에 머물렀는데 보고 위치는 0.551 m 움직였다(B-G4와 같은 모양). 그 뒤 두 로봇의 보고 위치가 틀어져 도킹이 실패했다. 서로의 라이다에는 상대의 라이다 머리만 보인다(gz_bugs.md 질문 b).
- g3exp: 상자가 놓이지 않았으므로 장애물 없는 D2 경기(트리 65901fc)로 읽는다. 후진 16번 모두 끝남, GOAL_ENDED_AT_TARGET 0, keepout 침범 15번(최대 0.06 m, 합 12.2 s). 일정 파일을 고쳐(sites 5a18d1f, at_s 15) 급행 상자 경기를 다시 돈다.
- g3exp2(65901fc, 상자 일정만 5a18d1f의 파일): 무엇이 로봇을 세웠나. **아무것도 세우지 않았다.** pinky2는 38.0 s 후진을 마치고 `FOLLOW_SENT to O`로 급행을 동쪽으로 달려 상자(서쪽 면 x 0.95)에 앞을 대고 섰다. 실제 위치는 46~62 s 동안 x 0.897~0.909, y 0.885~0.902에 머물며 yaw만 -0.59~0.36 rad로 흔들렸다. pinky_fsm은 BLOCKED를 내지 않았다(보고 상태는 줄곧 MOVING). 고정 경로 충돌 검사를 껐으므로 RPP도 서지 않았다. 상자는 정적 모델이라 밀리지 않았다. 로봇은 상자를 밀며 바퀴가 헛돌았고, 보고 위치(AMCL)는 그동안 0.82 m 앞으로 흘러 출고 도크(1.74, 0.90)에 닿았다고 보고했다. 도킹 대역이 보고 위치로 판정해 세 번 실패했고 격리됐다. 지시대로 조정 없이 이 경기를 그대로 적고 멈췄다.
- g3exp2 keepout 침범: 2번, 가장 깊게 0.03 m, 합 0.51 s.
- 1번째 시도(g3exp2_cut)는 11:59에 WSL이 꺼져 9분 만에 끊겼다. 위 숫자는 13:33에 다시 돈 한 번이다.
