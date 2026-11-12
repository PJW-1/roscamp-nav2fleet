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

- 트리: g2b, g2c는 wt-nf-gztest(sites aadeac3 + world 74e8264, 69e3fea 없음). g2d부터는 wt-nf-sites의 sites f11df6b(aadeac3 + nav2-base 69e3fea, world 포함)다.
- g2d: 주문 8건을 모두 냈다. FAIL은 시나리오 판정 `parked_while_backlog`(묶이지 않은 주문이 있는데 주차 자리에 들어감) 3번 때문이다. 85.0 s pinky2, 158.6 s pinky1, 240.6 s pinky2다. 관제가 도크가 찼을 때 주차 자리(P-2)를 대기 자리로 쓰는 것(`DOCK_QUEUED ... waits at P-2`)과 판정이 맞지 않는다.
- g2d: GOAL_ENDED_AT_TARGET 1번(9.3 s 서 있음). 후진 W > X_W가 X_W 0.006 m에서 yaw 오차 0.156 rad로 끝나지 않았다(gz_bugs.md B-G1, 원인 확정).
- g2d: 마당 keepout 침범(Nav2 goal 중, 실제 위치, 발자국 0.12 m 사각형): 15번, 가장 깊게 0.05 m, 가장 길게 2.38 s, 합 13.0 s. 장애물 없는 경기의 기준값이다.
- 69e3fea(목표 끝에 닿은 실패를 도착으로 봄)는 g2b, g2c 모두 들어 있지 않다. GOAL_ENDED_AT_TARGET 열은 69e3fea가 든 경기(g2d부터)에만 뜻이 있다.
- g2b: pinky2가 출고 도크에서 X_E로 물러난 뒤 끝에 닿고도 후진 goal이 끝나지 않았다. 재시도 둘이 바로 FAILED로 끝나 80 s에 격리됐다. 격리된 로봇이 J_E와 HALL을 쥐어 pinky1이 끝까지 기다렸다. gz_bugs.md B-G1.
- g2c: gz_bugs.md B-G2. 마당에서 로봇이 서로 길을 막았고, 관제는 교착으로 보지 않았다(DEADLOCK_SEEN 0). ZONE_MARKED_BLOCKED 16번, BLOCKED_WAIT 57번, HALL_YIELD 12번. 감시견(관제 로그가 420 s 조용함)이 시뮬 1136 s에 경기를 끝내서 summary.json과 events.csv가 없다. 숫자는 rec.csv(실제 위치, 관제 이벤트)와 fleet_check.json에서 냈다.
- g2c의 후진 7번은 모두 RETREAT_DONE으로 끝났다. 끝의 yaw 오차는 최대 0.055 rad였다(O > X_E 두 번은 -0.032, 0.055).
- 최소 거리 0.09 m는 발자국(0.12 x 0.12 m) 두 개가 겹치는 거리다. 시뮬 안에서 두 로봇이 닿았다.

## G3: C와 D(배치 고르기용, 같은 열)

트리: wt-nf-sites의 sites f11df6b(aadeac3 + nav2-base 69e3fea, world 포함). GOAL_ENDED_AT_TARGET 열이 뜻이 있다.

| 경기 | 배치 | 판정 | 주문 | 끝 시각 | 이동 한 건 | 정지 | 대기(횟수/합 s) | 최소 거리 | DEADLOCK_SEEN | OUTSIDE_ZONE | Retreat | 급행(방향 SET/HANDOVER) | GOAL_ENDED_AT_TARGET | 멈춘 것 | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| g3c1 | C | 멈춤(감시견) | 3/8 | 214.0 | 60.8(2건) | 12 | 0 / - | 0.214(pinky1-pinky3) | 0 | 0 | 6 | 3/0 | 0 | pinky1이 북쪽 통로 (1.39, 0.73)에서 막힌 뒤 관제가 다시 보내지 않음(B-G3) | 0.969 |
| g3d1 | D | 멈춤(감시견) | 5/8 | 430.6 | 34.2(23건) | 37 | 7 / - | 0.099(pinky2-pinky3) | 0 | 0 | 13 | 2/0 | 0 | pinky3이 CORR_M에서 막힌 뒤(396.2 s, "tried again in 60 s") 다시 보내지 않음. B-G3과 같은 모양 | 0.975 |

- g3c1: 감시견이 시뮬 618 s에 끝내 summary.json이 없다. 숫자는 rec.csv와 fleet_check.json에서 냈다. 이동 한 건은 끝난 이동이 2건뿐이라 대표값이 아니다.
- g3d1: 트리 f11df6b(02563a8 없음). 감시견이 시뮬 837 s에 끝냈다. GOAL_STALLED 1번(이동), 후진 13번은 모두 끝났다(yaw 오차 0.12 이하).

## 로봇 몸이 닿았나(리드 질문 c)

몸은 pinky_description의 base_link 충돌 메시 경계(x -0.075~0.039 m, y -0.041~0.047 m)로 보고, 실제 위치에서 두 몸 사이 틈을 쟀다(touch.py).

| 경기 | 최소 중심 거리 | 몸 사이 최소 틈 | 닿았나 |
|---|---|---|---|
| g2c | 0.09 m | 0(149.1 s pinky1-pinky3, 중심 0.098 m) | 닿았다 |
| g2d | 0.099 m | 0(173.6 s pinky1-pinky2, 중심 0.113 m) | 닿았다 |
| g3c1 | 0.214 m | 0.124 m | 닿지 않았다 |
| g3d1 | 0.099 m | 0(292.4 s pinky2-pinky3, 중심 0.111 m) | 닿았다 |

- g2d의 최소 거리 0.099 m에서 몸은 닿았다. 메시 경계 상자 기준이라 실제 메시보다 조금 크게 본다. 그래도 경계 상자가 겹쳤으므로 시뮬 안에서 부딪쳤다고 본다.
