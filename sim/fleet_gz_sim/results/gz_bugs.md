# Gazebo 경기에서 찾은 결함(관제, 핑키 쪽)

## B-G1. 출고 도크에서 X_E로 물러난 로봇이 끝에 닿고도 실패로 격리된다

- 어디서: 경기 g2b(D2, 3대, 주문 8건, 고정 경로 충돌 검사 끔). 트리 wt-nf-gztest(sites aadeac3 + world 74e8264).
- 재현: `BUILD=wt-nf-gztest game.sh g2b layout:=d2 fixed_line_collision_check:=false`. 66.5 s에 pinky2가 출고 도크에서 `out_out`(O > X_E, 후진)을 시작했다.
- 일어난 일(events.csv와 robots.csv, 시뮬 시각):
  - 66.5 s `RETREAT_START backs to X_E`. 로봇은 70.1 s에 (1.586, 0.889)에 닿았고, 그 뒤 (1.578, 0.887)에 서 있었다. X_E는 (1.58, 0.89)이므로 거리 허용치 0.05 m 안이다.
  - 그런데 후진 goal이 끝나지 않았다. 75.8 s `GOAL_STALLED retreat goal 15 open, no movement for 6 s`, `GOAL_RETRY STALLED, attempt 1`.
  - 다시 보낸 후진 두 번은 pinky_fsm이 바로 FAILED로 끝냈다(`goal ended: FAILED` 두 줄, 77.9 s와 79.9 s). 로봇이 이미 경로 끝에 있어서 `_too_short`(core.py:369~372)가 FAILED를 돌려준 것으로 본다.
  - 80.0 s `GOAL_FAILED FAILED after 3 attempts`, `ROBOT_ISOLATED FAILED; holds ['J_E', 'HALL']`.
  - 그 뒤 pinky1이 J_E+X_E를 97.7 s부터 기다렸고(`ZONE_WAIT`), 1800 s 시나리오 한도까지 아무것도 진행되지 않았다. 주문 1/8.
- 확인한 것:
  1. [확정] 로봇은 후진 목적지 X_E에 닿아 서 있었다(위치 기록).
  2. [확정] 재시도 두 번은 움직이지 않고 FAILED로 끝났다. 끝에 이미 있는 로봇의 후진 재시도를 실패로 세어 격리까지 간다(pc/fleet_core watch.py의 재시도 규칙과 pinky_fsm `_too_short`가 만나는 곳).
  3. [추측] 첫 시도가 끝나지 않은 까닭은 방향이다. FollowPathReverse는 `use_rotate_to_heading: false`(nav2_params.yaml:213)라서 끝에서 제자리 회전을 하지 않는다. 그런데 goal checker는 yaw 허용치 0.12 rad(nav2_params.yaml:109)를 요구한다. 도크 대역(sim_dock_stub)이 도크 4.1 cm 앞에서 성공을 냈고, 후진 중 조향으로 yaw가 0.12 rad 넘게 틀어졌다면 goal은 끝나지 않는다. 이 경기에는 yaw 기록이 없어서 확인하지 못했다. 다음 경기부터 실제 yaw를 함께 기록한다(rec_truth.py).
- 영향: 격리된 로봇이 J_E와 HALL을 쥔 채 남아 경기가 멈춘다(운영자 처리 전까지는 설계대로다).
- 후속: 관제 수정 69e3fea(끝에 닿은 실패나 멈춤을 도착으로, GOAL_ENDED_AT_TARGET)가 nav2-base에 들어왔다. g2d부터 이것을 넣은 sites(f11df6b)로 돈다.
- g2c(69e3fea 없음)의 후진 7번은 모두 끝났다. O > X_E 두 번의 끝 yaw 오차는 -0.032, 0.055 rad였다. 0.12를 넘은 후진은 아직 없다.
- **g2d에서 방향 원인을 확인했다([확정]).** sites f11df6b(69e3fea 포함)로 돈 g2d에서 후진 16번 중 1번이 끝나지 않았다.
  - pinky2, 299.8 s `RETREAT_START backs to X_W`(W > X_W). 실제 위치는 X_W에서 0.006 m였고, 끝 yaw 오차는 0.156 rad였다(허용치 0.12 rad).
  - 309.1 s `GOAL_STALLED retreat goal 85 open, no movement for 6 s`, 이어서 `GOAL_ENDED_AT_TARGET ... the robot stands at X_W`. 69e3fea 덕분에 격리 없이 이어졌다. 서 있던 시간은 9.3 s다.
  - 나머지 15번은 yaw 오차 0.12 rad 이하에서 RETREAT_DONE으로 끝났다. g2c까지 합하면 후진 23번 중 1번(약 4%)이다.
  - 측정: ana_retreat.py. 기대 yaw는 출발 지점 S를 바라보는 방향 atan2(S - T)이고, 끝 이벤트 시각의 실제 yaw와 비교했다.
- 가장 작은 Nav2 변경 제안(만들지 않음, 실물 로봇 작업 목록용. 근거: 후진 23번 중 1번, 그 1번에 9.3 s를 섰다. pinky_nav_config 기본값은 바꾸지 않았다):
  - controller_server에 yaw를 보지 않는 둘째 goal checker를 더한다. 예: `goal_checker_plugins: ["general_goal_checker", "reverse_goal_checker"]`, `reverse_goal_checker`는 `nav2_controller::SimpleGoalChecker`이고 `xy_goal_tolerance: 0.05`, `yaw_goal_tolerance: 3.14`다.
  - pinky_fsm은 후진 경로(reverse=True)를 보낼 때만 FollowPath 목표의 `goal_checker_id`를 `reverse_goal_checker`로 준다. 앞으로 가는 이동과 마당 goal은 그대로다.
  - 까닭: FollowPathReverse는 `allow_reversing` 때문에 `use_rotate_to_heading`을 켤 수 없어, 끝에서 yaw를 고칠 방법이 없다. 그러니 yaw를 요구하지 않는 편이 맞다. 다음 앞 이동이 시작할 때 RPP가 제자리 회전으로 방향을 맞춘다.
- 제안(B-G1, 담당과 합의 전): 후진 재시도에서 로봇이 이미 끝에 있으면 FAILED가 아니라 성공(또는 따로 구별되는 결과)으로 답한다. 또는 관제가 STALLED 뒤 로봇이 목적지 허용치 안에 있으면 성공으로 본다.

## B-G2. 출고 대기 자리(O_Q)로 가던 로봇이 N_E 바로 아래에 서서 다른 로봇의 길을 막는다

- 어디서: 경기 g2c(D2, 3대, 주문 8건). 트리 wt-nf-gztest(sites aadeac3 + world 74e8264), 69e3fea 없음.
- 재현: `BUILD=wt-nf-gztest REC='pinky1 pinky2 pinky3' game.sh g2c layout:=d2 fixed_line_collision_check:=false`.
- 일어난 일(rec.csv: 실제 위치와 관제 이벤트, 시뮬 시각):
  - 180.5 s pinky2 `NAV_GOAL_SENT to O_Q`(O_Q는 (1.78, 0.55)). 211~234 s에 pinky3에게 HALL_YIELD를 세 번 하고 goal을 다시 보냈다. 마지막은 239.3 s다.
  - 242 s부터 pinky2는 실제 위치 (1.605, 0.552), yaw 0.49 rad에 서서 끝까지 움직이지 않았다. O_Q까지 0.175 m, N_E (1.58, 0.65)까지 0.10 m다.
  - 그 뒤 pinky2의 관제 이벤트는 742.7 s `ZONE_WAIT has waited 502 s for N_E` 하나뿐이다. 즉 240 s 무렵부터 관제는 pinky2를 N_E를 기다리는 로봇으로 보았다.
  - pinky3은 307.9 s부터 742.7 s까지 N_E로 가는 마당 goal을 되풀이했다. `ZONE_MARKED_BLOCKED ...->N_E in HALL`이 열 번 넘게 나왔고, 결국 `GOAL_FAILED NAV_ABORTED after 3 attempts`, `ROBOT_ISOLATED NAV_ABORTED; holds ['HALL']`가 났다.
  - 633 s에 pinky2와 pinky3의 중심 사이가 0.09 m였다(발자국이 겹친다). pinky1은 P-2에서 출고 도크를 기다렸다. 주문은 2/8에서 멈췄다.
  - DEADLOCK_SEEN은 0이었다.
- 확인한 것:
  1. [확정] pinky2는 O_Q에 닿지 않고 N_E 0.10 m 아래 마당에 섰다. 관제는 그 뒤 pinky2를 N_E 대기로 다뤘다.
  2. [확정] pinky3의 N_E 마당 goal은 되풀이해 막혔다(ZONE_MARKED_BLOCKED 16번, BLOCKED_WAIT 57번). 그동안 pinky3은 pinky2 옆 (1.6~1.7, 0.5~0.6)을 맴돌았다. 관제는 이것을 교착으로 보지 않았다.
  2-1. [추측] 막은 것은 pinky2의 몸이다. 마당에 다른 장애물은 없었고, pinky2는 N_E 0.10 m 아래에 서 있었다.
  3. [추측] pinky2의 마당 goal이 O_Q 0.175 m 앞에서 성공으로 끝났다. AMCL 보고 위치가 실제와 0.1 m 넘게 달랐거나, 관제가 이 위치를 O_Q 도착으로 받아들였을 수 있다. 이 경기에는 보고 위치 기록이 없다. 다음 경기부터 rec_truth.py가 /pinkyN/state(보고 위치, 상태, current_goal)도 1 Hz로 남긴다.
- 영향: 출고 쪽 마당에서 두 로봇이 서로 막아 경기가 멈춘다.
- 제안(담당과 합의 전): 대기 자리 도착은 대기 자리 지점과의 거리(보고 위치)로 확인하고, 마당에서 같은 goal의 ZONE_MARKED_BLOCKED가 되풀이되면 막은 로봇이 누구인지 이벤트에 남긴다.

## B-G3. 배치 C: 북쪽 통로에서 막힌 로봇에게 "60 s 뒤 다시"라고 하고 다시 보내지 않는다

- 어디서: 경기 g3c1(C, 3대, 주문 8건). 트리 wt-nf-sites의 sites f11df6b(69e3fea 포함).
- 재현: `BUILD=wt-nf-sites REC='pinky1 pinky2 pinky3' game.sh g3c1 layout:=c fixed_line_collision_check:=false`.
- 일어난 일(rec.csv: 실제 위치, 보고 위치와 상태, 관제 이벤트):
  - 84.8 s pinky1 `NAV_GOAL_SENT to P-2`(도크가 차서 P-2에서 기다리러 감). 북쪽 통로 CORR_N을 서쪽으로 가다 (1.45, 0.727)에서 BLOCKED가 됐다가 다시 움직였고, 99.8 s부터 (1.396, 0.726)에서 BLOCKED로 섰다.
  - 109.5 s `ZONE_MARKED_BLOCKED N_E->P-2 in CORR_N, tried again in 60 s`. 같은 때 `NAV_GOAL_SENT to N_W`. 121.0 s `GOAL_RETRY NAV_ABORTED, attempt 1`(pinky_fsm 로그 `navigate_to_pose #3 aborted: 104`). 123.0 s 다시 N_W.
  - 138.0 s `ZONE_MARKED_BLOCKED N_E->N_W in CORR_N, tried again in 60 s`. 그 뒤 pinky1은 IDLE(current_goal 비어 있음)로 (1.392, 0.726)에 서 있었고, 관제는 감시견이 끝낸 시뮬 약 620 s까지 pinky1에게 아무것도 보내지 않았다.
  - pinky3은 201.6 s에 P-2로 가다 (0.183, 0.794)에 섰고, pinky2는 P-1에 있었다. 214.0 s 뒤로 관제 이벤트가 없다. 주문 3/8.
- 확인한 것:
  1. [확정] 138 s의 "tried again in 60 s" 뒤에 pinky1에게 다시 보낸 goal이 없다(관제 이벤트, 보고 상태 IDLE과 빈 current_goal).
  2. [확정] 그동안 통로에 다른 로봇은 없었다(130 s pinky2 (1.74, 0.50), pinky3 (0.45, 0.41)).
  3. [추측] Nav2가 막힌 까닭은 keepout이다. pinky1의 발자국은 133.5 s부터 keepout 칸 안으로 0.01 m 들어가 있었다(ana_keepout.py). 시작 자리가 keepout 안이면 planner가 길을 못 낼 수 있다. Nav2 로그로는 확인하지 못했다.
- 영향: 관제가 로봇을 통로 한가운데 IDLE로 둔 채 멈춘다. 격리도 교착 판정도 없다.
- 제안(담당과 합의 전): "tried again in 60 s"를 낸 경우 60 s 뒤 실제로 다시 보내는지 관제 시험으로 고정한다. 다시 보낼 길이 없으면 WARN 이벤트를 남긴다.
- 후속(B-G3): 관제 수정 02563a8(통로 안에서 나갈 길을 nav-goal 연결의 끝 지점으로 찾고, 정말 길이 없으면 ROBOT_IDLE)이 nav2-base에 들어왔다. 리드가 헤드리스 하니스에서 확인한 원인은 keepout이 아니라 CORR_N에 지점이 없는 것이다. keepout 추측은 아직 열려 있다. planner가 그 자리에서 길을 못 내면 이제는 침묵 대신 NAV_ABORTED 재시도와 격리가 보일 것이다.

## B-G2 리드 질문의 답(추가 경기 없이 g3c1, g3d1 기록으로)

a) 마당 goal이 성공으로 끝날 때 목표 지점과의 거리(navend.py: 보고 상태가 MOVING에서 IDLE로 바뀐 순간, 관제가 끊은 goal은 뺐다):
   - g3d1(D): NAV 14번. 보고 위치는 중앙값 0.050 m, 최대 0.081 m. 실제 위치는 중앙값 0.057 m, 최대 0.095 m다.
   - g3c1(C): NAV 2번. 보고 0.043/0.047 m, 실제 0.070/0.071 m다.
   - 고정 경로(FOLLOW) 끝은 g3d1 15번에서 보고 최대 0.049 m, 실제 최대 0.072 m다.
   - 목표에서 0.14~0.57 m 앞에서 멈춘 경우(g3d1 139.0 s O_Q 0.183 m, 147.8 s P-3 0.566 m, 322.6 s W_Q 0.141 m)는 모두 그 직전 2 s 안에 관제가 HALL_YIELD나 ZONE_MARKED_BLOCKED로 goal을 끊은 경우였다. Nav2가 다른 로봇 때문에 목표 앞에서 "성공"한 경우는 보지 못했다.
   - 그러므로 B-G2의 pinky2(O_Q 0.175 m 앞)도 HALL_YIELD로 끊긴 뒤 그 자리에 남은 것일 가능성이 크다([추측], 그 경기에는 보고 상태 기록이 없다).
b) 한 로봇의 라이다가 다른 로봇의 무엇을 보나(pinky_description 모델):
   - 라이다 빔 높이는 바닥에서 0.125 m다(base_footprint > base_link 0.028, > rplidar_mount 0.067, > rplidar_link 0.030).
   - 몸(base_link 충돌 메시)의 꼭대기는 0.028 + 0.085 = 0.113 m다. 빔이 몸 위로 지나간다.
   - 빔에 걸리는 것은 다른 로봇의 라이다 머리(rplidar_c1 메시, 반지름 0.028 m, 높이 0.095~0.136 m)뿐이다. 머리 중심은 로봇 중심에서 뒤로 0.017 m다.
   - 즉 costmap에는 다른 로봇이 지름 약 5.6 cm의 점으로 찍히고, 몸(0.114 x 0.088 m)은 보이지 않는다. 시뮬에서 몸이 닿는 까닭과 맞는다.
c) g2d 최소 거리 0.099 m: 몸이 닿았다(stage4_results.md의 "로봇 몸이 닿았나" 표).

## B-G4. 고정 경로 검사를 끈 로봇이 진열대 동쪽 끝 옆에 끼고, 보고 위치가 벽 너머로 흘러 도킹이 실패한다

- 어디서: 경기 g2e(D2, 3대, 주문 8건). 트리 sites 65901fc(nav2-base 02563a8 포함). 고정 경로 충돌 검사 끔(리드 결정).
- 재현: `BUILD=wt-nf-sites REC='pinky1 pinky2 pinky3' game.sh g2e layout:=d2 fixed_line_collision_check:=false`.
- 일어난 일(rec.csv):
  - 196.2 s pinky2 `FOLLOW_SENT to O`(out_in: N_E > X_E > O). 실제 위치는 약 200 s부터 (1.556, 0.764), yaw 2.69 rad에 서서 끝까지 움직이지 않았다. 진열대 shelf_1의 동쪽 끝은 x 1.48, y 0.775~0.79다.
  - 그런데 보고 위치(AMCL)는 계속 움직였다. 202.9 s (1.656, 0.952), 205.1 s (1.814, 0.925), 208.3 s부터 (1.730, 1.045)에 섰다. 북쪽 벽(y 0.99) 너머다. 실제와 0.3 m 넘게 다르다.
  - 207.9 s `OUTSIDE_ZONE at (1.73, 1.05), 0.06 m outside ['OUT_DOCK']`. 도킹 대역 로그 `dock OUTBOUND FAILED (-24.4, -12.6) cm` 세 번, 219.4 s `GOAL_FAILED DOCK_FAILED after 3 attempts`, `ROBOT_ISOLATED DOCK_FAILED; holds ['OUT_DOCK']`.
  - 그 뒤 출고 도크를 쓸 수 없어 경기가 3/8에서 멈췄다.
- 확인한 것:
  1. [확정] 실제 로봇은 진열대 동쪽 끝 옆에 서 있었고, 보고 위치는 벽 너머까지 흘렀다.
  2. [추측] 몸이 진열대 끝에 걸려 바퀴가 헛돌았고, 오도메트리를 따라 AMCL이 흘렀다. 고정 경로 충돌 검사를 껐기 때문에 RPP가 진열대를 보고 서지 않았다. 몸이 진열대에 닿았는지는 메시 접촉으로 확인하지 않았다.
- 영향: 고정 경로 검사를 끈 시뮬 설정의 대가다. 실물 로봇에서도 고정 경로 검사가 꺼져 있으면 같은 일이 날 수 있다.
- 제안(담당과 합의 전): N_E > X_E 모퉁이의 제자리 회전 자리를 진열대 끝에서 떼거나(G1 제안 1과 같다), 고정 경로에서도 진열대만은 보도록 한다.

## B-G5. 마당 서쪽(W_Q, N_W 근처)에서 보고 위치가 흐른 뒤 작업 도킹이 실패하고 격리된다

- 어디서: g3yard(D2, 마당 상자)와 g3d2(D), 둘 다 sites 65901fc. 3대, 주문 8건, 고정 경로 충돌 검사 끔.
- 일어난 일(rec.csv, freeze.py):
  - g3d2: 314.1~322.7 s pinky1의 실제 위치는 (0.257, 0.537) 0.05 m 안에서 제자리로 돌았고(yaw 3.03 > 0.61 > -1.13 > 0.72), 보고 위치(AMCL)는 0.401 m 움직였다. 331.3 s `FOLLOW_SENT to W`, 343.5~353.7 s `DOCK_FAILED` 세 번, `ROBOT_ISOLATED DOCK_FAILED; holds ['HALL_W', 'J_W', 'WORK_DOCK']`. 작업 도크를 쥔 채 격리되어 경기가 7/8에서 멈췄다.
  - g3yard: 203.1~209.6 s pinky2의 실제 위치는 (0.363, 0.558) 근처, 보고 위치는 0.551 m 움직였다. 208.4 s에 pinky1과 몸이 닿았다. 그 뒤 pinky2(출고)와 pinky1(작업) 모두 도킹 3번 실패로 격리됐다.
- 확인한 것:
  1. [확정] 두 경기 모두 실제 위치는 거의 서 있는데 보고 위치가 0.4~0.55 m 흐른 직후 도킹이 세 번 실패하고 격리됐다.
  2. [추측] 마당 서쪽 구석에서 다른 로봇과 엇갈리며 제자리 회전이나 접촉으로 바퀴가 미끄러지고, AMCL이 오도메트리를 따라 흘렀다. 서로의 라이다에는 상대의 라이다 머리만 보여(질문 b) 가까이 붙기 쉽다.
- 제안(담당과 합의 전): 도킹 실패가 나면 같은 자리에서 세 번 다시 하기 전에 보고 위치를 다시 잡는다(예: 마지막 지점으로 되돌아가 AMCL을 다시 맞춘다). 시뮬에서는 도킹 대역이 보고 위치만 보므로, 실물 도킹(pinky_dock)이 붙으면 다시 재야 한다.
- 원인 후보(2026-11-12 추가, 리드가 g4o3_on2 rec.csv에서 찾고 flip.py로 다시 셈): 점대칭 경기장에서 AMCL이 180° 뒤집힌 자세에 붙는다.
  - g4o3_on2(sites-order c02eb9c, order 모델 켬): 244.8 s에 pinky1과 pinky3의 몸이 닿았다(touch.py 몸 틈 0). 리드 확인으로는 pinky1이 341~363 s에 실제 위치가 거의 제자리((1.05, 0.33)에서 (1.02, 0.35))인 채 방향 오차가 44°, 71°, 114°, 147°, 180°로 커졌다. freeze.py도 pinky1을 348.3~359.0 s(보고 위치 0.313 m 이동)에 B-G4로 잡았다. flip.py로 세면, pinky1의 보고 위치가 "실제 위치를 경기장 중심(0.975, 0.52)으로 점대칭시킨 자리"에서 0.08 m 안이고 방향 오차가 150°를 넘는 보고가 376~903 s에 498번 있었다(가장 가까이 0.035 m). 388.8 s에는 W 도크(0.23, 0.89)에 있다고 보고했지만 실제는 (1.72, 0.11)이었다.
  - 다른 경기(같은 flip.py 기준): g3yard는 pinky2가 212~691 s에 456번 맞았다(가장 가까이 0.056 m). g3d2, g3exp2, g2e, g4o3_off1, g4o3_on1, g4o3_off2는 맞는 보고가 없었다. g4o3_off2의 pinky3은 끝에 방향 오차가 약 179°였지만 보고 위치가 점대칭 자리와 0.9 m 넘게 떨어져 있어 뒤집힘이 아니라 흐름으로 본다.
  - [추측] 순서는 접촉, 바퀴 헛돎, 방향 추정이 돌아감, 그리고 점대칭 경기장에서 AMCL이 뒤집힌 자세를 맞는 것으로 받아들임이다. g3d2의 흐름(0.401 m)은 이 기준으로는 뒤집힘이 아니었다.

## B-G6. 고정 경로 검사를 끄면 급행 위 상자 앞에서 아무것도 로봇을 세우지 않는다

- 어디서: g3exp2(D2, 3대, 급행 가운데 0.08 m 상자, 시뮬 15 s부터). sites 65901fc, 상자 일정은 sites 5a18d1f.
- 일어난 일: pinky2가 급행을 동쪽으로 달리다 상자에 앞을 대고 섰다(실제 x 0.90, 46~62 s). 보고 상태는 MOVING 그대로였고 BLOCKED가 없었다. 보고 위치는 0.82 m 앞으로 흘러 출고 도크에 닿았다고 했고, 도킹 3번 실패로 격리됐다.
- 비교: G1의 g1box1(한 대, 충돌 검사 켬)에서는 같은 상자 0.10 m 앞에서 섰고 BLOCKED를 냈다.
- [확정] 고정 경로 충돌 검사를 끈 시뮬 설정에서는 pinky_fsm의 막힘 판정도 로봇을 세우지 못했다. 막힘 판정은 로봇이 멈춰야 동작하는데, 바퀴가 헛돌아 오도메트리가 움직인다고 보고했기 때문으로 본다([추측]).
- 제안(담당과 합의 전): 실물에서 고정 경로 검사를 끄려면 그 대신 무엇이 앞의 물체를 볼지 먼저 정해야 한다. 예: 고정 경로 controller의 충돌 검사는 켜 두고 G1에서 찾은 회전 자리 문제를 지도에서 푼다.

## B-G7. 마당 동쪽(N_E 아래)에서 두 로봇이 몸 틈 0.012 m까지 붙은 뒤 둘 다 보고 위치가 흐르고 격리된다

- 어디서: g4o3_off2(D2, 3대, 주문 8건, order 모델 끔). sites-order c02eb9c, 고정 경로 충돌 검사 끔. 결과는 order_gz_results.md(sites-order)에 있다.
- 일어난 일(rec.csv, touch.py, drift.py):
  - 149.1 s: pinky1(P-3로 가는 중, 실제 (1.700, 0.383))과 pinky3(O_Q로 가는 중, 실제 (1.619, 0.457))의 중심 거리 0.110 m, 몸 틈 0.012 m. 이때 pinky1의 실제 yaw는 -2.93, 보고 yaw는 -1.24로 이미 1.7 rad 어긋나 있었다.
  - 보고 위치와 실제 위치의 차이는 150 s까지 0.06 m 아래였고, 180 s에 pinky1 0.49 m, pinky3 0.53 m, 195 s부터 pinky3 0.90 m, pinky1 0.41 m로 남았다.
  - 198.4 s: pinky3 `ROBOT_ISOLATED DOCK_FAILED; holds ['J_E', 'OUT_DOCK']`, O-003을 다시 줄에 넣었다.
  - 264.2 s: pinky1 `GOAL_FAILED NAV_ABORTED after 3 attempts`, `ROBOT_ISOLATED`(N_W로 가는 길을 못 짬, 보고 위치 (1.25, 0.69)). 그 뒤 관제 로그가 끊겨 감시견이 경기를 멈췄다. 주문 8건 중 2건이 나갔다.
- 확인한 것:
  1. [확정] B-G4 판정(실제는 서 있고 보고만 움직임)에는 걸리지 않았다(freeze.py 0건). 이 경우는 두 로봇이 같이 움직이는 중에 흘렀다.
  2. [추측] B-G5와 같은 종류다. 두 로봇이 서로의 라이다 머리만 보며 가까이 엇갈리다 몸이 스치거나 바퀴가 미끄러져, 오도메트리와 AMCL이 실제와 갈라졌다. touch.py의 몸 모양 판정으로는 닿지 않았지만(0.012 m) 몸 모양은 어림값이다.
- 같은 배치의 다른 세 판(g4o3_off1, on1, on2)에서는 일어나지 않았다. 최소 몸 틈은 off1 0.031 m, on1 0.042 m였다.

## B-G8. 시나리오 판정이 주차 자리 0.10 m를 실제 위치로 재서, 0.102 m에 선 로봇 때문에 판정 줄이 없다

- 어디서: g4o1_off1(D2, 1대, 주문 3건, order 모델 끔). sites-order c02eb9c.
- 일어난 일: 주문 3건은 197.5 s까지 모두 나갔다. pinky1은 206.9 s에 P-3(1.85, 0.15)로 가는 NavigateToPose를 받아 222 s에 IDLE이 됐다. 그때 보고 위치는 (1.777, 0.181)로 P-3에서 0.080 m, 실제 위치는 (1.764, 0.206)으로 0.102 m였다.
- 원인(코드 확인): scenario_monitor는 모든 로봇이 IDLE이고 주차 자리에서 `PARK_NEAR_M = 0.10` m 안에 있어야 경기를 끝낸다(sim/fake_robots/fake_robots/laps.py:6, :56). 이때 위치는 실제 위치를 먼저 쓴다(zone_checks.py:19). Nav2는 보고 위치로 목표에 닿았다고 했지만, 실제 위치가 0.002 m 넘쳐 판정이 나지 않았다. 관제는 할 일이 없어 로그가 끊겼고, 감시견이 시뮬 630 s에 멈췄다.
- 같은 1대 경기의 다른 다섯 판은 P-3에서 0.06~0.08 m 안에 서서 PASS였다.
- 제안(담당과 합의 전): 판정은 고치지 않았다. 고친다면 주차 판정 거리를 Nav2 목표 허용 오차와 AMCL 오차를 더한 값(예: 0.15 m)으로 넓히거나, 보고 위치로 재는 것을 고를 수 있다.
