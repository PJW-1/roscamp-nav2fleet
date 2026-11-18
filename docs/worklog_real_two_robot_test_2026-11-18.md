# 두 대 실물 시험 준비: 가짜 주문과 PC 도킹 대역 (2026-11-18)

사용자 요청(11-18): "일단 시뮬처럼만 테스트하려고 하는 거니까 두 대용 실물 테스트 준비를 해 줘. 똑같이 도킹 구역에 도달하고, 도킹은 아직 개발이 덜 됐으니 임시로 시뮬처럼 몇 초만 기다렸다가 가는 식으로. 주문도 가짜 주문 넣으면 되잖아."

## 1. 바꾼 것

| 쪽 | 브랜치 | 내용 |
|---|---|---|
| 로봇 | `feat/real-dock-aside` (1d753df, 서브에이전트) | `start.sh`에 `DOCK_STUB=1`을 주면 `real_pode.launch.py`에 `dock_aside:=true`를 넘긴다. 그러면 pode_fsm의 Dock 서버가 `/<ns>/dock`에서 `/<ns>/dock_fsm`으로 비킨다(시뮬 `sim_dock_stub`와 같은 remap). 안 주거나 `0`이면 예전과 같다. 그 밖의 값은 아무것도 띄우기 전에 exit 2 |
| PC | `feat/real-two-robot-orders` (41148e2, 2ac02c6) | `fake_robots`에 `real_dock_wait_node`. 로봇마다 `/<robot>/dock`을 받아 시나리오의 `dock.duration_s`(3 s)를 벽시계로 기다린 뒤, 로봇이 보고한 자세(AMCL, `/<robot>/state`)가 도크 지점에서 0.15 m 안이면 성공, 아니면 실패. `fakes.launch.py`의 `dock_wait_robots`로 켠다. `pc.launch.py`가 `monitor`, `dock_wait_robots`를 넘긴다. 시나리오 `real_two_orders`(v7, pode1 P-1, pode2 P-2). e68446e부터 주문은 Gazebo `gz_v7`의 앞 4건(R-001~R-004, 품목·20 s 간격·팔·도킹 시간·재고 같음)을 20 s 늦춘 것(사용자 결정 11-18: "시뮬이랑 같게 하는데 건수만 4건으로") |

- 시뮬 대역(`fleet_gz_sim.dock_stand_in`)은 시뮬 시계와 격리망에서만 돌게 막혀 있어 실물에 쓰지 않았다. 새 노드는 반대로 벽시계에서만 돈다(`use_sim_time`이면 시작을 거부하고, 나중에 켜는 것도 거부).
- 로봇 코드(pode_fsm)는 바꾸지 않았다. 로봇에 가는 것은 실행 묶음(`tools/real_pode`)뿐이라 로봇에서 다시 빌드할 필요가 없다.

## 2. 시험 날 순서

모두 도메인 12(`tools/real_pode/env.sh`). PC에서도 `ROS_DOMAIN_ID=12`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.

1. 로봇을 P-1(pode1 = 27a6), P-2(pode2 = 07d7)에 북쪽을 보게 놓는다.
2. PC(관제, 가짜 central·팔, 도킹 대역). 기록 폴더는 먼저 만든다(관제는 폴더를 만들지 않고 파일만 연다, `fleet_manager_node.py:78`):

   ```
   mkdir -p ~/real_runs
   ros2 launch iponic_bringup pc.launch.py use_fake:=true use_sim_clock:=false fake_robots:=none \
       robots:="pode1 pode2" map_file:=v7 scenario:=real_two_orders monitor:=false \
       dock_wait_robots:="pode1 pode2" record:=$HOME/real_runs/peer_on_order_on_1.jsonl
   ```

   - `fake_robots:=none`: `ros2 launch`는 빈 값(`fake_robots:=`)을 받지 않는다(2ac02c6 전 문서의 명령은 "malformed launch argument"로 뜨지 않았다).
   - `use_sim_clock:=false`: 기본값 true면 시뮬 시계가 돌아 실물과 맞지 않는다.
   - `monitor:=false`: 시나리오 감시 노드는 시뮬 전용 자세(`/sim/truth/<robot>`)를 읽고, 시나리오의 로봇 수만큼 기다리다 시간이 다 되면 launch를 내린다.
   - 주문은 PC launch가 뜬 뒤 대략 20, 40, 60, 80 s에 들어온다(벽시계, 한 번만, 반복 없음). 로봇이 늦게 떠도 관제가 쌓아 둔다.
   - 판 이름에 조건을 넣는다(`peer_on_order_on_1`, `peer_on_order_off_1`). order를 끄는 판은 `use_order_model:=false`를 더한다.
3. 로봇마다:

   ```
   PEER_ROBOTS=pode1,pode2 DOCK_STUB=1 ~/nav2fleet_real/start.sh pode1 0.12 0.10 1.5708   # 27a6
   PEER_ROBOTS=pode1,pode2 DOCK_STUB=1 ~/nav2fleet_real/start.sh pode2 0.38 0.10 1.5708   # 07d7
   ```

4. 확인:
   - 로봇 launch 로그에 `real_pode: dock_aside on, pode_fsm Dock -> /pode1/dock_fsm`.
   - PC 로그에 `REAL DOCK WAIT for ['pode1', 'pode2']`. `another node also serves /pode1/dock` 오류가 나오면 그 로봇을 `DOCK_STUB=1` 없이 띄운 것이다(로봇의 pode_fsm도 도킹을 받아 DOCK_UNAVAILABLE로 실패시킨다). 다시 띄운다.
   - 도킹마다 PC 로그에 `pode1: dock WORK ok (x, y) cm` 또는 `FAILED`.
5. 끌 때: 로봇마다 `~/nav2fleet_real/stop.sh`, PC는 Ctrl-C.

## 3. 시험하면서 볼 것 (사용자가 준 목록을 코드와 맞춰 고침, 11-18)

사용자: 고정 경로에 장애물은 두지 않으므로 "고정 경로에서 막혔을 때 끝나는 방식(BLOCKED/NAV_ABORTED)"은 이번에 보지 않는다.

| # | 볼 것 | 어떻게 | 근거와 고친 점 |
|---|---|---|---|
| 1 | 통신 끊김 | 로봇이 움직일 때 PC 네트워크를 몇 초 끊었다 잇는다(Wi-Fi 끄기나 랜선). 로봇이 약 1.5 s 뒤 서기 시작하는지(기준은 끊은 순간이 아니라 관제 heartbeat를 마지막으로 받은 때이고, 정확히 1.5 s에 완전히 선다는 보장은 아니다), 다시 이어지면 계속 가는지 | 로봇 `heartbeat_timeout_s` 1.5(`real_pode.launch.py:101`), `comm_loss_policy` 기본 stop(`pode_fsm/params.py:17`)이고 real 묶음은 이 값을 넘기지 않는다. 관제는 HEARTBEAT_LOST를 실패로 세지 않고(`_goal_failed`를 부르지 않아 격리 횟수에 안 들어감) 2 s 뒤부터 다시 보낼 수 있게 한다(`results.py:209`). 실제로 보내는 것은 통신 상태 같은 다른 조건도 맞을 때다. 3 s 넘게 state가 없으면 COMM_LOST, 돌아오면 COMM_RESTORED 뒤 위치를 확인하고 다시 간다(`health.py`). **관제(PC launch)를 끄는 방법은 판 끝에만 쓴다**: 가짜 central·팔·도킹 대역도 같이 꺼지고, 다시 켜면 주문이 R-001부터 다시 나온다 |
| 2 | 급행 북쪽 벽 여유 | 첫 주문(R-001)은 한 대만 움직이므로 그 급행 통과를 가까이서 본다(멈출 준비). 두 대가 출고를 번갈아 하며 `O_X`에서 기다리는 장면도 본다 | 급행 중심선 0.895, 도면 기준 몸 원 여유 약 2 mm, 실측 벽값으로는 음수(`worklog_merge_real_test_2026-11-18.md` 38행). `O_X`는 v7 급행 서쪽 끝 대기 자리 |
| 3 | 위치 추정 | 띄운 직후 state 자세가 놓은 자리(P-1 0.12,0.10 / P-2 0.38,0.10, 북쪽)와 맞는지. 도크에 섰을 때 로봇이 바닥 표시 위에 있는지 눈으로 | **고친 점**: LOCALIZATION_SUSPECT는 2 s 안에 갈 수 있는 거리보다 0.15 m 넘게 튈 때만 남는다(`localization.py`, `pose_jump_*`). 천천히 흐르는 것은 잡지 못하므로 눈으로 본다. PC 로그의 `dock WORK ok (x, y) cm`는 AMCL 자세와 도크 지점의 차이라서 흐름을 보여 주지 않는다(관제가 AMCL로 데려다 놓기 때문) |
| 4 | peer 층 켜고 order 켬/끔 | 두 판(`peer_on_order_on_N`, `peer_on_order_off_N`). 끝나고 `grep -c '"BASE_END_WAIT"' 기록.jsonl` | **고친 점**: BASE_END_WAIT가 많다는 것만으로 "관제 ETA 0.12가 실제보다 빠르다"고 볼 수 없다. 관제는 보통 추정 남은 시간(남은 거리/구간 속도, NAV·FOLLOW 0.12, RETREAT 0.05)이 `lookahead_s` 2 s 이하가 되면 다음 구역을 잡아 본다(`move_orders.py:123`, `trips.py:185`). 로봇이 이미 base 끝에서 기다리는 중이면 이 시간 조건은 건너뛴다(그래도 이미 쥔 구간이 남았으면 그것을 base에 붙이는 update를 먼저 보내고, 도크·horizon 조건에 걸리면 새 구역을 잡지 않는다). 이미 쥔 구간을 base에 붙일 때는 `standby_lookahead_s` 0.5 s를 쓴다(`move_orders.py:116`). 그래서 다른 조건이 같다면 로봇이 0.12보다 느릴수록 업데이트할 여유가 늘어나는 쪽이다(경향일 뿐, 가감속이나 지연 때문에 반드시 줄지는 않는다). 기다림이 생기는 까닭의 예: 다음 구역이나 도크를 다른 로봇이 쥐고 있을 때(정상), Wi-Fi 왕복이 남은 실제 시간보다 길 때, 로봇이 추정 속도보다 빠를 때. 이벤트는 같은 base에서 거듭 기다려도 한 번만 남고, base를 새로 보낼(update) 때마다 다시 남을 수 있다(`order_answers.py:31`, `move_orders.py`의 `_send_update`). order를 끈 판에는 이 이벤트가 없으므로 두 판은 주문 끝난 시각으로 견준다 |
| 5 | 두 대 마주치기 | 마주치면 우선순위 낮은 쪽이 후진했다 다시 가는지(LOCAL_YIELD backup → retry). 끝나고 `python3 tools/yield_report.py 기록.jsonl --encounters`로 LOCAL_YIELD와 HALL_YIELD를 센다 | peer 층은 두 로봇 모두 `PEER_ROBOTS=pode1,pode2`일 때만 켜진다. 4건 시나리오에서 정면으로 마주칠지는 모른다 |
| 6 | 도크 | **바뀐 점**: 이제 로봇이 DOCK_UNAVAILABLE로 거절하지 않고, 유효한 도크 goal이 중간에 끊기지 않으면 PC 대역이 3 s 기다린 뒤 로봇이 보고한 자세로 성공/실패를 정한다(최신 state가 없거나 0.15 m 밖이면 실패). 지도에 없는 도크 ID는 기다리지 않고 바로 실패하고, 취소·새 goal·종료는 기다리는 도중 다음 확인에서 실패로 끝난다(반복마다 20 ms를 쉰 뒤 확인한다). PC 로그에 도킹마다 `dock WORK ok` / `dock OUTBOUND ok`가 나오는지, `another node also serves` 오류가 없는지 | DOCK_UNAVAILABLE(재시도 3번)은 로봇을 `DOCK_STUB=1` 없이 띄웠을 때만 나온다. `FAILED`가 나오면 AMCL 자세가 도크에서 0.15 m 밖이라는 뜻이고, 관제가 두 번 더 시도한 뒤 격리한다(`move_retry_limit: 2`) |
| 7 | 도리도리 | 줄 입구 Q2로 가는 Nav2 goal 구간에서 로봇이 고개를 흔드는지 | 나오면 plan_keep을 넣을 근거(관찰만) |
| - | 고정 경로에서 막힘 | 이번에는 보지 않는다(사용자) | |

판마다: `record:=$HOME/real_runs/<판이름>.jsonl`(위 명령). 같은 이름이면 덮어쓰므로(`open(..., "w")`) 판마다 이름을 바꾼다. 기록에는 관제 이벤트가 `"out": "EventOut"` 줄로 들어가 위 grep과 `yield_report.py`가 읽는다. 주문은 가짜 central이 넣으므로 `use_fake:=true`이고 그래서 `use_sim_clock:=false`가 꼭 필요하다(위 명령에 있음).

## 4. 돌려 본 것

- 새 시험(먼저 써서 실패를 본 뒤 구현):
  - `sim/fake_robots/test/test_dock_wait.py`(판정, 켜도 되는 조건, 시나리오), `test_ros_dock_wait.py`(실제 Dock 액션: 성공, 멀면 실패, state 없음, 없는 도크, 두 로봇 따로, 취소, 새 목표가 옛 목표를 끝냄, LOCALIZING은 자세가 아님, stop이 돌던 목표를 끝내고 기다림, use_sim_time 켜기 거부, 같은 이름의 서버 찾기), `test_launch_dock_wait.py`(기본 꺼짐, 켜기, 시뮬 시계 거부, 가짜 로봇과 겹침 거부, `none`).
  - 41148e2 전에 처음 쓴 21개: 모듈이 없어 수집 오류(실패). 41148e2에서 21 passed. 리뷰 반영으로 6개를 더 쓰고 5개 실패를 본 뒤 2ac02c6에서 27 passed.
  - `pc/fleet_manager/test/test_launch_args.py`(이 파일만, ROS 도메인을 쓰지 않는다): 새 시험 1개 실패를 본 뒤 3 passed.
- fake_robots 전체 시험(도메인 66, LOCALHOST, 혼자 씀):
  - 41148e2: 1 failed, 1071 passed, 1 skipped, 27 xfailed. 실패는 `test_exactly_the_six_scenarios_are_shipped`(새 시나리오가 목록에 없었음). 70cbece에서 목록에 넣었다.
  - 70cbece: 1082 passed, 1 skipped, 27 xfailed (769 s). 그 뒤의 세 시험 커밋(75ae9e7, f961723, 9b4eb3c)은 `test_dock_wait.py`, `test_ros_dock_wait.py`만 바꿨고(나머지 커밋은 이 기록 문서뿐), 두 파일은 따로 돌려 통과했다(`test_ros_dock_wait.py` 13 passed 도메인 67, `test_dock_wait.py` 11 passed).
  - 이 결과는 앞서 도메인 67에서 집중 시험과 전체 시험을 겹쳐 돌려 오염됐던 결과를 대신한다.
- PC 쪽 실제 실행(도메인 67, LOCALHOST, 로봇 없음, 별도 설치 공간): 위 PC 명령으로 `fleet_manager`, `fake_central`, `fake_arm3`, `real_dock_wait`가 떴고 액션 `/pode1/dock`, `/pode2/dock`, `/workcell/fill_basket`, `/arm3/swap_basket`이 보였다. `/central/orders`에 O-001, O-002가 RUN으로 올라왔다. 내 launch 프로세스 묶음만 내렸다.
- 남은 작은 위험: `test_losing_localization_drops_the_last_pose`는 LOCALIZING이 도킹 대기(1 s)보다 늦게 처리되면 정상 노드에서도 실패할 수 있다(Codex [추측], 같은 PC 안 메시지라 1 s 넘게 늦을 일은 드물어 고치지 않음).
- 로봇 쪽: 서브에이전트가 `tools/test/test_real_pode.py`에 시험을 먼저 넣고(8 failed 62 passed) 구현 뒤 70 passed. 리뷰 반영으로 시험을 다듬어 `feat/real-dock-aside`는 9c1db1e까지 왔다. 실제 remap은 `test_launch_dock_aside_moves_real_fsm_server`가 진짜 pode_fsm을 real_pode.launch.py의 `_fsm()`으로 띄워 `ros2 action info`로 확인한다: `dock_aside:=true`면 `/pode1/dock_fsm` 서버가 `/pode1/pode_fsm` 하나이고 `/pode1/dock`은 서버 0, `false`면 반대. 5ad6e57과 9c1db1e에서 내가 도메인 68(LOCALHOST)로 파일 전체를 다시 돌려 둘 다 73 passed(환경을 맞춰 이 시험도 건너뛰지 않고 돈다). Nav2와 bringup까지 띄운 확인은 아니다.

## 5. 시뮬과 다른 점, 조심할 것

- PC가 도킹을 맡는 동안 로봇의 state는 DOCKING이 아니라 IDLE이다. Gazebo `sim_dock_stub`도 같다(pode_fsm 서버를 비키고 다른 노드가 답한다). 관제는 보고 DOCKING을 처음 배치와 통신 복구 때의 재배치 판단에만 쓴다(`fleet_core/traffic/mission.py`의 NOT_PLACEABLE). 도킹 중 통신이 끊겼다 돌아오는 경우만 시뮬 대역과 같은 차이가 난다.
- 도킹 성공은 AMCL 자세가 도크 지점에서 0.15 m 안이어야 한다. 관제의 move_to가 도크 지점에 데려다 놓은 뒤라 보통 맞지만, AMCL이 크게 틀리면 도킹이 실패하고 관제가 두 번 더 시도한 뒤 로봇을 격리한다(`move_retry_limit: 2`).
- 로봇이 위치를 못 잡으면(TF가 오래됨) pode_fsm은 state의 자세를 (0, 0)으로 보낸다. 도크에서 멀어서 실패로 판정되므로 잘못된 성공은 나지 않는다.
- 감시 노드를 끄므로 `summary.json` 같은 결과 판정은 없다. 주문 결과는 관제 로그와 `/fleet/...` 토픽으로 본다.
- 속도는 실물 `start.sh` 기본 0.12 m/s다. 관제 ETA(`nav_speed_mps`, `follow_speed_mps` 0.12)와 기록된 주문 게임(G2~G4, 0.12)과 같다.
- 급행 중심선 0.895의 북쪽 벽 여유는 여전히 실물에서 눈으로 본다(`worklog_merge_real_test_2026-11-18.md` 3절).

## 6. 남은 것

- 사용자 "곧 할 거니까 준비해"(11-18)에 따라 했다:
  - main 병합: 4fa6124에 `feat/real-two-robot-orders`(5bc150e), `feat/real-dock-aside`(8f1b487)를 --no-ff로 합쳐 푸시. 두 브랜치는 고친 파일이 겹치지 않는다. 합친 상태에서 fake_robots 관련 177 passed, `test_launch_args.py` 3 passed, `tools/test/test_real_pode.py` 73 passed(도메인 68).
  - 이 PC: main 체크아웃을 8f1b487로 올리고 `colcon build --packages-select fake_robots iponic_bringup`(2 packages finished). 설치에 `real_dock_wait_node`, `real_two_orders.yaml`, pc.launch의 `dock_wait_robots`가 있다.
  - 로봇: 두 대 모두 launch가 돌지 않는 것을 보고 `tools/real_pode/deploy.sh`로 8f1b487을 보냈다. 192.168.0.200 = wlan e4:5f:01:b9:27:a7(27a6, pode1), 192.168.0.201 = d8:3a:dd:26:07:d8(07d7, pode2). 로봇 패키지는 4fa6124 뒤로 바뀌지 않아 로봇에서 빌드하지 않았다. 두 대 모두 start.sh에 DOCK_STUB이 있고 문법 검사를 통과했으며, `DOCK_STUB=7`은 아무것도 띄우기 전에 거부된다.
  - 실물 시험은 아직 하지 않았다.
