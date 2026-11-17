# 문제점 정리: 지그재그, 가운데 통로, 주차구역 앞 교착, BLOCKED 깜빡임 (2026-11-17)

main 956140d(11-17 인계 문서까지)를 받아 사용자가 말한 문제를 조사한 기록이다. 다음 세션이나 다른 AI가 이 문서만 보고 이어 갈 수 있게 썼다.
조사는 코드, 지도, 파라미터, 머리 없는 세계(헤드리스 모의)로만 했다. Gazebo 판의 기록(`~/nf_logs/chk/<판>/`)은 집 컴퓨터(WSL)에 있어 여기서는 보지 못했다.
그래서 "Gazebo에서 실제로 이렇게 됐다"는 주장은 모두 [추측]이고, 9절의 기록을 보면 가를 수 있다.

표기: [확정] = 코드나 계산으로 확인함, [추측] = 근거는 있지만 실제 판의 기록으로 확인하지 못함.
Codex 리뷰(91cb480)의 지적 7건과 서브에이전트 숫자 정정(판 수)을 반영했다.

## 1. 요약

| 문제 | 가장 유력한 원인 | 상태 |
|---|---|---|
| 지그재그(O_OUT을 나온 뒤 끊김) | 인계 기록의 분석: NavFn이 5 cm 격자에서 내는 계단 경로(재계획과 컨트롤러 값의 몫은 따로 재지 않음). 개선 후보 smoother와 Smac 2D의 붙일 자리와 준비물을 3절에 정리 | 참고용으로 알아 둠. 지금 적용하지 않음(사용자 2026-11-17) |
| 큰 통로(가운데 통로) 가운데를 못 지나감 | 통로가 계획상 로봇 중심 4칸뿐이라, 가운데의 상자 하나(장애물 판의 상자 4)가 전역 costmap에 찍히거나, peer 층이 켜진 채 로봇 하나가 서 있으면 Nav2 전역 계획에서 통로 전체가 막힐 수 있다(셀 양자화에 따라 다름) | 계산 [확정], 실제 판 [추측] |
| 주차구역 앞 교착 한 번 | 칸 입구 줄에서 Nav2 단계의 대치(주차한 로봇, 칸막이 끝, 지나가는 로봇). 관제의 교착 감시와 가까이 붙음 감시는 이 경우를 보지 않는다. 남는 것은 막힘 처리, 멈춤 감시, 로봇 쪽 복구이고, BLOCKED가 깜빡이면 막힘 처리가 늦거나 안 된다(6절) | [추측]. 헤드리스 1,840판(v7 1,440판)에서 재현 안 됨 |
| BLOCKED 깜빡임으로 관제가 우회하지 않음 | 관제가 BLOCKED 아닌 보고 한 번에 10초 타이머를 0으로 되돌린다 | 코드 [확정]. 고친 브랜치 둘 푸시(6절, main 아님) |

## 2. 레포 최신화

- main 08241c0 -> 956140d(89파일). 11-17 인계 문서 `docs/worklog_drive_handoff_2026-11-17.md`가 내용을 정리한다:
  고정 경로 끝 방향 안 맞추기(D), O_OUT 1 cm, O_IN 이동, 대기 order의 목적지 일찍 정하기, 코드 정리 세 묶음.
- 새 원격 브랜치: `feat/nav-goal-smooth`(앞보기 0.15 + 문턱 0.45, main에 없음), `feat/nav-goal-theta-star`(비교용, 11-17 사용자 지시로 `feat/oout-smooth`와 함께 원격에서 지움. 백업 `~/ros2camp/branch-backup-2026-11-17/discarded.bundle`),
  묶음별 브랜치 6개(모두 main에 들어감).
- 공유 install은 새 main으로 다시 빌드했다. 시험 결과는 8절.

## 3. 지그재그: smoother 또는 Smac 2D

**지금 적용하지 않는다.** 사용자 지시(2026-11-17): "바로 적용하라는 건 아니고 개선 방안을 인지만 해 놓으라는 것". 이 절은 나중에 고를 때 쓸 참고 기록이다.

### 지금까지 잰 것(인계 6.2절, `feat/nav-goal-smooth`의 기록 3~5절, Gazebo `gz_v7_rush`)

| 판 | 경기 시간 | 전환당 선 횟수 | collision ahead | 자가 복구 |
|---|---|---|---|---|
| NavFn(main) | 259~265 s | 5.5~5.8 | 133 | 0 |
| Theta* 첫 판 / 둘째 판 | 247.2 / 277.4 s | 2.0 / 1.8 | 118 / 178 | 3 / 3 |
| 앞보기 0.15 + 문턱 0.45(브랜치) | 251.3 / 255.2 s | 2.0 / 2.5 | 83 / 107 | 1 / 3 |

- 원인(인계 기록의 분석, 가설): 처음 0.6 m에서 방향 변화 합이 중앙 142~151도인데 순 변화는 35도. 계획은 1 Hz로 다시 나온다.
- Theta*의 약점: 경로가 팽팽해 좁은 틈(주차한 로봇과 진열대 사이)의 가운데로 가지 않는다. goal 칸이 막히면 실패한다.
- 앞보기/문턱 조정의 부작용: 주차하러 가는 로봇이 칸 입구에서 2~4초 섰다. 장애물 판에서는 나빠졌다(649.4 -> 658.0 s).

### main의 설정 [확정]

- 플래너 `nav2_navfn_planner::NavfnPlanner`, tolerance 0.5, use_astar false(`pode/pode_nav_config/params/nav2_params.yaml:361-366`).
- `smoother_server`는 설정에도 launch 노드 목록(`pode/pode_nav_config/launch/nav2.launch.py:26`, 129~136행)에도 없다.
- Nav2 goal은 `NavigateToPose` + BT `pode/pode_nav_config/bt/navigate_to_pose_w_replanning_and_recovery_goal.xml`:
  `RateController 1 Hz { ComputePathToPose }`(25~33행) 다음 `FollowPath`(35행, goal checker `general_goal_checker` 고정). `SmoothPath` 없음.
- 고정 경로는 FSM이 `FollowPath` 액션을 직접 보낸다. BT를 거치지 않으므로 smoother나 플래너를 바꿔도 영향이 없다.
- 전역 costmap: 해상도 0.05, 발자국 0.12 정사각형 + padding 0.03(내접 0.09), inflation 0.25 / cost_scaling 6.0.

### 붙이는 자리

- smoother: BT의 `ComputePathToPose`와 `FollowPath` 사이에 `SmoothPath`를 넣고, launch 노드 목록에 `nav2_smoother::SmootherServer`를 더한다.
  `pode_nav_config/package.xml`에 `nav2_smoother` exec 의존을 더한다.
- Smac 2D: `planner_plugins`의 `GridBased`를 `nav2_smac_planner::SmacPlanner2D`로 바꾸면 되고 BT는 그대로다. package.xml에 `nav2_smac_planner`.
- 이 기계에는 `nav2_smoother`(SimpleSmoother, SavitzkyGolaySmoother), `nav2_constrained_smoother`, `nav2_smac_planner`가 있다 [확정].
  **실물 로봇에 깔렸는지는 모른다.** 로봇에서 `ros2 pkg list | grep -E "smac|smoother"`로 먼저 본다.
- 시뮬레이션의 `nav_overrides`는 Nav2 설정 전체에 깊이 덮이고 controller plugin 교체와 매핑을 비매핑으로 바꾸는 덮어쓰기를 거부하고, 병합 뒤 RPP controller에 충돌 검사를 다시 켜고, `nav_max_speed`(기본 0.20)를 주면 `desired_linear_vel` 상한을 건다. Gazebo 기본값(`fixed_line_check` 거짓)에서는 고정 경로 controller 셋의 검사를 끈다. 마당 goal(FollowPathGoal)은 `desired_linear_vel` 키가 있을 때만(plugin 이름이 아니라 키로 가린다) 이 둘을 받고, 회피 담당의 다른 plugin이면 건드리지 않는다(`launch_parts.py:206-215`, `sim/fleet_gz_sim/launch/gz.launch.py:103-116`,
  `sim/fleet_gz_sim/fleet_gz_sim/launch_parts.py:175-200`). 그래서 Gazebo에서 플래너 plugin(`planner_server`의 `GridBased.plugin`)은 `nav_overrides`로 바꿔 볼 수 있다.
  smoother는 launch 노드 목록과 BT를 바꿔야 해서 `nav_overrides`만으로는 안 된다.

### 볼 것 [추측 포함]

- smoother는 costmap 충돌 검사를 켜야 한다(통로 빈 칸이 6~7개). SimpleSmoother는 충돌이면 되돌린다.
- keepout 마스크의 막다른 주머니(`keepout_openings`, W 둘레)를 다듬은 경로가 지나가는지 본다.
- 1 Hz 재계획마다 다듬은 모양이 달라지면 RPP가 다시 흔들린다. 경로가 깜박이는지 RViz에서 본다.
- Smac 2D는 비용을 따라 칸 가운데로 가는 성질은 NavFn과 비슷하다. `tolerance`, `allow_unknown`의 뜻이 NavFn과 다르고,
  goal 칸이 막혔을 때의 동작은 확인해야 한다(Theta*가 여기서 약했다).
- 두 방법 모두 4절의 "가운데 통로가 4칸뿐"인 문제는 풀지 못한다. 그것은 costmap 설정의 문제다.

## 4. 큰 통로(가운데 통로)에서 가운데를 못 지나감

가운데 통로는 남쪽 진열대(y 0.365~0.405)와 북쪽 진열대(y 0.77~0.81) 사이, 실제 폭 0.365 m다. 평소에는 우회로로만 쓴다
(O_IN -> M_E -> Q2, W -> M_E).

### 계산 [확정]

- 5 cm 정적 지도(`sim/fleet_gz_sim/maps/arena_v7.pgm`, 원점 -0.15)는 진열대가 걸친 칸을 다 칠한다. 통로의 빈 칸은 y 0.45~0.75의 6칸(0.30 m)이다.
- 전역 발자국의 내접 반경 0.09 안은 비용 253(지나갈 수 없음)이다. 로봇 중심이 설 수 있는 칸은 y 0.50~0.70의 4칸이고,
  비용은 237, 176, 176, 237이다(x 0.80, 0.95, 1.40에서 같음).
- 장애물 판 `sim/fleet_gz_sim/obstacles/v7_yard_mix.yaml`의 상자 4(0.08 m, 370초에 놓을 예정이고 놓은 뒤 60초에 치움, 480초마다 되풀이. 로봇이 가까우면 늦어지거나 빠진다)는
  O_IN -> M_E의 0.4 지점 (0.952, 0.595), 곧 통로 한가운데에 놓인다. 상자가 전역 costmap에 찍히면 그 x에서 네 칸이 모두 253이 될 수 있어 가운데 통로로는 길이 없다(셀 경계와 상자 방향에 따라 조금 다를 수 있다).
- 실제 틈(상자 옆 0.15 / 0.135 m)은 로봇 폭 0.112 m보다 넓어 실물은 지나갈 수 있을 것이다 [추측]. 계획은 틈이 2 x 0.09 = 0.18 m보다 넓어야 통과시킨다.
- 다른 로봇도 같다. peer 층이 켜져 있으면(Gazebo 기본 켬 `sim/fleet_gz_sim/launch/gz.launch.py:76`, 실물 `nav2.launch.py` 기본 끔) peer 노드가 다른 로봇을 반경 0.08 원으로 찍으므로(`pode/pode_peer_obstacles/pode_peer_obstacles/node.py:53`)
  통로 가운데의 로봇 하나가 단면 전체를 막는다(연속 공간 근사, 셀 양자화에 따라 다를 수 있음). 마주 오는 두 로봇은 이 통로에서 비켜 갈 수 없다(남는 길 약 2 cm).
- M_E 동쪽 목(x 1.65~1.70)은 5 cm 한 칸 폭이다. M_E에 로봇이 서 있으면 통로 동쪽 출구가 닫힌다(peer 층이 켜져 있을 때, 근사).

### 실제 판과의 연결 [추측]

- 인계 4절과 `feat/nav-goal-smooth` 기록 5절: 장애물 판(nvbvar)에서 관제가 pode2를 O_IN으로 우회시켰지만 그 자리에서 움직이지 못하고
  `NO_START`가 세 번 났다(481~521초). 이것이 계획 실패 때문인지는 모른다. `NO_START`는 5초 동안 2 cm를 못 움직였다는 경고일 뿐이다.
  계획이 실패하면 BT는 재시도 뒤 복구(RoundRobin)로 costmap을 지우고 5초 기다린다(BT 25~33, 42~54행).
- 다른 로봇의 마지막 자세는 지우지 않는다(peer 노드 docstring, 의도된 설계). 보고가 끊긴 로봇이 통로에 있었다면 떠난 뒤에도 막혀 보일 수 있다.
- 헤드리스 세계도 계획할 때 다른 로봇을 0.08 + r + 0.03으로 키워(`sim/fake_robots/fake_robots/world/nav_goal.py:18,34`) 가운데 로봇 하나로 막힌다.
  다만 상자, 정적 지도 칠하기, inflation은 Nav2와 다르다.
- 사용자가 말한 "가운데"가 남쪽 진열대 사이의 틈(x 0.99~1.25)이라면 그것은 일부러 막은 것이다(`keepout_extra`, `fleet_map_v7.yaml:62`).

### 고치는 방안(바꾸지 않음, 사용자가 고를 것)

| 방안 | 효과 | 대가 |
|---|---|---|
| A. 전역 padding 0.03 -> 0~0.01, 전역 해상도 0.025 | 상자 옆이 계획상 열리고 두 대가 비킬 여지 | 경로가 진열대에 붙는다(Gazebo 2판에서 0.067 m까지 붙어 위치를 잃은 적). 계산 4배 |
| B. 지도를 걸친 칸 전부가 아니라 실제 경계로 칠함 | 통로 한 칸 넓어짐 | 상자나 로봇이 있으면 여전히 막힘. A와 함께여야 효과 |
| C. 관제가 가운데 통로를 한 대만 다니게(정원 1 또는 한 방향) | 마주침 없어짐 | 정원 1 구역은 첫 판에서 길을 잠근 이력(지도 머리말) |
| D. 우회한 뒤에도 계획이 실패하면(NO_START) 그 길도 막힘으로 표시해 다시 고름 | 로봇이 서 있지 않음 | 근본 해결 아님 |
| E. 상자 4를 통로 가장자리로 옮김 | 시험이 "통로 안에서 피해 가기"를 보여 줄 수 있음 | 시험 배치만 바뀜 |

## 5. 주차구역 앞 교착

### 기록

- "주차구역 앞 교착"이라는 글은 main과 모든 원격 브랜치의 문서, 커밋 메시지에 없다 [확정].
- 가장 가까운 기록 [확정]: `feat/nav-goal-smooth`의 `docs/worklog_nav_goal_smooth_2026-11-17.md` 5절.
  장애물 판(main 값 oinvar, 브랜치 값 nvbvar) **두 판 모두 420~460초쯤 칸 2, 3 앞 (0.49~0.56, 0.23~0.30)에서 로봇이 서서 막힘 판정을 받았다.**
  기록은 이를 "바꾸기 전에도 있는 약점(칸막이 끝과 상자, 주차한 로봇 사이)"이라고 적었다. 같은 기록 4절에는 값을 바꾼 판에서만 칸 입구에서 2~4초 서는 일이 있다.
- 사용자가 들은 교착이 이 장면이라면 6절의 BLOCKED 깜빡임 설명과 시각이 겹친다 [추측].

### 헤드리스 세계 [확정]

- 서브에이전트가 돌린 1,840판(v7 1,440판: 상황 10개 x 씨드 1~20 x order 켬/끔 x 회피 흉내 perfect/mixed/none, 주문 12건 몰아넣기 240판. 그 밖에 d 400판):
  v7에서 `DEADLOCK_SEEN` 0번, 8번은 모두 d. 결과 파일의 판 수와 교착 수는 다시 셌다(서브에이전트는 2,840판이라고 잘못 보고했다).
  칸에 선 로봇이 주차 상태가 아닌 채 입구를 2초 넘게 쥔 일도 0번. 스크립트와 결과는 이 세션의 scratch(`parkdead/`)에 있었다(지워질 수 있음).
- 앞서 본 d 배치 `express_obstacle`의 `DEADLOCK_SEEN`(order 켬)은 20씨드 중 8개에서 87~89초에 나고, 순환은 P-3과 P-2에서 base 끝 대기로 선 두 로봇이 서로의 주차 지점을 지나는 길을 원한 것이다.
  0.5초 뒤 `HORIZON_CHANGED`로 약 3초 만에 풀리고 다 끝난다. v7의 칸은 `bay_in`/`bay_out`으로만 이어진 막다른 곳이라 같은 순환이 생기지 않는다.

### 후보

1. **[추측] Nav2 단계의 대치, 관제는 못 봄.**
   - 칸 입구 E-k(y 0.24)는 북쪽을 보고 주차한 로봇의 몸 가운데(y 0.082)에서 0.158 m다. 전역 계획은 다른 로봇 둘레 0.08 + 0.09 = 0.17 m 안으로 가지 않으므로
     주차한 로봇 앞의 입구는 계획상 막힌 칸이다 [추측](연속 공간 근사. 실제는 peer 층, 점구름의 셀 양자화, goal tolerance에 달림. E는 `through`라 이어진 goal에서는 그 점을 목표로 삼지 않는다). O_OUT -> E-3 곧은 선은 E-2에서 0.161 m를 지난다 [확정, 계산].
   - 그래서 칸 입구 줄에서 지나가는 로봇, 칸으로 후진해 들어가려는 로봇, 칸에서 나오는 로봇, 칸막이 끝 keepout이 겹치면 서로 길을 막는다.
   - 관제의 가까이 붙음 감시 `_watch_near`는 두 로봇이 모두 Nav2 goal로 움직일 때만 본다(`pc/fleet_core/fleet_core/traffic/zone/watch.py:86-95`) [확정].
     후진해 들어가거나 고정 경로로 나오는 로봇, 주차해 goal이 없는 로봇과의 대치는 빠진다. 교착 감시는 구역/지점 허락의 대기 그래프만 본다(같은 파일 115~124행) [확정].
   - 이 두 감시로는 잡히지 않는다. 남는 처리는 막힘 처리(BLOCKED 10초 -> 양보, 막힘 표시, 뒤가 막혔으면 기다림), 멈춤 감시(`_watch_stall`), 로봇 쪽 복구다.
     6절의 깜빡임은 이 가운데 막힘 처리를 늦추거나 막는다.
2. **[확정, 주차 앞은 아님] v7의 줄 입구는 Q2 하나다.** Q2에서 격리되거나 고장 난 로봇이 있으면 나머지는 칸에 선 채 끝까지 기다린다.
   순환이 아니라 `DEADLOCK_SEEN`은 안 나오지만 보는 사람에게는 "주차구역에서 안 움직임"으로 보일 수 있다.
   헤드리스: `front_robot_stuck` v7 0/6, `pose_jumps` v7 order 끔 0/6(둘 다 이미 알려진 것).
3. **[추측] order 켬에서 비킬 로봇이 없는 순환.** 교착 비키기 `_to_parking`은 goal이 열린 로봇이면 하지 않는다(`watch.py:143`) [확정].
   order 켬에서 base 끝에서 기다리는 로봇은 goal이 열린 채 대기 그래프에 들어가므로, 순환 속 로봇이 모두 그러면 horizon 다시 짜기에만 기댄다. v7에서는 재현 안 됨.

### 고치는 방향(바꾸지 않음)

- 먼저 6절의 BLOCKED 깜빡임을 고친다(브랜치 둘 푸시, main 아님). 그러면 깜빡여도 10초 뒤 막힘 처리(양보 또는 막힘 표시)가 시작될 수 있다.
  DOCK, BACKOFF는 빠지고, 후진(RETREAT)과 뒤가 막힌 경우는 다른 시간 제한을 탄다.
- 1번을 줄이는 방안: `_watch_near`가 고정 경로/후진 로봇과 서 있는 로봇도 보게 함(빨리 풀리지만 칸에 들어가는 로봇을 멈추게 할 위험),
  칸 입구 줄을 지나는 Nav2 goal에 옆 입구 지점을 쥐게 함(확실하지만 처리량 감소), 입구를 y 0.26 이상으로(0.17 m 간격, 대신 칸 줄이 길어짐: 사용자가 전에 줄인 값).
- 2번: 두 번째 줄 입구나 Q2 격리 경보. 배치 결정이라 사용자가 정한다.
- 3번: order 켬에서 순환 속 로봇의 order를 끊고 비키게(`_end_both` 활용). 정지가 한 번 늘어난다.

## 6. BLOCKED 깜빡임과 관제가 우회하지 않는 문제

사용자가 붙여 준 분석(다른 세션)을 코드로 확인했다.

- 관제: `pc/fleet_core/fleet_core/traffic/zone/blocked.py` `_watch_blocked`(25~34행)는 `robot_state == BLOCKED`(또는 RECOVERING 45초 넘게)일 때만 막힌 것으로 보고,
  아닌 보고가 한 번만 와도(RECOVERING이 45초를 넘지 않았으면) `blocked_since`를 비운다 [확정]. 막힘 표시는 막힌 상태(BLOCKED, 또는 45초 넘은 RECOVERING)가 `blocked_mark_s`(10초) 이어져야 한다 [확정].
- 로봇: RECOVERING이 BLOCKED보다 먼저 보고된다(`pode/pode_fsm/pode_fsm/core.py:103-106`) [확정]. Nav2 goal의 진전 기준은 `nav_progress_m` 0.05(`params.py:52`) [확정].
  로봇 쪽 수정 브랜치에서 더 확인한 것: 상태 보고에서 RECOVERING은 BLOCKED보다 앞서므로 둘이 한 보고에 같이 나오지 않는다. 그래서 깜빡임은 BLOCKED와 MOVING(또는 RECOVERING) 사이로 보인다. 어느 쪽이 많은지는 재지 않았다 [추측].
  Nav2 goal에서는 5 cm 움직이면 BLOCKED가 풀리고 다시 BLOCKED가 되기까지 `blocked_after_s` 10초가 걸려, 막힘 아닌 틈이 10초 가까이 될 수 있다 [확정, 코드].
- 그래서 좁은 곳에서 막힌 상태와 아닌 상태가 10초보다 짧게 번갈아 나오면 관제 타이머가 계속 0으로 돌아가 막힘 표시와 우회가 일어나지 않는다 [확정, 코드]. 실제로 그렇게 번갈아 나왔는지는 재지 않았다.
  Gazebo의 420~460초 장면이 이것인지는 [추측]. 9절의 기록으로 가른다.

### 고친 브랜치(푸시, main 아님. main에 넣을지는 사용자가 정한다)

- `fix/blocked-hysteresis-fleet`(62bfb98, 7e0a771, 1f758ac). 관제는 같은 goal의 막힘 일화 안에서는 짧은 틈에 시계를 다시 세지 않는다. 다만 막힘 표시를 하거나 다른 로봇에게 양보한 뒤에는 원래처럼 시계를 비우고 새 일화로 시작한다(`blocked.py`의 `_mark_blocked`, `yard_retreat.py`의 `_yield_to_robot`).
  - 일화가 끝나는 때: 처음 막힌 자리에서 `blocked_reset_m` 0.30 m 넘게 나아감, 막힘 아닌 보고가 `blocked_reset_s` 20초 이어짐(RECOVERING 보고가 오면 이 20초를 처음부터 다시 센다. 다만 그 보고가 올 때 이미 20초가 찼으면 일화가 끝난다: `blocked.py`의 `_blocked_gap`이 `_gap_over`를 먼저 본다), goal이 바뀌거나 끝남.
  - 20초는 위의 "10초 가까운 틈"의 두 배, 0.30 m는 Pode 자가 후진 최대 0.15 m의 두 배다. 두 값 모두 [추측]이고 실제 깜빡임 주기는 재지 않았다.
  - 짧은 틈이 한 번 이상 있었으면 `ZONE_MARKED_BLOCKED`(두 곳)와 막힌 채 시간이 다 된 `GOAL_FAILED`(뒤에 로봇이 있을 때, 물러나는 중) 끝에 `, after N short gaps`가 붙는다. 0이면 예전 문구 그대로이고, 다른 `GOAL_FAILED`에는 붙지 않는다.
  - 시험: 고치기 전 코드로 당시 시험 9개를 돌려 8개가 실패했다(최종 시험은 11개). 고친 뒤 fleet_core `ZONE_TICK_ORDER=manager`와 `zone` 모두 1115 passed, 1 skipped, 5 xfailed(직접 다시 돌림).
    fake_robots 1019 passed, 27 xfailed와 관제 시나리오 11 passed는 에이전트가 돌린 결과다.
  - 남은 점: Nav2가 BLOCKED 없이 0.3 m 안에서 20초 넘게 꿈틀거리면 막힘 일화가 끝난다. 그 꿈틀거림이 12초(`stall_s`) 안에 매번 5 cm(`stall_move_m`) 넘게 움직이면 멈춤 감시(`watch.py`)도 시계를 다시 세므로 잡지 못한다.
- `fix/blocked-hysteresis-pode`(d1d9ac8, 4f87eab, e935998, 3724af2, 1ab81f7). 로봇은 판단에 쓰는 막힘 플래그를 main 그대로 두고, 상태 보고만 늦게 끈다.
  - 그래서 자가 복구, 교착 처리 시작, aborted 처리는 main과 같다. 첫 커밋 d1d9ac8은 판단 플래그까지 바꿔 Codex가 확정 결함 4건을 짚었고, 4f87eab에서 보고만 늦추는 방식으로 바꿨다.
  - 고정 경로(FollowPath): 막힌 자리에서 `unblock_m` 0.15 m 벗어나거나 물체가 `unblock_clear_s` 3초 계속 안 보이면 보고를 끈다.
  - Nav2 goal: 막힘이 풀린 뒤 막힌 자리에서 0.15 m 벗어나거나, goal을 다시 보내거나, 교착 처리를 시작하거나, 목표가 끝나면 끈다.
  - 가짜 로봇(nav_goal 둘)도 같은 0.15 m 보고 유지를 넣었다. follow_path 가짜는 쓰지 않는 모드라 그대로 두었다.
  - 시험: pode_fsm 366 passed, 8 skipped(직접 다시 돌림). ROS 포함 467 passed, 6 skipped와 fake_robots 1027 passed, 27 xfailed는 에이전트가 돌린 결과다.
- 두 브랜치를 합친 상태의 시험(로컬 브랜치 `tmp/blocked-combined` = main + 두 브랜치, 6개 패키지 따로 빌드, 직접 돌림):
  관제 시나리오 `test_scenario_integration.py`(도메인 90~98) 11 passed, `sim/fake_robots/test`(도메인 78) 1022 passed, 27 xfailed.
  가짜 로봇 수가 로봇 쪽 브랜치의 1027보다 5개 적은 것은 그쪽이 `sim/fleet_gz_sim/test/test_timing_agreement.py`(시험 5개)를 같이 돌렸기 때문이다.
  `pc/fleet_manager/test`의 나머지(도메인 50~59를 스스로 고름)는 돌리지 않았다. Gazebo와 실물 로봇에서 깜빡임 주기와 우회 효과는 아직 재지 않았다.
- 각 브랜치의 기록: `docs/worklog_blocked_hysteresis_fleet_2026-11-17.md`, `docs/worklog_blocked_hysteresis_pode_2026-11-17.md`.

## 7. 그 밖의 열린 문제(기존 기록에서 모음)

- 인계 8절: O_OUT 1 cm 효과 미확인, `standby_lookahead_s` 0.5초 추정값, O_IN은 Gazebo에서 지나간 적 없음, `feat/nav-goal-smooth`를 넣을지(주차 칸 입구에서 서는 부작용).
- 인계 7절: fleet_manager 통합 시험의 `stock_shortage` 간헐 실패(`J_W released ... 0.08 m`), 물러나기 중 이른 놓기 계열.
- order 병합 기록 6절: 물러나기를 끝낸 뒤 늦은 보고로 이른 놓기(켬·끔 strict xfail, 정책 결정 필요), FOLLOW_PATH O -> O_OUT의 같은 모양,
  가짜 Pode 이어진 조각, d `express_obstacle`의 `DEADLOCK_SEEN`(5절에서 원인 확인: 3초 만에 풀림), pose_jumps I3(B7, B 보관 중).
- 주행 계획서 8절: 튄 위치로 구역을 놓음(B7), 도크 교대 여유/구조, 시작 조건, 물러나는 거리(E-3), O_OUT에서 서는 것, 관제 화면 표시들.

## 8. 시험(새 main 956140d, 공유 install 다시 빌드)

- 공유 install 다시 빌드: 15개 패키지 성공.
- fleet_core: 1097 passed, 5 xfailed(인계 7절과 같음).
- fake_robots: 1019 passed, 27 xfailed(인계 7절과 같음).

## 9. 집 컴퓨터 기록에서 볼 것(가르는 측정)

- oinvar, nvbvar의 420~460초: 칸 2, 3 앞에 선 로봇의 `BLOCKED_WAIT` 횟수(같은 goal에서 여러 번이면 깜빡임 후보가 강해진다).
  같은 goal의 상태 보고 순서와 실제 이동량, 막힘 표시와 취소 사건의 시각을 시간순으로 맞추고, 주변 로봇의 상태(주차, 후진, 고정 경로)도 본다.
- nvbvar 481~521초 `NO_START` 시각의 `planner_server` 로그("Failed to create plan" 여부)와 `global_costmap/costmap`의 그 로봇 둘레.
- 장애물 판에서 상자 4가 실제로 놓인 동안: `global_costmap/costmap_raw`(nav2_msgs/Costmap, 253/254)나 `global_costmap/costmap`(OccupancyGrid, 99/100으로 바뀐 값)의
  x 0.95 단면이 y 0.50~0.70에서 모두 지나갈 수 없는 값인지. 셀 좌표와 시각을 함께 적는다.
- 한 대를 (1.10, 0.59)에 세우고 다른 로봇에 O_IN -> M_E goal을 줘서 `/plan`이 남쪽으로 돌거나 실패하는지(4절 확인용 Gazebo 판).
