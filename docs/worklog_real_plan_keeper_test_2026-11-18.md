# 실물 두 대 시험용: main + goal 길 유지(plan_keep) (2026-11-18)

사용자 요청: 다음 실물 두 대 시험(pode1 = pinky_27a6, pode2 = pinky_07d7)에서 `origin/feat/goal-plan-keeper`(771c971)도
돌린다. 로봇이 꺼져 있어 준비만 했다. ssh, 배포, Gazebo는 하지 않았다.

브랜치: `test/real-plan-keeper-2026-11-18` (개인 브랜치, main에 넣지 않음).

표기: [확정] = 코드나 명령 출력으로 확인함. [추측] = 확인하지 못한 판단.

## 1. 병합

| 커밋 | 내용 |
|---|---|
| (출발) `origin/main` 1f1089d | 가져온 시점의 origin/main. a14b2e5가 아직 없었다 [확정] |
| 914c625 | `origin/feat/real-fixed-line-check-off`(245c244) 병합. 트리가 로컬 main a14b2e5와 같다(`git diff a14b2e5 914c625` 비어 있음) [확정] |
| 9cfdc5f | `origin/feat/goal-plan-keeper`(771c971) `--no-ff` 병합 |
| 9333407 | `start.sh`에 `PLAN_KEEP=0` 스위치 추가(시험 먼저 작성) |
| (이 문서) | 기록 |

충돌은 하나였다 [확정].

- `tools/real_pode/README.md` "Gazebo와 다른 실물 기본값": main은 DOCK_STUB 줄과 "도킹 없음(DOCK_STUB 없이)" 줄을, keeper는
  plan_keep 줄과 옛 "도킹 없음" 줄을 같은 자리에 넣었다. plan_keep 줄 + main의 DOCK_STUB 두 줄을 남겼다. 고정 경로 충돌 검사
  꺼짐 줄(main)은 충돌 밖이라 그대로다.
- `tools/test/test_real_pode.py`, `tools/real_pode/deploy.sh`, `real_pode.launch.py`는 자동 병합됐다. 결과를 읽어 보았다:
  `deploy.sh` 목록에 `pode/pode_plan_keep`이 들어갔고, launch는 `plan_keep`을 nav2에 넘기며, 시험의 패키지 목록과 nav2 인자
  목록이 둘 다 맞춰졌다. FIXED_LINE_CHECK, DOCK_STUB 처리는 그대로다 [확정].
- 고정 경로 충돌 검사는 기본 꺼짐 그대로다(`make_params.py`, `start.sh:22-28`) [확정].

## 2. 바꾼 것: `PLAN_KEEP` (9333407)

병합만으로는 `start.sh`에서 plan_keep을 끌 방법이 없었다(`plan_keep:=false`는 `ros2 launch`를 손으로 칠 때만). 실물에서
문제가 나면 바로 끌 수 있게 더했다.

- `PLAN_KEEP=0 ./start.sh ...` → launch에 `plan_keep:=false`. 안 주거나 `1`이면 인자를 붙이지 않는다(launch 기본 true).
  다른 값은 `make_params.py` 전에 종료 2 (`tools/real_pode/start.sh:30-36, 53`) [확정].
- 꺼도 노드는 떠 있고 행동 트리는 여전히 `compute_path_keep`을 부른다. 노드가 planner 결과를 그대로 넘길 뿐이다
  (`pode/pode_plan_keep/pode_plan_keep/node.py:131-137`). 노드를 계획 경로에서 아예 빼는 스위치는 없다 [확정].

## 3. 시험 (이 PC, `ROS_DOMAIN_ID=69 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, Gazebo 없음)

빌드는 worktree 안 `_build_t/`, `_install_t/`에만 했다(`colcon build --packages-up-to pode_plan_keep pode_nav_config fleet_gz_sim`,
11개 패키지). `nav2fleet_ws/build`, `install`은 건드리지 않았다.

병합 커밋 9cfdc5f에서 [확정]:

```
tools/test/test_real_pode.py                88 passed in 119.50s (0:01:59)
pode/pode_plan_keep/test                    49 passed in 2.51s
pode/pode_nav_config/test                   112 passed in 8.84s
sim/fleet_gz_sim/test/test_dock_stub.py test_launch_wiring.py test_one_pode_wiring.py
  test_sim_nav_params.py test_stand_in_battery.py      151 passed in 17.00s
```

9333407에서: 새 시험 9개는 start.sh 수정 전 6 failed, 3 passed(기본값 시험 셋은 원래 통과) → 수정 후
`tools/test/test_real_pode.py` `97 passed in 146.97s (0:02:26)` [확정].

fleet_gz_sim 시험은 이 브랜치가 고친 파일 셋과 `stand_in_node`/`launch_parts`/`plan_keep`을 쓰는 두 파일만 돌렸다. 전체 594개는
돌리지 않았다.

## 4. 실물에서 달라지는 것

### 4.1 기본으로 켜지나, 어떻게 끄나

- 켜진다. `real_pode.launch.py:110`의 `plan_keep` 기본 true → nav2에 넘김(`:52`) → `nav2.launch.py:186-191`이 로봇 namespace에
  `plan_keep` 노드를 띄운다(`plan_keep_node` 기본 true, `:252`). 실물 launch는 `plan_keep_node`를 넘기지 않는다 [확정].
- 끄기: `PLAN_KEEP=0 ./start.sh ...` 또는 `ros2 launch ./real_pode.launch.py ... plan_keep:=false`. 2절대로 노드는 남는다 [확정].

### 4.2 로봇에 무엇을 복사하고 빌드하나

- `deploy.sh`는 `pode/pode_plan_keep`을 복사한다(`tools/real_pode/deploy.sh:9-11`) [확정].
- 로봇에서 빌드해야 할 것: 새 패키지 `pode_plan_keep`(ament_python, 의존 rclpy, action_msgs, nav2_msgs, nav_msgs: 로봇의 Jazzy에
  이미 있을 것 [추측])과 `pode_nav_config`(BT xml과 nav2.launch.py가 바뀌었다). `tools/real_pode` 묶음은 빌드 없이
  `~/nav2fleet_real/`로 간다(start.sh, README) [확정].
- 가장 쉬운 것은 README대로 `cd ~/nav2fleet_ws && colcon build` 전체 빌드다. 일부만 빌드하면
  `--packages-select pode_plan_keep pode_nav_config`를 빼먹지 않는다. `pode_nav_config`만 새로 빌드하고 `pode_plan_keep`이 없으면
  nav2 launch가 패키지를 찾지 못해 시작하지 못할 것이다 [추측: launch_ros가 실행 때 패키지를 찾는다].

### 4.3 어느 controller에 영향이 있나

- Nav2 goal(NavigateToPose)의 계획만이다. goal BT의 `ComputePathToPose`가 `server_name="compute_path_keep"`으로 이 노드를 부른다
  (`pode/pode_nav_config/bt/navigate_to_pose_w_replanning_and_recovery_goal.xml:32`). 그 경로를 따르는 controller는 이 BT의
  기본 `FollowPathGoal`이다 [확정].
- 관제의 고정 경로는 `pode_fsm`이 `FollowPath` 액션으로 controller_server에 바로 보낸다(`pode/pode_fsm/pode_fsm/follow_ros.py:58`).
  planner와 BT를 거치지 않으므로 FollowPath, FollowPathThrough, FollowPathReverse에는 영향이 없다 [확정].

### 4.4 Pi의 CPU, 스레드

- 프로세스 하나(rclpy) 더. Nav2는 실물에서 `use_composition: true`라 한 컨테이너지만 plan_keep은 따로 돈다. DDS 참여자도
  하나 는다 [확정: `real_pode.launch.py:56`, `nav2.launch.py:186`].
- 스레드: `MultiThreadedExecutor(num_threads=4)` (`node.py:45-46, 235`) [확정].
- 요청 처리 중 planner 결과를 5 ms 간격으로 기다린다(`node.py:42, 146-151`). 계획은 1 Hz이고 동시에 둘까지라, 계획이 걸리는
  동안만 초당 약 200번 깨어난다 [확정: 코드], CPU 부담은 작다 [추측].
- 전역 비용 지도 구독(`node.py:71`): 1 Hz 전체 지도(`always_send_full_costmap: true`, `nav2_params.yaml:356`, `publish_frequency: 1.0`).
  실물 지도 `arena_v7_a`는 92×52칸이라 메시지는 작다 [확정: 지도 크기]. 다만 Nav2 costmap 발행기는 구독자가 있을 때만 전체 지도를
  만들어 보낼 수 있으니, PC에 RViz가 안 붙은 때에는 이 노드 때문에 global costmap 발행이 새로 생길 수 있다 [추측].
- planner 호출 수는 늘지 않는다. BT가 1 Hz로 부르는 것이 그대로 planner로 간다 [확정: BT RateController 1.0 그대로].
- LOCALIZING 깜빡임, controller 4~12 Hz 문제와의 관계: 고정 경로 주행 중에는 BT가 돌지 않아 계획 요청이 없다. 그래도 노드는
  떠서 비용 지도를 받는다. 마당 goal 중에는 계획 1 Hz마다 Python 일이 조금 생긴다. 이것이 Pi의 CPU를 눈에 띄게 늘릴지는 재지
  않았다 [추측: 작다]. CPU가 모자라면 rclpy의 goal 승인이 늦어질 수 있는데, BT에 `server_timeout="1000"`(1 s)이 있다. 1 s를 넘기면
  BT가 계획 실패로 보고 복구(지도 지우기, Wait)로 간다 [확정: xml:32. 실물에서 1 s를 넘길지는 추측].

### 4.5 원인을 모르는 멈춤

- 무엇: `pode_plan_keep` 시험의 첫 실행(754fd25, PC 부하가 높고 Gazebo가 돌던 때)이 끝나지 않았고 `timeout`의 TERM에도 끝나지
  않아 KILL로 끝냈다. 어느 시험인지 출력이 없다. 바로 뒤 6번과 이후 여러 번은 2~3 s에 통과했다. Gazebo 열 판에서는 계획이
  멈춘 일이 없었다(`docs/worklog_goal_plan_keeper_2026-11-18.md` 5절) [확정: 문서 내용]. 이번 시험도 49 passed in 2.51s였다.
- 실물에서의 위험 [추측]:
  - 노드가 요청 하나를 끝내지 못해도 BT가 goal을 취소하면 `_wait`가 취소를 보고 돌아온다(`node.py:146-151`). 행동 트리가 계속
    기다리면 그 goal의 계획이 멈추고, 동시 요청 한도(2, `node.py:45, 90-96`)가 차면 다음 계획은 거절된다 → goal 실패 → 관제의
    재시도/격리로 이어진다.
  - planner가 응답하지 않으면 노드도 기다린다. 이것은 planner를 바로 부를 때와 같다(keeper 문서 5절).
  - 노드가 죽으면 `respawn=True`(`nav2.launch.py:188`)로 다시 뜬다. 그 사이 계획은 실패한다.
  - 시험 프로세스가 TERM을 무시한 것과 같은 일이 로봇 노드에서 나면 `stop.sh`는 20 s 뒤 "still alive"로 끝나고 KILL하지 않는다
    (`tools/real_pode/stop.sh:9-13`). 그때는 손으로 내려야 한다.
  - 어느 경우든 고정 경로 주행에는 걸리지 않는다(4.3).

### 4.6 main에 넣지 않은 이유가 아직 맞나

keeper 문서 6절의 이유 셋:

1. 실물에서 돌려 보지 않았다. CPU가 모자라면 계획이 늦거나 실패할 수 있다 → **아직 맞다.** 지난 실물 판에서 controller가
   20 Hz를 못 지켰으니(4~12 Hz) 오히려 더 무겁다 [확정: 실물에서 안 돌림 / 영향 크기는 추측].
2. 로봇 배포가 달라진다(새 패키지) → **맞다.** 이 브랜치로 배포하면 해결되는 준비 문제다.
3. 시험이 한 번 멈춘 원인을 모른다 → **아직 맞다.** 이번에 새로 찾은 것은 없다.

그래서 이 브랜치는 시험용으로만 쓰고, 실물 결과를 본 뒤 main을 정하는 것이 맞다 [추측: 권고].

## 5. `origin/fix/goal-replan-on-invalid`(b33e426)와의 관계

- 같은 문제(길이가 비슷한 두 길 사이 "도리도리")를 BT만으로 푸는 **다른 방법**이다. keeper 문서 2절에서 버린 방법으로
  기록했다(장애물 판에서 뒤집힘 9번, IsPathValid가 자주 실패해 거의 매초 다시 계획) [확정: 문서].
- 겹침: 둘 다 goal BT의 같은 `ComputePathToPose` 줄과 머리 설명을 바꾼다. 같이 병합하면 텍스트 충돌이 나고, 기능도 둘 다
  계획 바꾸기를 막으려는 것이라 둘을 함께 둘 이유가 없다 [확정: `git diff origin/main...origin/fix/goal-replan-on-invalid`].
- 이 브랜치에는 넣지 않았다.

## 6. 두 로봇 배포 명령 (실행하지 않았다)

PC에서 이 브랜치를 꺼낸 깨끗한 checkout에서(`deploy.sh`는 커밋된 HEAD만 보낸다):

```bash
git fetch origin && git switch test/real-plan-keeper-2026-11-18
tools/real_pode/deploy.sh pinky@192.168.0.200   # pode1 = pinky_27a6
tools/real_pode/deploy.sh pinky@192.168.0.201   # pode2 = pinky_07d7
```

각 로봇에서:

```bash
cd ~/nav2fleet_real && ./stop.sh
cd ~/nav2fleet_ws && colcon build        # 또는 --packages-select pode_plan_keep pode_nav_config (둘 다)
cd ~/nav2fleet_real
PEER_ROBOTS=pode1,pode2 DOCK_STUB=1 ./start.sh pode1 <x> <y> <yaw>   # pode2는 pode2와 그 자리
# 길 유지를 끄고 비교: PLAN_KEEP=0 PEER_ROBOTS=pode1,pode2 DOCK_STUB=1 ./start.sh ...
```

확인할 것:

```bash
ros2 action list | grep compute_path_keep          # /pode1/compute_path_keep, /pode2/compute_path_keep
grep -E "compute_path_keep -> compute_path_to_pose|kept the route|changed the route|acknowledge goal request" \
  ~/nav2fleet_real/logs/launch_*.log
top -H -p "$(pgrep -f 'plan_keep' | head -1)"      # 노드 CPU
```

주소는 메모(11-16, 11-18)대로 적었다. 시험 전에 공유기에서 다시 확인한다 [추측].

## 7. 열린 위험

- [추측] Pi CPU: plan_keep의 부하를 실물에서 재지 않았다. LOCALIZING 깜빡임과 control rate 경고가 늘면 먼저 `PLAN_KEEP=0`으로
  비교한다. 그래도 노드는 남으므로 완전한 비교는 main(plan_keep 없음)으로 다시 배포해야 한다.
- [추측] goal 승인 1 s 초과 시 계획 실패 → 복구. 로그의 "acknowledge goal request for compute_path_keep"을 본다.
- [확정] 원인 모를 시험 멈춤 1회가 풀리지 않았다.
- [추측] 부분 빌드 때 `pode_plan_keep`을 빼면 nav2 launch가 뜨지 않는다.
- [확정] keeper의 Gazebo 결과는 실물 지도(`arena_v7_a`, 92×52칸)가 아닌 시뮬 지도에서 낸 것이다. 실물 효과(뒤집힘 감소)는 아직 모른다.
