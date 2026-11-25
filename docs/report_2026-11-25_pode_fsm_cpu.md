# pode_fsm CPU 원인 찾기 (2026-11-25)

브랜치 `perf/pode-fsm-cpu-2026-11-25` (origin/main `c29de003`에서 땄다). 로봇에는 접속하지 않았고 배포도 하지 않았다.

## 요약

- 11-24 실물 Pi4에서 pode_fsm이 쓴 CPU는 대부분 우리 콜백이 아니라, rclpy 실행기가 깨어날 때마다 wait set을 다시 만드는 비용이었다.
- 주 노드에는 실체가 많다. 구독 8, 타이머 3, 서비스 11, 클라이언트 1, waitable 30(QoS 이벤트 처리기 24, 액션 클라이언트 3, 액션 서버 3)이다. 깨어날 때마다 이 실체를 모두 wait set에 다시 넣는데, 그 한 바퀴가 PC에서 418 µs 걸린다(`spin_once(timeout 0)` 2000번 평균).
- 이 노드를 /tf(66 msg/s)와 odom(30 Hz)이 초당 약 96번 깨웠다.
- 고친 것: TF와 odom을 작은 입력 노드(`<노드 이름>_inputs`)가 받게 하고, 그 노드는 전용 `SingleThreadedExecutor` 스레드에서 돌린다. 주 노드는 타이머(20·5·2 Hz)와 드문 입력에만 깨어난다.
- PC에서 같은 조건으로 세 갈래를 동시에 세 바퀴 쟀다. 옛 키트(`b9e86c5`)와 main(`c29de003`)은 둘 다 평균 17.8 %, 고친 것은 7.5 %다(58 % 줄었다).
- set_mode에서 늘어난 관리 상태(`pode_fsm.admin`)와 상태 보고 칸은 CPU 차이를 만들지 않았다. 옛 키트와 main의 값이 같다(main은 상태 보고도 10 Hz에서 5 Hz로 줄었다).
- Pi4에서 같은 비율로 줄면 62 %가 약 26 %가 된다는 계산이 나오지만, 이것은 추정이다. 다음 실물 측정에서 잰다.

## 배경: 11-24 실물 값

- `results/2026-11-24/probe/top_amcl_base_oin_start.txt`(브랜치 `data/real-2026-11-24`): pode_fsm 62.5 %, Nav2 컨테이너 106.2 %, load 8.01. 끝 파일에서는 53.3 %다.
- pode_fsm 누적 CPU 시간(TIME+)은 704 s이고, 키트를 띄운 21:24:39부터 21:43:41까지는 1142 s다. 그래서 한 순간 값이 아니라 그 시간 내내 평균 약 62 %를 썼다.
- 그날 로봇은 대부분 서 있었고, goal은 pode_fsm을 거치지 않고 Nav2 `navigate_to_pose`에 바로 보냈다. 그러니 62 %는 pode_fsm이 한가할 때의 값이다.
- 키트는 nav2fleet `b9e86c5`(태그 `rpp-baseline-2026-11-21`)였다.

## 실물 입력 (bag `1124_amcl_base_pode2`, 1336.8 s)

| 토픽 | 주기 | pode_fsm이 받는가 |
|---|---|---|
| `/pode2/tf` | 65.7 msg/s: odom→base_footprint 29.9 Hz, map→odom 10.0 Hz, 바퀴·캐스터 변환 4개 묶음 11.1 Hz, 변환 없는 빈 메시지 14.9 Hz | 받는다 |
| `/pode2/odom` | 29.8 Hz | 받는다(속도만 쓴다) |
| `/pode2/scan` | 10.0 Hz | 받지 않는다 |
| `/pode2/amcl_pose` | 214개(이따금) | 받지 않는다. AMCL은 map→odom TF로 들어온다 |
| `/pode2/state` | 9.9 Hz | 내보낸다(옛 키트 `state_rate_hz` 10) |

`tf_rates.py`로 셌다.

## 재현 방법

모든 스크립트는 `docs/pode_fsm_cpu_2026-11-25/`에 있다.

- `replay_inputs.py`: bag의 60~150초 구간에서 odom과 tf를 꺼내, 시각을 지금으로 바꿔 기록된 간격대로 되풀이해 발행한다. tf_static은 한 번 낸다.
  - 함께 띄우는 것: 가짜 Nav2 액션 서버 셋(follow_path, backup, navigate_to_pose, goal은 오지 않는다), clear costmap 서비스, local costmap(2 Hz, 120×120, 0.025 m).
  - local costmap 값은 `nav2_params.yaml`의 local_costmap(3 m × 3 m, 0.025 m, publish 2 Hz)과 같게 했다.
- `measure.sh`: 입력을 띄운 뒤 pode_fsm을 `/pode2`로 띄운다(`through_controller_id` FollowPathThrough, heartbeat 1.5 s).
  - 15초 워밍업 뒤 60초 동안 `/proc/<pid>/stat`의 utime + stime 차이로 CPU를 잰다.
  - 끝에서 상태가 IDLE인지 본다. 모든 판이 IDLE이어서, 실물처럼 TF 자세가 있는 상태로 돌았다.
- 갈래마다 따로 빌드했다(fleet_interfaces + pode_fsm). 세 갈래(옛 키트, main, 고친 것)는 도메인 44, 45, 47에서 동시에 돌려 같은 부하를 받게 했다.
  - PC는 다른 세션 때문에 load가 9~13(12코어)이었다. 같은 순간에 잰 값끼리만 비교한다.
- 프로파일: py-spy 0.4.4로 `record -r 200 -f raw`를 썼다. `measure.sh`에 측정초 45를 주어 pode_fsm 시작부터 60초(앞 15초 포함)를 기록했다. py-spy는 ptrace 권한 때문에 pode_fsm을 자식으로 띄운다.

## 프로파일 결과

`spy_sum.py`는 py-spy가 일하는 것으로 잘못 센 C 안의 대기 표본(wait set 대기 `executors.py:781`, 스레드 풀 큐 대기 `thread.py:89`)과 시작 때 import 표본을 빼고 묶는다. 결과는 `spy_old_summary.txt`, `spy_main_summary.txt`, `spy_fix_summary.txt`에 있다.

| 갈래 | 일한 표본 | 주 실행기: wait set 만들기와 고르기 | 콜백 | 입력 노드 스레드 | 기타 |
|---|---|---|---|---|---|
| 옛 키트 `b9e86c5` | 1706 | 84.3 % | 7.5 % | 없음 | 8.2 % |
| main `c29de003` | 1856 | 86.9 % | 5.5 % | 없음 | 7.5 % |
| 고친 것 | 352 | 28.7 % | 5.7 % | 51.7 % | 13.9 % |

- "wait set 만들기와 고르기"는 `Executor._wait_for_ready_callbacks`가 하는 일이다.
  - 깨어날 때마다 실체를 하나씩 wait set에 넣는다. 여기에 QoS 이벤트 처리기 24개의 `add_to_wait_set`과 `ExitStack` 들어가기·나오기가 포함된다.
  - 깨어난 뒤에는 준비된 것을 하나씩 고른다.
- 옛 키트와 main에서 실제 콜백(TF 넣기, odom, 제어 tick, 상태 발행)은 5.5~7.5 %뿐이다. 고친 것의 콜백 칸에는 TF·odom 콜백이 들어가지 않는다. 그 콜백은 입력 노드 스레드에서 돌아서 입력 노드 스레드 칸에 들어간다.
- py-spy는 파이썬 스레드만 본다. DDS 수신 스레드처럼 C로만 도는 스레드의 CPU는 표본에 없다. 그래서 일한 표본은 81 % 줄었지만, `/proc`로 잰 CPU는 58 % 줄었다.

## 고친 것

- `pode/pode_fsm/pode_fsm/node.py`: `FrequentInputs` 클래스.
  - 주 노드의 TransformListener와 odom 구독을 작은 입력 노드로 옮겼다. 입력 노드는 파라미터 서비스와 rosout이 없다.
  - 입력 노드는 자기 `SingleThreadedExecutor`를 데몬 스레드에서 돌리고, `destroy_node`에서 함께 정리한다.
  - 토픽 이름은 주 노드가 푼 이름(`resolve_topic_name`: 이름공간과 다시 잇기 포함)을 그대로 쓴다. 확인한 경우는 셋이다. 시험처럼 이름공간이 없으면 `/tf`, `/tf_static`, `/odom`이다. 실물 launch처럼 이름공간과 `/tf:=tf`를 주면 `/pode9/tf`처럼 이름공간 안의 토픽이다(시험). PC 재현에서는 `/pode2`로 띄워 TF 자세를 받아 IDLE이 됐다. Gazebo(`use_sim_time`)에서는 돌려 보지 않았다.
  - 입력 노드는 `__init__` 맨 끝에 띄운다. 앞에서 초기화가 실패하면 스레드가 남지 않는다.
  - TF 버퍼는 전과 같이 하나를 함께 쓴다. tf2 버퍼는 안에서 잠그기 때문에 다른 스레드에서 넣고 읽어도 된다.
- ROS 그래프에 노드가 하나 는다: `/<ns>/pode_fsm_inputs`.
- 시험(`pode/pode_fsm/test/test_ros_fsm.py`):
  - `test_tf_and_odom_are_taken_by_the_input_node_not_the_main_node`: 주 노드가 /tf, /tf_static, /odom을 구독하지 않고 입력 노드가 그 셋을 구독한다. 자세(TF)와 속도(odom)도 그대로 들어온다. 고치기 전에 실패하는 것을 먼저 봤다.
  - `test_the_input_node_follows_the_namespace_and_tf_remaps_like_the_real_launch`: 이름공간 `pode9`와 `/tf:=tf`, `/tf_static:=tf_static`을 주면 입력 노드가 `/pode9/tf`, `/pode9/tf_static`, `/pode9/odom`을 받는다.
  - `test_a_failed_start_leaves_no_input_thread_running`: `control_rate_hz` 0으로 초기화가 실패하면 입력 스레드가 남지 않는다. 입력 노드를 맨 끝으로 옮기기 전에 실패하는 것을 먼저 봤다.

## CPU 결과 (PC, 같은 조건, 동시 측정)

| 바퀴 | 옛 키트 `b9e86c5` | main `c29de003` | 고친 것 |
|---|---|---|---|
| 1 | 19.4 % | 19.3 % | 7.9 % |
| 2 | 17.0 % | 17.4 % | 7.3 % |
| 3 | 17.0 % | 16.7 % | 7.2 % |
| 평균 | 17.8 % | 17.8 % | 7.5 % |
| 4(다시 확인) | 재지 않음 | 21.0 % | 8.2 % |

평균은 1~3바퀴 값이다. 4바퀴는 입력 노드를 `__init__` 끝으로 옮긴 마지막 판을 다시 잰 것으로, 그때 pode_fsm 전체 시험이 같이 돌아 부하가 더 컸다. 원자료는 `ab_rounds.txt`에 있다. PC 한 코어 기준이며, Pi4(Cortex-A72 1.5 GHz)는 같은 일에 몇 배 더 쓴다.

## 시험

- `pode/pode_fsm/test` 전체: 524 passed, 6 skipped(158 s, 마지막 판).
  - 명령: `ROS_DOMAIN_ID=46 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONPATH=pode/pode_fsm:sim/fake_robots:pc/fleet_core python3 -m pytest -q pode/pode_fsm/test`
- 돌리지 않은 것: `sim/fleet_gz_sim`(Gazebo), `sim/fake_robots`, `tools/test`. `PodeFsmNode`를 import하는 시험은 `pode/pode_fsm/test` 안에만 있다. Gazebo 시험처럼 노드를 실행 파일로 띄우는 시험은 이번에 돌리지 않았다.

## 실물에서만 볼 수 있는 것 (다음 실물 측정 항목)

1. **Pi4에서 고치기 전과 후의 pode_fsm CPU.** 같은 조건(서 있음, Nav2 켜짐, goal 없음)에서 잰다. TIME+를 60초 간격으로 두 번 읽거나 `top -b -d 60 -n 2 -p <pid>`를 쓴다. 기준은 11-24의 평균 62 %다.
2. **주행 중 CPU.** goal이 있으면 FollowPath 피드백(controller 20 Hz)이 액션 클라이언트 구독으로 들어와 여전히 주 실행기를 깨운다. PC에서는 재지 않았다.
3. **Pi4 전체에 생긴 변화.** load, Nav2의 "Control loop missed its desired rate" 경고 수, AMCL이 initialpose를 버리는 일이 줄었는지 본다. 11-24에는 odom tf가 23~38 ms 늦어 두 번 버려졌다.
4. **`/pode2/tf`의 빈 메시지(14.9 Hz)를 누가 내는지.** `ros2 topic info -v /pode2/tf`로 확인한다. 빈 메시지도 TF를 구독하는 모든 노드(Nav2 포함)를 깨운다.
5. **배포 순서.** 이 변경을 로봇에 올리려면 main 병합과 nav2fleet 동기화가 먼저다(사용자 지시가 있어야 한다). 그 뒤 `tools/real_pode/deploy.sh`로 올린다.

## 이번에 하지 않은 후보

- **주 노드의 QoS 이벤트 처리기 24개.** rclpy가 발행자와 구독마다 기본으로 붙이는 처리기다(QoS가 맞지 않으면 경고하는 용도). 끄면 wait set이 작아지지만 그 경고를 잃는다.
- **rclpy EventsExecutor.** Jazzy에는 `rclpy.experimental`에 들어 있다. wait set을 다시 만들지 않는 장점이 있지만, 실험 기능이고 단일 스레드다. pode_fsm의 액션 서버 실행 콜백은 끝날 때까지 기다리며 돌기 때문에 단일 스레드와 맞지 않는다.
- **입력 노드 스레드 자체.** 고친 뒤 남은 파이썬 표본의 절반이 여기서 나온다(TF 66 msg/s 받기). pode_fsm에 필요한 것은 map→odom→base_footprint뿐이지만, TF를 걸러서 받을 방법은 없다.
