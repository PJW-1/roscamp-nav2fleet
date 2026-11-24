# pode_fsm 동작 없음 처리(설계 4.5) — 2026-11-24

- 브랜치: `feat/no-motion-2026-11-24` (main c62c03d에서 시작, 인터페이스 0.6)
- 기준 문서: 팀 저장소 docs/design 브랜치 `docs/설계/주문_흐름_1차_인터페이스.md` 4.5, 9장
- 타입과 contract.yaml은 바꾸지 않았다. 새 정의도 없다.

## 켜는 법

```bash
ros2 launch pode_fsm pode_fsm.launch.py namespace:=pode1 no_motion:=true
```

- `no_motion` 실행 인자 하나로 켠다. 기본은 `false`(실주행)다.
- 동작 없음일 때는 Nav2 없이 pode_fsm 혼자 돈다. bringup, Nav2, 라이다가 필요 없다.
- `real_pode.launch.py`와 `start.sh`에는 넣지 않았다. 그쪽은 Nav2를 함께 띄우는 실주행용이다.
- 시작 자세는 `initial_x`, `initial_y`, `initial_yaw`(rad)로 준다. 기본은 P-1이다. 세 대를 띄울 때는 로봇마다 다른 주차 자리를 줘야 한다. 주지 않으면 세 대가 모두 P-1에 있다고 알린다.

| 로봇 | 자리 | 인자 (v7 지도) |
|---|---|---|
| pode1 | P-1 | `initial_x:=0.12 initial_y:=0.10 initial_yaw:=1.5708` (기본) |
| pode2 | P-2 | `initial_x:=0.38 initial_y:=0.10 initial_yaw:=1.5708` |
| pode3 | P-3 | `initial_x:=0.64 initial_y:=0.10 initial_yaw:=1.5708` |

## 방식

FSM, 액션 서버(`GoalServers`), heartbeat 판단, 상태 발행은 실주행 코드를 그대로 쓴다. Nav2 실행기 세 개(FollowPath, NavigateToPose, BackUp)만 새 파일 `pode_fsm/no_motion.py`의 대역(`FollowStub`, `NavGoalStub`, `BackUpStub`)으로 바꾼다.

- 대역은 정해진 시간 뒤 Nav2처럼 `succeeded`를 알리고, 자세를 끝점으로 옮긴다.
- 취소는 다음 틱에 "취소 수락 + canceled"로 알린다.
- 그래서 목표 교체, 취소, order 이어 붙이기, PATH_NOT_CONNECTED, heartbeat 판단은 실주행과 같은 코드가 처리한다.
- 대역 일은 FSM tick(heartbeat 검사) 뒤에 처리한다. 그래야 heartbeat가 끊긴 틱에 목표가 성공으로 끝나지 않는다.

| 명령 | 파라미터 | 기본 | 끝난 뒤 |
|---|---|---|---|
| move_to (조각 묶음마다) | `no_motion_move_s` | 2.0 s | 자세 = 경로 끝 또는 Nav2 goal |
| retreat | `no_motion_retreat_s` | 2.0 s | 곧은 후진: 보는 방향 반대로 `distance_m`, 경로 후진: 경로 끝 |
| dock | `no_motion_dock_s` | 3.0 s | 성공(오차 0), 상태 DOCKED |

- 자세는 기다리는 동안 그대로이고, 끝나는 순간 목표로 옮긴다. FSM이 같은 끝점으로 경로를 다시 보내면 처음 기다림을 잇는다(0.15 m보다 짧은 이동에서 생긴다).
- 도킹은 PC 도킹 대역(`real_dock_wait`)처럼 DOCKING을 거쳐 DOCKED가 되고, 피드백을 APPROACH → ALIGN → DONE 순서로 보낸다. DONE은 성공 직전에 꼭 보낸다. 3.0 s는 실물 시나리오의 `dock.duration_s`와 같다.
- 도킹은 `dock_id`나 스테이션과의 거리를 확인하지 않는다. pode_fsm은 스테이션 좌표를 모르기 때문이다. PC 대역은 0.15 m 밖이면 실패시킨다.
- `check_clear`는 늘 `clear`로 답한다.
- 상태는 동작 없음일 때 기본 5 Hz로 보낸다. 끄면 지금처럼 10 Hz다. LOCALIZING 단계는 없다.
- heartbeat가 1.5 s 끊기면 하던 이동, 후진, 도킹을 `HEARTBEAT_LOST`로 끝내고 IDLE로 선다. 후진은 계약상 reason이 넷뿐이라 `FAILED`가 되고, FakePode도 같다. 자세는 바뀌지 않는다.
- 동작 없음에서는 로봇단 후진 양보(local_yield)를 끈다. 바퀴가 없는데 양보하면 자세만 뒤로 뛰기 때문이다. `no_motion_move_s`는 `blocked_after_s`(10 s)보다 짧아야 한다.
- 끄면 바뀌는 것이 없다. 분기는 모두 `self._nm is not None` 뒤에 있고, 기존 시험이 그대로 통과한다.

## FakePode와 같은 것

v7 구역 지도에서 P-1 출발, 두 노드에 같은 목표를 보내고 비교했다. 피드백은 같은 값이 이어지면 하나로 묶었고, 상태는 바뀐 순서만 남겼다.

| 경우 | 둘이 같았던 순서 |
|---|---|
| 고정 경로 P-1 → E-1 | 결과 (True, "", 0), 상태 IDLE → MOVING → IDLE, current_goal E-1, 피드백 (0, False, "") |
| Nav2 goal | 성공, 끝 자세 = 목표 |
| 목표 교체 | 앞 목표 (False, CANCELED, 0), 뒤 목표 성공 |
| 취소 | (False, CANCELED, 0) |
| PATH_NOT_CONNECTED | 상태 IDLE 그대로 |
| 경로 후진, 곧은 후진 | (True, ""), 상태 IDLE → MOVING → IDLE |
| 도킹 | 상태 IDLE → DOCKING → DOCKED, 다음 이동에서 DOCKED → MOVING |
| order (follow, nav, horizon nav) | 피드백 (0, F) → (1, F) → (2, True), base 끝에서 상태 MOVING. update를 보내면 앞 목표 (False, SUPERSEDED, 2), 뒤 목표 (True, "", 3). order 취소 (False, CANCELED, 0) |
| heartbeat 끊김 | (False, HEARTBEAT_LOST, 0), 둘 다 끊긴 뒤 1.4~2.2 s 안에 결과 |

## FakePode와 일부러 다른 것

`test_ros_no_motion.py`의 `KNOWN_DIFFERENCES`에 두고 하나씩 시험으로 고정했다.

| 항목 | 동작 없음 | FakePode | 이유 |
|---|---|---|---|
| distance_remaining | 끝날 때까지 시작 거리 | 줄어듦 | 움직이지 않으므로 |
| 도킹 DONE 피드백 | 늘 보냄 | 틱 순서에 따라 빠질 수 있음 | PC 대역과 같게 |
| 도킹 오차 | 0 | 계산값 | 움직이지 않으므로 |
| heartbeat 끊긴 뒤 자세 | 시작 자리 | 가던 중간 | 끝나야 자세가 옮겨지므로 |
| 배터리 | -1(모름) | 시나리오 값 | 배터리 토픽이 없으므로 |
| check_clear | 서비스 있음, 늘 clear | 서비스 없음 | 설계 4.5 |
| 상태 주기 | 5 Hz | 10 Hz | 설계 4.5 |
| order current_goal | 지금 달리는 조각 묶음의 끝(E-1, 다음 O_OUT) | base 끝(O_OUT) | 실주행 pode_fsm에 원래 있던 차이 |

## 시험

`ROS_DOMAIN_ID=65`(관제 세션 확인은 67), `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

| 명령 | 결과 |
|---|---|
| `python3 -m pytest -q pode/pode_fsm/test` | 487 passed, 6 skipped (바꾸기 전 main: 467 passed, 6 skipped) |
| `test_ros_no_motion.py`만 반복 | 두 번 모두 통과 |
| `python3 -m pytest -q tools/test/test_real_pode.py -k "fsm or dock_aside"` | 7 passed |

더한 시험 20개:
- `test_no_motion.py` 10개: 대역과 FSM 도킹 대기 (ROS 없음)
- `test_ros_no_motion.py` 8개: FakePode 비교 3개, 거리 차이, 5 Hz와 끝 자세, check_clear, 기본 기다림 2/2/3 s, heartbeat와 대역 완료가 같은 틱에 오는 회귀
- `test_launch.py` 2개: 기본 꺼짐, 노드 파라미터로 전달

잰 값: 상태는 3 s에 12~18개, heartbeat는 끊긴 뒤 1.4~2.2 s 안에 결과가 나왔다.

아직 하지 않은 것: 실제 `ros2 launch ... no_motion:=true`로 띄워 관제와 붙여 보기, 로봇 Pi에서 띄우기.

## 리뷰 (Codex)

| 지적 | 처리 |
|---|---|
| 같은 틱에 대역 완료가 heartbeat 검사보다 먼저 처리되어 성공으로 끝날 수 있음 | 시험으로 재현한 뒤 고침 (5f6946f). 다시 리뷰한 결과 결함 없음 |
| 후진 중 heartbeat가 끊기면 FAILED | 계약상 Retreat reason이 넷뿐이고 FakePode도 같아서 그대로 둠 |
| 도킹이 스테이션 위치를 확인하지 않음 | 사용자 결정으로 남김 (아래) |
| 도킹 피드백이 취소 뒤에 나갈 수 있음 | goal을 끝내는 것도 같은 execute 스레드라 끝난 뒤에는 나가지 않음. 결함 아님 |
| 비교 시험이 거리 등 일부 칸을 빼고 비교함 | 모두 의도한 차이로 목록에 두고 시험으로 고정함 |

## TODO 표시

`TODO(no-motion)`: `no_motion.py` 머리, `node.py`(파라미터, `_start_no_motion`), `core.py`와 `order_fsm.py`의 `dock_wait`, `server_ros.py` 도킹 분기, launch 인자.

## 정할 것과 다음 주

- 도킹을 `dock_id`나 스테이션 위치와 관계없이 성공시켜도 되는지. 맞추려면 pode_fsm이 지도(스테이션 좌표)를 읽어야 한다.
- order current_goal을 조각 묶음의 끝으로 둘지, FakePode처럼 base 끝으로 맞출지.
- 시작 자세를 로봇 이름으로 주차 자리에서 고르게 할지(지금은 인자로 준다).
- 실주행으로 돌아갈 때는 `no_motion`을 주지 않으면 된다. 정리할 때 TODO 코드를 지운다.
