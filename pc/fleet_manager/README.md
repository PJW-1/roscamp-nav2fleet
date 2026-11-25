# fleet_manager

`fleet_core`의 `FleetManager`를 ROS 2 노드(`fleet_manager_node`)로 감싼 패키지입니다.

## 동시성 구조 (IR-30)

`main()`은 `MultiThreadedExecutor`(`make_executor()`)로 돕니다. 코어 상태는 지금처럼 한 곳에서만 바뀝니다(단일 writer).

| 그룹 | 콜백 | 하는 일 |
|---|---|---|
| 코어 그룹 (`MutuallyExclusiveCallbackGroup` 하나) | 틱, heartbeat, `/fleet/robots` 발행, 서버(`confirm_clear` · `reinstate` · `force_ship`) | `self.core`와 노드 상태(`_pode_states`, `_goals` 등)를 바꾸는 유일한 곳 |
| 받은 편지함 그룹 (코어 그룹과 다른 그룹) | 구독(`robot_state` · `arm_state` · `order_board` · `system_mode`), 액션 goal 수락 · feedback · 결과, 서비스 응답(`check_clear` · `report_order` · `set_mode`) | 처리 함수와 인자를 `Inbox`(스레드 안전한 큐)에 넣기만 함 |

- **큐 비우기**: 틱 시작과 서버 콜백 시작에서 큐를 비우고, 넣은 순서대로 처리 함수를 부릅니다. 그래서 큐에서 꺼낸 순서가 코어 입력 기록(`record_to`, `--record`) 순서이고, `tools/replay.py`로 재생하면 같은 판단이 나옵니다.
- **future 완료 콜백**: rclpy는 future의 완료 콜백을 실행기 작업으로 돌려 콜백 그룹의 제약을 받지 않습니다. 그래서 완료 콜백도 `_post`로 큐에 넣기만 합니다.
- **heartbeat를 코어 그룹에 둔 이유**: 코어가 멈추면 heartbeat도 끊겨 로봇이 IR-34대로 서야 하기 때문입니다.
- **콜백 안에서 기다리지 않음**: 동기 `call()`, `spin_until_future_complete`, `spin_once`를 쓰지 않습니다(`test/test_no_blocking_calls.py`가 검사).
- **틱 처리 시간**: 큐 비우기를 포함한 틱이 주기(`1 / tick_hz`)를 넘으면 5초에 한 번까지 경고 로그를 냅니다. 큐 길이 최대값은 `Inbox.peak`에 남습니다.
