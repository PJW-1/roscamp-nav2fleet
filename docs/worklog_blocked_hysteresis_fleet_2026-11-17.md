# 관제 막힘 시계 이력 현상 (2026-11-17)

브랜치 `fix/blocked-hysteresis-fleet` (main 956140d에서). 관제 쪽만 고쳤다. 로봇(pode_fsm)은 그대로다.

## 무엇이 잘못됐나

- `pc/fleet_core/fleet_core/traffic/zone/blocked.py` `_watch_blocked`는 `BLOCKED`(또는 45 s 넘은
  `RECOVERING`)가 아닌 보고가 한 번만 와도 `blocked_since`, `blocked_noted`, `rear_noted`를 지웠다.
- 막힘 표시와 다른 길(`ZONE_MARKED_BLOCKED`, `REROUTE`)은 `blocked_mark_s`(10 s) 동안 이어진
  `BLOCKED` 뒤에만 나간다.
- 로봇은 좁은 곳에서 깜빡인다: 약 1 s 서면 `RECOVERING`, 2 s 뒤 `BLOCKED`, Nav2 goal이면 5 cm
  (`nav_progress_m`)만 움직여도 `BLOCKED`가 풀리고, 고정 경로면 장애물이 1 s 안 보이면 풀린다.
- 그래서 `BLOCKED`가 10 s보다 자주 끊기면 시계가 매번 0으로 돌아가 표시도 다른 길도 없이 제자리에서
  꿈틀댔다(Gazebo에서 약 40 s 선 로봇을 봤다. 로그로 증명하지는 않았다).

## 규칙 (막힘 일화)

같은 goal 안에서 첫 `BLOCKED`가 일화를 연다. 시작 시각과 시작 위치를 적는다.

- 일화 안의 막힘 아닌 보고(`MOVING` 등)와 `RECOVERING`은 시계를 다시 세지 않는다. 기다린 시간은
  일화 시작부터 잰다.
- 일화가 끝나는 때(다음 `BLOCKED`는 새 일화):
  - 시작 위치에서 `blocked_reset_m`(0.30 m) 넘게 나아갔다(막힘 아닌 보고에서 잰다);
  - `BLOCKED`도 `RECOVERING`도 아닌 보고가 `blocked_reset_s`(20 s) 이어졌다. `RECOVERING`은 이
    시간을 멈춘다(자가 복구는 막힘의 일부);
  - goal이 바뀌었거나 없어졌다(DOCK, BACKOFF goal로 바뀐 경우도 포함).
- 끝낼 때(거리, 시간)는 막힘 아닌 보고에서도, 틈 뒤 BLOCKED가 돌아온 첫 보고에서도 본다(Codex 지적 뒤 더함).
- 표시·양보·실패는 지금 막힘을 보고할 때만 한다. 기다린 시간이 넘었으면 `BLOCKED`가 돌아오는 첫
  보고에서 바로 한다.
- `BLOCKED_WAIT`는 일화마다 한 번. 틈 뒤 다시 막힌 횟수를 세어 `ZONE_MARKED_BLOCKED`와
  `GOAL_FAILED` 끝에 `, after N short gaps`로 붙인다(0이면 붙이지 않아 예전 문구 그대로).
- 표시(`_mark_blocked`)와 양보(`_yield_to_robot`)가 `blocked_since`를 지우는 것은 그대로다. 그 뒤
  다시 막히면 시계를 새로 연다(예전과 같다).
- goal이 바뀐 채 막히면 `BLOCKED_WAIT`를 다시 낸다(예전에는 끊김 없이 이어 막혀 있으면 다시 내지
  않았다). 틈 사이에 goal이 바뀐 일화를 옛 goal의 것으로 두지 않기 위해서다.

## 파라미터 (`fleet_params.yaml`, `params.py`)

| 키 | 값 | 근거 |
|---|---|---|
| `blocked_reset_m` | 0.30 (추정) | Pode 자가 후진 최대 `backup_max_m` 0.15 m(pode_fsm/params.py)의 두 배. 후진했다가 다시 다가와도 일화가 끝나지 않는다. 마당 물러나기 `yard_backoff_m` 0.12 m보다도 크다 |
| `blocked_reset_s` | 20.0 (추정) | Nav2 goal 모드에서 로봇은 5 cm 움직이면 BLOCKED를 풀고(nav_goal.py:100-101), 다시 BLOCKED가 되기까지 `blocked_after_s` 10 s가 걸린다(nav_goal.py:105-106). 그래서 틈은 10 s 남짓까지 간다. 처음에 10 s로 했다가 부모 세션의 로봇 쪽 조사(직접 다시 확인함)를 받고 그 두 배로 올렸다. 0.12 m/s로 20 s면 2.4 m라 정말 움직이는 로봇은 거리로 먼저 끝난다 |

검사: 두 키 모두 `_s`, `_m` 끝말이라 기존 `check_measures`가 유한, 0보다 큼을 본다
(`test_params_validate`에 0 거부 경우를 더했다).

## 시험

새 파일 `pc/fleet_core/test/test_zone_blocked_flicker.py`(11개). 하네스는 `zone_harness.Harness`
(속도 0, pode2가 Nav2 goal을 받고 선 채 보고만 바꾼다).

- 3 s BLOCKED / 0.5 s MOVING 반복 → 일화 10~11 s에 표시, `BLOCKED_WAIT` 한 번, `after 3 short gaps`;
- BLOCKED 2 s / MOVING 0.5 s / RECOVERING 2 s 반복 → 13 s 안에 표시;
- 틈마다 ±4 cm 꿈틀거림 → 표시(진전으로 치지 않음);
- 9 s 틈 → 11 s에는 표시 없음, BLOCKED가 돌아온 12 s에 표시;
- BLOCKED 1 s / MOVING 9 s·11 s(±4.5 cm 꿈틀거림) 반복 → 두 번째 BLOCKED에서 표시(Nav2 goal 다시 막힘 틈 흉내);
- 첫 틈에 0.35 m 나아감 → 새 일화(표시가 그만큼 늦음);
- blocked_reset_s + 1 s 틈 → 표시 없음, `BLOCKED_WAIT` 일화마다 한 번(3번);
- 틈 사이 goal 바뀜 → 옛 goal의 기다림을 넘기지 않음;
- BLOCKED가 돌아온 보고에서 0.30 m를 넘음 → 그 보고에서 새 일화;
- 막힘 아닌 보고가 정확히 blocked_reset_s 이어진 뒤의 BLOCKED → 새 일화(긴 틈 시험은 멈춤 감시가 goal을 거두지 않게 `stall_s` 1000, `start_limit_s` 0으로 돌린다).

결과는 아래 "결과"에 그대로 적는다.

## 결과

(아래 채움)

## Codex 리뷰 (62bfb98)

- [확정] 거리 기준을 BLOCKED가 돌아온 보고에서 넘으면 옛 시계를 이어 썼다 → 고침(시험 먼저, 고치기 전 실패 확인).
- [확정] 막힘 아닌 시간이 BLOCKED가 돌아온 보고에서 blocked_reset_s에 닿으면 옛 시계를 이어 썼다 → 고침(같음).
- [확정, 고치지 않음] 같은 goal을 둔 채 상대를 주차로 보내는 양보(`yard_retreat.py` `_yield_to_robot`) 뒤 계속 막혀 있으면 시계는 새로 열지만 `BLOCKED_WAIT`는 다시 나오지 않는다. 예전 코드와 같은 동작이라 두었다.

## 남은 것

- Gazebo/실제 로봇 로그로 깜빡임 주기를 아직 재지 않았다. 두 값은 추정이다.
- 로봇이 `BLOCKED`를 보고하면서 움직이는 경우(Nav2가 BLOCKED 표시를 단 채 꿈틀댐)는 거리 검사를
  하지 않는다(막힘 보고 중 진전은 예전처럼 무시).
- Nav2가 BLOCKED 없이 0.3 m 안에서 20 s 넘게 꿈틀대면 일화가 끝나고, 멈춤 감시(`stall_move_m` 0.05 m)도 5 cm 넘는 꿈틀거림에는 걸리지 않는다. 그런 로봇은 관제가 아직 못 잡는다.
- 로봇 쪽 고침(별도 브랜치, BLOCKED를 0.15 m 또는 3 s 뒤에 풂)이 들어오면 틈이 짧아진다. 관제 규칙은 그 고침 없이도 돌게 했다.
- 일화 시작 전의 `RECOVERING`(로봇은 BLOCKED 전에 RECOVERING을 보낸다)은 기다린 시간에 넣지 않는다.
  넣으면 표시가 약 2~3 s 빨라진다. 바꿀지는 실측 뒤에 정한다.
