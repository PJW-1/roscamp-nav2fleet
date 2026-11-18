# 줄 앞자리 로봇이 142 s 선 문제 (run8_1, 2026-11-18)

브랜치 `fix/queue-head-stall-2026-11-18` (출발: `test/real-run8-2026-11-18` 667a394).
커밋: 2e9fc25 (고침 + 시험), 0cc95bc (세 로봇 시험), 8bc6fb1 (시험 단언 보강: 도킹 성공,
pode1 turn 트립이 Q1을 지남. Codex 0cc95bc 검토 지적), 이 문서.

## 문제

실물 두 대 run8_1. WORK는 한 방향 줄 Q2 -> Q1 -> W (지도 v7, `queue: [Q1, Q2], line: true`).
시각은 epoch 1795000xxx의 끝 자리.

- 771.4 pode2 NAV_ABORTED + LOCALIZING(첫 보고 771.38). pode2의 WORK 주문이 빠지고(TO_PARK),
  같은 771.4에 pode1이 O_IN 경유 W 주문(pode1-2)을 받았다. pode1은 772.28에 LOCALIZING(8~10 s,
  PC에서 RViz를 켠 때와 겹침).
- 780.2 / 781.8 둘 다 다시 자세를 보고. pode1은 pode1-3 (base O_IN, horizon W).
- 782.9 pode2가 WORK 주문을 다시 받아 Q2 -> Q1로 갔다. 787.4 "stands at Q1".
- 785.1 pode1 `ZONE_WAIT waits for Q2`. 789.0 pode1이 O_IN에 닿은 뒤 930.9까지 141.9 s 두 로봇 모두
  새 구간을 받지 못하고 섰다. DEADLOCK 이벤트 없음.
- 930.9 pode1이 다시 LOCALIZING으로 트립을 잃자 그제야 pode2가 WORK_DOCK을 받았다.

## 재생으로 본 내부 상태

`tools/replay.py`의 `_read`, `start_core`, `_arguments`로 기록을 다시 넣고 시각마다 상태를 찍었다
(스크립트는 세션 scratchpad `qh/qstate.py`, 저장소에 넣지 않음). 고치기 전 코어는 기록과 같다
(`6484 inputs, 9 inside calls, 67 commands / same as recorded`).

재생에서 상태를 찍은 시각은 783.5, 786.0, 788.0, 790.0, 850.0, 930.0, 931.5다. 790.0, 850.0, 930.0의
세 시각에서 상태가 같았다(관찰은 이 세 시각뿐이다. 그 사이에도 같았다는 것은 기록에 새 명령이 없다는 데서 나온 추정이다):

| | pode1 | pode2 |
|---|---|---|
| node | O_IN | Q1 |
| trip | E-1 O_IN M_E Q2 Q1 W, goal W, **turn True**, done 2 | Q2 Q1 W, goal W, turn False, done 2 |
| `_line_spot` | 2 (줄 밖) | 0 (앞자리) |
| `_ahead(.., WORK)` | [] | **[pode1]** |
| `_is_next` | True | **False** |
| want | M_E, Q2 (Q2는 pode2가 쥠) | 없음 |
| wait key | (Q2,) | (WORK_DOCK,) |

쥔 노드: Q1=pode2, Q2=pode2, W 비었음, WORK_DOCK 비었음, O_IN=pode1.

pode1의 turn은 771.4에 `_grab`의 `_take_turn`(goals.py:153)에서 생겼다. 그때 pode2는 WORK로 가지
않아 앞선 로봇이 없었으므로 DOCK_TURN_TAKEN 이벤트가 나오지 않았다.

## 막은 규칙

1. `Turns._ahead`의 순위(고치기 전 turns.py:116-124) 첫 항이 `not (turn or dock_tries)`라 turn을 가진
   pode1이 줄 자리(spot 2 대 0)와 상관없이 pode2보다 앞이다.
2. `_waits_for(pode1, pode2)`(turns.py:126-128)는 `pode1.goal is None`을 요구하는데 pode1은 order
   goal(base O_IN)이 살아 있어 거짓. 그래서 `_is_next(pode2)`가 거짓이다.
3. order 연장(move_orders.py:129-133)에서 다음 구간이 도크라 `_is_next`를 보고 `_stay`로 돌아간다.
   이 분기는 새 want를 남기지 않는다(있던 want를 지우지도 않는다. 재생에서 pode2의 want는 없었다).
   그래서 `DeadlockWatch.due`(deadlock_watch.py:19-27)의 그래프에는 pode1 -> pode2
   한 줄뿐이고 순환이 없다. 그래서 DEADLOCK도 안 났다.

pode1은 pode2가 쥔 Q2를 기다리고, pode2는 순위상 pode1을 기다린다. 한쪽(순위 대기)이 대기 그래프에
없는 순환 대기다. Codex 진단 검토(~/codex-review.md "queue-head stall diagnosis")도 1~3을 [확정]으로 봤다.

## 고침 (2e9fc25, turns.py `_ahead`, `_blocks`)

줄 스테이션에서만, 두 로봇 중 한쪽만 상대의 남은 길(`trip.route()`, stale이 아닌 트립만) 위의 줄 자리를
쥐고 있으면 쥔 로봇이 앞이다. 둘 다이거나 둘 다 아니면 예전 순위 그대로다. 한 방향 줄에서 남의 길
위 자리를 쥔 로봇은 보통 몸으로 앞에 있고, 이번 사례에서는 그 로봇이 먼저 가야 뒤 로봇도 지나갈 수 있었다(`_blocks`는
위치가 아니라 노드를 쥔 로봇과 아직 잡지 않은 horizon까지 본다. 늘 몸의 순서와 같다는 증명은 없다). 줄 아닌
스테이션(OUTBOUND)과 지도, 길, 지름길은 그대로다.

## 시험 (테스트 먼저)

- `test_v7_the_robot_at_the_front_goes_before_a_turn_taken_from_behind[order_off|order_on]`:
  pode1(P-1)이 먼저 불려 turn을 잡고, 그 뒤 Q1의 pode2를 부른다. pode2가 먼저 도킹하고, 풀어 주면
  pode1이 도킹해야 한다.
- `test_v7_three_robots_all_dock_after_a_turn_taken_from_behind[order_off|order_on]`: 같은데 pode3이
  P-3에서 온다. 세 로봇 모두 도킹하고 첫째가 pode2여야 한다.

고치기 전 코드(turns.py만 667a394 = 2e9fc25^로 되돌림):

```
FAILED test/test_zone_line_queue.py::test_v7_the_robot_at_the_front_goes_before_a_turn_taken_from_behind[order_on]
FAILED test/test_zone_line_queue.py::test_v7_three_robots_all_dock_after_a_turn_taken_from_behind[order_on]
2 failed, 2 passed, 22 deselected in 1.36s
```

order_off는 고치기 전에도 통과한다(실물은 order 모델). 고친 뒤:

```
4 passed, 22 deselected in 0.43s
```

전체 `cd pc/fleet_core && python3 -m pytest -q`:
- 2e9fc25: `1108 passed, 7 xfailed in 51.16s`
- 0cc95bc: `1110 passed, 7 xfailed in 46.81s`
- 8bc6fb1: `1110 passed, 7 xfailed in 44.36s`. 보강한 두 시험도 고치기 전 turns.py에서는
  order_on 둘이 실패한다(`2 failed, 2 passed, 22 deselected in 1.34s`).

세 로봇 탐색(scratchpad `qh/three.py`, 시작 자리 세 가지 x order 켬/끔): 고친 뒤 여섯 경우 모두 셋 다
도킹. 고치기 전에는 order 켬 + (P-1, Q1, P-3)만 300 s 동안 아무도 도킹하지 못했다.

## 재생으로 본 효과

고친 코어로 같은 기록을 재생하면 1113줄에서 처음 달라진다(782.9). 고친 코어는 783.4에
`ZONE_GRANTED HALL+WORK_DOCK to W`와 pode2-5 update 2(base W)를 낸다. 기록에서는 930.9에 받았다.
즉 147.5 s 일찍 도크를 준다.

재생은 기록된 로봇 입력을 그대로 넣으므로 갈라진 뒤는 볼 수 없다. 실제 로봇은 update 2를 받은 적이
없어 고친 코어는 785.5에 `ORDER_UPDATE_REJECTED no answer in 2 s`를 내고 다시 보낸다. 이것은 재생의
한계이지 고친 코드의 동작이 아니다. 도킹 이후의 흐름은 위 시뮬레이션 시험으로만 확인했다.

## 확인하지 못한 것과 남은 위험

- 실물과 Gazebo에서는 돌려 보지 않았다.
- Codex 검토 [확정]: 쌍별 비교라 일반적으로는 추이적이지 않다. v7처럼 들어오는 곳이 하나인 한 방향
  줄에서는 줄에 선 로봇이 뒤 로봇 모두의 길 위 자리를 쥐므로 순서가 몸의 순서와 같을 것으로 본다(추측,
  증명 없음). 셋째 로봇이 stale이 아닌 트립이 없고 `dock_tries`나 부른 시각으로 순위가 앞서면 세 쌍이
  어긋나 셋 다 설 수 있다는 반례가 코드상 있다. 그런 상태가 실제로 생기는지, 고치기 전에도 서는지는
  확인하지 않았다(추측). 시험은 세 로봇 탐색 여섯 경우뿐이다.
- `_blocks`는 아직 잡지 않은 horizon까지 본다(Codex [확정]). order 모델은 대기 트립 끝에 도크까지
  붙이므로 그 경우에도 순서를 바꿀 수 있다. 시험 범위에서는 문제가 보이지 않았다.
- turn과 `dock_tries`보다 `_blocks`가 먼저다. 이미 Q1 -> W로 보낸 이동은 취소되지 않지만
  (move_orders.py:123), 아직 도크 구간을 잡지 않은 재도킹 로봇이 줄 앞 로봇에게 밀리는 경우는
  시험하지 않았다.
- `_stay`가 새 want를 만들지 않아(있던 want를 지우지도 않는다) 순위 대기가 DeadlockWatch에 보이지
  않을 수 있는 점은 그대로다. 다른 순위
  대기가 또 생기면 같은 식으로 조용히 설 수 있다.
- 771.4의 LOCALIZING(RViz와 겹침) 원인은 이 작업 범위가 아니다.
