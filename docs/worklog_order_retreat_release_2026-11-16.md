# 작업 기록: order 물러나기 중 구역을 일찍 놓는 버그 (2026-11-16)

브랜치 `fix/order-retreat-release` (출발점 `fix/fake-pode-order` f9bee09). 대상은 `pc/fleet_core`만이다.

## 1. 버그

- ROS 시나리오 stock_shortage에서 판정기가 `t=107.4 pode1 released J_W 0.076 m from it`을 냈다
  (`/tmp/claude-1000/fakeorder_ws/scen4_tmp/test_scenario_finishes_and_mat3/safety.json`).
  `releases.csv`: 같은 시각 WORK_DOCK(참 0.0846 m, 보고 0.1379 m), J_W(참 0.0762 m, 보고 0.1215 m).
  평소 놓는 거리는 보고 위치 기준 0.135 m 이상이다.
- order `pode1-38`(fms.jsonl:4446)은 한 goal에 seq 1(retreat, W → N_W)과 seq 2(nav_goal, P-2)를 담았다.
  seq 1을 끝냈다는 피드백은 t=109.501(fms.jsonl:4808)로, 놓은 뒤 2초다.
- 원인(코드로 확인):
  1. `progress.py` `_track`은 goal의 마지막이 아니고 NAV_GOAL도 아닌 링크를, 보고가 링크 끝
     `ON_NODE_M`(0.03 m) 안에 들면(`passed()`, 옆 거리 제한 없음) 넘기고 `m.node`를 링크 끝으로 바꾼다.
  2. order가 꺼져 있으면 물러나기는 따로 RetreatCmd라 늘 goal의 마지막 링크여서 넘기지 않았다.
     켜면 물러나기 뒤에 다음 조각이 붙어 마지막 링크가 아니게 된다.
  3. `_release_behind`는 보고가 `m.node`에서 `release_margin_m`(0.05 m) 안이면 노드 좌표로 잰다.
     N_W는 J_W에서 0.14 m라 놓는다. 그러나 물러나는 로봇은 놓을 구역 쪽에서 노드로 다가가므로,
     노드 앞 0.03~0.05 m에 보고 잡음까지 더하면 몸이 아직 구역에 걸쳐 있다.
- 같은 모양을 하나 더 찾았다: 급행에서 막혀 갈림목을 거쳐 마당 지점(N_E)으로 물러나는 yard retreat도
  order가 켜져 있으면 물러나기 뒤에 다음 조각이 붙어 J_E를 일찍 놓는다(아래 시험 2).

## 2. 시험 (먼저 쓰고 실패 확인)

1. `test_zone_release.py::test_a_robot_backing_out_is_measured_from_its_report_until_the_retreat_is_done[order_off|order_on]`
   W에서 주차 자리로 나가는 pode1을 N_W 0.025 m 앞(보고 기준 J_W 0.115 m)에 세우고 답하지 않게 한다.
   J_W, WORK_DOCK을 쥐고 있어야 한다. 그 뒤 움직이게 하면 RETREAT_DONE 뒤에 J_W를 놓는다.
2. `test_zone_yard_retreat.py::test_the_junction_is_kept_until_the_robot_says_it_has_backed_down[order_off|order_on]`
   급행에서 막혀 N_E로 물러나는 pode1을 N_E 0.028 m 안에서 세운다. J_E를 쥐고 있어야 한다.

수정 전 코드(사본에 HEAD의 progress.py를 넣고 돌림), 나온 그대로:

```
== manager
E       AssertionError: ['HALL', 'J_W', 'WORK_DOCK']
E       assert 'J_E' in ['HALL', 'J_W', 'WORK_DOCK']
E       AssertionError: ['HALL']
E       assert ('J_W' in ['HALL'])
FAILED fleet_core/test/test_zone_yard_retreat.py::test_the_junction_is_kept_until_the_robot_says_it_has_backed_down[order_on]
FAILED fleet_core/test/test_zone_release.py::test_a_robot_backing_out_is_measured_from_its_report_until_the_retreat_is_done[order_on]
2 failed, 24 passed in 3.87s
== zone
(같은 두 개 실패)
2 failed, 24 passed in 3.14s
```

order_off는 수정 전에도 통과한다(끈 길은 원래 안전했다).

## 3. 수정과 이유

`progress.py` `_track`: 보고가 RETREAT 링크 끝 노드에서 `release_margin_m`(노드 좌표로 재는 거리)
안에 있는 동안은 진행으로 넘기지 않는다. 끝냈다는 답(order의 SEGMENT_DONE, 끈 길의 물러나기 결과)을
기다린다. `passed()`가 참이고 끝 노드까지의 직선거리가 `release_margin_m`보다 크면(보통은 노드를
그만큼 지나갔을 때, 옆으로 벗어난 보고에서도 참이 될 수 있음) 진행으로 넘긴다. FOLLOW_PATH 링크는
그대로 넘긴다.

처음에는 RETREAT 링크를 답으로만 넘기게 했다. 그러자 풀 스위트에서 test_zone_move_orders.py의 세 시험이
실패했다(`3 failed, 1073 passed, 1 skipped, 3 xfailed`, 두 틱 순서 모두). 세 시험은 `hold_answers`로
모든 로봇의 답을 막는다. 그래서 출고 도크에서 물러난 pode2가 N_E를 한참 지나 P-2로 가는데도 N_E 노드를
계속 쥐었고(`_still_needed`가 물러나기 링크의 끝 노드를 아직 필요하다고 봄), pode1의 update가 나가지
못했다. 답이 늦거나 사라지면 실제로도 같은 일이 생긴다(노드를 쥔 채 다른 로봇을 막음). 그래서
"노드 좌표로 잴 거리 안에서만 답을 기다린다"로 좁혔다. 그 거리 밖의 보고는 `_release_behind`가
(`zone_clear_m`이 0인 기본 설정에서) 노드 좌표가 아니라 보고 위치로 잰다. 다만 보고 잡음 자체는 남고,
넘긴 뒤 다음 보고가 다시 노드 0.05 m 안으로 들어오면 노드 좌표로 잴 수 있다(5절 2, 6절 3). 이 동작은
`test_an_order_retreat_whose_answer_is_late_is_passed_once_the_robot_is_beyond_the_node`가 지킨다.

후보 비교:
- (a) `_track`이 RETREAT 링크를 넘기지 않는다(노드 가까이에서만) — 고름.
  - 잘못된 상태(끝내지 않은 물러나기를 끝낸 것으로 친 trip.done, m.node)를 생기는 곳에서 막는다.
    그래서 `_release_behind`뿐 아니라 `_via()`(물러나는 중인데 nav_goal로 보임: blocked.py의 뒤 막힘
    판정, `_extend_order`의 yard 판정)와 `_still_needed`도 맞게 본다.
  - 끈 길과 같은 뜻이 된다: 끈 길에서도 물러나는 동안 `m.node`는 떠난 도크이고, 결과가 와야 바뀐다.
  - FOLLOW_PATH의 앞으로 가는 진행은 그대로다. 끈 길은 물러나기 goal이 링크 하나라
    (`goal.links - 1` 조건으로 원래 넘기지 않음) 바뀌지 않는다. 지도들(d2, d, c, v7, d2_shelf)에
    이어진 물러나기 링크는 없다(직접 확인).
- (b) 물러나는 중에는 `_release_behind`에서 노드 좌표로 재지 않는다 — 고르지 않음.
  - Codex의 진단 리뷰는 (b)를 더 안전한 직접 수정으로 보았다. 그러나 `_via(m) == RETREAT`로 가르면
    이 경우를 놓친다(Codex도 지적함): `_track`이 이미 nav_goal 링크로 넘겼다.
    넘긴 뒤에도 남는 표시(지난 링크 종류, 끝내지 않은 retreat seq)를 따로 들고 다녀야 하고,
    잘못 넘긴 trip.done이 다른 곳에 주는 영향은 그대로 남는다.
  - Codex가 제안한 "보고 거리에 잡음 여유를 더해 판정 반경을 넘을 때만 노드 좌표 사용"은 이번 경우를
    막지 못한다: 놓던 순간 보고 거리가 J_W 0.1215 m로, 0.085 + 0.03 = 0.115 m를 이미 넘었다.
- 노드 좌표 규칙 자체는 없애지 않는다. O_OUT에 0.04 m 못 미쳐 선 로봇(test_zone_release.py의
  stopped_short_of_o_out)과, 물러나기를 끝낸 뒤 N_W에서 놓는 order 길(같은 파일 40-42행 docstring)이
  그 규칙에 기댄다. 두 시험 모두 통과한다.

## 4. 명령과 결과

수정 후, 새 시험 파일 (나온 그대로):

```
$ for t in manager zone; do ZONE_TICK_ORDER=$t PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider \
    pc/fleet_core/test/test_zone_move_orders.py pc/fleet_core/test/test_zone_release.py pc/fleet_core/test/test_zone_yard_retreat.py | tail -2; done
== manager
55 passed in 6.58s
== zone
55 passed in 5.91s
```

fleet_core 전체(커밋 d00449b 기준, 기준선 f9bee09는 1074 passed, 1 skipped, 3 xfailed):

```
$ ZONE_TICK_ORDER=manager PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test
1079 passed, 1 skipped, 3 xfailed in 66.95s (0:01:06)
$ ZONE_TICK_ORDER=zone PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test
1079 passed, 1 skipped, 3 xfailed in 64.12s (0:01:04)
```

뒤에 더한 늦은 보고 시험(6절 1). 처음 커밋(d956fe5)의 strict xfail은 Codex 지적대로 앞 단언의 실패를
가리고 있었다: 정확한 보고로는 하네스가 N_W에서 WORK_DOCK을 이미 정당하게 놓아, xfail이 앞 조건
("RETREAT_DONE 때 WORK_DOCK을 쥠")에서 실패하고 있었다. ba297be에서 고쳤다. 물러나며 (0.35, 0.69)를
지난 뒤로 보고가 거기 멈추게 하고(현장의 늦은 보고), 앞 조건은 보통 시험
`test_the_late_report_case_is_set_up_as_seen_in_the_field`가 본다. strict xfail에는 마지막 단언만 남겼다.
ba297be 기준(나온 그대로):

```
$ ZONE_TICK_ORDER=manager|zone PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test/test_zone_release.py
20 passed, 2 xfailed in 1.25s        (manager)
20 passed, 2 xfailed in 1.21s        (zone)
$ ... -k late_report_at --runxfail   (두 틱 순서 모두)
E       AssertionError: assert 'WORK_DOCK' in ['J_W', 'HALL', 'EXPRESS', 'J_E', 'OUT_DOCK']
pc/fleet_core/test/test_zone_release.py:291: AssertionError
2 failed, 20 deselected
$ ZONE_TICK_ORDER=manager PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test
1081 passed, 1 skipped, 5 xfailed in 43.59s
$ ZONE_TICK_ORDER=zone ...
1081 passed, 1 skipped, 5 xfailed in 42.24s
```

291행은 xfail 시험의 마지막 단언이다.

world 시험(d00449b 기준):

```
$ ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONPATH=pc/fleet_core:sim/fake_robots \
    python3 -m pytest -q -p no:cacheprovider sim/fake_robots/test/test_world_*.py sim/fake_robots/test/test_situation*.py
FAILED sim/fake_robots/test/test_situation_avoid_modes.py::test_the_yard_back_off_does_not_stand_a_robot_on_a_node_it_does_not_hold[order_burst-12-True]
1 failed, 420 passed, 27 xfailed in 453.13s (0:07:33)
```

실패한 것은 `assert outcome.world.metrics.count("HALL_BACKOFF_DONE") >= 1, "the back-off was used"`다.
`outcome.failures == []`(불변식)는 지킨다. 시점이 바뀌어 씨드 12가 더는 물러나기를 쓰지 않는다.
수정 전 사본(f9bee09의 progress.py)에서는 `3 passed`. 이 시험의 docstring에 적힌 대로 씨드가
물러나기를 쓰지 않게 되면 씨드를 바꿔 왔다. `sim/`은 이 작업 범위 밖이라 고치지 않았다(6절).

## 5. Codex

- 진단 리뷰(코디네이터가 돌려 전달): 로그가 진단과 맞는다고 확인. (b)를 권했으나 `_via` 판정으로는
  놓친다고 함. 노드 좌표 규칙을 통째로 없애지 말라고 함. 반영: 3절.
- d00449b 리뷰(`~/codex-review.md`, 도착해서 읽음). 직접 확인한 결과:
  1. [확정, 확인함] order의 SEGMENT_DONE, `_arrived_anyway`(results.py:146, `_link_reached`는 끝 노드
     0.075 m 안이면 참: base.py:61-63, arrival_tolerance_m 0.15), 통신 복구의 `_resume`(health.py:50)은
     구역을 빠져나갔는지 확인하지 않고 `m.node`를 끝 노드로 둔다(SEGMENT_DONE은 위치를 보지 않고,
     `_arrived_anyway`와 `_resume`, 끈 길의 물러나기 성공(results.py:106 `_succeeded`)은 도착 허용치
     안인지만 본다). 끈 길에도 똑같고 이번 수정 전부터 있었다. 남은 점으로 둔다.
  2. [확정, 확인함] 옆 방향 잡음으로 끝 노드에서 0.05 m 넘게 보고되면 길 방향으로는 못 미쳤어도 링크를
     넘긴다. 그 보고에서는 보고 위치로 재지만, 뒤 보고가 다시 0.05 m 안이면 노드 좌표로 잰다. 좁지만
     실제로 있을 수 있는 길이다. 남은 점.
  3. [확정] 길 위에서 끝까지 0.03 m 넘게 남았으면 `passed()`가 원래 거짓이라 새 지연은 주로 0.03 m
     안에서 생긴다(옆으로 벗어난 보고는 직선거리 0.03~0.05 m에서도 passed가 참일 수 있다). 답이 끝내 안
     오면 정지 감시(stall_s 12초)가 도착 추정을 시도한다. 다만 정지 감시가 돌고 새 보고가 도착 판정을
     채울 때만이고, BLOCKED 등에서는 막힘 처리가 따로 돈다. 늘 12초 뒤 풀린다는 보장은 아니다.
  4. [확정] 끈 길에서 한 goal에 물러나기 링크가 여럿이면 새 조건이 걸린다. 지도 다섯 개 모두 이어진
     물러나기 링크가 없어 지금은 해당 없음(직접 확인).
  5. 시험이 1초만 멈추고 틱 순서를 환경 변수로 고른다는 지적: 두 틱 순서 모두 직접 돌려 통과했다.
     마당 시험이 취약할 수 있다는 [추측]: 두 틱 순서 모두 통과.
- d956fe5(strict xfail 시험) 리뷰: 도착해서 읽음. [확정] xfail이 앞 단언 실패를 가릴 수 있다는 지적이
  실제로 맞았다(4절). ba297be에서 고쳤다. "정책 선택만은 아니다, 다음 길이 도크에 다시 다가갈 때만 노드
  좌표를 막는 국소 수정 후보"라는 [추측]은 6절 1의 후보와 같다.
- d12c6cb(이 기록 첫판) 리뷰: 도착해서 읽음. 표현이 실제 조건보다 강하다는 [확정] 지적 넷(위치 확인 없이,
  노드를 지나가면, 일찍 놓을 길이 없다, 12초 뒤 막힘 없음)을 직접 확인하고 3절과 5절의 표현을 고쳤다.
  시험 범위가 좁다는 지적(1초 정지, yard 시험은 놓는 순서를 단언하지 않음)은 맞다. 남은 점에 더한다.
- ba297be(시험 나눔) 리뷰: 커밋 직후 시작했다.
- 이 기록 고친 판 리뷰: 커밋 직후 시작한다.

커밋: d00449b(수정과 시험), d956fe5(strict xfail 시험), d12c6cb(이 기록), ba297be(xfail 앞 조건 나눔),
이 기록 고친 판.

## 6. 남은 점

1. **끈 길에서도 생기는 조기 놓기(물러나기를 끝낸 뒤의 늦은 보고)** — strict xfail 시험
   `test_a_late_report_at_the_node_after_backing_out_does_not_free_the_dock[order_off|order_on]`.
   origin/main(order 끔)을 부하 아래에서 돌린 9개 시나리오에서 다섯 번 나왔다(코디네이터 측정). 모두
   따로 보낸 RetreatCmd(W→N_W, O→N_E)가 끝난 뒤였다: `_segment_done`이 `m.node`를 N_W로 두고, 0.6~0.8초
   안에 다음 MoveCmd가 나가 로봇은 x 0.35를 따라 X_W 쪽으로 올라가는데, 보고가 늦어 N_W 0.05 m 안에
   머물면 노드 좌표(0.149 m)로 재어 WORK_DOCK을 놓았다(참 0.065~0.080 m, 보고 0.076~0.104 m).
   이 다음 길은 작업 도크 구역에서 0.05 m를 지나므로, 보고 위치로만 재면 X_W를 지나 급행으로 들어간
   뒤에야 놓는다(도크에 들어오려는 로봇이 몇 초 더 기다림). 어느 쪽을 택할지(판정 반경 0.08 m를 지킬지,
   처리량을 지킬지)는 정책 결정이라 고치지 않았다. 후보: 다음 goal의 길이 그 구역에 release gap보다
   가까이 지나면 노드 좌표를 쓰지 않는다(O_OUT에 못 미쳐 선 경우는 다음 길이 구역에서 멀어지므로 그대로).
2. **FOLLOW_PATH 링크도 같은 모양**: v7의 O → O_OUT(출고 도크에서 나오는 고정 경로)이 order goal의
   마지막 링크가 아니면, 달리는 중 O_OUT 0.025 m 앞에서 `_track`이 넘기고 노드 좌표로 재어 OUT_DOCK을
   보고 0.12 m에서 놓는다. 끄면 결과가 올 때까지 쥔다. 하네스로 확인함:
   `order 0.025 0.12 ['HALL'] O_OUT` / `off 0.025 0.12 ['OUT_DOCK', 'HALL'] O`. 판정 반경(0.08 m) 아래로
   놓은 시험은 아직 없고, 과제가 FOLLOW_PATH 진행을 그대로 두라고 해서 고치지 않았다. 같은 규칙
   (끝 노드에서 release_margin_m 안이면 답을 기다림)을 FOLLOW_PATH에도 넓히면 막힌다.
3. Codex 지적 1, 2(완료 답이나 도착 추정이 노드를 정한 뒤의 노드 좌표, 옆 잡음으로 넘긴 뒤 다시 들어옴).
4. world 시험 `test_the_yard_back_off_does_not_stand_a_robot_on_a_node_it_does_not_hold[order_burst-12-True]`의
   씨드를 바꿔야 한다(4절). order_burst, d2, yard_backoff 켬, order 켬, avoid none으로 씨드 1~24를
   돌렸는데 HALL_BACKOFF_DONE을 쓰는 씨드가 없었고, 불변식 실패도 없었다(출력: `done`만 나옴).
   다른 시나리오에서 켠 길의 물러나기를 찾거나 그 매개변수를 끈 길 둘로 줄이는 판단이 필요하다.
5. ROS 시나리오(test_scenario_integration.py, test_fms_restart.py)는 이 작업에서 돌리지 않았다(코디네이터 몫).
6. 새 시험의 범위가 좁다: 노드 앞에서 1초만 멈춰 보고, 여러 잡음 방향이나 긴 지연은 보지 않는다.
   yard 시험은 J_E를 RETREAT_DONE 뒤에 놓는 순서까지는 단언하지 않는다.
7. 늦은 보고 시험은 현장 상태를 재현하지만(보고를 멈추고 몸을 옮김) 현장의 명령 흐름을 그대로 달리지는
   않는다. 하네스에는 잡음이 없어, 그냥 달리면 N_W에서 정당하게 놓는다(참 거리 0.115 m 이상, 직접 잼).
