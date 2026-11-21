# 출고 출구 고정 경로를 keepout 띠 안에서 끊던 문제 (2026-11-21)

브랜치 `fix/exit-cancel-keepout-2026-11-21`, 기준 `origin/main` = a6265b0.

## 일어난 일

Gazebo 묶음(v7 배치, 48판) 가운데 한 판에서 pode2가 출고 도크 바로 아래에서 갇혀 격리됐다.

- pode2는 order pode2-29로 주차하러 가고 있었다(base: seq1 follow_path O -> O_OUT(out_out),
  seq2 nav_goal E-1, seq3 retreat P-1). out_out을 가는 도중 WORK 호출이 왔다.
- `calls.py` `_head_for`: 지금 방식이 follow_path라 route를 자르고 `route.stale = True`.
- `move_orders.py` `_extend_order`의 stale 갈래는 follow_path를
  `at_yard_node = mode != RETREAT and _in_yard(m) and _on_node(m)`일 때 끊는다.
- 시뮬 82.624의 보고 위치 (0.181, 0.762). HALL 사각형 `[0.0, 0.63, 1.98, 0.765]`
  (`fleet_map_v7.yaml:87`) 안이라 `_in_yard`가 참. 로봇의 `m.node`는 아직 출발 노드 O이고
  O (0.285, 0.868)에서 0.148이라 `_on_node`(한도 `arrival_tolerance_m` 0.15)도 참.
- 기록: 82.7에 `CancelCmd goal 29`, 82.8에 새 order pode2-30, base는 (0.179, 0.754)에서
  시작하는 Q2행 nav_goal.
- Nav2 keepout 마스크는 OUT_DOCK을 0.06 넓힌다(y 0.705까지 막힘). out_out은 follow_path라 열린
  틈도 없다(`dock_openings`). `arena_v7_keepout.pgm`에서 (0.175, 0.752), (0.19, 0.741)은 막힌 칸,
  O_OUT은 열린 칸. 플래너가 그 자리에서 42번 실패했다.

즉 y 0.705~0.765 띠에서 관제는 "넓은 구역의 노드 위"로 보고, Nav2는 "keepout 안"으로 본다.
O에서 0.15 안이면서 HALL 안인 out_out 위 구간은 y 0.761~0.765, 약 4 mm뿐이라 드물다(48판에 한 번).
보통은 `STANDBY_LEFT` 또는 `ORDER_UPDATED update 1, stitch O_OUT`으로 다음 nav_goal이 O_OUT에서
시작한다.

## 고른 규칙

stale 갈래가 고정 경로를 끊는 조건에 하나를 더한다: 로봇이 가까이 선 노드(`m.node`)도 넓은
구역(SHARED) 노드여야 한다(`MoveOrders._yard_node`). 로봇 위치가 HALL 안이어도 그 노드가 도크
노드(O)면 끊지 않고, 고정 경로가 끝까지 가서 O_OUT에서 다음 길을 받는다.

고친 뒤 시험 세계에서: 띠 안에서는 취소가 없고, 취소와 nav_goal로 시작하는 새 goal은 로봇이
keepout 밖에 나온 뒤에야 나간다(out_out을 지나 다음 조각이 nav_goal이 되면 stale 갈래가 nav_goal로
보고 끊는다).

### 다른 후보와 고르지 않은 까닭

1. **도크를 떠나는(또는 모든) 고정 경로는 조각 끝까지 둔다.** 넓은 구역 노드에서 끊는 지금
   동작(여러 노드를 지나는 고정 경로)도 함께 사라진다. 지금 동작을 다른 곳에서 바꾸지 않는다는
   조건에 어긋난다.
2. **로봇 위치가 keepout(구역 + 0.06) 밖일 때만 끊는다.** 먼저 이것으로 짰다
   (`fleet_map.keepout.keepout_rects`를 씀). 띠 사고는 막았지만 기존 시험
   `test_zone_edges.py::test_a_purpose_changed_and_changed_back_still_arrives_at_the_dock[order_on]`이
   깨졌다: N_W를 지나 W로 들어가는 고정 경로 위(WORK_DOCK keepout 띠 안, 노드 N_W는 HALL)에서
   취소하고 같은 고정 경로를 다시 보내는 동작이 바뀌었다. 다시 보내는 것이 follow_path라 Nav2
   플래너가 필요 없는 경우까지 막는다.
3. **새 order를 고정 경로 끝(O_OUT)에 잇기(standby처럼).** 맞는 방향이지만 stale route의 끝을
   다시 짜고 update를 보내는 갈래가 새로 들어가 바뀌는 곳이 크다.

고른 규칙은 "노드 위에 섰다"의 뜻을 바로잡는다: 이 갈래가 노리는 것은 넓은 구역의 노드에 선
로봇인데, 출발한 도크 노드에서 0.15 안에 있다는 것으로 그 노드 위로 쳤다. 도크 노드는 둘레가
keepout이고 넓은 구역 노드는 Nav2 goal 자리라 keepout 밖에 두게 지도를 짰다(`fleet_map_v7.yaml`의
O_OUT 설명). 지도 모양과 keepout 생성기는 건드리지 않았다.

## 시험

- 새 시험 `pc/fleet_core/test/test_zone_exit_band.py`
  `test_a_call_inside_the_dock_keepout_band_does_not_cut_the_exit_path[order_off/order_on]`:
  pode1 하나가 출고를 끝내고 주차하러 out_out을 가다가 (0.181, 0.762)에 있을 때 WORK 호출이 온다.
  그 한 걸음만 로봇을 세워 관제가 그 자리를 보게 한다(앞선 조건으로 `_in_yard`, `_on_node`, keepout
  안을 확인). 그 뒤 W에 닿을 때까지 로봇이 keepout 안에 있을 때 CancelCmd나 nav_goal로 시작하는
  새 goal이 나가지 않는지 본다.
  - 고치기 전: order_on 실패(`CancelCmd goal 2`가 (0.181, 0.762)에서 나감), order_off 통과(order를
    쓰지 않으면 이 갈래를 지나지 않는다. 지킴 시험으로 둔다).
  - 고친 뒤: 둘 다 통과.
- fleet_core 전체와 fleet_manager 전체 결과는 보고에 적는다.

## 남은 것

- 비슷한 띠가 다른 도크 노드에도 있을 수 있으나 이 규칙은 노드가 도크 노드이면 모두 막으므로
  같은 꼴은 함께 막힌다. 반대로, 넓은 구역 노드에서 0.15 안이면서 다른 도크의 keepout 띠 안인
  자리(예: O_IN (0.49, 0.625) 위쪽, OUT_DOCK에서 0.14)에서 고정 경로를 끊고 nav_goal을 주는 경우는
  그대로다. 지금 out_in 위의 로봇은 호출로 목적이 바뀌지 않아(`_redirect`는 TO_OUTBOUND를
  돌리지 않는다) 이 갈래를 타지 않는다.
- Gazebo로 다시 확인하지 않았다(다른 작업이 GPU를 쓴다).
