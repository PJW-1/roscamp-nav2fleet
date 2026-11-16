# 작업 기록: 위치가 튄 뒤 구역 붙잡기(suspect_hold = check) (2026-11-16)

브랜치 `feat/pose-jump-hold`. `feat/order-default-on-2`(order 모델 기본 켬)의 5eecfd1 위로 rebase했다(시작은 01dbd12). 푸시와 병합은 하지 않았다.

## 1. 결론

- 사용자가 고른 고침 "B"를 넣고 출하 파라미터에서 켰다(`suspect_hold: check`). 보고 위치가 튀거나 쥔 구역 밖에 머문 로봇은 의심하고, 의심하는 동안 쥔 구역을 하나도 놓지 않는다. 로봇은 base 끝까지 가고 거기서 위치를 확인한다.
- order를 켠 `pose_jumps`에서 I3_OUTSIDE_HELD(서 있는 구역을 남에게 줌)가 없어졌다. 전에는 strict xfail이었다. fake_robots의 27칸 가운데 21칸이 이제 그대로 통과한다.
- 대가: 배치 d의 씨드 1, 3(회피 모드 셋 모두, 6칸)은 대기 자리에서 보고 위치가 20 s 틀린 채로 남아, base 끝 확인이 `NOT_AT_TARGET` 세 번 뒤 그 로봇을 격리한다. 주문 6개 중 1개만 끝난다. 설계대로이며(틀린 위치를 오래 믿지 않음), 그 칸은 이유를 적은 strict xfail로 두었다.
- 프로토타입(`exp/order-midcheck` 4eedcaf)의 "A"(order_end_check)는 옮기지 않았다.

## 2. 무엇이 바뀌었나

| 커밋 | 내용 |
|---|---|
| 0c86fab | `suspect_hold` 키. 값은 `""`(끔)과 `check`. `OPTIONAL`에 넣어 이 키가 없는 파일과 기록도 읽힌다(코드 기본값 `""`). `OPTIONAL`의 뜻은 "파일에 없어도 되는 키"로 바꾸고 시험을 그 뜻으로 고쳤다(출하 파일이 빼는 키는 여전히 `zone_clear_m` 하나) |
| f5b2815 | B 본체. `doubts.py`(믹스인 Doubts), `robots.py`(`doubt`, `doubt_by`), `controller.py`(믹스인 연결), `progress.py`(튐과 구역 밖 감시를 놓기 앞에서, 의심 중 놓지 않음), `move_orders.py`(의심 중 order를 늘리지 않고 base 끝에서 확인), `results.py`(`_segment_done`이 의심을 풂), `base.py`(`_fail`이 의심을 `pose_doubt`로), `watch.py`(구역 밖 횟수가 차면 의심), `localization.py`(튄 까닭을 돌려줌). 시험 `test_zone_doubts.py`를 먼저 썼다. `zone_harness.py`에 로봇별 보고 위치 치우침(`bias`)을 넣었다 |
| 08e2b51 | 출하 파일 `suspect_hold: check`. 바뀐 시험 기대(아래 4절) |
| 4f5bbd5 | Codex 리뷰 고침: `LOCALIZING` 중에는 확인하지 않음, 확인 기한을 goal마다 새로(`doubt_by = (goal id, 기한)`), 도킹 성공 때 보고 위치가 도크 지점에서 `arrival_tolerance_m` 안이면 의심을 풂 |
| fd83a0a | yaml 주석: `OUTSIDE_ZONE`, `LOCALIZATION_SUSPECT`가 "기록만"이 아니게 됨 |
| 375612d, 76eb239 | `pose_jumps` order 켬 칸의 기대를 B에 맞춤(아래 4절). 못 끝낸 실행은 의심 중 `NOT_AT_TARGET`으로 격리된 로봇이 있을 때만 받는다 |
| (이 문서) | `docs/관제_구역교통.md` 규칙 10, 놓는 때 표, 사건 표. `docs/ZoneTraffic_구조.md` 믹스인 표 |

동작(켬):

1. 의심 걸기: `LOCALIZATION_SUSPECT`가 난 보고, 또는 쥔 구역에서 `outside_zone_m` 넘게 벗어난 보고가 `outside_zone_samples`번 이어질 때. 사건 `ZONES_HELD_SUSPECT`(`<까닭>: keeps [...]`). 이미 의심 중이면 다시 걸지 않는다. 고장이나 통신 끊김 중에는 걸지 않는다.
2. 의심 중: `_release_behind`가 아무것도 놓지 않는다. order는 늘리지 않는다(이미 받은 base 끝까지만 간다). 멈추지 않고 운영자 단계도 없다.
3. base 끝에서 기다릴 때(`waiting_at_base_end`) 보고 위치가 goal 끝에서 `arrival_tolerance_m`(0.15 m) 안이면 `DOUBT_CLEARED`. 아니면 `arrival_settle_s`(1 s) 뒤 goal을 취소하고 `NOT_AT_TARGET`으로 실패(재시도 `move_retry_limit`번, 그다음 `ROBOT_ISOLATED`).
4. 이동 goal의 성공(`_segment_done`: 닿음 확인을 거친 성공, `GOAL_ENDED_AT_TARGET` 포함)도 의심을 푼다. 도킹 성공은 보고 위치가 도크 지점 가까이일 때만 푼다.
5. goal 없이 서 있을 때 걸린 의심은 풀 기회가 없다. 다음 goal의 base 끝이나 성공에서 확인한다(그때까지 놓지 않는다). 다시 움직이면 반드시 확인 지점(base 끝 대기, goal 성공, 실패와 재시도)에 닿으므로 영영 붙잡지는 않는다. 시험: 서서 튄 뒤 새 goal, 갱신 답을 기다리는 중 튐.
6. 격리되면 의심은 `pose_doubt`가 되어 `ZONES_RETURNED`도 하지 않는다.
7. 켜면 구역 밖 감시가 놓기 뒤에서 앞으로 옮겨진다(같은 보고로 놓기 전에 의심을 건다).

## 3. 스위치

- `pc/fleet_core/fleet_core/data/fleet_params.yaml`의 `suspect_hold`. `check`(출하값) 또는 `""`(끔, 예전과 같음).
- 파일에 키가 없으면 `""`. `tools/replay.py`는 기록의 파라미터로 `FleetParams`를 바로 만들므로 이 키가 없는 옛 기록은 끈 채로 재생한다. 프로토타입으로 만든 기록(A의 키 `order_end_check` 등)은 지원하지 않는다(재생이 "these parameters do not fit this code"로 거절).
- fleet_manager는 새 ROS 파라미터가 없다. `params_file`로 yaml을 읽는다.

## 4. 바뀐 시험 기대와 이유

- `test_params_validate.py`: `OPTIONAL == {"zone_clear_m", "suspect_hold"}`, 출하 파일이 빼는 키는 `{"zone_clear_m"}`이고 그것이 `OPTIONAL` 안에 든다. 코드 기본값과 파일 값이 다른 키에 `suspect_hold`(`""` 대 `check`)를 더했다(옛 기록은 끔, 파일은 B).
- `test_zone_edges.py` 셋(들어 옮긴 로봇, Nav2가 다른 마당 구역으로 데려감, 찬 마당 구역)과 `test_zone_situations.py`의 구역 밖 튐: 끔/켬 둘로 돌린다. 켜면 구역 밖 횟수가 먼저 의심을 걸어 `ZONES_HELD_SUSPECT`가 `OUTSIDE_ZONE` 앞에 온다. 들어 옮긴 로봇은 켜면 `WORK_DOCK`과 지점 W를 쥔 채 남는다(goal이 없어 아직 확인할 기회가 없다). 끄면 예전처럼 돌려준다.
- `sim/fake_robots/test/situation_helpers.py`, `test_situation_faults.py`, `test_situation_avoid_modes.py`: order 켬 `pose_jumps`의 strict xfail(`PoseJumpsOpen`, I3)을 지웠다. 이제 I3를 포함한 불변식은 늘 지켜야 한다. 주문 못 끝냄은 `NOT_AT_TARGET` 격리가 있었을 때만 `PoseJumpsIsolated`로 내고, 그런 칸(d-1, d-3)만 strict xfail이다.

## 5. 시험 결과 (나온 그대로)

마지막 커밋(76eb239, rebase 뒤) 기준. ROS가 닿는 것은 `ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

| 명령 | 결과 |
|---|---|
| `ZONE_TICK_ORDER=manager PYTHONPATH=pc/fleet_core python3 -m pytest -q -p no:cacheprovider pc/fleet_core/test` | 1106 passed, 1 skipped, 3 xfailed |
| 같은 것, `ZONE_TICK_ORDER=zone` | 1106 passed, 1 skipped, 3 xfailed |
| `cd pc/fleet_manager && python3 -m pytest -q -p no:cacheprovider $(ls test/test_*.py \| grep -v -e test_scenario_integration -e test_fms_restart)` (내 임시 폴더에 colcon으로 지은 fleet_interfaces, fleet_core, fleet_manager, fake_robots, pode_fsm을 source) | 113 passed |
| `python3 -m pytest -q -p no:cacheprovider tools/test` | 96 passed |
| `python3 -m pytest -q -p no:cacheprovider sim/fleet_gz_sim/test` | 406 passed, 17 skipped |
| `python3 -m pytest -q -p no:cacheprovider sim/fake_robots/test` | 901 passed, 6 skipped, 6 xfailed (7분 41초). xfail 6칸은 d-1, d-3의 order 켬 pose_jumps(perfect, none, mixed) |

중간에 나온 것:

- B를 켜기만 하고(fake_robots 기대를 고치기 전) 돌린 fake_robots: 27 failed, 880 passed, 6 skipped. 21칸은 XPASS(strict)(I3가 없어져 옛 xfail이 통과), 6칸은 `finished 1/6 by 900 s`(d-1, d-3).
- 튐 창 시작을 14~24 s로 옮긴 측정(`jumpshift_b.py check`, order 켬, 배치 d2/d/c x 씨드 1~3, 54 실행): **I3 0/54, 끝낸 주문 269/324**. 못 끝낸 실행: 16 s d-2, 18 s d-1/d-2/d-3, 20 s d-1/d-3, 22 s d2-2/d2-3, 24 s d2-1/d2-2/d2-3(모두 1/6). 프로토타입 측정(I3 0/54, 269/324)과 같다. 고치기 전(order 켬, B 없음)은 I3 45/54(앞 작업의 측정).
- 하네스에서 같은 틀린 보고 위치(-0.3 m)로 도크가 비어 O까지 가는 경우: 끄면 O에서 OUT_DOCK을 놓아 I3, 켜면 없음(`test_the_same_wrong_pose_frees_a_zone_the_robot_is_in_only_with_the_key_off`).

## 6. 돌리지 않은 것

- Gazebo, 실물 로봇. `test_scenario_integration.py`, `test_fms_restart.py`(도메인 50~59를 씀, 사용자가 57에서 작업 중).

## 7. 열린 점

1. **아는 약한 경우**: 위치 추정이 틀린 채로 base 끝에 "닿았다"고 하는 로봇. AMCL이 틀린 추정으로 로봇을 몰면, 로봇은 틀린 추정 기준으로 goal 끝에 서고 보고 위치도 goal 끝이다. B는 보고 위치만 보므로 의심을 풀고 다시 놓는다. 몸은 다른 곳에 있다. 이것을 잡으려면 보고 위치가 아닌 다른 근거(비전 위치, 도크 마커)가 필요하다.
2. **격리 비용**: 보고 위치가 base 끝에서 15 s(재시도 세 번 x (확인 1 s + 재시도 대기 2 s) 남짓)보다 오래 틀리면 로봇을 격리한다. fake_robots d-1, d-3에서 주문 6개 중 1개만 끝난다. 사용자 우선순위(성능 우선)와 부딪칠 수 있다. 줄이려면 확인 실패 뒤 재시도 대기를 늘리거나 격리 전에 위치가 돌아오기를 기다리는 정책이 필요하다(사용자 결정).
3. **구역 밖 보고로 재시도를 못 짬**: `NOT_AT_TARGET` 재시도는 보고 위치에서 길을 짠다. 보고 위치가 공유 구역 밖이고 지점 위도 아니면 길이 없어 `ROBOT_IDLE`만 되풀이하고 격리되지도 않는다(구역은 쥔 채). 끈 관제에서도 같다(`_way`, goals.py). 하네스에서 보고 위치를 0.3 m 위로 치우치면 재현된다.
4. **서서 걸린 의심**: goal 없이 선 로봇은 다음 goal까지 쥔 구역을 놓지 않는다. 도크에 선 로봇이 도킹 성공 때 위치가 어긋나 있었으면 스테이션 일 내내 쥔다(Codex [확정] 지적, 정책으로 남김). 그 뒤 위치가 돌아와도 다시 보지 않는다.
5. **확인 기한과 LOCALIZING**: `LOCALIZING` 중에는 확인하지 않지만 기한은 흐른다. 다시 보고할 때 기한이 지났고 위치가 아직 어긋나 있으면 바로 `NOT_AT_TARGET`이다(Codex [확정], 그대로 둠).
6. 도킹 성공 확인은 마지막 정상 보고의 위치를 쓴다. 그 보고가 오래됐는지는 보지 않는다(Codex [확정], 그대로 둠).
7. fake_robots의 xfail 칸은 "의심 중 `NOT_AT_TARGET`으로 격리된 로봇이 있었다"까지만 본다. 그 격리가 주문을 못 끝낸 원인인지는 보지 않는다(Codex [확정], 76eb239 리뷰). 재투입 뒤 새 의심 없이 격리되는 경우를 앞 의심으로 칠 수 있다(Codex [추측], pose_jumps에는 재투입이 없다).
8. 켬과 끔에서 구역 밖 감시의 순서가 다르다. 놓기와 구역 밖 횟수는 같은 보고에서 서로 겹치지 않으므로(구역 밖이면 놓기도 막힘) 차이는 `_take_where_it_stands`가 같은 보고에서 잡는 구역뿐이다. 이 순서를 따로 고정하는 시험은 없다.
