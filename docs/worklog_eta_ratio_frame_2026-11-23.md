# ETA 순항 속도 비율: 틀만 (2026-11-23)

ETA 결정 E1(`docs/worklog_eta_cause_2026-11-22.md` 5절, 브랜치 `feat/eta-cause-2026-11-22`)의 틀이다.
사용자 결정 2026-11-23: 틀만 만들고, 값은 제어기와 지도가 정해진 뒤 실측해서 넣는다.
11-23 실주행에서 로봇은 0.102 m/s로 달렸고 관제는 `nav_speed_mps` 0.12로 잡아 ETA가 이르다.

## 바꾼 것

- 새 파라미터 `nav_eta_speed_ratio` (기본 1.0)
  - `pc/fleet_core/fleet_core/common/params.py`: 필드와 검사. 0 초과 1 이하인 유한한 수만 받는다
    (불리언, 문자열, nan, inf 거절). 곱 `nav_speed_mps x 비율`이 0으로 언더플로하는 아주 작은 값(5e-324)도 거절한다
  (Codex 지적). 검사는 `__post_init__`에 있어 파일 읽기와 `replace` 둘 다 막는다.
  - `pc/fleet_core/fleet_core/data/fleet_params.yaml`: `nav_eta_speed_ratio: 1.0`과 짧은 설명.
  - 끝말이 `_ratio`라 시간·거리 검사(`MEASURE_SUFFIXES`) 대상이 아니고, 따로 검사한다.
  - 옛 기록에는 이 키가 없다. 코드 기본값 1.0으로 재생되어 예전과 같다.
- `pc/fleet_core/fleet_core/traffic/zone/routes.py`
  - `speed()`는 그대로 둔다. 길 비용(route 고르기)용이다.
  - 새 `eta_speed()`: Nav2 goal이면 `nav_speed_mps x nav_eta_speed_ratio`, 다른 모드는 `speed()` 그대로.
  - `travel_s`, `left_s`, `nav_end_s`(넘어가는 거리 `nav_pass_m` 부분)가 `eta_speed()`를 쓴다.
- `pc/fleet_core/fleet_core/traffic/zone/order_answers.py`: `_left_s`에서 남은 거리(distance_remaining)를
  시간으로 바꾸는 두 곳이 `eta_speed()`를 쓴다.

## 비율이 들어가는 곳 (모두 `_left_s` 또는 `left_s`/`travel_s`/`nav_end_s`를 거친다)

| 쓰는 곳 | 무엇 |
|---|---|
| `routes.left_s`, `routes.travel_s`, `routes.nav_end_s`(PASS의 넘어가는 거리) | 남은 시간 계산 바탕 |
| `order_answers._left_s` | 남은 거리 보고를 시간으로, 투영, 끝 보정 |
| `move_orders` 118행 `left <= standby_lookahead_s` | 대기 지점 앞보기(update 보낼 때) |
| `move_orders` 126행 `_left_s(...) > lookahead_s` | horizon 앞보기 |
| `goals._take` 170행 `eta[lane]`(ARRIVE) → `lane_policy` | 차선 들어가기 순서 ETA 비교 |
| `order_answers._hold_base` | 통신 끊김 때 base 끝까지 구역 쥐는 시간 |

## 비율이 들어가지 않는 곳 (일부러)

- `goals._cost`: edge 비용 `price_m / speed()`와 차선 대기 `length / follow_speed_mps`. 길 고르기다.
- `goals._way`: 첫 임시 edge 비용 `first.length / nav_speed_mps`. 길 고르기다(대기 지점 고르기 포함).
- 고정 경로(`follow_speed_mps`)와 후진(`retreat_speed_mps`)의 ETA. `eta_speed()`가 두 모드는 그대로 둔다.
- 끝에서 도는 시간(`nav_turn_rps`). 속도가 아니라 각속도다.
- `nav_speed_mps` 자체의 출처는 바꾸지 않았다.

## 시험

- 새 시험 `pc/fleet_core/test/test_eta_speed_ratio.py` 36개
  - 기본값 1.0(코드, 파일), 받는 값과 거절하는 값(`replace`, 파일 읽기)
  - 비율 1.0이면 `eta_speed == speed`(모든 모드)
  - 비율 0.85: Nav2 goal만의 `left_s`/`travel_s`가 정확히 1/0.85배, 섞인 길은 Nav2 goal 부분만,
    고정 경로·후진은 그대로, 넘어가는 거리 보정은 1/0.85배, 끝 회전은 그대로
  - 관제 `_left_s`(PASS, ARRIVE, 남은 거리 보고 있음/없음): Nav2 goal 부분만 1/0.85배, 뒤 고정 경로는 그대로
  - `_hold_base`가 더 긴 ETA를 쓴다
  - 비율 0.5에서도 모든 edge의 길 비용과 고른 route가 같다. `goals._cost`에 비율을 넣으면 이 시험이
    실패하는 것을 확인했다(돌려 놓음)
- 기존 시험(비율 1.0): 결과는 아래 "시험 결과"
- `tools/eta_cause/eta_split.py`(위 브랜치)를 main 코드와 이 브랜치 코드로 같은 실기록 두 개
  (today, run9)에 돌려 출력과 `--rows` 표본이 바이트까지 같았다. `tools/replay.py`도 "same as recorded".
  나중에 비율을 1이 아닌 값으로 두면 `eta_split.py`의 `speed` 분해(`(used - adj) * speed`)는
  `nav_speed_mps x 비율`을 써야 한다.

## 시험 결과

- `cd pc/fleet_core && python3 -m pytest -q test`: 1269 passed, 7 xfailed
  (이 변경 전 시험 1233개 + 새 36개. 새 시험 없이 돌린 1차도 1233 passed, 7 xfailed)
- `tools/test` + `docs/distance_remaining_2026-11-21/test_eta_error.py`: 267 passed, 3 skipped, 2 failed.
  실패 2개는 `test_eta_error.py`의 옛 기대값(끝 보정 `nav_pass_m` 전)이라 main 코드로 돌려도 똑같이 실패한다.
  이 변경과 무관하다.
- `pc/fleet_manager/test`(install 환경 source, ROS_DOMAIN_ID 63): 102 passed, 34 skipped, 1 failed.
  실패한 `test_fms_restart.py::test_robots_stop_without_the_fms_and_a_new_one_waits_for_all_of_them`은
  fleet_core 전체 시험과 같이 돌 때 났고, 혼자 다시 돌리니 통과(1 passed). launch 시험은 install 폴더의
  노드를 띄워 이 브랜치 코드를 직접 타지 않는다.
- Codex 검토 1회: (1) 아주 작은 비율의 언더플로 → 고침, 시험 추가. (2) 노드 밖에서 출발할 때
  `goals._way` 첫 edge 비용을 지키는 시험이 없다 → 코드는 `nav_speed_mps`를 그대로 쓰고 이번에 손대지 않아 남김.
  고친 것의 재검토: 결함 없음.

## 내일 남은 것

- 비율 값 재기(실측 도구 포함). 제어기와 지도가 정해진 뒤.
- 고정 경로 속도(`follow_speed_mps`)도 같은 식으로 둘지.
- 길 비용(`travel_s`를 쓰지 않는 `goals._cost`)도 같은 속도를 쓸지(E1의 "그때 정한다").
- 통신 끊김 여유(`comm_base_hold_extra_s`) 바꿀지.
