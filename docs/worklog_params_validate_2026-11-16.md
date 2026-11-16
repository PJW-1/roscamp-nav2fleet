# 관제 파라미터 검증과 PC·로봇 안전 관계 (2026-11-16)

브랜치 `feat/params-validate` (origin/main 7c7d04b에서).

## 결론

- `load_params`가 이제 yaml을 검사한다. 빠진 키, 모르는 키, 매핑이 아닌 파일은 키 이름을 적은
  `ValueError`다. 일부러 빼는 키는 `OPTIONAL = {"zone_clear_m"}` 하나뿐이다.
- 이름이 `_s`, `_m`, `_mps`, `_pct`로 끝나는 값(시간·거리·속도·백분율)은 유한하고 음수가 아니어야
  한다. 0은 `ZERO_MEANS_OFF = {zone_clear_m, idle_watch_s, silent_limit_s, start_limit_s,
  battery_low_pct}`에서만 허용하고, 나머지는 0보다 커야 한다. `_pct`는 100 이하.
- 코드 기본값: `stall_s`만 6 → 12로 yaml과 맞췄다. `idle_watch_s`, `silent_limit_s`,
  `start_limit_s`의 기본값 0은 **그대로 뒀다**. 이유는 아래 "의도된 차이".
- PC·로봇 시간 관계(`stall_s ≥ blocked_after_s + 2`)는 `sim/fleet_gz_sim/test/test_timing_agreement.py:21`이
  이미 본다. 중복 시험은 만들지 않았다. 이번 변경으로 params_file에서 stall_s가 빠지는 길(6 s가 조용히
  쓰이던 길)이 막혔다.
- 해제·후진 관계는 시험으로 고정하지 않았다. 두 식이 서로 다른 기준(구역 다각형 거리 vs 경로 꼭짓점에서의
  경로 거리)을 써서 파라미터만으로 정해지는 관계가 없다. 아래 "해제·후진".

## 바꾼 파일

- `pc/fleet_core/fleet_core/common/params.py`: `OPTIONAL`, `ZERO_MEANS_OFF`, `MEASURE_SUFFIXES`,
  `check_measures()` 추가, `load_params` 검사 추가, `stall_s` 기본값 12.0, 세 감시 기본값에 기록 호환 주석.
- `pc/fleet_core/test/test_params_validate.py`: 새 시험 38개(매개변수화 포함).
- 이 문서.

## yaml에 빠진 키와 판단

FleetParams 필드와 `fleet_params.yaml` 키 비교 결과: 빠진 키 `zone_clear_m` 하나, 모르는 키 없음.

- `zone_clear_m`: 의도. `test_zone_release.py:17`이 yaml에 없음을 고정하고, `progress.py:26`의
  `release_gap`은 0이면 `footprint_radius_m + release_margin_m`를 쓴다. `zone_clear_m`을 가진 옛 기록은
  그 값으로 놓는다(`test_zone_release.py:46` 등).

## 의도된 차이(코드 기본값 ≠ yaml)

| 키 | 코드 | yaml | 판단 |
|---|---|---|---|
| stall_s | 6 → **12** | 12 | 맞춤. 6은 Pode BLOCKED(10 s)보다 짧아 위험 |
| idle_watch_s | 0 | 10 | 유지. 0=끔, 키 없는 옛 기록을 감시 없이 재생 |
| silent_limit_s | 0 | 60 | 유지. `test_zone_silent.py:30`이 "records without it run without the watch"로 고정 |
| start_limit_s | 0 | 5 | 유지. `test_zone_no_start.py:34`가 같은 이유로 고정 |

처음에는 네 값을 모두 yaml에 맞췄고 fleet_core 전체 시험에서 위 두 시험이 실패했다(2 failed, 781 passed).
기본값은 `tools/replay.py:97`의 `FleetParams(**head["params"])`처럼 키 없는 옛 기록을 재생할 때만 쓰이므로
(운영 파일은 이제 `load_params`가 빠진 키를 막는다) 세 감시는 0으로 되돌렸다. `idle_watch_s`는 고정한 시험이
없지만 같은 모양(알리기만 하는 감시, 0=끔)이고 SILENT_STALL 판정에 영향을 주므로 같이 뒀다.
`test_code_defaults_match_the_shipped_file_except_record_compat_watches`가 이 차이를 정확히 세 키로 고정한다.

`FleetParams(**fields)`로 직접 만드는 시험(`test_traffic_mode.py:29`, `test_zone_release.py:49,143,152`)은
`asdict(load_params())`에서 시작해 모든 필드를 넘기므로 기본값 변경의 영향이 없다.

## 해제·후진

- 관제(`traffic/zone/progress.py:25-26`, :103-110): 보고 중심이 뒤 **구역 다각형**에서
  `footprint_radius_m + release_margin_m`(0.135 m) 넘게 떨어진 보고가 `release_confirm_samples`번 이어지면
  놓는다. 지점은 `node_clear_m`(0.20 m).
- Pode(`pode/pode_fsm/pode_fsm/recovery.py:48-50`, `backup.py:33`): 후진 거리 =
  `min(backup_max_m, 마지막 경로 꼭짓점부터 경로를 따라 온 거리 - (release_distance_m + backup_margin_m))`.
  중심을 마지막 **경로 꼭짓점**에서 0.23 m 앞에 둔다.
- 경로 꼭짓점이 관제 지점과 같다는 보장도, 구역 경계가 꼭짓점에서 얼마인지도 파라미터에 없다(지도 기하에
  달렸다). 그래서 "로봇이 물러난 뒤에도 관제가 놓은 구역 밖에 있다"를 파라미터만으로 쓸 수 없다. 시험을
  지어내지 않았다. 확인하려면 v7 지도의 각 구역 경계와 경로 꼭짓점 거리를 재는 시험이 필요하다.
- 참고: `release_distance_m`(yaml:12)은 구역 교통이 읽지 않는다(yaml 주석). 이름만 공유하는 값이다.

## 시험 명령과 결과

- `PYTHONPATH=pc/fleet_core python3 -m pytest pc/fleet_core/test/test_params_validate.py -q`
  - 시험 먼저: 28 failed, 10 passed (상수만 둔 상태). 고친 뒤 38 passed.
- `PYTHONPATH=pc/fleet_core python3 -m pytest pc/fleet_core/test -q`
  - 네 기본값을 모두 바꿨을 때: 2 failed, 781 passed (test_zone_no_start, test_zone_silent의 기록 호환 시험).
  - 최종: 783 passed (36.5 s).
- `ROS_DOMAIN_ID=64 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONPATH=pc/fleet_core:pode/pode_fsm:sim/fake_robots:sim/fleet_gz_sim python3 -m pytest sim/fleet_gz_sim/test/test_timing_agreement.py sim/fake_robots/test/test_world_invariants.py sim/fake_robots/test/test_world_reaction.py -q`: 24 passed.
- `pc/fleet_manager/test/test_order_goals.py` (ROS 환경, 도메인 64): 4 passed.

## 돌리지 않은 것

- `pc/fleet_manager/test/test_scenario_integration.py`, `test_fms_restart.py` (도메인 50-59를 직접 씀, 지시로 제외).
- Gazebo, `sim/fake_robots`의 나머지 ROS 시험, `pode/pode_fsm` 시험(바꾼 파일이 없음).

## 남은 위험

- 0을 금지한 시간·거리 키 가운데 0이 정당할 수 있는 값(예: `arrival_settle_s`, `yard_backoff_wait_s`)이
  있다. 지금 yaml은 모두 0보다 크다. 0을 쓰려면 `ZERO_MEANS_OFF`에 넣고 코드가 0을 다루는지 확인해야 한다.
- 정수 키(`basket_capacity`, `release_confirm_samples` 등)와 문자열 키(`traffic_mode`)는 값 검사를 하지 않는다.
- `FleetParams(**fields)`를 직접 쓰는 길(replay, 시험)은 `check_measures`를 거치지 않는다.
- 해제·후진의 기하 관계는 시험으로 고정되지 않았다.
