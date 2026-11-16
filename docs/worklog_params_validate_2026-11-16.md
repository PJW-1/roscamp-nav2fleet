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

## 후속 커밋(Codex 리뷰 e0acac6 반영)

- 타입 검사 `check_types()`: 모든 숫자 키(int, float)는 불리언과 문자열을 거부하고, 실수 키는 유한해야
  한다. int 키에 2.5 같은 실수도 거부한다. 불리언 키는 불리언만, 문자열 키는 문자열만 받는다.
  실수 키에 정수(예: `stall_s: 12`)는 받는다.
- `release_confirm_samples`는 1 이상의 정수다. 0이나 음수면 `progress.py:145`(`_counted`)에서 첫 보고만으로
  구역을 놓는다.
- `eta_fill_s_per_item`은 접미사 규칙(`_s` 등으로 끝남)에 걸리지 않아 `NONNEGATIVE_EXTRA`로 따로 지정했다:
  유한, 0 이상.
- 옛 기록 재생: `tools/replay.py`에 `RECORD_DEFAULTS = {"stall_s": 6.0}`. stall_s가 없는 옛 기록은 그
  기록을 만든 코드의 기본값 6 s로 재생한다(코드 기본값을 12로 바꿔 재생 결과가 바뀌지 않게). stall_s가 있는
  기록은 그 값을 쓴다. 운영 경로(`load_params`)는 지금처럼 키가 빠지면 오류다.
- **고치지 않고 남긴 것**: 사용자가 준 파일에 `stall_s: 6`처럼 양수지만 Pode의 `blocked_after_s`(10 s) + 2보다
  짧은 값이 있으면 `load_params`를 통과한다. `blocked_after_s`는 계약(`interfaces/fleet_interfaces/contract.yaml`)에
  없어서 PC 쪽은 그 값을 알 수 없다. 숫자를 PC에 또 적는 대신 계약에 넣을지는 사용자가 정한다. 지금은
  출하 yaml만 `test_timing_agreement.py:21`이 본다.

## 바꾼 파일

- `pc/fleet_core/fleet_core/common/params.py`: `OPTIONAL`, `ZERO_MEANS_OFF`, `MEASURE_SUFFIXES`,
  `NONNEGATIVE_EXTRA`, `check_types()`, `check_measures()` 추가, `load_params` 검사 추가, `stall_s` 기본값 12.0,
  세 감시 기본값에 기록 호환 주석.
- `tools/replay.py`: `RECORD_DEFAULTS`(옛 기록의 stall_s 6.0).
- `pc/fleet_core/test/test_params_validate.py`: 새 시험(매개변수화 포함 278개).
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

- 관제(`traffic/zone/progress.py:25-26`, :103-110): 기준점과 거리가 세 갈래다(`zone_clear_m`이 0일 때).
  - 스테이션에 선 로봇(`AT_STATION`, 지점 위): 그 **지점 좌표**에서 `footprint_radius_m + dock_error_m`.
  - 지점에서 `release_margin_m` 안에 있는 로봇: 그 **지점 좌표**에서 `footprint_radius_m + release_margin_m`(0.135 m).
  - 그 밖: **보고 중심**에서 0.135 m.
  - 기준점이 구역 다각형에서 그 거리 넘게 떨어진 판정이 `release_confirm_samples`번 이어지면 놓는다(아직 갈 길에
    필요한 구역은 빼고). `zone_clear_m`이 있는 옛 기록은 늘 보고 중심에서 그 값이다. 지점은 `node_clear_m`(0.20 m).
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
- 후속 커밋:
  - `test_params_validate.py` 시험 먼저: 42 failed, 236 passed. 고친 뒤 278 passed.
  - fleet_core 전체: 1023 passed (34.8 s).
  - 위 sim 세 파일 + `tools/test/test_triage.py`: 55 passed.

## 돌리지 않은 것

- `pc/fleet_manager/test/test_scenario_integration.py`, `test_fms_restart.py` (도메인 50-59를 직접 씀, 지시로 제외).
- Gazebo, `sim/fake_robots`의 나머지 ROS 시험, `pode/pode_fsm` 시험(바꾼 파일이 없음).

## 남은 위험

- 0을 금지한 시간·거리 키 가운데 0이 정당할 수 있는 값(예: `arrival_settle_s`, `yard_backoff_wait_s`)이
  있다. 지금 yaml은 모두 0보다 크다. 0을 쓰려면 `ZERO_MEANS_OFF`에 넣고 코드가 0을 다루는지 확인해야 한다.
- 정수 키는 타입만 본다. 범위를 보는 것은 `release_confirm_samples`(1 이상)뿐이다(`basket_capacity`,
  `outside_zone_samples`, `clear_confirm` 등의 0이나 음수는 통과). 문자열 키는 `comm_loss_policy`만 값을 본다.
- `FleetParams(**fields)`를 직접 쓰는 길(replay, 시험)은 `check_types`, `check_measures`를 거치지 않는다.
- `stall_s: 6` 같은 사용자 파일은 통과한다(위 "고치지 않고 남긴 것").
- `RECORD_DEFAULTS`에는 stall_s만 있다. 다른 키의 옛 기본값 변경 이력은 2026-11-12 패키지 복사 전 기록이
  이 저장소에 없어 확인하지 못했다.
- 해제·후진의 기하 관계는 시험으로 고정되지 않았다.
