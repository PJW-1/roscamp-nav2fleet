# 시뮬 관측 API 작업 기록 (2026-11-16, 브랜치 feat/zone-observe)

## 결론
- 시뮬이 교통 제어기 내부(`traffic.zone.*` 모듈, `_blocked`, `_marks`)를 직접 읽지 않는다.
- 막힘 판정은 읽기 전용 질의 `live_block(keys, point, radius, now) -> bool` 하나로 한다. 내부 가변 객체는 내보내지 않고 bool만 돌려준다.
- 순수 계산(`keys_at`, `segment_end`)과 상수(`CORRIDOR`, `DESTINATION`, `IN_ZONE`, `KEEPOUT`)는 `fleet_core.traffic.api`에서 다시 내보낸다.

## 바꾼 파일
- `pc/fleet_core/fleet_core/traffic/zone/views.py`: `Views.live_block` 추가. 판정은 예전과 같다(막힌 키의 만료 시각 > now, 또는 표시 위치와 point의 거리 <= radius).
- `pc/fleet_core/fleet_core/traffic/api.py`: `TrafficApi.live_block` 계약 추가, 순수 함수와 상수 재공개.
- `sim/fake_robots/fake_robots/world/block_watch.py`: import를 `traffic.api`로, `_live`는 `live_block` 호출. 교통 객체에 `live_block`이 없으면(관제 없음) 예전 `getattr(..., {})`처럼 빈 상태(False)로 본다. 반경은 `near_m + box.radius` 그대로.
- `sim/fleet_gz_sim/fleet_gz_sim/fleet_view.py`: `segment_end`를 `traffic.api`에서 가져온다.
- 시험: `test_zone_block_marks.py`(예전 사설 접근과 같은 값, 상태 불변), `test_traffic_api.py`(공개 이름), `test_fms_boundary.py`(ast 경계).

## 공개 위치 판단
- `live_block`은 상태 관측이라 `Views` 믹스인의 인스턴스 메서드로 둔다.
- `keys_at`, `segment_end`는 상태가 아닌 순수 계산이라 인스턴스 메서드로 옮기지 않았다. `traffic/api.py`는 이미 공개 경계이고 시험의 KEPT 목록에 있어, 코드를 옮기지 않고 거기서 다시 내보내는 것이 변경이 가장 작다. 구현은 zone 안에 그대로 있다.
- 새 폴더와 새 모듈은 만들지 않았다.

## 시험(ROS_DOMAIN_ID=67, 로컬호스트)
- 시험 먼저: 새 시험이 실패하는 것을 확인한 뒤 구현했다(처음 4개 실패. 스냅샷 방식에서 질의 방식으로 바꿀 때 3개 실패).
- `PYTHONPATH=pc/fleet_core:pc/fleet_core/test python3 -m pytest pc/fleet_core/test -q`: 751 passed (44 s).
- fake_robots(situation_table, traffic_bugs, world_obstacle_view, world_invariants): 74 passed.
- fleet_gz_sim `test_fleet_view.py`: 41 passed.
- 경계 시험은 `sim/*/*/**/*.py`를 ast로 훑어 `fleet_core.traffic.zone` import, `traffic._x` 읽기, `getattr(traffic, "_x")`를 막는다. 반례 문자열로 시험이 실제로 실패할 수 있음도 확인한다.

## Codex 지적 반영(후속 커밋)
- 경계 시험이 `from fleet_core.traffic import zone [as z]`를 놓치던 것을 고쳤다(반례를 먼저 넣어 실패를 확인). `SIM_ROOT`는 시험 파일 기준 경로(`Path(__file__).resolve().parents[3]`)로 바꿨다.

## 돌리지 않은 것 / 한계
- Gazebo, test_scenario_integration, test_fms_restart는 돌리지 않았다.
- 경계 시험은 정적 검사라 `getattr(traffic, 변수)` 같은 동적 접근은 못 잡는다.
- 별칭(`t = world.traffic; t._blocked`)과 동적 접근은 정적 검사로 못 잡는다. 억지로 추적하지 않았다.
- `live_block`이 없는 관제를 막힘 없음으로 보는 대체 동작은 지금 ZoneTraffic 하나뿐이라 그대로 뒀다.
- `Views.zones`(views.py)가 가변 `grants`를 그대로 내보내는 것은 이번 변경 전부터 있던 것이다. 고치지 않았다.
