# 이름 바꾸기: link → edge, via → mode, Trip → Route (2026-11-19)

지시서 `nav2fleet_이름변경_지시서_2026-11-19.md`대로 관제 내부 이름 세 개를 바꿨다. 동작은 바꾸지 않았다.
브랜치 `refactor/rename-edge-mode-route`, 기준 ae89d81(origin/main 3d4254c + v7 기본 배치, run8 문서 정리 병합).
팀 레포(지시서 4절)는 사용자 지시로 하지 않았다. main 병합은 코디네이터가 확인한 뒤 한다.

## 커밋

| 커밋 | 내용 |
|---|---|
| c81113c | 그래프 함수 `route()` → `shortest_path()` |
| 6f1189e | `Link` → `Edge`, `links()/links_from()/link()` → `edges()/edges_from()/edge()`, `free_link/back_link` 등 → edge |
| a32ce9f | 주행 방식 `via` → `mode` (`Edge.mode`, `Goal.mode`, `_mode()`, `_end_mode()`) |
| 8b214bc | `Trip` → `Route`, `Trip.route()` → `Route.nodes_left()`, `m.trip` → `m.route`, `trips.py` → `routes.py`, `test_zone_trips.py` → `test_zone_routes.py` |
| ac2287b | 주석, docstring, 시험 assert 문구의 링크/트립/link/trip → edge/route |
| a8ef55a | 시험 docstring 한 곳의 `free_link` → `free_edge` (3.4 grep에서 찾음) |

바꾸는 방법: 파이썬 토큰(NAME)만 바꾸는 스크립트로 바꿔 문자열과 주석은 건드리지 않았고, 단계마다 남은 것을 손으로 분류했다.

## 바꾸지 않은 것과 이유

- 밖으로 나가는 것(지시서 0절): 메시지, contract.yaml, 지도 yaml 키, fleet_params.yaml, 이벤트 코드와 문구, 조회 dict 키(`"route"` 등), 기록 형식, `on_*`.
  views.py는 지역 변수 `route`를 `nodes`로 바꿔 키 `"route"`를 그대로 뒀다.
- 장애물 일정 yaml 키 `link`(sim/fleet_gz_sim/obstacles, tools/triage/readers.py). 시험의 `entry(link=...)`는 이 키를 만드는 인자라 그대로다. 파이썬 속성 `Obstacle.link`는 `Obstacle.edge`로 바꿨다.
- 오류 문구: FleetMapError("link from ... to itself" 등), PlanError, ScheduleError. 사람이 읽는 출력이라 그대로 뒀다.
- 다른 뜻의 link: fake_robots `world/link.py`의 `Link`, `_setup_link`(통신 링크), `pode_report.py` 주석, `tools/yield_report.py`의 `_link`(묶기), Gazebo SDF `<link>`, `base_link`.
- via point 뜻의 via: `back_edge`의 `via`(꺾는 노드), `yard_retreat` 결과와 `_behind_on_the_bent_way`의 `via`, goals.py의 `via`(retreat_via 노드의 구역)와 `via_node`, `Avoid.retreat_via`, fake_robots `Motion.via`(거칠 점), `test_zone_block_marks.py:263`의 `via`(거쳐 가는 edge 목록).
- 다른 뜻의 trip: fake_robots `order_metrics.py`의 `trips`, `_trip`, `_track_trip`, `trip_text`(이동 한 건의 시간 지표), `test_manager.py`의 `test_a_real_fill_opens_the_trip_out_by_name`(영어 일반어), `test_zone_goal_life.py:90`의 검사기 입력 문자열.
- 지도 yaml 주석 `fleet_map_v7.yaml:192`의 "트립": 지도 본문이 기록 첫 줄(`map`)에 그대로 들어가므로 이번에는 두었다(확인 필요).
- docs/와 README는 건드리지 않았다(README에 바뀐 이름을 쓰는 코드 예시가 없다).

## 결과

전과 후 모두 같은 방식으로 돌렸다: 따로 만든 colcon 빌드(build/install을 scratch에), ROS_DOMAIN_ID 69(전)/72(후),
LOCALHOST, rmw_fastrtps_cpp, 시나리오 도메인 80~88(전)/90~98(후).

| 묶음 | 전(ae89d81) | 후(a8ef55a) |
|---|---|---|
| pc/fleet_core | 1111 통과, 7 xfail | 1111 통과, 7 xfail |
| tools | 156 통과 | 156 통과 |
| pc/fleet_manager(ROS 통합 포함) | 131 통과 | 131 통과 |
| sim/fleet_gz_sim | 598 통과 | 598 통과 |
| pode_avoidance / nav_config / peer_obstacles / scan_filter | 26 / 106 / 19 / 23 통과 | 같음 |
| pode_fsm | 467 통과, 6 건너뜀 | 467 통과, 6 건너뜀 |
| sim/fake_robots(test_ros_* 포함) | 16 실패, 1049 통과, 1 건너뜀, 27 xfail | 같은 16개 실패, 나머지 같음 |

- 시험마다 결과를 junit으로 맞대어 보았다: 공통 시험의 결과 차이 0, 이름만 바뀐 시험(fleet_core 21, fleet_gz_sim 17) 말고는 같다.
- fake_robots의 16개 실패(test_ros_pode.py 12, test_ros_pode_order.py 4)는 바꾸기 전 ae89d81에서도 똑같이 난다. 이번 변경과 관계없다.
  추측: 시험은 `load_fleet_map("d2")` 좌표를 기대하는데 노드가 기본 배치(v7, 9da87f2부터)로 뜬다.
- tools/test_real_pode.py의 dock_aside 2개 실패(3d4254c에서 알려진 것)는 ae89d81에서 나지 않았다.
- colcon build: 전후 모두 15개 패키지 성공.
- ruff 0.17.5: 전후 모두 106건이고 목록이 같다(103건 docs/oout_wobble 스크립트, 3건 fleet_gz_sim의 긴 줄). 새로 생긴 것 없음.
- 재생: run8 기록이 재부팅으로 없어져 ~/real_runs의 8개(fixedline_off_1, peer_on_order_on_1~3, run5_1, run5_2,
  run6_1, run7_1)로 대신했다. `tools/replay.py` 출력과, 코어가 낸 줄 전체를 덤프한 것을 비교했다.
  바꾸기 전 두 번 돌려 바이트 단위로 같음(결정적)을 확인했고, 단계마다와 마지막에 8개 모두 바이트 단위로 같았다.
  (fixedline_off_1, peer_on_order_on_2, peer_on_order_on_3은 바꾸기 전부터 기록과 다른 줄이 있다. 기록을 만든 뒤 코드가 바뀐 탓으로 보지만 확인하지 않았다. 전후 출력은 같다.)
- 지시서 3.4의 grep(`git grep -nwE "Link|links_from|free_link|back_link|Trip|trips" -- '*.py' ':!docs'`,
  `git grep -nE "\.via\b|\bvia\s*[=:]" -- 'pc/*.py' 'sim/*.py' 'tools/*.py'`): 첫째는 fake_robots의 통신 링크 `Link`(3줄)와 order_metrics의 `trips`(3줄)만 남았다.
  둘째는 via point 뜻만 남았다(blocked.py, goals.py, routes.py, yard_retreat.py, test_zone_block_marks.py, fake_robots의 거칠 점).
