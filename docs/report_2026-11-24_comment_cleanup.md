# 주석 정리 기록 (2026-11-24)

사용자 지시: "문단별로 무슨 역할을 하는지 간단하게 요약하고, 중요하면 한두 줄만 덧붙여라. 전체적으로 수정해 놔."
(`fleet_map_v7.yaml`이 반 넘게 주석인 것을 보고 한 말이다.)

## 규칙

- 문단(코드 덩어리, YAML 구역)마다 역할을 한국어 한 줄로 쓴다. 중요하면 한두 줄만 더한다.
- 값을 바꾼 이력(날짜, `0.68 -> 0.63 -> 0.70`, 누가 정했는지, 계산 과정)은 주석에 두지 않는다. git log와 기록 문서에 둔다.
- 영어 주석은 쓰지 않는다. `# ---- 입력 ----` 같은 구역 표시는 둔다.
- 코드, 문자열, 식별자, YAML 키와 값은 바꾸지 않는다.

## 범위와 결과

브랜치 `chore/comment-cleanup-2026-11-24`(main `aff00d7`에서 출발). 디렉터리별 7개 작업을 커밋 하나씩으로 나눴다.

| 커밋 | 범위 | 주석 줄 (전 → 후) |
|---|---|---|
| `7aa50ac` | `pc/fleet_core/fleet_core`, `pc/fleet_manager`, `bringup` 등 | 1082 → 826 |
| `8e352d4` | `pc/fleet_core/test` | 1531 → 969 |
| `e4cbd06` | `sim/fleet_gz_sim` | 1143 → 745 |
| `a01989b` | `learning` | 1086 → 787 |
| `104019f` | `pode` | 1324 → 942 |
| `2de81be` | `tools`, `scripts`, `docs`, `results` | 893 → 648 |
| `59d5444` | `sim/fake_robots` | 1768 → 1525 |
| 합계 | .py, .yaml, .sh 577개 파일 | 8827 → 6442 (주석 글자 비율 13% → 8%) |

- 바뀐 파일 319개, +2240 −4584줄.
- `fleet_map_v7.yaml`: 270줄(주석 198줄) → 113줄(주석 41줄). 배치 그림은 날짜를 빼고 남겼다.
- 남은 주석은 대부분 구역 표시, 한 줄 요약, 단위·뜻을 적은 짧은 줄 끝 주석, `# noqa`다.

## 코드가 그대로인지 확인한 방법

검사기(작업용 스크립트)로 `aff00d7`과 비교했다. 319개 파일 모두 통과했다.

- .py: docstring을 뺀 AST가 같다.
- .yaml: `yaml.safe_load_all` 결과가 같다.
- .sh: 주석을 뺀 줄이 같고 `bash -n`이 통과한다.
- `__doc__`를 argparse 설명으로 쓰는 파일은 짧은 한국어 모듈 docstring을 남겼다.

## 시험 (ROS 노드를 띄우는 `test_ros_*` 제외, `ROS_DOMAIN_ID=78`, LOCALHOST)

| 패키지 | 결과 |
|---|---|
| `fleet_core` | 1269 통과, xfail 7 |
| `pode_nav_config` · `pode_avoidance` · `pode_scan_filter` · `pode_peer_obstacles` | 111 · 26 · 23 · 19 통과 |
| `tools/test` | 286 통과, 건너뜀 3 |
| `fake_robots` (작업 브랜치에서) | 993 통과, 건너뜀 2, xfail 30 |
| `docs/distance_remaining_2026-11-21` | 2 실패, 8 통과. `aff00d7`에서도 같은 2건이 실패한다(원래 실패) |
| `learning` | `torch`가 없어 못 돌림. 검사기만 통과 |

## 검토에서 되살린 설명

Codex 검토와 지운 주석 재검토에서, 줄이다가 빠지거나 뜻이 틀어진 설명 7곳을 되살렸다.

- `tools/real_pode/make_params.py`: 기본 실행은 고정 경로 컨트롤러 셋의 충돌 검사를 끈다(`--fixed-line-check`로 켬). "속도만 바꾼다"는 틀린 줄 끝 주석을 지웠다.
- `fleet_params.yaml` `release_distance_m`: 구역 교통은 이 값을 읽지 않는다(`footprint_radius_m + release_margin_m`로 놓음).
- `fleet_params.yaml` `nav_eta_speed_ratio`: 범위(0 초과 1 이하).
- `nav2_params.yaml` `speed_limit_topic`: 버전 2에서 쓰니 지우지 않는다.
- `nav2_params.yaml` 다각형 발자국: FollowPath 셋의 충돌 검사와 `pode_fsm`의 막힘 판단이 쓴다.
- `avoidance.yaml` `peer_layer`: 이 키가 없는 회피 패키지로 `peer_obstacles:=true`를 주면 `nav2.launch.py`가 뜨지 않는다.
- `fake_robots/world/nav_goal.py` `UNBLOCK_M`: Pode의 `unblock_m`과 같아야 한다.

## 손대지 않은 것과 까닭

- `docs/oout_wobble_2026-11-16/analyze_cause.py`의 `# ---- odom -> 지도` 표시: 같은 폴더의 스크립트 넷이 이 문자열로 소스를 쪼갠다(`exec(src.split(...)[0])`).
- `tools/real_pode`의 .py·.sh 주석은 한글을 포함해야 한다(`test_comments_are_korean`).
- xml, sdf, world, md, cfg 파일.

## 다른 브랜치와의 관계

- 아직 main에 없는 개인 브랜치(`feat/plan-flip-2026-11-24`, `feat/fleet-net-2026-11-24` 등)는 main을 합칠 때 주석 줄에서 충돌할 수 있다. 코드는 그 브랜치 것을, 주석은 이 형식을 쓰면 된다.
- 팀 레포에는 다음 PR(IR-62와 함께)로 같은 정리를 넣는다. 팀 `dev`(`e3ee8a5`)에서 새 브랜치를 따서 3-way merge로 얹는다.
