# 팔 이름 바꾸기(설계 4.6, 0.6 기준) — 2026-11-24

- 브랜치: `feat/arm-rename-0.6-2026-11-24` (main c62c03d에서 시작)
- 기준 문서: 팀 저장소 docs/design 브랜치 `docs/설계/주문_흐름_1차_인터페이스.md` 4.6
- contract 버전은 0.6 그대로다. 0.7 브랜치(`feat/interface-align-2026-11-24`)에서 가져온 것은 없다.

## 바꾼 이름

| 0.6 | 바꾼 이름 | 로봇 |
|---|---|---|
| `arm1` | `arm_left` | Phody 왼팔 |
| `arm2` | `arm_right` | Phody 오른팔 |
| `arm3` | `arm_ship` | Phony 출고 팔 (`/arm_ship/swap_basket`) |

Pode 이름(`pode1` ~ `pode3`)과 `/workcell/fill_basket`은 그대로다.

## 커밋

| 커밋 | 내용 |
|---|---|
| 4eba091 | 이름 바꾸기 한 커밋(IR-90). contract, 설정, 코드, 시험, 시나리오를 함께 넣었다. 64개 파일, 158줄 |
| 다음 커밋 | 바꾼 이름 뒤의 조사 고치기(`arm_left과` → `arm_left와`, `arm_left이` → `arm_left가`, `arm_ship가` → `arm_ship이`, `arm_ship는` → `arm_ship은`). 주석과 설명만 바뀌고 동작은 같다. 이 보고서도 같은 커밋에 넣었다 |

## 바꾼 곳

| 곳 | 파일 |
|---|---|
| 계약 | `interfaces/fleet_interfaces/contract.yaml`: `placeholders.arm`, `swap_basket`의 `instances` → `[arm_ship]` |
| 로봇 목록 | `config/robots.yaml` |
| fleet | `fleet_map_*.yaml` 다섯 개의 `arms` · `server`, `fleet_params.yaml` 주석, 시험 자료 `fleet_map_line3` |
| fleet 시험 | `pc/fleet_core/test` 7개, `pc/fleet_manager/test` 4개(`fms_restart.yaml` 포함) |
| 가짜 노드 · 실행 | `fake_arm_node`의 `arm_id` 기본값, `fake_workcell_node`, `common.py`, `fakes.launch.py`, `gz.launch.py`, `bringup/iponic_bringup/launch/pc.launch.py`의 `fake_arms` 기본값과 설명 |
| 시나리오 | `sim/fake_robots/scenarios` 12개, `sim/fleet_gz_sim/scenarios` 11개 |
| 그 밖의 시험 | `sim/fake_robots/test` 12개, `sim/fleet_gz_sim/test` 1개, `pode/pode_fsm/test` 1개 |
| 설계 문서 | `docs/IFP_공통설계.md` (지난 기록이 아니라 지금 쓰는 설계라서 바꿨다) |

- `README.md`, `.github/CODEOWNERS`, `arm/*/README.md`, `docs/개발_규칙.md`에는 옛 이름이 없었다.
- `orders/`(`pc/fleet_core/fleet_core/orders`)에는 옛 이름이 없었고, 바꾼 것도 없다.
- `f"arm{i}"`처럼 이름을 만들어 쓰는 코드는 없었다. 옛 이름이 든 기록 파일을 다시 돌리는 시험도 없었다.

## grep 결과

`git grep -nwE 'arm[123]'`는 날짜가 붙은 worklog 세 줄만 낸다. 지난 기록이라 그대로 두었다.

- `docs/worklog_real_two_robot_test_2026-11-18.md:74`
- `docs/worklog_rename_pode_2026-11-16.md:16`, `:109`

`results/`, `learning/`, `tools/`에는 남은 것이 없다.

## 시험

환경: 별도 설치 공간, `ROS_DOMAIN_ID=66 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST SCENARIO_FIRST_DOMAIN=80 FMS_RESTART_DOMAIN=79`. 패키지 폴더마다 `python3 -m pytest -q test`.

| 패키지 | 결과 |
|---|---|
| `pc/fleet_core` | 1269 passed, 7 xfailed |
| `pc/fleet_manager` | 137 passed |
| `pode/pode_fsm` | 610 passed |
| `tools` | 285 passed, 3 skipped |
| `sim/fleet_gz_sim` | 467 passed, 6 skipped |
| `sim/fake_robots` | 1136 passed, 1 skipped, 30 xfailed, 20 failed → 아래 |

- `sim/fake_robots`의 20개 실패는 모두 `test/test_ros_pode.py`(팔 이름 없음)에서 났다. 같은 도메인에서 다른 시험이 함께 돌던 때였다. 그 파일만 다시 돌리자 58개가 모두 통과했다.
- `scripts/check_interfaces.py`는 떠 있는 ROS 노드를 보는 도구라서, 노드 없이 돌리면 24개가 없다고 나온다. 대신 가짜 노드를 띄워 확인하는 `test_ros_check_interfaces.py`가 통과했다.

## 리뷰 (Codex)

main에 세 이름 바꾸기를 다시 적용해 만든 결과와 이 커밋이 같은지 비교했고, 결함이 없다고 답했다. 다시 고칠 것이 없어 한 번으로 끝났다.

## 팀원이 알아야 할 것

- 팔 PC는 새 이름으로 띄워야 한다. 옛 이름으로 띄우면 fleet이 찾지 못한다.
  - 가짜 팔: `fake_arms:="arm_left arm_right arm_ship"`
  - 출고 팔: `arm_id:=arm_ship`, 액션 `/arm_ship/swap_basket`
- `arm_left`와 `arm_right`는 `/workcell/fill_basket` 서버 하나를 함께 쓴다. 둘 중 하나만 적으면 오류다.
