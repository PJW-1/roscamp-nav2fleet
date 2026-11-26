# 작업 기록: Nav2 goal 길 유지(keeper) 기본 켬 (2026-11-26)

브랜치 `feat/plan-keep-default-on-2026-11-26` (origin/main 3ccf2a3f에서). 사용자 지시(11-26 밤): "키퍼는 그냥 이제 기본으로 킴으로 두면되잖아".

## 근거

- Gazebo 비교(11-24, `docs/report_2026-11-24_plan_flip.md`): 장애물 판에서 로봇이 받은 경로의 뒤집힘이 RPP 1 cm 12.7 → 1.7, 2.5 cm 19.0 → 2.0으로 줄었고 판 시간은 같았다.
- 실물(11-26, `data/real-2026-11-26` 브랜치 `results/2026-11-26/measure_2026-11-26.md`): 두 대 운반 둘째 판을 `PLAN_KEEP=1`로 돌렸다. 주문 4건 모두 SHIPPED, BLOCKED 0, 판 시간 2분 24초(끈 판 2분 25초). Pi4 CPU는 Nav2 컨테이너 109~113 %, pode_fsm 55~57 %였고 keeper 노드는 top 상위 목록에 들지 않았다. 두 판뿐이라 뒤집힘 감소 효과는 실물에서 가르지 못했다(그 판에서는 planner가 뒤집지 않음).

## 바꾼 것

- 기본값 false → true: `pode_nav_config` `nav2.launch.py`, 실물 `real_pode.launch.py`, Gazebo `gz.launch.py`·`one_pode.launch.py`의 `plan_keep`.
- 실물 `start.sh`: `PLAN_KEEP` 기본 1. 0이면 `plan_keep:=false`, 1이면 `plan_keep:=true`를 늘 넘긴다. 그 밖의 값은 거절(종료 2). `NO_MOTION=1`은 전처럼 `PLAN_KEEP`을 0이나 빈 값으로만 받는다.
- `bag_record.sh`: 토픽에 `plan_kept`(keeper가 돌려준 길)를 더했다. 11-26 bag에는 없어서 keeper가 무엇을 돌려줬는지 볼 수 없었다.
- 문서: `tools/real_pode/README.md`, `pode/pode_plan_keep/README.md`.
- 끄는 법: Nav2 `plan_keep:=false`, Gazebo `plan_keep:=false`, 실물 `PLAN_KEEP=0 ./start.sh ...`.

## 시험

- `tools/test/test_real_pode.py`, `tools/test/test_bag_record.py`: 165 passed, 3 skipped.
- `pode_nav_config/test/test_nav2_launch.py`, `pode_plan_keep/test`, `fleet_gz_sim` `test_one_pode_wiring.py`·`test_launch_wiring.py`·`test_dock_stub.py`: 243 passed, 12 failed. 실패 12개는 같은 명령으로 origin/main에서도 똑같이 실패한다(목록 diff 없음). 설치본(`podedrive_ws/install`)이 오래된 탓으로 보며 이 변경과 관계없다.
- `test_nav2_launch.py`의 `_expand`는 이제 `plan_keep="false"`를 기본으로 넣는다. 노드 9개를 세는 기존 시험이 keeper와 따로 돌게 하려는 것이고, launch 기본값은 `plan_keep=None`으로 주는 시험이 본다.

## 남은 것

- nav2fleet와 로봇 배포에는 아직 들어가지 않았다(동기화는 사용자 지시 때). 로봇에서 켜려면 지금처럼 `PLAN_KEEP=1`을 준다.
