# RPP와 MPPI 비교 묶음 (2026-11-21)

- 조건: Gazebo v7 경기장 한 대, O_IN → M_E, `nav_max_speed` 0.20 m/s, 시나리오마다 컨트롤러마다 5판(RPP와 MPPI 번갈아).
  RPP = `FollowPathGoal`, MPPI = `FollowPathMPPI`(RotationShim + MPPI, 커밋 4456358).
- 시나리오와 장애물 값: `tools/nav_bench/scenarios.yaml`. 지표 정의와 해석: `docs/worklog_mppi_controller_2026-11-21.md`의 "3단계".
- 파일: `table.md`(평균 ± 표준편차, 성공 수, 충돌·BLOCKED 합), `legs.csv`(판별 값), `compare.png`(시간·도리도리·몸 최소 거리),
  `tracks.png`(실제 위치 궤적), `runs.txt`(시나리오별 시작·끝 시각과 bench 결과).
- 판별 원본 JSON과 launch 로그는 WSL `~/gz_runs/nav_bench_2026-11-21/`에 있다.
- 다시 만들기: `python3 tools/nav_bench/report.py --config tools/nav_bench/scenarios.yaml --legs ~/gz_runs/nav_bench_2026-11-21/legs --out results/2026-11-21/mppi_benchmark`
