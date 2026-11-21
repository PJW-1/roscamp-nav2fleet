# RPP와 MPPI 비교 묶음, 4단계 (2026-11-21)

- 조건: Gazebo v7 경기장 한 대, O_IN → M_E, `nav_max_speed` 0.20 m/s, 시나리오마다 컨트롤러마다 10판(ABBA 순서).
  RPP = `FollowPathGoal`, MPPI = `FollowPathMPPI`(RotationShim + MPPI, `CostCritic` 1.0, `vx_std` 0.2, 커밋 c1c03e6).
- 판마다 출발 자세를 확인해 3 cm·5°를 넘으면 Gazebo에서 출발점으로 옮겼다. 계획 경로와 실제 궤적이 통로 띠(y 0.405~0.77) 안이면 "통로" 판이다.
- 시나리오와 장애물 값: `tools/nav_bench/scenarios.yaml`. 지표 정의와 해석: `docs/worklog_mppi_controller_2026-11-21.md`의 "4단계".
- 파일: `table.md`(본 표, 시간 나누기 표, 공정성 표), `legs.csv`(판별 값), `compare.png`, `tracks.png`, `runs.txt`.
  `runs.txt`에서 끝 줄이 없는 a_hall 첫 시작(19:26)은 Gazebo가 시작 때 죽은 시도로, 판을 남기지 않았다.
- 판별 원본 JSON과 launch 로그는 WSL `~/gz_runs/nav_bench_p3/full/`에 있다.
- 다시 만들기: `python3 tools/nav_bench/report.py --config tools/nav_bench/scenarios.yaml --legs ~/gz_runs/nav_bench_p3/full/legs --out results/2026-11-21/mppi_benchmark_p3`
