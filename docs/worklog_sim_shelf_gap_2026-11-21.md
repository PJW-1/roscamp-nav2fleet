# 시뮬 진열대 A 바퀴 여유 (2026-11-21)

시뮬 전용 변경이다. 실측 `shelves`, 실물 키트 지도(`tools/real_pode/maps`), 급행 줄 y 0.868은 그대로다.

## 원인

- 2026-11-20에 `sim_shelves`의 진열대 A 북쪽 면을 1 cm 물렸다(0.803 -> 0.793). 이 1 cm는 몸 반폭 0.056으로 셌다.
- 시뮬 바퀴 충돌 구는 몸보다 더 나온다: 반지름 0.028, 중심이 몸 중심에서 y +-0.04855(관절 0.04055 + 0.008).
  바깥 끝이 중심에서 0.0766이다(`pinky.urdf.xacro`).
- 면 0.793이면 로봇 중심이 y 0.8696 아래일 때 바퀴가 진열대에 닿는다. 줄 0.868은 이미 1.6 mm 겹친다.
- Gazebo 0.12 m/s 경기에서 로봇이 줄 위 y 0.870~0.875(닿거나 거의 닿음)로 갔고, 한 대는 진열대 동쪽 끝(x 1.5605)에서
  0.864로 내려가 걸렸다. 바퀴 오도메트리는 계속 돌아 관제가 닿지 않은 도크에 닿았다고 믿었다.

## 고른 값

진열대 A 북쪽 면 0.783. 실측 면 0.803에서 실물 로봇이 벽 쪽으로 붙는 약 2 cm(시뮬 로봇에는 없다)를 뺀 값이다.
바퀴 닿는 선은 0.783 + 0.0766 = 0.8596이라 줄 0.868에서 8.4 mm 아래다. 0.780~0.786 안에서 다른 값을 쓸 검사 사정은 없어 제안값 그대로 썼다.

## 다시 만든 파일

`cd sim/fleet_gz_sim && PYTHONPATH=../../pc/fleet_core:. python3 -m fleet_gz_sim.arena --layout v7` (WSL).
바뀐 것은 `worlds/arena_v7.sdf`(shelf_1 한 개: 중심 y 0.7815 -> 0.7765, 두께 0.023 -> 0.013)와 `maps/arena_v7_fine.pgm`뿐이다.
5 cm 지도(`arena_v7`, `_a`, `_b`, `_ab`)와 keepout은 0.783이 같은 칸이라 그대로다. `--real` 키트 지도는 실측 진열대라 건드리지 않았다.

## 시험

- `pc/fleet_core/fleet_core/data/fleet_map_v7.yaml`: 진열대 A 값과 주석.
- `pc/fleet_core/test/test_layout_v7.py`: `SIM_SHELF_A` 0.783, 실측과 차이 0.02, 새 시험 `test_v7_sim_wheel_spheres_clear_shelf_a_on_the_express_lane`
  (줄 0.868 - 바퀴 바깥 끝 0.0766 - `sim_shelves` 면이 5 mm 이상).
- `sim/fleet_gz_sim/test/test_arena_v7.py`: 월드 상자 0.783, 1 cm 지도 점유 칸(0.785는 진열대, 0.795는 빈칸).
- `sim/fleet_gz_sim/test/test_fleet_setup.py`: 좁은 띠 문자열 y 0.823 -> 0.813.
- 시험 명령과 결과는 보고에 적었다. Gazebo는 띄우지 않았다.
