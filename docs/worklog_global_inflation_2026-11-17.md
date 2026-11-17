# 전역 inflation 조이기 A/B 파일 (2026-11-17)

사용자 요청(11-17): "1번 2번 같이 진행해서테스트해" = cost_scaling_factor를 올리고 inflation_radius를 낮추는 것을 함께 시험.

배경: v7에서 O_OUT(0.16, 0.62) -> E-1(0.12, 0.24) 전역 경로가 동쪽으로 약 0.15 m 부푼다.
원인 후보는 global inflation(radius 0.25, scaling 6.0)이 서쪽 벽 0.25 m 띠 전체를 비싸게 만드는 것.

## 파일 (sim/fleet_gz_sim/nav_overrides, 시뮬 전용)

| 파일 | 덮는 값 (global_costmap.global_costmap.ros__parameters) |
|---|---|
| global_inflation_tight.yaml | inflation_layer.cost_scaling_factor 15.0, inflation_layer.inflation_radius 0.15 |
| global_padding_0p01_inflation_tight.yaml | 위 둘 + footprint_padding 0.01 |

local costmap, controller, 실물 params 파일은 그대로. setup.py의 glob(nav_overrides/*.yaml)이 설치한다.

## 사용

nav_overrides는 파일 하나만 받는다.

- 조이기만: `nav_overrides:=<share 또는 소스 경로>/global_inflation_tight.yaml`
- 지도 갈래 A와 함께: `nav_map_variant:=a nav_overrides:=<...>/global_padding_0p01_inflation_tight.yaml`

시험: test_launch_wiring.py의 test_tight_global_inflation_overrides_change_only_the_global_costmap.
