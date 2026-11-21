# Pode(Pinky Pro) URDF를 Isaac Lab으로 가져오기

`make_pinky_urdf.sh`가 제조사 모델(`pinky_pro` 014a09f, 패키지 `pinky_description`)을 Windows 도구가 읽을 수 있는 URDF와 메시 묶음으로 만든다. 생성물은 커밋하지 않는다.

## 실행

WSL(Ubuntu 24.04, ROS 2 Jazzy)에서 한다. 기본 출력은 `C:\isaac_assets\pinky`다.

```bash
bash learning/assets/make_pinky_urdf.sh                      # /mnt/c/isaac_assets/pinky
bash learning/assets/make_pinky_urdf.sh /some/dir            # 다른 폴더
bash learning/assets/make_pinky_urdf.sh --abs-prefix C:/isaac_assets/pinky   # 메시 절대 경로
bash learning/assets/test_make_pinky_urdf.sh                 # 임시 폴더에 만들어 확인, 끝에 PASS
```

- 입력은 `urdf/robot.urdf.xacro`(`is_sim:=false`)다. Gazebo 시뮬의 `sim/fleet_gz_sim/urdf/pode_sim.urdf.xacro`도 같은 `insert_robot`(`is_sim=false`, 카메라 8도, 화면 -25도)를 쓰므로 같은 로봇이다. 시뮬은 그 위에 Gazebo 플러그인만 얹는다.
- 출력은 `pinky.urdf`와 `meshes/visual/*.dae`, `meshes/collision/*.stl`이다.
- 메시 경로는 기본이 URDF 파일 기준 상대 경로(`meshes/visual/base_link.dae`)다. `package://`는 하나도 남지 않는다. 폴더째 옮겨도 된다. 절대 경로가 필요하면 `--abs-prefix`를 쓴다(이때는 존재 확인을 건너뛴다).
- `package://`가 남거나 메시가 없거나 `check_urdf`가 실패하면 0이 아닌 값으로 끝난다.
- 소스 폴더는 환경 변수 `PINKY_DESCRIPTION_DIR`로 바꾼다(기본 `~/pinky_deps_ws/src/pinky_pro/pinky_description`).

## 생성된 URDF에서 읽은 값

| 항목 | 값 | 근거 |
|---|---|---|
| 바퀴 관절 | `l_wheel_joint`, `r_wheel_joint` (continuous, 축 `0 1 0`, 부모 `base_link`) | 관절 이름 |
| 바퀴 관절 위치 | `base_link` 기준 y = +0.04055 / -0.04055 | joint origin |
| 바퀴 간격(관절 origin) | 0.0811 m | 0.04055 x 2 |
| 바퀴 간격(충돌 구 중심) | 0.0971 m | 충돌 구가 바깥으로 0.008 m 치우침: (0.04055+0.008) x 2 |
| 바퀴 반지름 | 0.028 m | 바퀴 충돌 구 `radius` |
| 라이다 프레임 | `rplidar_link` | |
| 라이다 위치 | `base_link` 기준 (-0.017, 0, 0.097), yaw 180도 / `base_footprint` 기준 (-0.017, 0, 0.125) | `rplidar_mount` (-0.017, 0, 0.067) + `rplidar_link` (0, 0, 0.030) + `base_link` z 0.028 |
| 전체 질량 | 1.54 kg | 링크 질량 합 |

링크별 질량(kg): `base_link` 0.8, `l_wheel` 0.2, `r_wheel` 0.2, `rplidar_mount` 0.11, `caster_mount_link` 0.1, `caster_roate_link` 0.05, `caster_wheel` 0.05, `robot_lamp` 0.01, `front_camera_mount` 0.01, `screen_mount` 0.01. 나머지 링크(`base_footprint`, `rplidar_link`, `front_camera_link`, `ultrasonic_link`, `ir_*_link`, `imu_link`)는 질량이 없다(0).
링크 이름 `caster_roate_link`의 오타는 제조사 원본 그대로다.

## 불일치

Gazebo 시뮬(`sim/fleet_gz_sim/urdf/pode_sim.urdf.xacro:22-23`)의 DiffDrive는 `wheel_separation` 0.0961, `wheel_radius` 0.028이다. 반지름은 URDF와 같다. 간격은 URDF에서 어떻게 재도 0.0961이 나오지 않는다(관절 0.0811, 충돌 구 중심 0.0971). 양쪽 다 고치지 않았다. Isaac Lab에서 cmd_vel을 바퀴 속도로 바꿀 때는 실물 bringup(`pinky_params.yaml`)과 같은 0.0961을 그대로 쓴다(2026-11-21 사용자 결정). 시뮬 모델의 바퀴 위치는 고치지 않고, 실물의 회전 비율은 응답 측정 도구(`tools/real_pode/cmd_step_probe.py`)로 재서 맞춘다. 참고로 `pode/pode_nav_config/test/test_nav_config.py:62`의 주석은 바퀴 축을 y ±0.04055로 적는다(URDF와 일치).
