# Pinky URDF 내보내기 작업 기록 (2026-11-21)

목적: Pode(Pinky Pro) 모델을 Isaac Lab(Windows)에서 가져올 수 있게 URDF와 메시를 `C:\isaac_assets\pinky`에 만든다.

## 한 일

- `learning/assets/make_pinky_urdf.sh`: xacro를 URDF로 펼치고, 참조된 메시를 `meshes/`로 복사하고, `package://`를 URDF 기준 상대 경로로 바꾼다. 남은 `package://`, 없는 메시, `check_urdf` 실패는 오류로 끝난다. `--abs-prefix`로 절대 경로도 쓸 수 있다.
- `learning/assets/test_make_pinky_urdf.sh`: 임시 폴더에 만들어 `check_urdf`, `package://` 없음, 메시 경로 존재, `l_wheel_joint`/`r_wheel_joint` 존재, 절대 경로 옵션을 확인한다.
- `learning/assets/README.md`: 실행 방법과 URDF에서 읽은 값.
- 입력은 `robot.urdf.xacro`(`is_sim:=false`)다. 시뮬의 `pode_sim.urdf.xacro`와 같은 `insert_robot` 인자라 같은 로봇이다.

## 명령과 결과

```
bash learning/assets/make_pinky_urdf.sh      -> meshes: 17 references, 16 files / OK: /mnt/c/isaac_assets/pinky/pinky.urdf
bash learning/assets/test_make_pinky_urdf.sh -> PASS
```

## 발견

- 바퀴 간격: URDF 관절은 0.0811 m, 충돌 구 중심은 0.0971 m인데 Gazebo DiffDrive는 0.0961 m다. 어느 쪽도 고치지 않았고 README에 적었다. Isaac 쪽 값은 사용자가 정해야 한다.
- 반지름 0.028 m, 라이다 `rplidar_link`(base_footprint 기준 높이 0.125 m), 총 질량 1.54 kg은 기대와 같다.
