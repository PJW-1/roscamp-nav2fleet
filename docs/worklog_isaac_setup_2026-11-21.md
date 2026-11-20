# Isaac Lab 설치 작업 기록 (2026-11-21)

계획 문서 「nav2fleet_학습기반_주행_계획_2026-11-21」의 5-4 1번(설치와 예제 학습)과 5-5(설치 순서)를 했다. 버전, 위치, 다시 설치하는 법, 결과 표는 `learning/isaac_setup.md`에 있다. 이 문서는 한 일의 순서와 판단만 적는다.

## 순서

1. **01:40 버전 확인:** Isaac Lab 공식 설치 문서와 Isaac Sim 6.2 요구 사항을 읽었다. 계획과 다른 점(Isaac Lab 3.0, Isaac Sim 6.2, Python 3.12, uv, Newton 기본, RTX 3080은 Isaac Sim 최소 사양 미달)을 사용자에게 먼저 보고했다.
2. **01:45 기존 설치 정리:** 시스템 Python 3.12에 있던 Isaac Sim 6.0.1(isaacsim 패키지 25개)을 pip로 지우고, 남은 폴더와 Omniverse 캐시(`%LOCALAPPDATA%\ov`, `~\.nvidia-omniverse`, `Documents\Kit`)를 지웠다. 이 설치를 쓰던 `Desktop\ros2_amr_fleet_control`은 사용자가 더 쓰지 않는다고 했다. 폴더는 지우지 않았다.
3. **01:47 Newton 판 설치:** `D:\isaac\IsaacLab`에 `release/3.0.0`을 받고 `uv sync`(78 s).
4. **01:49 Newton Cartpole:** 4096개 환경, 150회, 학습 79 s, 보상 0.13 → 4.93.
5. **01:52 Isaac Sim 더함:** `uv sync --inexact --extra isaacsim --extra importers`(208 s). 가상 환경이 18 GB가 됐다.
6. **01:58 Isaac Sim PhysX Cartpole:** 처음 4096개는 시작 중 접근 위반으로 죽었다. 256개로 줄여 성공했고, 다시 4096개로 돌려도 성공했다. 첫 실행 도중 다른 작업이 `UV_PYTHON_INSTALL_DIR` 없이 `uv run`을 해서 가상 환경을 다시 만들려 한 것이 원인일 가능성이 높다. 그 뒤로 가상 환경은 uv 기본 위치의 Python(`%APPDATA%\uv\python`)을 쓴다.

## 판단

- **Newton을 주 엔진으로:** 우리 학습은 화면 없이 돌리고 장면이 단순하다(바닥, 상자, 작은 로봇). Newton은 3080 사양을 충족하고 GPU 메모리를 적게 쓴다. Isaac Sim은 지울 이유가 없어 같은 환경에 남겼다(D 드라이브라 여유가 충분하다).
- **관리자 작업 없음:** 긴 경로 허용이 이미 켜져 있었고 드라이버(610.88)가 요구 버전(Isaac Sim 6.2용 595.97, Newton용 580.88)보다 새것이었다.
- **EULA:** 사용자가 승인이 필요한 일을 모두 승인한다고 해서 `OMNI_KIT_ACCEPT_EULA=YES`로 진행했다.

## 남은 것

- 학습한 정책을 화면으로 보기(`--viz newton_gl`)는 하지 않았다.
- Isaac Sim 첫 실행 실패를 같은 조건으로 다시 일으켜 보지는 않았다. 다시 나면 `D:\isaac\logs\p2_cartpole_physx.log`와 비교하고, 그때 다른 `uv` 명령이 돌고 있었는지 본다.
