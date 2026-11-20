# Isaac Lab 설치 기록 (집 PC)

2026-11-21 새벽에 계획 문서 「nav2fleet_학습기반_주행_계획_2026-11-21」의 5-4 1번과 5-5를 했다. 이 파일은 설치한 버전과 다시 설치하는 법, Cartpole 시험 결과를 남긴다.

## 설치한 버전

| 항목 | 값 |
|---|---|
| Isaac Lab | 3.0.0. 브랜치 `release/3.0.0`, 커밋 39394453c(`git describe`: v3.0.0-9-g39394453c) |
| Isaac Sim | 6.2.0.0 (pip, Isaac Lab의 extra `isaacsim`) |
| 물리 엔진 | Newton(warp-lang 1.17.0, 해석기 MuJoCo-Warp)이 기본이다. PhysX는 Isaac Sim과 함께 쓴다 |
| Python | 3.12.14 (uv 가상 환경) |
| PyTorch | 2.11.0+cu130 (CUDA 13.0) |
| NVIDIA 드라이버 | 610.88 (바꾸지 않음) |
| GPU, RAM | RTX 3080 10 GB, 32 GB |
| OS | Windows 11 Home 10.0.26200 |
| uv | 0.12.20 |

## 계획 문서와 달라진 점

계획 문서 5-5는 2025년 기준이라 아래가 바뀌었다(공식 설치 문서 https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation.md 기준).

- **버전:** 계획은 Isaac Sim 5.x와 Python 3.11을 가정했다. Isaac Lab 3.0은 Isaac Sim 6.2와 Python 3.12를 쓰고, Isaac Sim 5.1 이하는 지원하지 않는다.
- **물리 엔진:** Isaac Lab 3.0은 Newton을 기본으로 설치하고, Newton만으로는 Isaac Sim 없이 돈다. Isaac Sim은 extra로 고른다.
- **가상 환경:** conda 대신 공식 권장인 uv를 썼다. `isaaclab.bat --install` 대신 `uv sync`다.
- **실행 명령:** `isaaclab.bat -p scripts\reinforcement_learning\rsl_rl\train.py --task Isaac-Cartpole-v0 --headless` 대신 `uv run isaaclab train --rl_library rsl_rl --task Isaac-Cartpole-Direct physics=newton_mjwarp`이다. 화면은 기본으로 꺼져 있고(headless), 보려면 `--viz newton_gl`을 붙인다.
- **사양:** Isaac Sim 6.2의 최소 GPU는 GeForce RTX 4080 16 GB라 RTX 3080 10 GB는 표에 못 미친다. Newton만 쓸 때는 Turing 이상 GPU와 드라이버 580.88 이상이면 되고, Isaac Sim의 메모리 요구는 적용되지 않는다. 실제로는 아래처럼 둘 다 Cartpole이 돌았다.
- **사전 작업:** 긴 경로 허용(`LongPathsEnabled`)은 이미 1이었다. 관리자 작업, 재부팅, 드라이버 업데이트는 하지 않았다.
- **설치 위치:** C 드라이브 여유가 59 GB라 D 드라이브에 두었다.

## 위치

| 무엇 | 어디 |
|---|---|
| Isaac Lab 소스와 가상 환경 | `D:\isaac\IsaacLab`, `D:\isaac\IsaacLab\.venv` (Newton만: 4.5 GB, Isaac Sim 포함: 18 GB) |
| uv 캐시 | `D:\isaac\uv-cache` (환경 변수 `UV_CACHE_DIR`) |
| Omniverse 캐시 | `%LOCALAPPDATA%\ov`는 `D:\isaac\ov`로 가는 junction이다 |
| 설치·학습 로그 | `D:\isaac\logs` |
| 학습 결과 | `D:\isaac\IsaacLab\logs\rsl_rl\cartpole_direct\<날짜시각>` |
| C에 남는 것 | Warp 커널 캐시 `%LOCALAPPDATA%\NVIDIA\warp\Cache`, 내려받은 asset 캐시 `%LOCALAPPDATA%\Temp\...`, uv가 쓰는 Python `%APPDATA%\uv\python`(138 MB, 전부터 있던 것) |

예전에 시스템 Python 3.12에 pip로 깔았던 Isaac Sim 6.0.1(패키지 25개, 18.4 GB)과 Omniverse 캐시(4.8 GB)는 지웠다. C 드라이브 여유가 59 GB에서 81 GB로 늘었다. torch 등 다른 패키지는 다른 프로그램이 써서 남겼다.

## 다시 설치하는 법 (Git Bash)

먼저 PowerShell에서 Omniverse 캐시를 D로 돌린다(`%LOCALAPPDATA%\ov`가 이미 있으면 내용을 옮기거나 지운 뒤에 한다).

```powershell
New-Item -ItemType Directory -Force D:\isaac\ov | Out-Null
New-Item -ItemType Junction -Path "$env:LOCALAPPDATA\ov" -Target D:\isaac\ov
```

그다음 Git Bash에서 설치한다. 같은 코드를 받으려면 기록한 커밋으로 checkout한다(`release/3.0.0` 브랜치는 계속 움직인다).

```bash
export UV_CACHE_DIR='D:\isaac\uv-cache' OMNI_KIT_ACCEPT_EULA=YES PYTHONUTF8=1
git -c core.longpaths=true clone --branch release/3.0.0 https://github.com/isaac-sim/IsaacLab.git /d/isaac/IsaacLab
cd /d/isaac/IsaacLab
git checkout 39394453c
uv sync                                                    # Newton만. 78 s
uv sync --inexact --extra isaacsim --extra importers       # Isaac Sim 6.2와 URDF 변환기를 더함. 208 s
```

- **extra를 빼고 `uv sync`를 다시 하지 않는다.** `uv sync`는 기본이 "정확히 맞추기"라 extra로 깐 Isaac Sim을 지운다. 실행은 `uv run --extra isaacsim --extra importers ...`로 한다(`uv run`은 지우지 않는다).
- **EULA:** `OMNI_KIT_ACCEPT_EULA=YES`로 동의했다. 사용자가 2026-11-21 01시 40분쯤 "승인이 필요한 것은 다 승인한 것"이라고 해서 이 설정으로 진행했다.

## Cartpole 학습 확인 (rsl_rl PPO, headless, 기본 150회)

| 판 | 환경 수 | 결과 | 학습 시간 | 반복 1회 | 평균 보상 (0 → 29 → 59 → 99 → 149회) | GPU 메모리 최고 (시작 전) |
|---|---|---|---|---|---|---|
| Newton (`physics=newton_mjwarp`) | 4096 | 성공 | 79 s (준비 포함 148 s) | 0.32 s | 0.13 → 3.14 → 4.71 → 4.95 → 4.93 | 1562 MiB (1158 MiB) |
| Isaac Sim PhysX, 처음 띄움 | 4096 | 시뮬레이션 시작 중 접근 위반으로 종료(55 s) | - | - | - | 4005 MiB (1160 MiB) |
| Isaac Sim PhysX | 256 | 성공 | 68 s (전체 83 s) | 0.45 s | 0.09 → 2.07 → 4.86 → 4.94 → 4.94 | 3686 MiB |
| Isaac Sim PhysX, 다시 | 4096 | 성공 | 74 s (전체 91 s) | 0.49 s | 0.13 → 2.76 → 4.92 → 4.94 → 4.94 | 4257 MiB (1045 MiB) |

- GPU 메모리는 nvidia-smi가 2초마다 잰 GPU 전체 사용량이라 화면과 다른 프로그램 몫(시작 전 약 1.0~1.2 GB)이 들어 있다. "최고 − 시작 전"은 Newton 판에서 약 0.4 GB, Isaac Sim PhysX 4096개 판에서 약 3.2 GB였다. 다만 PhysX 판은 아래처럼 다른 GPU 작업과 겹쳤을 수 있어서, 이 차이는 같은 조건에서 잰 엔진별 사용량이 아니라 대략적인 크기로만 본다.
- Newton 판의 준비 시간은 해석기 초기화 23 s, 시뮬레이션 시작 39 s였다. Isaac Sim 판은 두 번째부터 시뮬레이션 시작이 2.3 s였다.
- Isaac Sim의 첫 실행 실패는 `physx_manager.py:590`(`play`)에서 텐서 뷰 백엔드가 먼저 내려가며(`RuntimeError: Tensor view backend has been unloaded`) 접근 위반이 난 것이다. 같은 명령을 다시 돌리니 성공했다. 첫 실행 때 캐시를 만드는 과정과 겹친 것으로 보지만 원인은 확인하지 않았다. 로그는 `D:\isaac\logs\p2_cartpole_physx.log`다.
- Isaac Sim 판 두 번은 핑키 Newton 시험이 같은 GPU를 쓰던 때와 겹쳤을 수 있다. 시간 비교는 Newton 판만 깨끗하다.
- 학습한 정책을 눈으로 보는 `isaaclab play ... --viz newton_gl`은 화면이 필요해 아직 하지 않았다.

## 판단에 쓸 것

- 우리 학습은 거의 화면 없이 돌리므로 Newton으로 충분하다. GPU 메모리도 Newton 쪽이 적게 쓰는 것으로 보인다(Cartpole 4096개에서 대략 0.4 GB 대 3.2 GB. PhysX 판은 다른 GPU 작업과 겹쳤을 수 있어 정확한 비교는 아니다).
- Isaac Sim은 지금 3080에서도 Cartpole이 돌지만 공식 최소 사양 밖이다. 화면이 필요한 일(Kit viewer, 카메라 센서)에만 쓴다.
- 핑키를 Newton에 올린 바퀴 속도 제어와 접촉 시험은 브랜치 `feat/newton-pinky-check-2026-11-21`에 따로 남긴다.
