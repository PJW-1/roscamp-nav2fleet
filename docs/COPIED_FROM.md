# 어디서 복사했나

이 레포의 첫 커밋은 `PJW-1/roscamp-freefloor`의 `main` 커밋 `9375a80`에서 아래 경로를 **그대로 복사**한 것이다(fork가 아니다). 그 뒤의 커밋에서 자유바닥·레인 전용 코드를 덜어 내고 새 교통 층을 만든다.

| 이 레포의 경로 | 원래 경로 (`roscamp-freefloor` 9375a80) | 비고 |
|---|---|---|
| `interfaces/fleet_interfaces` | 같음 | |
| `pc/fleet_core`, `pc/fleet_manager`, `pc/central` | 같음 | 첫 커밋에는 `traffic/floor/*`와 레인 전용 파일도 들어 있고, 다음 커밋에서 지운다 |
| `pinky/pinky_fsm`, `pinky/pinky_nav_config`, `pinky/pinky_dock` | 같음 | |
| `sim/fake_robots`, `sim/fleet_gz_sim` | 같음 | 옛 지도·site·시나리오는 다음 커밋에서 지운다 |
| `tools/` | 같음 | `replay.py`와 시험 틀이 쓰는 것만 남긴다 |
| `arm/`, `bringup/` | 같음 | |
| `docs/핑키_계약.md`, `docs/실측_2026-11-10.md`, `docs/GG_공통설계.md` | 같음 | |
| `.gitattributes`, `.gitignore`, `ruff.toml`, `iponic.repos`, `config/`, `scripts/` | 같음 | 레포 설정, 벤더 의존(pinky_pro), 로봇 목록, 인터페이스 점검 |

가져오지 않은 것: `.github/`(CI), `README.md`(새로 씀), `docs/`의 작업 기록들과 `docs/freefloor/`, `docs/관제_동작.md`, `docs/개선점_2026-11-06.md`.

옛 작업 기록은 원래 레포에 있다: <https://github.com/PJW-1/roscamp-freefloor/tree/9375a80/docs>

확인하는 법: 첫 커밋의 디렉터리 tree 해시가 원래 커밋과 같다.

```
git rev-parse <첫 커밋>:pc            # 이 레포
git rev-parse 9375a80:pc              # roscamp-freefloor
```
