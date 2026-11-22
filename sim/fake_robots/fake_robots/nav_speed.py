"""주행 최고 속도의 단일 출처(pode_nav_config/params/nav_speed.yaml, 사용자 결정 2026-11-22).

가짜 로봇은 이 값에 로봇마다 배율을 곱해 간다. 배율은 1을 넘지 않으므로 출처보다 빨리 가는
가짜 로봇은 없다. 배율 범위는 Isaac 대본 로봇의 robot_speed_scale에서 1 위를 뺀 것이다.
저장소 안에서 돌면 저장소의 파일을, 설치본이면 pode_nav_config의 share 파일을 읽는다.
"""
from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path

import yaml

SOURCE_REL = Path("params") / "nav_speed.yaml"
REPO_SOURCE = Path(__file__).resolve().parents[3] / "pode" / "pode_nav_config" / SOURCE_REL
SCALE_RANGE = (0.67, 1.0)  # 시나리오 파일과 세계의 로봇 배율. 시험 대역은 더 느려도 된다


def source_file() -> Path:
    if REPO_SOURCE.is_file():
        return REPO_SOURCE
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("pode_nav_config")) / SOURCE_REL


def nav_speed_mps(path=None) -> float:
    """단일 출처의 주행 최고 속도(m/s). path를 주지 않으면 한 번만 읽는다."""
    if path is None:
        return _from_source()
    return _read(Path(path))


@lru_cache(maxsize=1)
def _from_source() -> float:
    return _read(source_file())


def _read(path: Path) -> float:
    value = float(yaml.safe_load(path.read_text(encoding="utf-8"))["nav_speed_mps"])
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"{path}: nav_speed_mps must be above 0, got {value!r}")
    return value
