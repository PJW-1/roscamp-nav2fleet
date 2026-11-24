"""Gazebo 시뮬을 띄워도 되는 네트워크 설정인지 본다.
시뮬은 실물과 같은 토픽 이름을 쓰므로 도메인 43 이상, LOCALHOST 탐색, 고정 피어 없음, Fast DDS일 때만 통과시킨다.
"""
from collections.abc import Mapping
from pathlib import Path

MIN_DOMAIN = 43
# ROS 문서의 안전 도메인 범위 0~101(그 위는 리눅스 임시 포트와 겹치거나 프로세스 수가 적다)
MAX_DOMAIN = 101
DDS = "rmw_fastrtps_cpp"
# Fast DDS가 읽는 환경 변수 이름(2.14와 3.x)
FAST_DDS_VARIABLES = ("ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE",
                      "FASTDDS_DEFAULT_PROFILES_FILE", "FASTDDS_ENVIRONMENT_FILE",
                      "FASTDDS_BUILTIN_TRANSPORTS")
# 작업 폴더에 있으면 Fast DDS가 저절로 읽는 파일
FAST_DDS_FILES = ("DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml")


def problems(env: Mapping[str, str], *, rmw: str | None = None,
             workdir: Path | None = None) -> list[str]:
    """격리가 안 된 이유 목록. 비어 있으면 띄워도 된다.
    rmw: 실제 RMW(없으면 RMW_IMPLEMENTATION, 그것도 없으면 Jazzy 기본). workdir: 노드 작업 폴더.
    """
    found = []
    domain = env.get("ROS_DOMAIN_ID", "")
    # ASCII 숫자 세 자리까지만 int()에 넘긴다
    digits = domain.isascii() and domain.isdigit() and len(domain) <= 3
    if not digits or not MIN_DOMAIN <= int(domain) <= MAX_DOMAIN:
        found.append(f"ROS_DOMAIN_ID={domain!r}: {MIN_DOMAIN}~{MAX_DOMAIN} 사이로 정해야 한다")
    scope = env.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "")
    if scope != "LOCALHOST":
        found.append(f"ROS_AUTOMATIC_DISCOVERY_RANGE={scope!r}: LOCALHOST여야 한다")
    if env.get("ROS_STATIC_PEERS", ""):
        found.append("ROS_STATIC_PEERS가 있다: 다른 컴퓨터와 이어지므로 비워야 한다")
    rmw = rmw or env.get("RMW_IMPLEMENTATION", DDS)
    if rmw != DDS:
        found.append(f"RMW={rmw!r}: 저장소 기준인 {DDS}만 시험했다(RMW_IMPLEMENTATION)")
    for name in FAST_DDS_VARIABLES:
        if env.get(name, ""):
            found.append(f"{name}가 있다: LOCALHOST 밖으로 찾을 수 있으므로 비워야 한다")
    if workdir is not None:
        for name in FAST_DDS_FILES:
            if (workdir / name).exists():
                found.append(f"{workdir / name}가 있다: Fast DDS가 저절로 읽으므로 치워야 한다")
    return found


def require_isolated(env: Mapping[str, str], *, rmw: str | None = None,
                     workdir: Path | None = None) -> None:
    """격리가 안 됐으면 이유를 모두 담아 RuntimeError를 낸다."""
    found = problems(env, rmw=rmw, workdir=workdir)
    if found:
        raise RuntimeError("Gazebo 시뮬을 띄우지 않는다:\n  " + "\n  ".join(found))
