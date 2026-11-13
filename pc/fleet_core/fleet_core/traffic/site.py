# ---- 현장 지도 불러오기 진입점 ----
from __future__ import annotations

from fleet_core.fleet_map import fleet_map_from_text, load_fleet_map


# ---- 파일이나 문자열에서 현장 지도 읽기 ----
def load_site(path=None):
    return load_fleet_map(path)


def site_from_text(text: str):
    return fleet_map_from_text(text)
