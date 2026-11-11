"""지도(site)를 파일이나 기록에서 읽는 자리.

지도가 필요한 코드는 지도를 인자로 받는다. 파일(map_file 파라미터)이나 기록 첫 줄에서 지도를
읽어야 하는 곳만 여기를 부른다. 지도는 구역 지도(fleet_core/fleet_map)다.
"""
from __future__ import annotations

from fleet_core.fleet_map import fleet_map_from_text, load_fleet_map


def load_site(path=None):
    """지도 파일이나 배치 이름(d2, d, c)으로 읽는다. 없으면 기본 배치."""
    return load_fleet_map(path)


def site_from_text(text: str):
    """기록 첫 줄에 남긴 지도 본문(site.source_text)으로 지도를 다시 만든다."""
    return fleet_map_from_text(text)
