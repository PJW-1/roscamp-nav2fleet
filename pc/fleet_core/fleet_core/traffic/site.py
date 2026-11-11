"""지도(site)를 파일이나 기록에서 읽는 자리. 1단계에는 지도가 없다.

레인 지도는 지웠고 새 지도(fleet_core/traffic/fleet_map.py, 2단계)는 이 가지에 아직 없다.
지도가 필요한 코드는 지도를 인자로 받는다. 파일(map_file 파라미터)이나 기록 첫 줄에서 지도를
읽어야 하는 곳만 여기를 부르고, 그 모듈이 생기기 전에는 NoFleetMap을 낸다.

2단계가 맞출 것: fleet_map.load_fleet_map(path) (None이면 기본 배치)와
fleet_map.fleet_map_from_text(text)(재생용). 이름이 다르면 아래 두 줄만 고친다.
"""
from __future__ import annotations

import importlib

from fleet_core.pending import NEEDS_FLEET_MAP

FLEET_MAP_MODULE = "fleet_core.traffic.fleet_map"


class NoFleetMap(RuntimeError):
    """지도 모듈이 아직 없다."""


def _fleet_map():
    try:
        return importlib.import_module(FLEET_MAP_MODULE)
    except ModuleNotFoundError as error:
        missing = error.name or ""
        if missing != FLEET_MAP_MODULE and not FLEET_MAP_MODULE.startswith(missing + "."):
            raise  # 지도 모듈 안의 임포트가 틀렸다. 감추지 않는다
        raise NoFleetMap(f"{NEEDS_FLEET_MAP}: {FLEET_MAP_MODULE} is not on this branch") from None


def load_site(path=None):
    """지도 파일을 읽는다. path가 없으면 기본 배치."""
    return _fleet_map().load_fleet_map(path)


def site_from_text(text: str):
    """기록 첫 줄에 남긴 지도 본문(site.source_text)으로 지도를 다시 만든다."""
    return _fleet_map().fleet_map_from_text(text)
