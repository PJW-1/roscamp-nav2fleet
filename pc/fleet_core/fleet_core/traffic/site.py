from __future__ import annotations

from fleet_core.fleet_map import fleet_map_from_text, load_fleet_map


def load_site(path=None):
    return load_fleet_map(path)


def site_from_text(text: str):
    return fleet_map_from_text(text)
