"""fleet_core/pending.py의 WAITING 표: 건너뛰는 시험은 모두 실제 파일이고 이유는 하나다."""
from pathlib import Path

import pytest
from fleet_core.pending import NEEDS_ZONE_TRAFFIC, WAITING, waiting_reason

REPO = Path(__file__).resolve().parents[3]
PACKAGES = {"fleet_core": REPO / "pc" / "fleet_core", "fake_robots": REPO / "sim" / "fake_robots",
            "fleet_manager": REPO / "pc" / "fleet_manager",
            "fleet_gz_sim": REPO / "sim" / "fleet_gz_sim"}


def test_the_reason_is_the_shared_one():
    assert NEEDS_ZONE_TRAFFIC == "needs the zone traffic layer (stage 3)"
    assert set(WAITING.values()) <= {NEEDS_ZONE_TRAFFIC}


@pytest.mark.skipif(not (REPO / "pc" / "fleet_manager").is_dir(), reason="only in the source tree")
@pytest.mark.parametrize("package, name", sorted(WAITING))
def test_every_waiting_module_is_a_test_file_of_its_package(package, name):
    assert (PACKAGES[package] / "test" / name).is_file()


def test_the_reason_is_read_from_the_package_folder_and_the_file_name(monkeypatch):
    # 지금은 기다리는 모듈이 없다(시나리오 통합과 fms_restart도 되살렸다). 표에 넣으면 읽는지 본다
    monkeypatch.setitem(WAITING, ("fleet_manager", "test_fms_restart.py"), NEEDS_ZONE_TRAFFIC)
    assert waiting_reason(Path("x/fleet_manager/test/test_fms_restart.py")) == NEEDS_ZONE_TRAFFIC
    assert waiting_reason(Path("x/fleet_manager/test/test_scenario_integration.py")) is None
    assert waiting_reason(Path("x/fake_robots/test/test_manager.py")) is None
    assert waiting_reason(Path("x/fleet_core/test/test_zones.py")) is None
