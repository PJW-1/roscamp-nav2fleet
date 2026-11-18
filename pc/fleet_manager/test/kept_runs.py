# ---- 실패한 시험의 결과 폴더 남기기 ----
import os
import re
import shutil
import time
from pathlib import Path

import pytest


def keep_root() -> Path:
    return Path(os.environ.get("FLEET_TEST_KEEP_DIR") or Path.home() / "fleet_test_failures")


def keep(name: str, run_dir: Path, root: Path, stamp: str) -> Path:
    base = re.sub(r"[^\w.-]+", "-", name).strip("-") + "_" + stamp
    kept = root / base
    count = 1
    while kept.exists():
        count += 1
        kept = root / f"{base}_{count}"
    shutil.copytree(run_dir, kept)
    return kept


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    run_dir = getattr(item, "funcargs", {}).get("tmp_path")
    if report.when != "call" or not report.failed or run_dir is None:
        return
    try:
        kept = keep(item.name, run_dir, keep_root(), time.strftime("%Y%m%d_%H%M%S"))
    except OSError as error:
        report.sections.append(("kept run", f"the run could not be kept: {error}"))
        return
    report.sections.append(("kept run", f"the run is kept in {kept}"))
