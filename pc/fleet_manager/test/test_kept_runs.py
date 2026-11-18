import os
import subprocess
import sys
from pathlib import Path

import kept_runs

HERE = Path(__file__).parent
FAILING = """
def test_it_breaks(tmp_path):
    (tmp_path / "summary.json").write_text("{}", encoding="utf-8")
    (tmp_path / "bag").mkdir()
    (tmp_path / "bag" / "data.txt").write_text("x", encoding="utf-8")
    assert False


def test_it_holds(tmp_path):
    (tmp_path / "summary.json").write_text("{}", encoding="utf-8")


def test_it_breaks_with_no_folder():
    assert False
"""


def test_a_kept_run_is_a_copy_named_after_the_test_and_the_time(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    (run / "launch.log").write_text("log", encoding="utf-8")
    kept = kept_runs.keep("test_scenario[stuck_robot]", run, tmp_path / "kept", "20261118_101500")
    assert kept == tmp_path / "kept" / "test_scenario-stuck_robot_20261118_101500"
    assert (kept / "launch.log").read_text(encoding="utf-8") == "log"


def test_a_second_run_kept_in_the_same_second_gets_its_own_folder(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    first = kept_runs.keep("test_a", run, tmp_path / "kept", "20261118_101500")
    second = kept_runs.keep("test_a", run, tmp_path / "kept", "20261118_101500")
    assert first != second and second.name == "test_a_20261118_101500_2"


def test_the_keep_folder_comes_from_the_environment_or_the_home_folder(monkeypatch):
    monkeypatch.setenv("FLEET_TEST_KEEP_DIR", "/somewhere/else")
    assert kept_runs.keep_root() == Path("/somewhere/else")
    monkeypatch.delenv("FLEET_TEST_KEEP_DIR")
    assert kept_runs.keep_root() == Path.home() / "fleet_test_failures"


def test_only_a_failed_test_with_a_run_folder_is_kept_and_the_report_says_where(tmp_path):
    (tmp_path / "suite").mkdir()
    (tmp_path / "suite" / "test_sample.py").write_text(FAILING, encoding="utf-8")
    env = dict(os.environ, FLEET_TEST_KEEP_DIR=str(tmp_path / "kept"), PYTHONPATH=str(HERE))
    done = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "-p", "kept_runs", "-p", "no:cacheprovider",
         "--rootdir", str(tmp_path / "suite"), str(tmp_path / "suite")],
        env=env, capture_output=True, text=True, timeout=120)
    kept = sorted(path.name for path in (tmp_path / "kept").iterdir())
    assert len(kept) == 1 and kept[0].startswith("test_it_breaks_2"), (kept, done.stdout)
    folder = tmp_path / "kept" / kept[0]
    assert (folder / "summary.json").exists() and (folder / "bag" / "data.txt").exists()
    assert f"the run is kept in {folder}" in done.stdout
    assert "2 failed, 1 passed" in done.stdout


def test_a_run_that_cannot_be_kept_does_not_hide_the_failure(tmp_path):
    (tmp_path / "suite").mkdir()
    (tmp_path / "suite" / "test_sample.py").write_text(FAILING, encoding="utf-8")
    (tmp_path / "kept").write_text("a file where the folder should be", encoding="utf-8")
    env = dict(os.environ, FLEET_TEST_KEEP_DIR=str(tmp_path / "kept"), PYTHONPATH=str(HERE))
    done = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "-p", "kept_runs", "-p", "no:cacheprovider",
         "--rootdir", str(tmp_path / "suite"), str(tmp_path / "suite")],
        env=env, capture_output=True, text=True, timeout=120)
    assert "2 failed, 1 passed" in done.stdout and "INTERNALERROR" not in done.stdout, done.stdout
    assert "the run could not be kept:" in done.stdout
