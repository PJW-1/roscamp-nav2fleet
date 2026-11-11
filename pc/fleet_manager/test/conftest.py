"""교통 층(3단계)을 기다리는 시험은 건너뛴다(fleet_core/pending.py)."""
from fleet_core.pending import skip_waiting_module


def pytest_pycollect_makemodule(module_path, parent):
    return skip_waiting_module(module_path, parent)
