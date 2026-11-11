"""지도(2단계)를 기다리는 시험은 건너뛴다(fleet_core/pending.py). fleet_core 없이도 읽힌다."""
try:
    from fleet_core.pending import skip_waiting_module
except ImportError:  # fleet_core가 없는 곳에서는 기다리는 시험이 스스로 건너뛴다(importorskip)
    skip_waiting_module = None


def pytest_pycollect_makemodule(module_path, parent):
    return skip_waiting_module(module_path, parent) if skip_waiting_module else None
