import json
import os

import numpy as np
import pytest
import report_tables as rt


def fake_run(path, success):
    os.makedirs(path)
    n = len(success)
    code = np.where(np.array(success), 0, 1)
    np.savez(
        path + "/episodes.npz",
        env=np.arange(n),
        code=code,
        keys=np.array(["success", "collision", "oob", "timeout"]),
    )
    s = float(np.mean(success))
    json.dump(
        {"rates": {"success": s, "collision": 1 - s, "timeout": 0.0}, "w_flips_per_min": 50.0},
        open(path + "/eval.json", "w"),
    )


def test_item5_refuses_a_missing_eval(tmp_path, monkeypatch):
    r = str(tmp_path)
    monkeypatch.setattr(rt, "ITEM5", [("ctrl", "ctrl", 0.12), ("c", "c", 0.12)])
    for tag in ("ctrl", "c"):
        for s in rt.SETS:
            if (tag, s) != ("c", "head"):
                fake_run(rt.run_dir(r, tag, s), [1, 0, 1, 1])
    with pytest.raises(FileNotFoundError):
        rt.item5(r)


def test_item5_cap_rows_skip_only_handover(tmp_path, monkeypatch):
    r = str(tmp_path)
    monkeypatch.setattr(rt, "ITEM5", [("d", "d08", 0.08)])
    for s in ("rand", "head"):
        fake_run(rt.run_dir(r, "d08", s), [1, 0, 0, 1])
    (row,) = rt.item5(r)
    assert row["rand_success"] == 0.5 and row["ho_success"] == ""
