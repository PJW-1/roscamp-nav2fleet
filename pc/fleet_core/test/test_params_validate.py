"""load_params가 파일을 검사한다(2026-11-16).

params_file로 받은 yaml에서 키 하나가 빠지면 코드 기본값이 조용히 쓰였다. 예: stall_s가 빠지면 그때
기본값 6 s가 되어 Pode가 BLOCKED를 보고하기(10 s) 전에 관제가 goal을 거둔다. 그래서 빠진 키는 오류다.
일부러 빼 둔 키는 OPTIONAL에 이름으로 적는다. 시간·거리 값은 유한하고 양수여야 하며, 0이
'끔'인 키만 0을 허용한다.
"""
import dataclasses
import json
import math
import sys
from pathlib import Path

import pytest
import yaml
from fleet_core.common.params import (
    DEFAULT_PARAMS,
    OPTIONAL,
    ZERO_MEANS_OFF,
    FleetParams,
    load_params,
)

SHIPPED = yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
FIELDS = {f.name: f for f in dataclasses.fields(FleetParams)}


def write(tmp_path, data):
    path = tmp_path / "params.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def without(key):
    return {k: v for k, v in SHIPPED.items() if k != key}


def test_the_shipped_file_loads():
    assert load_params().stall_s == SHIPPED["stall_s"]


def test_the_only_intended_omission_is_zone_clear_m():
    """zone_clear_m은 0이면 footprint_radius_m + release_margin_m로 놓는다(test_zone_release)."""
    assert OPTIONAL == frozenset({"zone_clear_m"})
    assert set(FIELDS) - set(SHIPPED) == OPTIONAL


def test_optional_keys_have_a_code_default():
    for name in OPTIONAL:
        assert FIELDS[name].default is not dataclasses.MISSING


@pytest.mark.parametrize("key", ["stall_s", "idle_watch_s", "silent_limit_s", "start_limit_s"])
def test_a_missing_key_with_a_code_default_is_an_error(tmp_path, key):
    with pytest.raises(ValueError, match=key):
        load_params(write(tmp_path, without(key)))


def test_a_missing_key_without_a_code_default_names_the_key(tmp_path):
    with pytest.raises(ValueError, match="basket_capacity"):
        load_params(write(tmp_path, without("basket_capacity")))


def test_an_unknown_key_names_the_key(tmp_path):
    with pytest.raises(ValueError, match="stall_secs"):
        load_params(write(tmp_path, {**SHIPPED, "stall_secs": 12.0}))


def test_an_optional_key_may_be_given(tmp_path):
    assert load_params(write(tmp_path, {**SHIPPED, "zone_clear_m": 0.11})).zone_clear_m == 0.11


def test_a_file_that_is_not_a_mapping_is_an_error(tmp_path):
    path = tmp_path / "params.yaml"
    path.write_text("- 1\n- 2\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_params(path)


@pytest.mark.parametrize("bad", [-1.0, math.inf, math.nan])
@pytest.mark.parametrize("key", ["stall_s", "footprint_radius_m", "nav_speed_mps",
                                 "silent_limit_s", "battery_low_pct"])
def test_time_distance_and_speed_values_are_finite_and_not_negative(tmp_path, key, bad):
    with pytest.raises(ValueError, match=key):
        load_params(write(tmp_path, {**SHIPPED, key: bad}))


@pytest.mark.parametrize("key", ["stall_s", "heartbeat_period_s", "follow_speed_mps",
                                 "release_margin_m"])
def test_zero_is_an_error_where_zero_does_not_mean_off(tmp_path, key):
    with pytest.raises(ValueError, match=key):
        load_params(write(tmp_path, {**SHIPPED, key: 0.0}))


def test_zero_means_off_keys_are_the_documented_ones():
    assert ZERO_MEANS_OFF == frozenset({"zone_clear_m", "idle_watch_s", "silent_limit_s",
                                        "start_limit_s", "battery_low_pct"})


@pytest.mark.parametrize("key", sorted(ZERO_MEANS_OFF - OPTIONAL))
def test_zero_is_allowed_where_it_means_off(tmp_path, key):
    assert getattr(load_params(write(tmp_path, {**SHIPPED, key: 0.0})), key) == 0.0


def test_a_percentage_above_100_is_an_error(tmp_path):
    with pytest.raises(ValueError, match="battery_charged_pct"):
        load_params(write(tmp_path, {**SHIPPED, "battery_charged_pct": 120.0}))


def test_score_values_may_be_negative(tmp_path):
    params = load_params(write(tmp_path, {**SHIPPED, "score_same_call": -9.0}))
    assert params.score_same_call == -9.0


RECORD_COMPAT = {"idle_watch_s", "silent_limit_s", "start_limit_s"}


def test_code_defaults_match_the_shipped_file_except_record_compat_watches():
    """FleetParams(**fields)로 바로 만들 때(tools/replay.py의 옛 기록)의 기본값.

    stall_s는 파일과 같다(12 s). 0이 '끔'인 세 감시는 그 키가 없는 옛 기록을 감시 없이
    재생하려고 0으로 둔다(test_zone_silent, test_zone_no_start). load_params는 빠진 키를
    오류로 내므로 운영 파일에는 이 기본값이 쓰이지 않는다.
    """
    differ = {name: (f.default, SHIPPED[name]) for name, f in FIELDS.items()
              if f.default is not dataclasses.MISSING and name in SHIPPED
              and f.default != SHIPPED[name]}
    assert set(differ) == RECORD_COMPAT
    assert all(FIELDS[name].default == 0.0 for name in RECORD_COMPAT)
    assert RECORD_COMPAT <= ZERO_MEANS_OFF
    assert FIELDS["stall_s"].default == SHIPPED["stall_s"] == 12.0


# ---- 접미사 밖 숫자 키와 타입 검사(Codex 리뷰 e0acac6) ----
@pytest.mark.parametrize("bad", [0, -1, True, 2.5, "3"])
def test_release_confirm_samples_is_a_positive_integer(tmp_path, bad):
    """0이나 음수면 첫 보고만으로 구역을 놓는다(progress.py _counted)."""
    with pytest.raises(ValueError, match="release_confirm_samples"):
        load_params(write(tmp_path, {**SHIPPED, "release_confirm_samples": bad}))


@pytest.mark.parametrize("bad", [-1.0, math.nan, math.inf, "4", True])
def test_eta_fill_s_per_item_is_finite_and_not_negative(tmp_path, bad):
    with pytest.raises(ValueError, match="eta_fill_s_per_item"):
        load_params(write(tmp_path, {**SHIPPED, "eta_fill_s_per_item": bad}))


def test_eta_fill_s_per_item_may_be_zero(tmp_path):
    params = load_params(write(tmp_path, {**SHIPPED, "eta_fill_s_per_item": 0.0}))
    assert params.eta_fill_s_per_item == 0.0


NUMBER_KEYS = sorted(name for name, f in FIELDS.items() if f.type in ("int", "float"))


@pytest.mark.parametrize("key", NUMBER_KEYS)
@pytest.mark.parametrize("bad", [True, "1"])
def test_every_number_key_refuses_booleans_and_strings(tmp_path, key, bad):
    with pytest.raises(ValueError, match=key):
        load_params(write(tmp_path, {**SHIPPED, key: bad}))


@pytest.mark.parametrize("key", sorted(n for n, f in FIELDS.items() if f.type == "float"))
def test_every_float_key_refuses_nan(tmp_path, key):
    with pytest.raises(ValueError, match=key):
        load_params(write(tmp_path, {**SHIPPED, key: math.nan}))


@pytest.mark.parametrize("key", sorted(n for n, f in FIELDS.items() if f.type == "bool"))
def test_every_flag_refuses_numbers(tmp_path, key):
    with pytest.raises(ValueError, match=key):
        load_params(write(tmp_path, {**SHIPPED, key: 1}))


def test_a_float_key_written_as_an_integer_loads(tmp_path):
    assert load_params(write(tmp_path, {**SHIPPED, "stall_s": 12})).stall_s == 12


# ---- 옛 기록 재생: stall_s 없는 기록은 그때 기본값 6 s(Codex 리뷰 e0acac6) ----
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import replay  # noqa: E402


def recorded_head():
    from fleet_core.fleet_map import load_fleet_map
    from fleet_core.manager import FleetManager
    core = FleetManager(load_fleet_map("d2"), load_params(), ["pode1"], lambda: 0.0)
    lines = []
    core.record_to(lines.append)
    return json.loads(lines[0])


def test_replay_of_a_record_without_stall_s_uses_the_old_6_s():
    head = recorded_head()
    del head["params"]["stall_s"]
    assert replay.start_core(head, lambda: 0.0).params.stall_s == 6.0


def test_replay_of_a_record_with_stall_s_uses_it():
    head = recorded_head()
    assert replay.start_core(head, lambda: 0.0).params.stall_s == SHIPPED["stall_s"] == 12.0
