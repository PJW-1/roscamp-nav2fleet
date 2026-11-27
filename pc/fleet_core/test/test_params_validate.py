"""load_params가 yaml 파일을 검사한다: 빠진 키는 오류(OPTIONAL에 적은 키만 예외)고, 시간·거리 값은 유한하고 양수여야 하며 0이 '끔'인 키만 0을 허용한다."""
import dataclasses
import json
import math
import sys
from dataclasses import replace
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
                                 "release_margin_m", "blocked_reset_m", "blocked_reset_s"])
def test_zero_is_an_error_where_zero_does_not_mean_off(tmp_path, key):
    with pytest.raises(ValueError, match=key):
        load_params(write(tmp_path, {**SHIPPED, key: 0.0}))


def test_zero_means_off_keys_are_the_documented_ones():
    assert ZERO_MEANS_OFF == frozenset({"zone_clear_m", "idle_watch_s", "silent_limit_s",
                                        "start_limit_s", "battery_low_pct", "nav_pass_m",
                                        "nav_turn_rps"})


@pytest.mark.parametrize("key", sorted(ZERO_MEANS_OFF - OPTIONAL))
def test_zero_is_allowed_where_it_means_off(tmp_path, key):
    assert getattr(load_params(write(tmp_path, {**SHIPPED, key: 0.0})), key) == 0.0


def test_a_percentage_above_100_is_an_error(tmp_path):
    with pytest.raises(ValueError, match="battery_charged_pct"):
        load_params(write(tmp_path, {**SHIPPED, "battery_charged_pct": 120.0}))


def test_score_values_may_be_negative(tmp_path):
    params = load_params(write(tmp_path, {**SHIPPED, "score_same_call": -9.0}))
    assert params.score_same_call == -9.0


RECORD_COMPAT = {"idle_watch_s", "silent_limit_s", "start_limit_s", "nav_pass_m", "nav_turn_rps"}


def test_code_defaults_match_the_shipped_file_except_record_compat_watches():
    """FleetParams(**fields)로 바로 만들 때(tools/replay.py의 옛 기록)의 기본값. 0이 '끔'인 감시는 그 키가 없는 옛 기록을 감시 없이 재생하려고 0이다."""
    differ = {name: (f.default, SHIPPED[name]) for name, f in FIELDS.items()
              if f.default is not dataclasses.MISSING and name in SHIPPED
              and f.default != SHIPPED[name]}
    assert set(differ) == RECORD_COMPAT
    assert all(FIELDS[name].default == 0.0 for name in RECORD_COMPAT)
    assert RECORD_COMPAT <= ZERO_MEANS_OFF
    assert FIELDS["stall_s"].default == SHIPPED["stall_s"] == 12.0


# ---- 접미사 밖 숫자 키와 타입 검사 ----
@pytest.mark.parametrize("bad", [0, -1, True, 2.5, "3"])
def test_release_confirm_samples_is_a_positive_integer(tmp_path, bad):
    """0이나 음수면 첫 보고만으로 구역을 놓는다(progress.py _counted)."""
    with pytest.raises(ValueError, match="release_confirm_samples"):
        load_params(write(tmp_path, {**SHIPPED, "release_confirm_samples": bad}))


@pytest.mark.parametrize("bad", [0, -1, True, 2.5, "3"])
def test_report_max_in_flight_is_a_positive_integer(tmp_path, bad):
    """IR-41 동시 보고 상한. 0이면 보고가 하나도 나가지 않는다."""
    with pytest.raises(ValueError, match="report_max_in_flight"):
        load_params(write(tmp_path, {**SHIPPED, "report_max_in_flight": bad}))


def test_report_max_in_flight_ships_as_4():
    assert load_params().report_max_in_flight == 4


@pytest.mark.parametrize("bad", [0, -1, True, 2.5, "3"])
def test_bridge_max_in_flight_is_a_positive_integer(tmp_path, bad):
    with pytest.raises(ValueError, match="bridge_max_in_flight"):
        load_params(write(tmp_path, {**SHIPPED, "bridge_max_in_flight": bad}))


def test_the_bridge_retry_cannot_start_above_its_cap(tmp_path):
    with pytest.raises(ValueError, match="bridge_retry_first_s"):
        load_params(write(tmp_path, {**SHIPPED, "bridge_retry_first_s": 11.0}))


def test_the_bridge_timings_ship_as_the_ir_40_table():
    p = load_params()
    assert (p.bridge_reply_limit_s, p.bridge_retry_first_s, p.bridge_retry_max_s,
            p.bridge_max_in_flight) == (3.0, 1.0, 10.0, 4)


def test_the_station_time_estimates_are_gone():
    assert not {"eta_fill_s_per_item", "eta_fill_base_s", "eta_swap_s"} & set(FIELDS)


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


def test_a_huge_integer_in_a_float_key_names_the_key(tmp_path):
    """math.isfinite(10**400)은 OverflowError다. 칸 이름이 든 ValueError여야 한다."""
    with pytest.raises(ValueError, match="stall_s"):
        load_params(write(tmp_path, {**SHIPPED, "stall_s": 10**400}))


def test_a_float_key_written_as_an_integer_loads(tmp_path):
    assert load_params(write(tmp_path, {**SHIPPED, "stall_s": 12})).stall_s == 12


# ---- 옛 기록 재생: stall_s 없는 기록은 그때 기본값 6 s ----
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


@pytest.mark.parametrize("recorded, played", [(None, False), (False, False), (True, True)],
                         ids=["missing", "false", "true"])
def test_replay_keeps_the_order_model_of_the_record(recorded, played):
    """use_order_model 키가 없는 옛 기록은 그때의 기본값(끔)으로 재생하고, 키가 있으면 그 값이다."""
    head = recorded_head()
    if recorded is None:
        del head["params"]["use_order_model"]
    else:
        head["params"]["use_order_model"] = recorded
    assert replay.start_core(head, lambda: 0.0).params.use_order_model is played


def test_replay_of_a_record_with_stall_s_uses_it():
    head = recorded_head()
    assert replay.start_core(head, lambda: 0.0).params.stall_s == SHIPPED["stall_s"] == 12.0


def old_stall_record(path):
    """stall_s 키가 없던 때의 기록 흉내: 6 s 코어로 기록하고 머리에서 stall_s를 지운다."""
    from fleet_core.common.commands import OrderRecord
    from fleet_core.fleet_map import load_fleet_map
    from fleet_core.manager import FleetManager
    now = [0.0]
    core = FleetManager(load_fleet_map("d2"), replace(load_params(), stall_s=6.0), ["pode1"],
                        lambda: now[0])
    lines = []
    core.record_to(lines.append)
    x, y = core.site.node_pose("P-1")
    for _ in range(3):
        core.on_robot_state("pode1", x, y, 0.0, "IDLE", 90.0)
    core.on_order_board("RUN", [OrderRecord("O-001", {"A": 1})])
    for _ in range(100):
        now[0] += 0.1
        core.on_robot_state("pode1", x, y, 0.0, "MOVING", 90.0)
        core.tick()
    head = json.loads(lines[0])
    del head["params"]["stall_s"]
    path.write_text("\n".join([json.dumps(head), *lines[1:]]) + "\n", encoding="utf-8")
    return path, stall_times(lines)


def stall_times(lines):
    """pode1의 GOAL_STALLED 사건 시각."""
    events = (json.loads(line) for line in lines[1:])
    return [e["t"] for e in events
            if e.get("code") == "GOAL_STALLED" and e.get("robot") == "pode1"]


def replay_lines(path, monkeypatch):
    """replay()를 돌리고 재생 코어가 낸 줄도 받는다(기록 쓰기를 갈라 받음)."""
    produced = []
    start = replay.start_core

    def start_and_tee(head, clock):
        core = start(head, clock)
        record_to = core.record_to

        def tee(write):
            record_to(lambda line: (write(line), produced.append(line)))
        core.record_to = tee
        return core

    monkeypatch.setattr(replay, "start_core", start_and_tee)
    return replay.replay(path), produced


def test_replay_of_an_old_record_judges_the_stall_at_6_s(tmp_path, monkeypatch):
    path, recorded = old_stall_record(tmp_path / "old.jsonl")
    assert recorded and recorded[0] == pytest.approx(6.3, abs=0.05)
    result, produced = replay_lines(path, monkeypatch)
    assert result.same and result.compared, result
    assert stall_times(produced) == recorded


def test_without_the_compat_default_the_old_record_would_replay_differently(tmp_path,
                                                                            monkeypatch):
    """위 시험이 stall_s에 기대는지 확인: 호환 기본값을 빼면 12 s로 판정해 6.3 s 사건이 없다."""
    path, recorded = old_stall_record(tmp_path / "old.jsonl")
    monkeypatch.setattr(replay, "RECORD_DEFAULTS", {})
    result, produced = replay_lines(path, monkeypatch)
    assert result.compared and not result.same
    assert len(produced) > 1, "재생 줄을 받지 못하면 아래 단언이 빈 목록으로 통과한다"
    assert all(abs(t - recorded[0]) > 1.0 for t in stall_times(produced))


@pytest.mark.parametrize("bad", [-1.0, 0.0, float("inf")])
def test_the_lost_dock_heading_tolerance_is_a_positive_angle(tmp_path, bad):
    with pytest.raises(ValueError, match="lost_dock_yaw_deg"):
        load_params(write(tmp_path, {**SHIPPED, "lost_dock_yaw_deg": bad}))
