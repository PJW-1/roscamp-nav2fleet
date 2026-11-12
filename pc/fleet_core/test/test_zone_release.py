"""뒤 구역 놓기: 로봇 중심이 아니라 몸 전체가 빠져나간 뒤(추가 지시서 2의 5절).

보고 중심이 구역에서 footprint_radius_m(발자국 0.12 x 0.12 m의 반 대각선) + release_margin_m
넘게 떨어진 보고가 release_confirm_samples번 이어지면 놓는다.
"""
from dataclasses import asdict, replace

import yaml
from fleet_core.common.params import DEFAULT_PARAMS, FleetParams, load_params
from zone_harness import Harness

P = load_params()


def test_the_shipped_release_gap_is_the_footprint_plus_the_margin():
    assert (P.footprint_radius_m, P.release_margin_m) == (0.085, 0.05)
    assert "zone_clear_m" not in yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
    assert P.zone_clear_m == 0.0


def gap_at_release(params, zone="WORK_DOCK"):
    """도크에 선 pinky1이 출고로 떠날 때 zone을 놓은 주기의 (도크와의 거리, 그때 걸음 수)."""
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"}, params=params)
    world.report()
    call = world.call("WORK", "pinky1")
    assert world.run_until(lambda: world.result_of(call) is not None, 30.0)
    world.call("OUTBOUND", "pinky1")
    world.step()
    bot = world.bots["pinky1"]
    gaps = []
    while zone in world.view("pinky1")["held"]:
        gaps.append(world.site.distance_to_zone(bot.x, bot.y, zone))
        world.step()
        assert world.now < 120.0
    return gaps


def test_a_zone_behind_is_kept_until_the_whole_body_and_the_margin_are_out():
    gaps = gap_at_release(P)
    need = P.footprint_radius_m + P.release_margin_m
    outside = [gap for gap in gaps if gap >= need - 1e-9]
    assert len(outside) == P.release_confirm_samples - 1, "released on the third report out"
    assert max(gaps[:-len(outside)]) < need


def test_a_record_made_with_zone_clear_m_still_releases_at_it():
    fields = asdict(P)
    fields.update(zone_clear_m=0.10)
    gaps = gap_at_release(FleetParams(**fields))
    assert [gap >= 0.10 - 1e-9 for gap in gaps].count(True) == P.release_confirm_samples - 1


def test_a_docked_robot_is_measured_from_the_dock_without_the_margin():
    """도킹 성공이 자리를 보증한다: 도크 지점에서 발자국 반지름 넘게 떨어진 구역은 놓는다.

    W는 HALL에서 0.10 m, J_W에서 0.12 m다. 여유까지 더하면(0.135 m) 도크에 선 로봇이 갈림목을
    쥐고 있어 급행에서 물러나는 로봇이 J_W를 못 잡는다.
    """
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    call = world.call("WORK", "pinky2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    world.run(1.0)
    assert world.view("pinky2")["held"] == ["WORK_DOCK"]


def test_the_margin_is_a_parameter():
    wide = gap_at_release(replace(P, release_margin_m=0.10))
    assert len(wide) > len(gap_at_release(P))


def docked_at_work(params=P, error_cm=None):
    """pinky2가 W에 도킹을 마친 뒤 쥐고 있는 구역. error_cm(x 또는 (x, y))을 주면 도킹이 그 오차를
    보고한다."""
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"}, params=params)
    world.report()
    if error_cm is not None:
        real = world.traffic.on_dock_result
        x_cm, y_cm = error_cm if isinstance(error_cm, tuple) else (error_cm, 0.0)
        world.traffic.on_dock_result = lambda robot, goal_id, ok, *_: real(
            robot, goal_id, ok, x_cm, y_cm, 0.0)
    call = world.call("WORK", "pinky2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    world.run(1.0)
    return sorted(world.view("pinky2")["held"])


def test_a_docked_robot_keeps_what_its_reported_docking_error_may_still_touch():
    """도킹 결과는 제가 보고한 오차만큼만 자리를 보증한다. W는 HALL에서 0.10 m, J_W에서 0.12 m다."""
    assert docked_at_work(error_cm=1.0) == ["WORK_DOCK"]  # 0.085 + 0.01 < 0.10
    assert docked_at_work(error_cm=3.0) == ["HALL", "WORK_DOCK"]  # 0.115: HALL에 걸칠 수 있다
    assert docked_at_work(error_cm=4.0) == ["HALL", "J_W", "WORK_DOCK"]  # 0.125: 갈림목에도


def test_the_docking_error_counts_both_axes():
    assert docked_at_work(error_cm=(0.0, 3.0)) == ["HALL", "WORK_DOCK"]  # y만 3 cm
    assert docked_at_work(error_cm=(2.4, 3.2)) == ["HALL", "J_W", "WORK_DOCK"]  # 둘을 합쳐 4 cm


def test_a_record_made_with_zone_clear_m_releases_at_it_for_a_docked_robot_too():
    fields = asdict(P)
    fields.update(zone_clear_m=0.11)  # 옛 규칙: 보고 위치에서 0.11 m. HALL은 도크에서 0.10 m다
    assert docked_at_work(FleetParams(**fields)) == ["HALL", "WORK_DOCK"]


def test_a_record_made_with_zone_clear_m_measures_a_docked_robot_from_its_report():
    """옛 규칙은 도킹한 로봇도 도크 지점이 아니라 보고 위치에서 잰다: HALL에서 0.11 m 넘게 떨어진
    자리를 보고하면 HALL을 놓는다(도크 지점은 0.10 m라 거기서 재면 쥐고 있는다)."""
    fields = asdict(P)
    fields.update(zone_clear_m=0.11)
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"},
                    params=FleetParams(**fields))
    world.report()
    call = world.call("WORK", "pinky2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    bot = world.bots["pinky2"]
    bot.y += 0.03
    assert world.site.distance_to_zone(bot.x, bot.y, "HALL") > 0.11
    world.run(1.0)
    assert world.view("pinky2")["held"] == ["WORK_DOCK"]
