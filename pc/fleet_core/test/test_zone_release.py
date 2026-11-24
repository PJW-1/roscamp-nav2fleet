"""뒤 구역 놓기는 로봇 중심이 아니라 몸 전체가 빠져나간 뒤에 한다(footprint_radius_m + release_margin_m)."""
import math
from dataclasses import asdict, replace

import pytest
import yaml
from fleet_core.common.commands import MoveCmd, MoveOrderCmd, RetreatCmd
from fleet_core.common.params import DEFAULT_PARAMS, FleetParams, load_params
from zone_harness import Harness

P = load_params()
OFF = replace(P, use_order_model=False)


def test_the_shipped_release_gap_is_the_footprint_plus_the_margin():
    assert (P.footprint_radius_m, P.release_margin_m) == (0.085, 0.05)
    assert "zone_clear_m" not in yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
    assert P.zone_clear_m == 0.0


def gap_at_release(params, zone="WORK_DOCK"):
    """도크에 선 pode1이 출고로 떠날 때 zone을 놓은 주기의 (도크와의 거리, 그때 걸음 수)."""
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"}, params=params)
    world.report()
    call = world.call("WORK", "pode1")
    assert world.run_until(lambda: world.result_of(call) is not None, 30.0)
    world.call("OUTBOUND", "pode1")
    world.step()
    bot = world.bots["pode1"]
    gaps = []
    while zone in world.view("pode1")["held"]:
        gaps.append(world.site.distance_to_zone(bot.x, bot.y, zone))
        world.step()
        assert world.now < 120.0
    return gaps


def test_a_zone_behind_is_kept_until_the_whole_body_and_the_margin_are_out():
    """보고 위치로 재는 길(끈 길)을 본다. 노드 좌표 규칙은 아래 O_OUT 시험이 본다."""
    gaps = gap_at_release(OFF)
    need = P.footprint_radius_m + P.release_margin_m
    outside = [gap for gap in gaps if gap >= need - 1e-9]
    assert len(outside) == P.release_confirm_samples - 1, "released on the third report out"
    assert max(gaps[:-len(outside)]) < need


def test_with_the_order_model_the_zone_behind_is_kept_until_the_body_is_out():
    """켠 길(기본): 노드 좌표 규칙으로 여유보다 일찍 놓을 수 있지만 몸이 구역 밖에 나간 뒤다."""
    gaps = gap_at_release(replace(OFF, use_order_model=True))
    assert gaps[-1] >= P.footprint_radius_m
    assert max(gaps[:-1], default=0.0) < gaps[-1]


def test_a_record_made_with_zone_clear_m_still_releases_at_it():
    fields = asdict(P)
    fields.update(zone_clear_m=0.10)
    gaps = gap_at_release(FleetParams(**fields))
    assert [gap >= 0.10 - 1e-9 for gap in gaps].count(True) == P.release_confirm_samples - 1


def test_a_docked_robot_is_measured_from_the_dock_without_the_margin():
    """도킹 성공이 자리를 보증해, 도크 지점에서 발자국 반지름 넘게 떨어진 구역은 놓는다."""
    world = Harness(starts={"pode1": "P-1", "pode2": "P-2", "pode3": "P-3"})
    world.report()
    call = world.call("WORK", "pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    world.run(1.0)
    assert world.view("pode2")["held"] == ["WORK_DOCK"]


def stopped_short_of_o_out(params=P):
    """출고를 마친 pode1이 O_OUT에 0.04 m 못 미쳐 서고, 다음 goal을 받아 제자리에서 돈다."""
    world = Harness(layout="v7", starts={"pode1": "O", "pode2": "P-2", "pode3": "P-3"},
                    params=params)
    world.report()
    out = world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: world.result_of(out) is not None, 30.0)
    world.traffic.on_release("pode1")
    bot = world.bots["pode1"]
    assert world.run_until(lambda: bot.y <= 0.66, 30.0)
    assert world.site.distance_to_zone(bot.x, bot.y, "OUT_DOCK") < P.footprint_radius_m + 0.05
    assert "OUT_DOCK" in world.view("pode1")["held"]
    if bot.run is not None:
        bot.run.finish_segment()  # order: 로봇은 O_OUT 조각을 끝냈다고 본다
    bot.points = []
    world.speed = 0.0
    # 멈춘 자리는 O_OUT에서 0.04 못 미친 곳(구역에서 0.105)에 둔다
    bot.y = world.site.node_pose("O_OUT")[1] + 0.04
    world.run(1.0)
    assert world.view("pode1")["node"] == "O_OUT"
    assert bot.cmd is not None or bot.run is not None, "it has the next goal"
    return world


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_a_robot_standing_within_the_margin_of_its_node_is_measured_from_the_node(order):
    """O_OUT에 0.04 m 못 미쳐 선 로봇도 지점 좌표로 재어 출고 구역을 놓는다(지점이 구역에서 0.135 m 넘게 떨어져 있다)."""
    world = stopped_short_of_o_out(replace(P, use_order_model=order))
    assert "OUT_DOCK" not in world.view("pode1")["held"]
    assert not world.problems


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_a_robot_farther_than_the_margin_from_its_node_is_measured_from_its_report(order):
    world = stopped_short_of_o_out(replace(P, release_margin_m=0.03, use_order_model=order))
    assert "OUT_DOCK" in world.view("pode1")["held"], "0.04 m from the node, 0.105 m from the zone"


def test_a_record_made_with_zone_clear_m_measures_a_standing_robot_from_its_report():
    world = stopped_short_of_o_out(replace(P, zone_clear_m=0.11))
    assert "OUT_DOCK" in world.view("pode1")["held"]


def test_the_margin_is_a_parameter():
    wide = gap_at_release(replace(P, release_margin_m=0.10))
    assert len(wide) > len(gap_at_release(P))


def docked_at_work(params=P, error_cm=None):
    """pode2가 W에 도킹을 마친 뒤 쥐고 있는 구역. error_cm을 주면 도킹이 그 오차를 보고한다."""
    world = Harness(starts={"pode1": "P-1", "pode2": "P-2", "pode3": "P-3"}, params=params)
    world.report()
    if error_cm is not None:
        real = world.traffic.on_dock_result
        x_cm, y_cm = error_cm if isinstance(error_cm, tuple) else (error_cm, 0.0)
        world.traffic.on_dock_result = lambda robot, goal_id, ok, *_: real(
            robot, goal_id, ok, x_cm, y_cm, 0.0)
    call = world.call("WORK", "pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    world.run(1.0)
    return sorted(world.view("pode2")["held"])


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
    """옛 규칙은 도킹한 로봇도 보고 위치에서 재어, HALL에서 0.11 m 넘게 떨어진 보고면 HALL을 놓는다."""
    fields = asdict(P)
    fields.update(zone_clear_m=0.11)
    world = Harness(starts={"pode1": "P-1", "pode2": "P-2", "pode3": "P-3"},
                    params=FleetParams(**fields))
    world.report()
    call = world.call("WORK", "pode2")
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    bot = world.bots["pode2"]
    bot.y += 0.03
    assert world.site.distance_to_zone(bot.x, bot.y, "HALL") > 0.11
    world.run(1.0)
    assert world.view("pode2")["held"] == ["WORK_DOCK"]


def short_of_n_w(order: bool, short_m: float = 0.025):
    """W에서 나가는 pode1이 N_W에 short_m 못 미쳐 서고, 물러나기 조각을 끝냈다고 말하지 않는다."""
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"},
                    params=replace(P, use_order_model=order))
    world.report()
    kind = MoveOrderCmd if order else RetreatCmd
    assert world.run_until(lambda: world.sent(kind, "pode1"), 10.0)
    if order:
        base = world.sent(kind, "pode1")[0].base
        assert [(s.seq_id, s.mode) for s in base] == [(1, "retreat"), (2, "nav_goal")]
        assert base[0].end_node == "N_W"
    world._move = lambda bot: None  # 로봇이 멈추고 답하지 않는다
    (ax, ay), (bx, by) = world.site.edge("W", "N_W").points[-2:]
    span = math.dist((ax, ay), (bx, by))
    bot = world.bots["pode1"]
    bot.x, bot.y = bx - (bx - ax) * short_m / span, by - (by - ay) * short_m / span
    world.run(1.0)
    return world, bot


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_a_robot_backing_out_is_measured_from_its_report_until_the_retreat_is_done(order):
    """물러나는 로봇은 끝냈다는 답 전에는 N_W 좌표가 아니라 보고 위치로 재어 구역을 놓는다."""
    world, bot = short_of_n_w(order)
    need = P.footprint_radius_m + P.release_margin_m
    assert world.site.distance_to_zone(bot.x, bot.y, "J_W") < need
    held = world.view("pode1")["held"]
    assert "J_W" in held and "WORK_DOCK" in held, held
    assert "RETREAT_DONE" not in world.codes("pode1")
    del world._move
    assert world.run_until(lambda: "J_W" not in world.view("pode1")["held"], 30.0)
    assert "RETREAT_DONE" in world.codes("pode1") and not world.problems


def test_an_order_retreat_whose_answer_is_late_is_passed_once_the_robot_is_beyond_the_node():
    """끝냈다는 답이 늦어도 노드를 release_margin_m 넘게 지나가면 진행으로 넘긴다."""
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"},
                    params=replace(P, use_order_model=True))
    world.report()
    assert world.run_until(lambda: world.sent(MoveOrderCmd, "pode1"), 10.0)
    world.hold_answers = True
    bot, n_w = world.bots["pode1"], world.site.node_pose("N_W")
    assert world.run_until(lambda: world.view("pode1")["node"] == "N_W", 30.0)
    assert math.dist((bot.x, bot.y), n_w) > P.release_margin_m
    assert world.run_until(lambda: world.view("pode1")["held"] == ["HALL"]
                           and "N_W" not in world.view("pode1")["nodes"], 30.0)
    assert "SEGMENT_DONE" not in world.codes("pode1") and not world.problems


LATE_REPORT = (0.35, 0.69)  # N_W 0.04 m 위: 노드 좌표로 잴 거리 안
BODY_UP = (0.35, 0.745)  # 몸은 이미 도크 쪽으로 올라갔다(WORK_DOCK에서 0.067 m)


def late_report_after_backing_out(order: bool):
    """물러나기 결과가 온 뒤 보고가 N_W 앞에 늦게 머물러도 노드 좌표로 재어 WORK_DOCK을 놓지 않는다. 돌려준 world는 그 뒤 1초를 돈 상태다."""
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"},
                    params=replace(P, use_order_model=order))
    real = world.inputs.on_robot_state
    stuck = []

    def late(robot, x, y, *rest, **kw):
        if robot == "pode1" and (stuck or y <= LATE_REPORT[1]):
            stuck.append(True)
            x, y = LATE_REPORT
        return real(robot, x, y, *rest, **kw)

    world.inputs.on_robot_state = late
    world.report()
    world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: "RETREAT_DONE" in world.codes("pode1"), 30.0)
    held_at_done = list(world.view("pode1")["held"])
    world._move = lambda bot: None
    bot = world.bots["pode1"]
    bot.x, bot.y = BODY_UP
    world.run(1.0)
    return world, held_at_done


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_the_late_report_case_is_set_up_as_seen_in_the_field(order):
    """아래 xfail 시험의 앞 조건. xfail이 앞 단언의 실패를 가리지 않게 따로 본다."""
    world, held_at_done = late_report_after_backing_out(order)
    site, x_w = world.site, world.site.node_pose("X_W")
    assert "WORK_DOCK" in held_at_done
    assert world.traffic._robots["pode1"].node == "N_W"
    assert math.dist(LATE_REPORT, site.node_pose("N_W")) <= P.release_margin_m
    need = P.footprint_radius_m + P.release_margin_m
    assert site.distance_to_zone(*LATE_REPORT, "WORK_DOCK") < need, "the report alone keeps it"
    assert site.distance_to_zone(*BODY_UP, "WORK_DOCK") < 0.08
    if order:
        next_paths = [s.path for s in world.sent(MoveOrderCmd, "pode1")[0].base[1:]]
    else:
        next_paths = [c.path for c in world.sent(MoveCmd, "pode1")]
    assert any(tuple(p[:2]) == tuple(x_w) for path in next_paths for p in path), (
        "the next goal goes up past the dock through X_W")


@pytest.mark.xfail(strict=True, reason="남은 점: 끝낸 뒤 늦은 보고가 노드 앞이면 노드 좌표로 잰다")
@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_a_late_report_at_the_node_after_backing_out_does_not_free_the_dock(order):
    world, _ = late_report_after_backing_out(order)
    assert "WORK_DOCK" in world.view("pode1")["held"]
